`timescale 1ns/10ps
//======================================================================
//  CC : connection check + path builder for the dual-tree RRT (4 PEs)
//  [ rotating-priority arbitration + boundary-aware neighbourhood scan ]
//
//  Per accepted node it:
//    1. round-robin arbitrates the four PEs (rotating priority).  cur_pe =
//       which PE, sel = which tree owns it (0 = start : PE0/PE1,
//       1 = goal : PE2/PE3)
//    2. reads the node's own-tree cell occupancy, then writes the node
//       into its OWN tree at the next free slot
//    3. scans only the OTHER-tree cells that can contain a node within a
//       +/-1 voxel cube.  A neighbouring grid cell is visited only when the
//       candidate lies on offset 0/15 of that grid axis (1..8 cells total).
//    4a. no match -> ACK the source PE
//    4b. match    -> walk both trees to their roots and emit the full
//                    START..GOAL path into PATHMEM, then assert `path`
//
//  TIMING: every tree read is PIPELINED -- the memory return
//  (cc_read_node/parent/size) is REGISTERED in the *_RCV cycle and only
//  consumed one cycle later in the *_CMP cycle.  No combinational logic
//  (compare / next-state / register update) ever uses the external memory
//  output in the cycle it arrives.  Read sequence per access:
//      *_REQ  (drive address) -> *_RCV (latch return) -> *_CMP (use it)
//======================================================================
module CC (
    input  logic        clk,
    input  logic        rst_n,
    output logic        path,

    // ---- PE hand-offs ----
    input  logic        pe0_node_valid,  input logic [23:0] pe0_node,  input logic [15:0] pe0_parent,  output logic pe0_ack,
    input  logic        pe1_node_valid,  input logic [23:0] pe1_node,  input logic [15:0] pe1_parent,  output logic pe1_ack,
    input  logic        pe2_node_valid,  input logic [23:0] pe2_node,  input logic [15:0] pe2_parent,  output logic pe2_ack,
    input  logic        pe3_node_valid,  input logic [23:0] pe3_node,  input logic [15:0] pe3_parent,  output logic pe3_ack,

    // ---- tree read port (to CCREADCONT) ----
    output logic        cc_read_en,
    output logic        cc_read_sel,       // 0=start 1=goal
    output logic [3:0]  cc_read_gx, cc_read_gy, cc_read_gz, cc_read_slot,
    input  logic [23:0] cc_read_node,
    input  logic [15:0] cc_read_parent,
    input  logic [3:0]  cc_read_size,

    // ---- tree write port (to CCREADCONT) ----
    output logic        cc_write_en,
    output logic        cc_write_sel,      // 0=start 1=goal
    output logic [3:0]  cc_write_gx, cc_write_gy, cc_write_gz,
    output logic [23:0] cc_write_node,
    output logic [15:0] cc_write_parent,

    // ---- path memory write ----
    output logic [1:0]  path_we,
    output logic [23:0] path_wnode
);
    //  read sequence is 3 states: REQ (issue) -> RCV (latch return) -> CMP (use)
    localparam ARB=0, RD_SIZE=1, RCV_SIZE=2, CMP_SIZE=3, WR_CAND=4,
               SCAN_REQ=5, SCAN_RCV=6, SCAN_CMP=7, ACK=8,
               BS_REQ=9, BS_RCV=10, BS_CMP=11, BG_REQ=12, BG_RCV=13, BG_CMP=14, PDONE=15;
    logic [3:0] state, n_state;

    localparam [15:0] NONE = 16'hFFFF;

    logic [1:0]  arb_ptr, cur_pe;
    logic        sel;                     // own tree of the candidate
    logic [23:0] cand_node;
    logic [15:0] cand_parent;
    logic [3:0]  cand_slot;               // own-tree free slot (= size before write)

    logic [3:0]  sgx, sgy, sgz, sslot;    // opposite-tree scan cursor
    logic [3:0]  scan_size;
    logic [15:0] cur_s, cur_g;            // path-walk pointers

    // ---- PIPELINE registers for the tree-read return (break memory->logic path)
    logic [23:0] rd_node_q;
    logic [15:0] rd_parent_q;
    logic [3:0]  rd_size_q;

    // candidate cell (grid coords = voxel[7:4])
    wire [3:0] ccx = cand_node[23:20];
    wire [3:0] ccy = cand_node[15:12];
    wire [3:0] ccz = cand_node[7:4];

    // A +/-1 voxel match crosses a 16-voxel grid boundary only when the
    // candidate's in-cell offset is 0 or 15.  This is exact, not heuristic.
    function automatic [3:0] scan_lo(input [3:0] c, input [3:0] offset);
        scan_lo = (offset == 4'd0 && c > 4'd0) ? (c - 4'd1) : c;
    endfunction
    function automatic [3:0] scan_hi(input [3:0] c, input [3:0] offset);
        scan_hi = (offset == 4'd15 && c < 4'd15) ? (c + 4'd1) : c;
    endfunction
    wire [3:0] sxlo = scan_lo(ccx, cand_node[19:16]);
    wire [3:0] sylo = scan_lo(ccy, cand_node[11:8]);
    wire [3:0] szlo = scan_lo(ccz, cand_node[3:0]);
    wire [3:0] sxhi = scan_hi(ccx, cand_node[19:16]);
    wire [3:0] syhi = scan_hi(ccy, cand_node[11:8]);
    wire [3:0] szhi = scan_hi(ccz, cand_node[3:0]);

    // ---- +/-1 voxel connection test (on the REGISTERED read node) ----
    function automatic logic within1(input [7:0] a, input [7:0] b);
        logic [7:0] d; d = (a >= b) ? (a - b) : (b - a); within1 = (d <= 8'd1);
    endfunction

    logic [3:0] scan_now;
    logic       scan_valid, scan_last, is_match;
    assign scan_now   = (sslot == 4'd0) ? rd_size_q : scan_size;
    assign scan_valid = (sslot < scan_now);
    assign scan_last  = ((sslot + 4'd1) >= scan_now);
    assign is_match   = scan_valid &&
                        within1(cand_node[23:16], rd_node_q[23:16]) &&
                        within1(cand_node[15:8 ], rd_node_q[15:8 ]) &&
                        within1(cand_node[7:0  ], rd_node_q[7:0  ]);

    // ---- round-robin PE pick (rotating priority) ----
    logic [3:0] pe_valid;
    assign pe_valid = {pe3_node_valid, pe2_node_valid, pe1_node_valid, pe0_node_valid};
    logic       pick_valid;
    logic [1:0] pick;
    always @(*) begin
        pick = arb_ptr; pick_valid = 1'b0;
        for (int k = 0; k < 4; k++) begin
            if (!pick_valid && pe_valid[(arb_ptr + k[1:0])]) begin
                pick = arb_ptr + k[1:0]; pick_valid = 1'b1;
            end
        end
    end

    logic [23:0] pick_node;
    logic [15:0] pick_parent;
    always @(*) begin
        case (pick)
            2'd0: begin pick_node=pe0_node; pick_parent=pe0_parent; end
            2'd1: begin pick_node=pe1_node; pick_parent=pe1_parent; end
            2'd2: begin pick_node=pe2_node; pick_parent=pe2_parent; end
            2'd3: begin pick_node=pe3_node; pick_parent=pe3_parent; end
        endcase
    end

    //==================================================================
    //  PIPELINE stage : latch the tree-read return in every *_RCV cycle.
    //  Everything downstream uses rd_*_q (never cc_read_* directly).
    //==================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_node_q   <= 24'd0;
            rd_parent_q <= 16'd0;
            rd_size_q   <= 4'd0;
        end else begin
            case (state)
                RCV_SIZE: begin
                    // This access only queries occupancy.  An empty cell's
                    // payload is unwritten memory and must not enter control.
                    rd_node_q   <= 24'd0;
                    rd_parent_q <= 16'd0;
                    rd_size_q   <= cc_read_size;
                end
                SCAN_RCV: begin
                    rd_size_q <= cc_read_size;
                    if (sslot < cc_read_size) begin
                        rd_node_q   <= cc_read_node;
                        rd_parent_q <= cc_read_parent;
                    end else begin
                        rd_node_q   <= 24'd0;
                        rd_parent_q <= 16'd0;
                    end
                end
                BS_RCV, BG_RCV: begin
                    rd_node_q   <= cc_read_node;
                    rd_parent_q <= cc_read_parent;
                    rd_size_q   <= cc_read_size;
                end
                default: ;
            endcase
        end
    end

    //==================================================================
    //  next-state  (decisions use only registered data: rd_*_q, is_match)
    //==================================================================
    always_comb begin
        n_state = state;
        case (state)
            ARB:      n_state = pick_valid ? RD_SIZE : ARB;
            RD_SIZE:  n_state = RCV_SIZE;
            RCV_SIZE: n_state = CMP_SIZE;                                 // latch only
            CMP_SIZE: n_state = (rd_size_q >= 4'd8) ? ACK : WR_CAND;       // cell full -> drop
            WR_CAND:  n_state = SCAN_REQ;
            SCAN_REQ: n_state = SCAN_RCV;
            SCAN_RCV: n_state = SCAN_CMP;                                 // latch only
            SCAN_CMP: begin
                if (is_match)                 n_state = BS_REQ;
                else if (!scan_last)          n_state = SCAN_REQ;
                else if (sgz < szhi)          n_state = SCAN_REQ;
                else if (sgy < syhi)          n_state = SCAN_REQ;
                else if (sgx < sxhi)          n_state = SCAN_REQ;
                else                          n_state = ACK;             // neighbourhood exhausted
            end
            ACK:      n_state = ARB;
            BS_REQ:   n_state = BS_RCV;
            BS_RCV:   n_state = BS_CMP;                                   // latch only
            BS_CMP:   n_state = (rd_parent_q == NONE) ? BG_REQ : BS_REQ;
            BG_REQ:   n_state = BG_RCV;
            BG_RCV:   n_state = BG_CMP;                                   // latch only
            BG_CMP:   n_state = (rd_parent_q == NONE) ? PDONE : BG_REQ;
            PDONE:    n_state = PDONE;
            default:  n_state = ARB;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= ARB;
        else        state <= n_state;
    end

    //==================================================================
    //  Sequential state is split by role to keep the control cones compact.
    //==================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_ptr <= 2'd0;
        end else if (state == ARB) begin
            if (pick_valid) arb_ptr <= pick + 2'd1;
            else            arb_ptr <= arb_ptr + 2'd1;
        end
    end

    // Arbitration identity is control state and needs a known reset value.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_pe <= 2'd0;
            sel    <= 1'b0;
        end else if (state == ARB && pick_valid) begin
            cur_pe <= pick;
            sel    <= pick[1];               // PE0/1 -> start, PE2/3 -> goal
        end
    end

    // Candidate payload is reset along with the rest of the sequential state.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cand_node   <= 24'd0;
            cand_parent <= 16'd0;
        end else if (state == ARB && pick_valid) begin
            cand_node   <= pick_node;
            cand_parent <= pick_parent;
        end
    end

    // Scan cursors directly control memory addresses and FSM decisions.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cand_slot <= 4'd0;
            sgx       <= 4'd0;
            sgy       <= 4'd0;
            sgz       <= 4'd0;
            sslot     <= 4'd0;
            scan_size <= 4'd0;
        end else begin
            case (state)
                CMP_SIZE: begin
                    cand_slot <= rd_size_q;           // free slot in own cell
                    sgx       <= sxlo;
                    sgy       <= sylo;
                    sgz       <= szlo;
                    sslot     <= 4'd0;
                    scan_size <= 4'd0;
                end
                SCAN_CMP: begin
                    if (sslot == 4'd0) scan_size <= rd_size_q;

                    // A match leaves the scan immediately, so advancing the
                    // cursor is harmless.  Keeping this independent of
                    // is_match removes rd_node_q from the cursor timing cone.
                    if (!scan_last) begin
                        sslot <= sslot + 4'd1;
                    end else begin
                        sslot <= 4'd0;
                        if (sgz < szhi) begin
                            sgz <= sgz + 4'd1;
                        end else begin
                            sgz <= szlo;
                            if (sgy < syhi) begin
                                sgy <= sgy + 4'd1;
                            end else begin
                                sgy <= sylo;
                                if (sgx < sxhi) sgx <= sgx + 4'd1;
                            end
                        end
                    end
                end
                default: ;
            endcase
        end
    end

    // Path pointers are memory addresses, not payload-only registers.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_s <= 16'd0;
            cur_g <= 16'd0;
        end else begin
            case (state)
                SCAN_CMP: if (is_match) begin
                    if (sel == 1'b0) begin
                        cur_s <= {ccx, ccy, ccz, cand_slot};
                        cur_g <= {sgx, sgy, sgz, sslot};
                    end else begin
                        cur_g <= {ccx, ccy, ccz, cand_slot};
                        cur_s <= {sgx, sgy, sgz, sslot};
                    end
                end
                BS_CMP: if (rd_parent_q != NONE) cur_s <= rd_parent_q;
                BG_CMP: if (rd_parent_q != NONE) cur_g <= rd_parent_q;
                default: ;
            endcase
        end
    end

    //==================================================================
    //  combinational outputs
    //  cc_read_sel is held across REQ *and* RCV so CCREADCONT's return mux
    //  picks the tree actually read; the CMP cycles use rd_*_q instead.
    //==================================================================
    always @(*) begin
        cc_read_en=0; cc_read_sel=0; cc_read_gx=0; cc_read_gy=0; cc_read_gz=0; cc_read_slot=0;
        cc_write_en=0; cc_write_sel=0; cc_write_gx=0; cc_write_gy=0; cc_write_gz=0;
        cc_write_node=0; cc_write_parent=0;
        path_we=2'b00; path_wnode=0;
        pe0_ack=0; pe1_ack=0; pe2_ack=0; pe3_ack=0; path=0;

        case (state)
            RD_SIZE: begin
                cc_read_en=1; cc_read_sel=sel;                 // own-tree occupancy read
                cc_read_gx=ccx; cc_read_gy=ccy; cc_read_gz=ccz; cc_read_slot=4'd0;
            end
            RCV_SIZE: cc_read_sel=sel;                         // hold return mux while latching
            WR_CAND: begin
                cc_write_en=1; cc_write_sel=sel;
                cc_write_gx=ccx; cc_write_gy=ccy; cc_write_gz=ccz;
                cc_write_node=cand_node; cc_write_parent=cand_parent;
            end
            SCAN_REQ: begin
                cc_read_en=1; cc_read_sel=~sel;                // scan the OTHER tree
                cc_read_gx=sgx; cc_read_gy=sgy; cc_read_gz=sgz; cc_read_slot=sslot;
            end
            SCAN_RCV: cc_read_sel=~sel;                        // hold return mux while latching
            ACK: begin
                pe0_ack=(cur_pe==2'd0); pe1_ack=(cur_pe==2'd1);
                pe2_ack=(cur_pe==2'd2); pe3_ack=(cur_pe==2'd3);
            end
            BS_REQ: begin
                cc_read_en=1; cc_read_sel=1'b0;                // start tree
                cc_read_gx=cur_s[15:12]; cc_read_gy=cur_s[11:8]; cc_read_gz=cur_s[7:4]; cc_read_slot=cur_s[3:0];
            end
            BS_RCV: cc_read_sel=1'b0;                          // hold return mux while latching
            BS_CMP: begin path_we=2'b01; path_wnode=rd_node_q; end     // front-insert start half
            BG_REQ: begin
                cc_read_en=1; cc_read_sel=1'b1;                // goal tree
                cc_read_gx=cur_g[15:12]; cc_read_gy=cur_g[11:8]; cc_read_gz=cur_g[7:4]; cc_read_slot=cur_g[3:0];
            end
            BG_RCV: cc_read_sel=1'b1;                          // hold return mux while latching
            BG_CMP: begin path_we=2'b10; path_wnode=rd_node_q; end     // append goal half
            PDONE:  path=1;
            default: ;
        endcase
    end

endmodule
