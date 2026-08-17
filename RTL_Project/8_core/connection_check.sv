`timescale 1ns/10ps
//======================================================================
//  CC : connection check + path builder for the dual-tree RRT
//
//  Per accepted node it:
//    1. arbitrates between the two PEs (round-robin)
//    2. writes the candidate into its OWN tree
//    3. scans the OTHER tree for a node within a +/-1 cube (connection)
//    4a. no match  -> ACK the PE (let it carry on growing)
//    4b. match     -> walk both trees to the roots and emit the full
//                     START..GOAL path into PATHMEM, then assert `path`
//
//  sel = 0 : candidate came from the start-tree PE
//  sel = 1 : candidate came from the goal-tree PE
//======================================================================
module CC (
    input  logic        clk,
    input  logic        rst_n,
    output logic        path,                 // 1 once full path is built

    // ---- start-tree PE hand-off ----
    input  logic        pe0_node_valid,
    input  logic [23:0] pe0_node,
    input  logic [9:0]  pe0_parent,
    output logic        pe0_ack,

    // ---- goal-tree PE hand-off ----
    input  logic        pe1_node_valid,
    input  logic [23:0] pe1_node,
    input  logic [9:0]  pe1_parent,
    output logic        pe1_ack,

    // ---- start tree (CC read/write port) ----
    output logic        s_cread_en,
    output logic [9:0]  s_cread_addr,
    input  logic [23:0] s_cread_node,
    input  logic [9:0]  s_cread_parent,
    input  logic [9:0]  s_tree_size,
    output logic        s_write_en,
    output logic [23:0] s_write_node,
    output logic [9:0]  s_write_parent,

    // ---- goal tree (CC read/write port) ----
    output logic        g_cread_en,
    output logic [9:0]  g_cread_addr,
    input  logic [23:0] g_cread_node,
    input  logic [9:0]  g_cread_parent,
    input  logic [9:0]  g_tree_size,
    output logic        g_write_en,
    output logic [23:0] g_write_node,
    output logic [9:0]  g_write_parent,

    // ---- path memory write ----
    output logic [1:0]  path_we,
    output logic [23:0] path_wnode
);
    localparam ARB=0, WR_CAND=1, SCAN_REQ=2, SCAN_CMP=3, ACK=4,
               BS_REQ=5, BS_CMP=6, BG_REQ=7, BG_CMP=8, PDONE=9;
    localparam [9:0] NONE = 10'h3FF;

    logic [3:0] state;
    logic       turn, sel;
    logic [23:0] cand_node;
    logic [9:0]  cand_parent, cand_idx;
    logic [9:0]  scan_idx, scan_size;
    logic [9:0]  cur_s, cur_g;

    // ---------- candidate compare (+/-1 cube) ----------
    function automatic logic within1(input [7:0] a, input [7:0] b);
        logic [7:0] d;
        d = (a >= b) ? (a - b) : (b - a);
        within1 = (d <= 8'd1);
    endfunction

    logic [23:0] scan_node;
    logic        match;
    assign scan_node = (sel == 1'b0) ? g_cread_node : s_cread_node;   // scan the OTHER tree
    assign match     = within1(cand_node[23:16], scan_node[23:16]) &&
                       within1(cand_node[15:8 ], scan_node[15:8 ]) &&
                       within1(cand_node[ 7:0 ], scan_node[ 7:0 ]);

    // ---------- control / datapath FSM ----------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ARB; turn <= 1'b0; sel <= 1'b0;
            cand_node <= 0; cand_parent <= 0; cand_idx <= 0;
            scan_idx <= 0; scan_size <= 0; cur_s <= 0; cur_g <= 0;
        end else begin
            case (state)
                ARB: begin
                    if (turn == 1'b0) begin
                        if (pe0_node_valid)      begin sel<=1'b0; cand_node<=pe0_node; cand_parent<=pe0_parent; turn<=1'b1; state<=WR_CAND; end
                        else if (pe1_node_valid) begin sel<=1'b1; cand_node<=pe1_node; cand_parent<=pe1_parent; turn<=1'b0; state<=WR_CAND; end
                    end else begin
                        if (pe1_node_valid)      begin sel<=1'b1; cand_node<=pe1_node; cand_parent<=pe1_parent; turn<=1'b0; state<=WR_CAND; end
                        else if (pe0_node_valid) begin sel<=1'b0; cand_node<=pe0_node; cand_parent<=pe0_parent; turn<=1'b1; state<=WR_CAND; end
                    end
                end

                WR_CAND: begin
                    // cand is written this cycle (combinational write_en below);
                    // its index is the current size, scan the opposite tree next.
                    cand_idx  <= (sel == 1'b0) ? s_tree_size : g_tree_size;
                    scan_idx  <= 10'd0;
                    scan_size <= (sel == 1'b0) ? g_tree_size : s_tree_size;
                    state     <= SCAN_REQ;
                end

                SCAN_REQ: state <= SCAN_CMP;

                SCAN_CMP: begin
                    if (match) begin
                        if (sel == 1'b0) begin cur_s <= cand_idx; cur_g <= scan_idx; end
                        else             begin cur_s <= scan_idx; cur_g <= cand_idx; end
                        state <= BS_REQ;
                    end else if (scan_idx >= scan_size - 1'b1) begin
                        state <= ACK;
                    end else begin
                        scan_idx <= scan_idx + 1'b1;
                        state    <= SCAN_REQ;
                    end
                end

                ACK: state <= ARB;

                // ----- build start half : walk start tree to root, front-insert -----
                BS_REQ: state <= BS_CMP;
                BS_CMP: begin
                    if (s_cread_parent == NONE) state <= BG_REQ;
                    else begin cur_s <= s_cread_parent; state <= BS_REQ; end
                end

                // ----- build goal half : walk goal tree to root, append -----
                BG_REQ: state <= BG_CMP;
                BG_CMP: begin
                    if (g_cread_parent == NONE) state <= PDONE;
                    else begin cur_g <= g_cread_parent; state <= BG_REQ; end
                end

                PDONE: state <= PDONE;
                default: state <= ARB;
            endcase
        end
    end

    // ---------- combinational outputs ----------
    always_comb begin
        s_cread_en=0; s_cread_addr=0; s_write_en=0; s_write_node=0; s_write_parent=0;
        g_cread_en=0; g_cread_addr=0; g_write_en=0; g_write_node=0; g_write_parent=0;
        path_we=2'b00; path_wnode=0;
        pe0_ack=0; pe1_ack=0; path=0;

        case (state)
            WR_CAND: begin
                if (sel == 1'b0) begin s_write_en=1; s_write_node=cand_node; s_write_parent=cand_parent; end
                else             begin g_write_en=1; g_write_node=cand_node; g_write_parent=cand_parent; end
            end
            SCAN_REQ: begin
                if (sel == 1'b0) begin g_cread_en=1; g_cread_addr=scan_idx; end
                else             begin s_cread_en=1; s_cread_addr=scan_idx; end
            end
            BS_REQ: begin s_cread_en=1; s_cread_addr=cur_s; end
            BS_CMP: begin path_we=2'b01; path_wnode=s_cread_node; end
            BG_REQ: begin g_cread_en=1; g_cread_addr=cur_g; end
            BG_CMP: begin path_we=2'b10; path_wnode=g_cread_node; end
            ACK:    begin pe0_ack=(sel==1'b0); pe1_ack=(sel==1'b1); end
            PDONE:  path=1;
            default:;
        endcase
    end

endmodule
