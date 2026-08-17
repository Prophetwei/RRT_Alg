//======================================================================
//  TREECONT : synthesizable 2->1 round-robin arbiter for ONE tree's
//  PE nearest-neighbour read port.  Instantiated twice (start / goal),
//  each serving its two PEs.  Arbitration is work-conserving round robin:
//  an idle preference slot never blocks the other requester.  The PE selected
//  at cycle t gets its ready+data at t+1.
//======================================================================
module TREECONT (
    input  logic        clk,
    input  logic        rst_n,

    // ---- PE p0 ----
    input  logic        p0_read_en,
    input  logic [3:0]  p0_gx, p0_gy, p0_gz, p0_slot,
    output logic        p0_read_ready,
    output logic [23:0] p0_node,
    output logic [15:0] p0_parent,
    output logic [3:0]  p0_size,

    // ---- PE p1 ----
    input  logic        p1_read_en,
    input  logic [3:0]  p1_gx, p1_gy, p1_gz, p1_slot,
    output logic        p1_read_ready,
    output logic [23:0] p1_node,
    output logic [15:0] p1_parent,
    output logic [3:0]  p1_size,

    // ---- to TREEMEM port A ----
    output logic        t_read_en,
    output logic [3:0]  t_gx, t_gy, t_gz, t_slot,
    input  logic [23:0] t_node,
    input  logic [15:0] t_parent,
    input  logic [3:0]  t_size
);
    logic       pe_sel;        // round-robin preference
    logic       sel_pe;
    logic       grant_pe;
    logic       grant_valid;

    logic        sel_en;
    logic [3:0]  sel_gx, sel_gy, sel_gz, sel_slot;

    // Combinational request selection.  The PE receiving the current response
    // is masked because its level request drops only on this clock edge.
    always_comb begin
        sel_en = 1'b0;
        sel_pe = pe_sel;

        if (p0_read_en && !(grant_valid && grant_pe == 1'b0) &&
            p1_read_en && !(grant_valid && grant_pe == 1'b1)) begin
            sel_en = 1'b1;
            sel_pe = pe_sel;
        end else if (p0_read_en && !(grant_valid && grant_pe == 1'b0)) begin
            sel_en = 1'b1;
            sel_pe = 1'b0;
        end else if (p1_read_en && !(grant_valid && grant_pe == 1'b1)) begin
            sel_en = 1'b1;
            sel_pe = 1'b1;
        end

        if (sel_pe == 1'b0) begin
            sel_gx=p0_gx; sel_gy=p0_gy; sel_gz=p0_gz; sel_slot=p0_slot;
        end else begin
            sel_gx=p1_gx; sel_gy=p1_gy; sel_gz=p1_gz; sel_slot=p1_slot;
        end
    end

    assign t_read_en = sel_en;
    assign t_gx = sel_gx;  assign t_gy = sel_gy;  assign t_gz = sel_gz;  assign t_slot = sel_slot;

    // Sequential response tag and next round-robin preference.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pe_sel      <= 1'b0;
            grant_pe    <= 1'b0;
            grant_valid <= 1'b0;
        end else begin
            if (sel_en) pe_sel <= ~sel_pe;
            grant_pe    <= sel_pe;
            grant_valid <= sel_en;
        end
    end

    // returned data (registered in TREEMEM) routed to the granted PE
    always_comb begin
        p0_read_ready = grant_valid && (grant_pe == 1'b0);
        p1_read_ready = grant_valid && (grant_pe == 1'b1);
    end

    assign p0_node = t_node;  assign p0_parent = t_parent;  assign p0_size = t_size;
    assign p1_node = t_node;  assign p1_parent = t_parent;  assign p1_size = t_size;

endmodule
