`timescale 1ns/10ps
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
    output logic        p0_read_accept,
    output logic        p0_read_ready,
    output logic [3:0]  p0_rsp_gx, p0_rsp_gy, p0_rsp_gz, p0_rsp_slot,
    output logic [23:0] p0_node,
    output logic [15:0] p0_parent,
    output logic [3:0]  p0_size,

    // ---- PE p1 ----
    input  logic        p1_read_en,
    input  logic [3:0]  p1_gx, p1_gy, p1_gz, p1_slot,
    output logic        p1_read_accept,
    output logic        p1_read_ready,
    output logic [3:0]  p1_rsp_gx, p1_rsp_gy, p1_rsp_gz, p1_rsp_slot,
    output logic [23:0] p1_node,
    output logic [15:0] p1_parent,
    output logic [3:0]  p1_size,

    // ---- to TREEMEM port A ----
    output logic        t_read_en,
    output logic [3:0]  t_gx, t_gy, t_gz, t_slot,
    input  logic [23:0] t_node,
    input  logic [15:0] t_parent,
    input  logic [3:0]  t_size,

    // ---- opportunistic TREEMEM port B (CC has priority) ----
    input  logic        b_available,
    output logic        b_read_en,
    output logic [3:0]  b_gx, b_gy, b_gz, b_slot,
    input  logic [23:0] b_node,
    input  logic [15:0] b_parent,
    input  logic [3:0]  b_size
);
    logic pe_sel;
    logic a_sel_pe, b_sel_pe;
    logic a_sel_en, b_sel_en;
    logic a_to_p0, a_to_p1;
    logic b_to_p0, b_to_p1;
    logic [3:0] a_gx, a_gy, a_gz, a_slot;
    logic [3:0] a_rsp_gx_q, a_rsp_gy_q, a_rsp_gz_q, a_rsp_slot_q;
    logic [3:0] b_rsp_gx_q, b_rsp_gy_q, b_rsp_gz_q, b_rsp_slot_q;

    // Port A is always available.  When both contexts request and CC is not
    // reading this tree, port B serves the other context in the same cycle.
    always_comb begin
        a_sel_en = 1'b0;
        b_sel_en = 1'b0;
        a_sel_pe = pe_sel;
        b_sel_pe = ~pe_sel;

        if (p0_read_en && p1_read_en) begin
            a_sel_en = 1'b1;
            a_sel_pe = pe_sel;
            if (b_available) begin
                b_sel_en = 1'b1;
                b_sel_pe = ~pe_sel;
            end
        end else if (p0_read_en) begin
            a_sel_en = 1'b1;
            a_sel_pe = 1'b0;
        end else if (p1_read_en) begin
            a_sel_en = 1'b1;
            a_sel_pe = 1'b1;
        end

        if (a_sel_pe == 1'b0) begin
            a_gx=p0_gx; a_gy=p0_gy; a_gz=p0_gz; a_slot=p0_slot;
        end else begin
            a_gx=p1_gx; a_gy=p1_gy; a_gz=p1_gz; a_slot=p1_slot;
        end

        if (b_sel_pe == 1'b0) begin
            b_gx=p0_gx; b_gy=p0_gy; b_gz=p0_gz; b_slot=p0_slot;
        end else begin
            b_gx=p1_gx; b_gy=p1_gy; b_gz=p1_gz; b_slot=p1_slot;
        end
    end

    assign t_read_en = a_sel_en;
    assign t_gx = a_gx; assign t_gy = a_gy; assign t_gz = a_gz; assign t_slot = a_slot;
    assign b_read_en = b_sel_en;
    assign p0_read_accept = (a_sel_en && (a_sel_pe == 1'b0)) ||
                            (b_sel_en && (b_sel_pe == 1'b0));
    assign p1_read_accept = (a_sel_en && (a_sel_pe == 1'b1)) ||
                            (b_sel_en && (b_sel_pe == 1'b1));

    // Independent response tags keep the two registered memory ports aligned.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pe_sel        <= 1'b0;
            a_to_p0       <= 1'b0;
            a_to_p1       <= 1'b0;
            b_to_p0       <= 1'b0;
            b_to_p1       <= 1'b0;
            a_rsp_gx_q    <= 4'd0;
            a_rsp_gy_q    <= 4'd0;
            a_rsp_gz_q    <= 4'd0;
            a_rsp_slot_q  <= 4'd0;
            b_rsp_gx_q    <= 4'd0;
            b_rsp_gy_q    <= 4'd0;
            b_rsp_gz_q    <= 4'd0;
            b_rsp_slot_q  <= 4'd0;
        end else begin
            if (a_sel_en) pe_sel <= ~a_sel_pe;
            a_to_p0       <= a_sel_en && (a_sel_pe == 1'b0);
            a_to_p1       <= a_sel_en && (a_sel_pe == 1'b1);
            b_to_p0       <= b_sel_en && (b_sel_pe == 1'b0);
            b_to_p1       <= b_sel_en && (b_sel_pe == 1'b1);
            if (a_sel_en) begin
                a_rsp_gx_q   <= a_gx;
                a_rsp_gy_q   <= a_gy;
                a_rsp_gz_q   <= a_gz;
                a_rsp_slot_q <= a_slot;
            end
            if (b_sel_en) begin
                b_rsp_gx_q   <= b_gx;
                b_rsp_gy_q   <= b_gy;
                b_rsp_gz_q   <= b_gz;
                b_rsp_slot_q <= b_slot;
            end
        end
    end

    // Each PE receives at most one response per cycle.
    always_comb begin
        p0_read_ready = a_to_p0 || b_to_p0;
        p1_read_ready = a_to_p1 || b_to_p1;

        if (b_to_p0) begin
            p0_node=b_node; p0_parent=b_parent; p0_size=b_size;
            p0_rsp_gx=b_rsp_gx_q; p0_rsp_gy=b_rsp_gy_q;
            p0_rsp_gz=b_rsp_gz_q; p0_rsp_slot=b_rsp_slot_q;
        end else begin
            p0_node=t_node; p0_parent=t_parent; p0_size=t_size;
            p0_rsp_gx=a_rsp_gx_q; p0_rsp_gy=a_rsp_gy_q;
            p0_rsp_gz=a_rsp_gz_q; p0_rsp_slot=a_rsp_slot_q;
        end

        if (b_to_p1) begin
            p1_node=b_node; p1_parent=b_parent; p1_size=b_size;
            p1_rsp_gx=b_rsp_gx_q; p1_rsp_gy=b_rsp_gy_q;
            p1_rsp_gz=b_rsp_gz_q; p1_rsp_slot=b_rsp_slot_q;
        end else begin
            p1_node=t_node; p1_parent=t_parent; p1_size=t_size;
            p1_rsp_gx=a_rsp_gx_q; p1_rsp_gy=a_rsp_gy_q;
            p1_rsp_gz=a_rsp_gz_q; p1_rsp_slot=a_rsp_slot_q;
        end
    end

endmodule
