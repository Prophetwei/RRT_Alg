//======================================================================
//  CCREADCONT : synthesizable demux that routes the CC's single logical
//  tree read/write port to the correct physical tree (start / goal),
//  selected by cc_*_sel (0 = start, 1 = goal), and muxes the read data
//  back to the CC.  No arbitration is needed here (CC is the sole master
//  of both trees' port B and both write ports).
//======================================================================
module CCREADCONT (
    // ---- from CC : read ----
    input  logic        cc_read_en,
    input  logic        cc_read_sel,          // 0=start 1=goal
    input  logic [3:0]  cc_read_gx, cc_read_gy, cc_read_gz, cc_read_slot,
    output logic [23:0] cc_read_node,
    output logic [15:0] cc_read_parent,
    output logic [3:0]  cc_read_size,

    // ---- from CC : write ----
    input  logic        cc_write_en,
    input  logic        cc_write_sel,         // 0=start 1=goal
    input  logic [3:0]  cc_write_gx, cc_write_gy, cc_write_gz,
    input  logic [23:0] cc_write_node,
    input  logic [15:0] cc_write_parent,

    // ---- to START tree (port B + write) ----
    output logic        s_b_read_en,
    output logic [3:0]  s_b_gx, s_b_gy, s_b_gz, s_b_slot,
    input  logic [23:0] s_b_node,
    input  logic [15:0] s_b_parent,
    input  logic [3:0]  s_b_size,
    output logic        s_write_en,
    output logic [3:0]  s_write_gx, s_write_gy, s_write_gz,
    output logic [23:0] s_write_node,
    output logic [15:0] s_write_parent,

    // ---- to GOAL tree (port B + write) ----
    output logic        g_b_read_en,
    output logic [3:0]  g_b_gx, g_b_gy, g_b_gz, g_b_slot,
    input  logic [23:0] g_b_node,
    input  logic [15:0] g_b_parent,
    input  logic [3:0]  g_b_size,
    output logic        g_write_en,
    output logic [3:0]  g_write_gx, g_write_gy, g_write_gz,
    output logic [23:0] g_write_node,
    output logic [15:0] g_write_parent
);
    // ---- read demux ----
    assign s_b_read_en = cc_read_en && (cc_read_sel == 1'b0);
    assign g_b_read_en = cc_read_en && (cc_read_sel == 1'b1);
    assign s_b_gx = cc_read_gx;  assign s_b_gy = cc_read_gy;  assign s_b_gz = cc_read_gz;  assign s_b_slot = cc_read_slot;
    assign g_b_gx = cc_read_gx;  assign g_b_gy = cc_read_gy;  assign g_b_gz = cc_read_gz;  assign g_b_slot = cc_read_slot;

    // ---- read data mux (registered read: select on the return cycle is
    //      driven by cc_read_sel which the CC holds stable across REQ/RCV) ----
    assign cc_read_node   = (cc_read_sel == 1'b0) ? s_b_node   : g_b_node;
    assign cc_read_parent = (cc_read_sel == 1'b0) ? s_b_parent : g_b_parent;
    assign cc_read_size   = (cc_read_sel == 1'b0) ? s_b_size   : g_b_size;

    // ---- write demux ----
    assign s_write_en = cc_write_en && (cc_write_sel == 1'b0);
    assign g_write_en = cc_write_en && (cc_write_sel == 1'b1);
    assign s_write_gx = cc_write_gx;  assign s_write_gy = cc_write_gy;  assign s_write_gz = cc_write_gz;
    assign g_write_gx = cc_write_gx;  assign g_write_gy = cc_write_gy;  assign g_write_gz = cc_write_gz;
    assign s_write_node = cc_write_node;  assign s_write_parent = cc_write_parent;
    assign g_write_node = cc_write_node;  assign g_write_parent = cc_write_parent;

endmodule
