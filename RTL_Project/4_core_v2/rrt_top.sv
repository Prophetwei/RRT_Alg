`timescale 1ns/10ps
`include "PE.sv"
`include "connection_check.sv"
`include "TREECONT.sv"
`include "MAPCONT.sv"
`include "CCREADCONT.sv"

//======================================================================
//  RRT_TOP : synthesizable 4-core dual-tree RRT controller (LOGIC ONLY)
//
//  Contains every non-memory (synthesizable) block:
//     * 4x PE          (PE0,PE1 grow START tree; PE2,PE3 grow GOAL tree)
//     * 2x TREECONT     arbiters (2 PEs share each tree's NN read port)
//     * 1x MAPCONT      arbiter (4 PEs share the obstacle-map read port)
//     * 1x CC           connection check + path builder
//     * 1x CCREADCONT   routes CC's read/write to the correct tree
//
//  All actual storage (2x TREEMEM, MAPMEM, PATHMEM) is external and wired
//  in by TESTBED, so this core is synthesizable on its own (SRAM macros
//  attach to these ports on a real chip).
//======================================================================
module RRT_TOP (
    input  logic        clk,
    input  logic        rst_n,

    // ---- control / handshake (4 PEs) ----
    input  logic in_valid0, output logic out_valid0, input logic [23:0] rand_num0,
    input  logic in_valid1, output logic out_valid1, input logic [23:0] rand_num1,
    input  logic in_valid2, output logic out_valid2, input logic [23:0] rand_num2,
    input  logic in_valid3, output logic out_valid3, input logic [23:0] rand_num3,

    // ================= START TREEMEM =================
    // ---- tree sizes (for CC balanced selection) ----
    input  logic [15:0] s_tree_size,
    input  logic [15:0] g_tree_size,

    output logic        s_a_read_en,                         // port A (PE NN, via TREECONT)
    output logic [3:0]  s_a_gx, s_a_gy, s_a_gz, s_a_slot,
    input  logic [23:0] s_a_node,
    input  logic [15:0] s_a_parent,
    input  logic [3:0]  s_a_size,
    output logic        s_b_read_en,                         // port B (CC)
    output logic [3:0]  s_b_gx, s_b_gy, s_b_gz, s_b_slot,
    input  logic [23:0] s_b_node,
    input  logic [15:0] s_b_parent,
    input  logic [3:0]  s_b_size,
    output logic        s_write_en,                          // write
    output logic [3:0]  s_write_gx, s_write_gy, s_write_gz,
    output logic [23:0] s_write_node,
    output logic [15:0] s_write_parent,

    // ================= GOAL TREEMEM ==================
    output logic        g_a_read_en,
    output logic [3:0]  g_a_gx, g_a_gy, g_a_gz, g_a_slot,
    input  logic [23:0] g_a_node,
    input  logic [15:0] g_a_parent,
    input  logic [3:0]  g_a_size,
    output logic        g_b_read_en,
    output logic [3:0]  g_b_gx, g_b_gy, g_b_gz, g_b_slot,
    input  logic [23:0] g_b_node,
    input  logic [15:0] g_b_parent,
    input  logic [3:0]  g_b_size,
    output logic        g_write_en,
    output logic [3:0]  g_write_gx, g_write_gy, g_write_gz,
    output logic [23:0] g_write_node,
    output logic [15:0] g_write_parent,

    // ================= MAP (shared) ==================
    output logic        map_read_en,
    output logic [7:0]  map_read_x, map_read_y, map_read_z,
    input  logic        map_obstacle,

    // ================= PATH ==========================
    output logic [1:0]  path_we,
    output logic [23:0] path_wnode,

    // ================= STATUS ========================
    output logic        path
);

    // -------- PE <-> TREECONT (NN read) --------
    logic        p0_req, p1_req, p2_req, p3_req;
    logic [3:0]  p0_gx,p0_gy,p0_gz,p0_slot, p1_gx,p1_gy,p1_gz,p1_slot;
    logic [3:0]  p2_gx,p2_gy,p2_gz,p2_slot, p3_gx,p3_gy,p3_gz,p3_slot;
    logic        p0_rdy, p1_rdy, p2_rdy, p3_rdy;
    logic [23:0] p0_tnode, p1_tnode, p2_tnode, p3_tnode;
    logic [15:0] p0_tpar,  p1_tpar,  p2_tpar,  p3_tpar;
    logic [3:0]  p0_tsize, p1_tsize, p2_tsize, p3_tsize;

    // -------- PE <-> CC (node handoff) --------
    logic        p0_nv,p1_nv,p2_nv,p3_nv, p0_ack,p1_ack,p2_ack,p3_ack;
    logic [23:0] p0_node,p1_node,p2_node,p3_node;
    logic [15:0] p0_par, p1_par, p2_par, p3_par;

    // -------- PE <-> MAPCONT --------
    logic        m0_en,m1_en,m2_en,m3_en, m0_rdy,m1_rdy,m2_rdy,m3_rdy, m0_ob,m1_ob,m2_ob,m3_ob;
    logic [23:0] m0_node,m1_node,m2_node,m3_node;

    // -------- CC <-> CCREADCONT --------
    logic        cc_rd_en, cc_rd_sel;
    logic [3:0]  cc_rd_gx,cc_rd_gy,cc_rd_gz,cc_rd_slot;
    logic [23:0] cc_rd_node;  logic [15:0] cc_rd_par;  logic [3:0] cc_rd_size;
    logic        cc_wr_en, cc_wr_sel;
    logic [3:0]  cc_wr_gx,cc_wr_gy,cc_wr_gz;
    logic [23:0] cc_wr_node;  logic [15:0] cc_wr_par;

    //==================================================================
    //  PEs
    //==================================================================
    PE #(.SEED(8'd255), .STEP_DOWN(-5'sd1)) u_PE0 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid0), .out_valid(out_valid0), .rand_num(rand_num0),
        .read_tree_req(p0_req), .read_gx(p0_gx), .read_gy(p0_gy), .read_gz(p0_gz), .read_slot(p0_slot),
        .read_ready(p0_rdy), .tree_node(p0_tnode), .tree_parent(p0_tpar), .tree_size(p0_tsize),
        .node_valid(p0_nv), .node_out(p0_node), .parent_out(p0_par), .node_ack(p0_ack),
        .read_map_en(m0_en), .map_node(m0_node), .map_ready(m0_rdy), .obstacle(m0_ob)
    );
    PE #(.SEED(8'd255), .STEP_DOWN(-5'sd1)) u_PE1 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid1), .out_valid(out_valid1), .rand_num(rand_num1),
        .read_tree_req(p1_req), .read_gx(p1_gx), .read_gy(p1_gy), .read_gz(p1_gz), .read_slot(p1_slot),
        .read_ready(p1_rdy), .tree_node(p1_tnode), .tree_parent(p1_tpar), .tree_size(p1_tsize),
        .node_valid(p1_nv), .node_out(p1_node), .parent_out(p1_par), .node_ack(p1_ack),
        .read_map_en(m1_en), .map_node(m1_node), .map_ready(m1_rdy), .obstacle(m1_ob)
    );
    PE #(.SEED(8'd0), .STEP_DOWN(5'sd1)) u_PE2 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid2), .out_valid(out_valid2), .rand_num(rand_num2),
        .read_tree_req(p2_req), .read_gx(p2_gx), .read_gy(p2_gy), .read_gz(p2_gz), .read_slot(p2_slot),
        .read_ready(p2_rdy), .tree_node(p2_tnode), .tree_parent(p2_tpar), .tree_size(p2_tsize),
        .node_valid(p2_nv), .node_out(p2_node), .parent_out(p2_par), .node_ack(p2_ack),
        .read_map_en(m2_en), .map_node(m2_node), .map_ready(m2_rdy), .obstacle(m2_ob)
    );
    PE #(.SEED(8'd0), .STEP_DOWN(5'sd1)) u_PE3 (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid3), .out_valid(out_valid3), .rand_num(rand_num3),
        .read_tree_req(p3_req), .read_gx(p3_gx), .read_gy(p3_gy), .read_gz(p3_gz), .read_slot(p3_slot),
        .read_ready(p3_rdy), .tree_node(p3_tnode), .tree_parent(p3_tpar), .tree_size(p3_tsize),
        .node_valid(p3_nv), .node_out(p3_node), .parent_out(p3_par), .node_ack(p3_ack),
        .read_map_en(m3_en), .map_node(m3_node), .map_ready(m3_rdy), .obstacle(m3_ob)
    );

    //==================================================================
    //  Tree NN read arbiters (PE0,PE1 -> START ; PE2,PE3 -> GOAL)
    //==================================================================
    TREECONT u_TC_S (
        .clk(clk), .rst_n(rst_n),
        .p0_read_en(p0_req), .p0_gx(p0_gx), .p0_gy(p0_gy), .p0_gz(p0_gz), .p0_slot(p0_slot),
        .p0_read_ready(p0_rdy), .p0_node(p0_tnode), .p0_parent(p0_tpar), .p0_size(p0_tsize),
        .p1_read_en(p1_req), .p1_gx(p1_gx), .p1_gy(p1_gy), .p1_gz(p1_gz), .p1_slot(p1_slot),
        .p1_read_ready(p1_rdy), .p1_node(p1_tnode), .p1_parent(p1_tpar), .p1_size(p1_tsize),
        .t_read_en(s_a_read_en), .t_gx(s_a_gx), .t_gy(s_a_gy), .t_gz(s_a_gz), .t_slot(s_a_slot),
        .t_node(s_a_node), .t_parent(s_a_parent), .t_size(s_a_size)
    );
    TREECONT u_TC_G (
        .clk(clk), .rst_n(rst_n),
        .p0_read_en(p2_req), .p0_gx(p2_gx), .p0_gy(p2_gy), .p0_gz(p2_gz), .p0_slot(p2_slot),
        .p0_read_ready(p2_rdy), .p0_node(p2_tnode), .p0_parent(p2_tpar), .p0_size(p2_tsize),
        .p1_read_en(p3_req), .p1_gx(p3_gx), .p1_gy(p3_gy), .p1_gz(p3_gz), .p1_slot(p3_slot),
        .p1_read_ready(p3_rdy), .p1_node(p3_tnode), .p1_parent(p3_tpar), .p1_size(p3_tsize),
        .t_read_en(g_a_read_en), .t_gx(g_a_gx), .t_gy(g_a_gy), .t_gz(g_a_gz), .t_slot(g_a_slot),
        .t_node(g_a_node), .t_parent(g_a_parent), .t_size(g_a_size)
    );

    //==================================================================
    //  Map read arbiter (4 PEs -> 1 map port)
    //==================================================================
    MAPCONT u_MC (
        .clk(clk), .rst_n(rst_n),
        .pe0_read_en(m0_en), .pe0_read_node(m0_node), .pe0_read_ready(m0_rdy), .pe0_read_obstacle_out(m0_ob),
        .pe1_read_en(m1_en), .pe1_read_node(m1_node), .pe1_read_ready(m1_rdy), .pe1_read_obstacle_out(m1_ob),
        .pe2_read_en(m2_en), .pe2_read_node(m2_node), .pe2_read_ready(m2_rdy), .pe2_read_obstacle_out(m2_ob),
        .pe3_read_en(m3_en), .pe3_read_node(m3_node), .pe3_read_ready(m3_rdy), .pe3_read_obstacle_out(m3_ob),
        .read_map_en(map_read_en), .read_x(map_read_x), .read_y(map_read_y), .read_z(map_read_z),
        .read_obstacle_in(map_obstacle)
    );

    //==================================================================
    //  Connection check / path builder
    //==================================================================
    CC u_CC (
        .clk(clk), .rst_n(rst_n), .path(path),
        .pe0_node_valid(p0_nv), .pe0_node(p0_node), .pe0_parent(p0_par), .pe0_ack(p0_ack),
        .pe1_node_valid(p1_nv), .pe1_node(p1_node), .pe1_parent(p1_par), .pe1_ack(p1_ack),
        .pe2_node_valid(p2_nv), .pe2_node(p2_node), .pe2_parent(p2_par), .pe2_ack(p2_ack),
        .pe3_node_valid(p3_nv), .pe3_node(p3_node), .pe3_parent(p3_par), .pe3_ack(p3_ack),
        .cc_read_en(cc_rd_en), .cc_read_sel(cc_rd_sel),
        .cc_read_gx(cc_rd_gx), .cc_read_gy(cc_rd_gy), .cc_read_gz(cc_rd_gz), .cc_read_slot(cc_rd_slot),
        .cc_read_node(cc_rd_node), .cc_read_parent(cc_rd_par), .cc_read_size(cc_rd_size),
        .cc_write_en(cc_wr_en), .cc_write_sel(cc_wr_sel),
        .cc_write_gx(cc_wr_gx), .cc_write_gy(cc_wr_gy), .cc_write_gz(cc_wr_gz),
        .cc_write_node(cc_wr_node), .cc_write_parent(cc_wr_par),
        .path_we(path_we), .path_wnode(path_wnode)
    );

    //==================================================================
    //  CC read/write router to the two trees' port B / write ports
    //==================================================================
    CCREADCONT u_CCR (
        .cc_read_en(cc_rd_en), .cc_read_sel(cc_rd_sel),
        .cc_read_gx(cc_rd_gx), .cc_read_gy(cc_rd_gy), .cc_read_gz(cc_rd_gz), .cc_read_slot(cc_rd_slot),
        .cc_read_node(cc_rd_node), .cc_read_parent(cc_rd_par), .cc_read_size(cc_rd_size),
        .cc_write_en(cc_wr_en), .cc_write_sel(cc_wr_sel),
        .cc_write_gx(cc_wr_gx), .cc_write_gy(cc_wr_gy), .cc_write_gz(cc_wr_gz),
        .cc_write_node(cc_wr_node), .cc_write_parent(cc_wr_par),
        .s_b_read_en(s_b_read_en), .s_b_gx(s_b_gx), .s_b_gy(s_b_gy), .s_b_gz(s_b_gz), .s_b_slot(s_b_slot),
        .s_b_node(s_b_node), .s_b_parent(s_b_parent), .s_b_size(s_b_size),
        .s_write_en(s_write_en), .s_write_gx(s_write_gx), .s_write_gy(s_write_gy), .s_write_gz(s_write_gz),
        .s_write_node(s_write_node), .s_write_parent(s_write_parent),
        .g_b_read_en(g_b_read_en), .g_b_gx(g_b_gx), .g_b_gy(g_b_gy), .g_b_gz(g_b_gz), .g_b_slot(g_b_slot),
        .g_b_node(g_b_node), .g_b_parent(g_b_parent), .g_b_size(g_b_size),
        .g_write_en(g_write_en), .g_write_gx(g_write_gx), .g_write_gy(g_write_gy), .g_write_gz(g_write_gz),
        .g_write_node(g_write_node), .g_write_parent(g_write_parent)
    );

endmodule
