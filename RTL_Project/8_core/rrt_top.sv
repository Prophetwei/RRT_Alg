`timescale 1ns/10ps
`include "PE.sv"
`include "connection_check.sv"

//======================================================================
//  RRT_TOP : synthesizable dual-tree RRT controller (LOGIC ONLY)
//
//  Contains only the two PEs and the connection-check / path builder.
//  All memories are external (instantiated by the bench / a real chip
//  would wire SRAM macros to these ports):
//     * two tree memories  (start + goal)  -> *_tree_* ports
//     * obstacle map        (256^3)        -> *_read_map_* / *_obstacle
//     * path memory                        -> path_we / path_wnode
//
//  Pure synchronous logic, no behavioral memory arrays, so this is
//  synthesizable on its own.
//======================================================================
module RRT_TOP (
    input  logic        clk,
    input  logic        rst_n,

    // ---- control / handshake ----
    input  logic        in_valid0,
    output logic        out_valid0,
    input  logic [23:0] rand_num0,
    input  logic        in_valid1,
    output logic        out_valid1,
    input  logic [23:0] rand_num1,

    // ==================  START tree memory interface  ==================
    // PE read port (nearest-neighbour scan)
    output logic        s_read_tree_en,
    output logic [9:0]  s_read_tree_addr,
    input  logic [7:0]  s_read_tree_node_x,
    input  logic [7:0]  s_read_tree_node_y,
    input  logic [7:0]  s_read_tree_node_z,
    // CC read port (connection scan / path walk)
    output logic        s_cread_tree_en,
    output logic [9:0]  s_cread_tree_addr,
    input  logic [7:0]  s_cread_tree_node_x,
    input  logic [7:0]  s_cread_tree_node_y,
    input  logic [7:0]  s_cread_tree_node_z,
    input  logic [9:0]  s_cread_tree_parent,
    // CC write port
    output logic        s_write_tree_en,
    output logic [7:0]  s_write_tree_node_x,
    output logic [7:0]  s_write_tree_node_y,
    output logic [7:0]  s_write_tree_node_z,
    output logic [9:0]  s_write_tree_parent,
    // tree size
    input  logic [9:0]  s_max_tree_idx,

    // ==================  GOAL tree memory interface  ===================
    output logic        g_read_tree_en,
    output logic [9:0]  g_read_tree_addr,
    input  logic [7:0]  g_read_tree_node_x,
    input  logic [7:0]  g_read_tree_node_y,
    input  logic [7:0]  g_read_tree_node_z,
    output logic        g_cread_tree_en,
    output logic [9:0]  g_cread_tree_addr,
    input  logic [7:0]  g_cread_tree_node_x,
    input  logic [7:0]  g_cread_tree_node_y,
    input  logic [7:0]  g_cread_tree_node_z,
    input  logic [9:0]  g_cread_tree_parent,
    output logic        g_write_tree_en,
    output logic [7:0]  g_write_tree_node_x,
    output logic [7:0]  g_write_tree_node_y,
    output logic [7:0]  g_write_tree_node_z,
    output logic [9:0]  g_write_tree_parent,
    input  logic [9:0]  g_max_tree_idx,

    // ==================  external map read  ============================
    output logic        s_read_map_en,
    output logic [7:0]  s_read_x,
    output logic [7:0]  s_read_y,
    output logic [7:0]  s_read_z,
    input  logic        s_obstacle,

    output logic        g_read_map_en,
    output logic [7:0]  g_read_x,
    output logic [7:0]  g_read_y,
    output logic [7:0]  g_read_z,
    input  logic        g_obstacle,

    // ==================  path memory write  ===========================
    output logic [1:0]  path_we,
    output logic [23:0] path_wnode,

    // ==================  status  ======================================
    output logic        path
);

    //-------- PE <-> CC hand-off (internal) --------
    wire        pe0_node_valid, pe0_ack;  wire [23:0] pe0_node;  wire [9:0] pe0_parent;
    wire        pe1_node_valid, pe1_ack;  wire [23:0] pe1_node;  wire [9:0] pe1_parent;

    //-------- CC packed write-node (split out to the tree ports) --------
    wire [23:0] s_write_node_packed, g_write_node_packed;
    assign s_write_tree_node_x = s_write_node_packed[23:16];
    assign s_write_tree_node_y = s_write_node_packed[15:8];
    assign s_write_tree_node_z = s_write_node_packed[7:0];
    assign g_write_tree_node_x = g_write_node_packed[23:16];
    assign g_write_tree_node_y = g_write_node_packed[15:8];
    assign g_write_tree_node_z = g_write_node_packed[7:0];

    //==================================================================
    //  Start-tree PE  (samples bias toward GOAL = 255)
    //==================================================================
    PE #(.SEED(8'd255)) u_PE0 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid0), .out_valid(out_valid0), .rand_num(rand_num0),
        .read_tree_en(s_read_tree_en), .tree_node_addr(s_read_tree_addr),
        .tree_node_x_read(s_read_tree_node_x), .tree_node_y_read(s_read_tree_node_y), .tree_node_z_read(s_read_tree_node_z),
        .max_tree_idx(s_max_tree_idx),
        .node_valid(pe0_node_valid), .tree_node_write(pe0_node), .tree_parent_write(pe0_parent),
        .write_tree_en(pe0_ack),
        .map_x(s_read_x), .map_y(s_read_y), .map_z(s_read_z), .read_map_en(s_read_map_en), .obstacle(s_obstacle)
    );

    //==================================================================
    //  Goal-tree PE  (samples bias toward START = 0)
    //==================================================================
    PE #(.SEED(8'd0)) u_PE1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid1), .out_valid(out_valid1), .rand_num(rand_num1),
        .read_tree_en(g_read_tree_en), .tree_node_addr(g_read_tree_addr),
        .tree_node_x_read(g_read_tree_node_x), .tree_node_y_read(g_read_tree_node_y), .tree_node_z_read(g_read_tree_node_z),
        .max_tree_idx(g_max_tree_idx),
        .node_valid(pe1_node_valid), .tree_node_write(pe1_node), .tree_parent_write(pe1_parent),
        .write_tree_en(pe1_ack),
        .map_x(g_read_x), .map_y(g_read_y), .map_z(g_read_z), .read_map_en(g_read_map_en), .obstacle(g_obstacle)
    );

    //==================================================================
    //  Connection check / path builder
    //==================================================================
    CC u_CC (
        .clk(clk), .rst_n(rst_n), .path(path),
        .pe0_node_valid(pe0_node_valid), .pe0_node(pe0_node), .pe0_parent(pe0_parent), .pe0_ack(pe0_ack),
        .pe1_node_valid(pe1_node_valid), .pe1_node(pe1_node), .pe1_parent(pe1_parent), .pe1_ack(pe1_ack),

        .s_cread_en(s_cread_tree_en), .s_cread_addr(s_cread_tree_addr),
        .s_cread_node({s_cread_tree_node_x, s_cread_tree_node_y, s_cread_tree_node_z}),
        .s_cread_parent(s_cread_tree_parent), .s_tree_size(s_max_tree_idx),
        .s_write_en(s_write_tree_en), .s_write_node(s_write_node_packed), .s_write_parent(s_write_tree_parent),

        .g_cread_en(g_cread_tree_en), .g_cread_addr(g_cread_tree_addr),
        .g_cread_node({g_cread_tree_node_x, g_cread_tree_node_y, g_cread_tree_node_z}),
        .g_cread_parent(g_cread_tree_parent), .g_tree_size(g_max_tree_idx),
        .g_write_en(g_write_tree_en), .g_write_node(g_write_node_packed), .g_write_parent(g_write_tree_parent),

        .path_we(path_we), .path_wnode(path_wnode)
    );

endmodule
