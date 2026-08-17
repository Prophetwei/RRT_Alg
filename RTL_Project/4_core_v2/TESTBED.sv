`timescale 1ns/10ps
// ----------------------------------------------------------------------
//  Core selection:
//    RTL  sim (default)      : compile the RTL core (rrt_top.sv + submodules)
//    GATE sim (+define+SYN)  : do NOT include the RTL; compile the synthesized
//                              netlist Netlist/RRT_TOP_SYN.v on the vcs cmd line
//                              instead (module RRT_TOP, same ports).
//  The behavioural memories (TREEMEM/MAPMEM/PATHMEM) and PATTERN are always RTL.
// ----------------------------------------------------------------------
`ifdef RTL_TOP
  `include "rrt_top.sv"
`endif
`ifdef GATE_TOP
    `include "../02_SYN/Netlist/RRT_TOP_SYN.v"
`endif
`include "path_mem.sv"
`include "tree_mem.sv"
`include "map_mem.sv"
`include "PATTERN.sv"

//======================================================================
//  TESTBED : wires the synthesizable (logic-only) RRT_TOP core to the
//  external behavioural memories (2x TREEMEM, MAPMEM, PATHMEM) and the
//  pattern/checker.
//======================================================================
module TESTBED;
    logic clk, rst_n;

    // ---- core control (4 PEs) ----
    logic in_valid0, out_valid0;  logic [23:0] rand_num0;
    logic in_valid1, out_valid1;  logic [23:0] rand_num1;
    logic in_valid2, out_valid2;  logic [23:0] rand_num2;
    logic in_valid3, out_valid3;  logic [23:0] rand_num3;
    logic path_found;

    // ---- core <-> START TREEMEM ----
    wire        s_a_read_en;  wire [3:0] s_a_gx,s_a_gy,s_a_gz,s_a_slot;
    wire [23:0] s_a_node;     wire [15:0] s_a_parent;  wire [3:0] s_a_size;
    wire        s_b_read_en;  wire [3:0] s_b_gx,s_b_gy,s_b_gz,s_b_slot;
    wire [23:0] s_b_node;     wire [15:0] s_b_parent;  wire [3:0] s_b_size;
    wire        s_write_en;   wire [3:0] s_write_gx,s_write_gy,s_write_gz;
    wire [23:0] s_write_node; wire [15:0] s_write_parent;
    wire [15:0] s_tree_size;

    // ---- core <-> GOAL TREEMEM ----
    wire        g_a_read_en;  wire [3:0] g_a_gx,g_a_gy,g_a_gz,g_a_slot;
    wire [23:0] g_a_node;     wire [15:0] g_a_parent;  wire [3:0] g_a_size;
    wire        g_b_read_en;  wire [3:0] g_b_gx,g_b_gy,g_b_gz,g_b_slot;
    wire [23:0] g_b_node;     wire [15:0] g_b_parent;  wire [3:0] g_b_size;
    wire        g_write_en;   wire [3:0] g_write_gx,g_write_gy,g_write_gz;
    wire [23:0] g_write_node; wire [15:0] g_write_parent;
    wire [15:0] g_tree_size;

    // ---- core <-> MAP (shared) ----
    wire        map_read_en;  wire [7:0] map_read_x, map_read_y, map_read_z;  wire map_obstacle;

    // ---- core -> PATH ----
    wire [1:0]  path_we;  wire [23:0] path_wnode;

    // ---- pattern <-> map (load / clear / verify) ----
    logic       write_map_en;  logic [7:0] write_x, write_y, write_z;  logic write_obstacle_val;
    logic       global_clear_en;  wire clear_done;
    logic       p_map_en;  logic [7:0] p_map_x, p_map_y, p_map_z;  wire p_obstacle;

    // ---- pattern <-> path read-out ----
    logic       read_path_en;  logic [9:0] read_path_addr;  wire [23:0] read_path_node;  wire [9:0] path_size;

    //==================================================================
    //  Synthesizable core
    //==================================================================
    RRT_TOP u_TOP (
        .clk(clk), .rst_n(rst_n),
        .in_valid0(in_valid0), .out_valid0(out_valid0), .rand_num0(rand_num0),
        .in_valid1(in_valid1), .out_valid1(out_valid1), .rand_num1(rand_num1),
        .in_valid2(in_valid2), .out_valid2(out_valid2), .rand_num2(rand_num2),
        .in_valid3(in_valid3), .out_valid3(out_valid3), .rand_num3(rand_num3),

        .s_tree_size(s_tree_size), .g_tree_size(g_tree_size),

        .s_a_read_en(s_a_read_en), .s_a_gx(s_a_gx), .s_a_gy(s_a_gy), .s_a_gz(s_a_gz), .s_a_slot(s_a_slot),
        .s_a_node(s_a_node), .s_a_parent(s_a_parent), .s_a_size(s_a_size),
        .s_b_read_en(s_b_read_en), .s_b_gx(s_b_gx), .s_b_gy(s_b_gy), .s_b_gz(s_b_gz), .s_b_slot(s_b_slot),
        .s_b_node(s_b_node), .s_b_parent(s_b_parent), .s_b_size(s_b_size),
        .s_write_en(s_write_en), .s_write_gx(s_write_gx), .s_write_gy(s_write_gy), .s_write_gz(s_write_gz),
        .s_write_node(s_write_node), .s_write_parent(s_write_parent),

        .g_a_read_en(g_a_read_en), .g_a_gx(g_a_gx), .g_a_gy(g_a_gy), .g_a_gz(g_a_gz), .g_a_slot(g_a_slot),
        .g_a_node(g_a_node), .g_a_parent(g_a_parent), .g_a_size(g_a_size),
        .g_b_read_en(g_b_read_en), .g_b_gx(g_b_gx), .g_b_gy(g_b_gy), .g_b_gz(g_b_gz), .g_b_slot(g_b_slot),
        .g_b_node(g_b_node), .g_b_parent(g_b_parent), .g_b_size(g_b_size),
        .g_write_en(g_write_en), .g_write_gx(g_write_gx), .g_write_gy(g_write_gy), .g_write_gz(g_write_gz),
        .g_write_node(g_write_node), .g_write_parent(g_write_parent),

        .map_read_en(map_read_en), .map_read_x(map_read_x), .map_read_y(map_read_y), .map_read_z(map_read_z),
        .map_obstacle(map_obstacle),

        .path_we(path_we), .path_wnode(path_wnode),
        .path(path_found)
    );

    //==================================================================
    //  Gate-level timing back-annotation (only for +define+SDF)
    //  Adjust the path if you run VCS from a different directory.
    //==================================================================
`ifdef SDF
    initial $sdf_annotate("../02_SYN/Netlist/RRT_TOP_SYN.sdf", u_TOP);
`endif

    //==================================================================
    //  External START tree (root 0,0,0) / GOAL tree (root 255,255,255)
    //==================================================================
    TREEMEM #(.ROOT_X(8'd0), .ROOT_Y(8'd0), .ROOT_Z(8'd0)) u_STARTTREE (
        .clk(clk), .rst_n(rst_n),
        .a_read_en(s_a_read_en), .a_gx(s_a_gx), .a_gy(s_a_gy), .a_gz(s_a_gz), .a_slot(s_a_slot),
        .a_node(s_a_node), .a_parent(s_a_parent), .a_size(s_a_size),
        .b_read_en(s_b_read_en), .b_gx(s_b_gx), .b_gy(s_b_gy), .b_gz(s_b_gz), .b_slot(s_b_slot),
        .b_node(s_b_node), .b_parent(s_b_parent), .b_size(s_b_size),
        .write_en(s_write_en), .write_gx(s_write_gx), .write_gy(s_write_gy), .write_gz(s_write_gz),
        .write_node(s_write_node), .write_parent(s_write_parent),
        .tree_size(s_tree_size)
    );

    TREEMEM #(.ROOT_X(8'd255), .ROOT_Y(8'd255), .ROOT_Z(8'd255)) u_GOALTREE (
        .clk(clk), .rst_n(rst_n),
        .a_read_en(g_a_read_en), .a_gx(g_a_gx), .a_gy(g_a_gy), .a_gz(g_a_gz), .a_slot(g_a_slot),
        .a_node(g_a_node), .a_parent(g_a_parent), .a_size(g_a_size),
        .b_read_en(g_b_read_en), .b_gx(g_b_gx), .b_gy(g_b_gy), .b_gz(g_b_gz), .b_slot(g_b_slot),
        .b_node(g_b_node), .b_parent(g_b_parent), .b_size(g_b_size),
        .write_en(g_write_en), .write_gx(g_write_gx), .write_gy(g_write_gy), .write_gz(g_write_gz),
        .write_node(g_write_node), .write_parent(g_write_parent),
        .tree_size(g_tree_size)
    );

    //==================================================================
    //  External obstacle map (256^3)
    //==================================================================
    MAPMEM u_MAPMEM (
        .clk(clk), .rst_n(rst_n),
        .write_map_en(write_map_en), .write_x(write_x), .write_y(write_y), .write_z(write_z),
        .write_obstacle_val(write_obstacle_val),
        .global_clear_en(global_clear_en), .clear_done(clear_done),
        .pe_read_map_en(map_read_en), .pe_read_x(map_read_x), .pe_read_y(map_read_y), .pe_read_z(map_read_z),
        .pe_read_obstacle_out(map_obstacle),
        .p_read_map_en(p_map_en), .p_read_x(p_map_x), .p_read_y(p_map_y), .p_read_z(p_map_z),
        .p_read_obstacle_out(p_obstacle)
    );

    //==================================================================
    //  External path memory
    //==================================================================
    PATHMEM u_PATHMEM (
        .clk(clk), .rst_n(rst_n),
        .we(path_we), .wnode(path_wnode),
        .read_path_en(read_path_en), .read_path_addr(read_path_addr), .read_path_node(read_path_node),
        .path_size(path_size)
    );

    //==================================================================
    //  Pattern / driver / checker
    //==================================================================
    PATTERN u_PATTERN (
        .clk(clk), .rst_n(rst_n),
        .in_valid0(in_valid0), .out_valid0(out_valid0), .rand_num0(rand_num0),
        .in_valid1(in_valid1), .out_valid1(out_valid1), .rand_num1(rand_num1),
        .in_valid2(in_valid2), .out_valid2(out_valid2), .rand_num2(rand_num2),
        .in_valid3(in_valid3), .out_valid3(out_valid3), .rand_num3(rand_num3),
        .write_map_en(write_map_en), .write_x(write_x), .write_y(write_y), .write_z(write_z),
        .write_obstacle_val(write_obstacle_val),
        .global_clear_en(global_clear_en), .clear_done(clear_done),
        .p_read_map_en(p_map_en), .p_read_x(p_map_x), .p_read_y(p_map_y), .p_read_z(p_map_z), .p_read_obstacle(p_obstacle),
        .read_path_en(read_path_en), .read_path_addr(read_path_addr), .read_path_node(read_path_node), .path_size(path_size),
        .path(path_found)
    );

endmodule
