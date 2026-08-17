`timescale 1ns/10ps
`include "PATTERN.sv"
`include "rrt_top.sv"
`include "tree_mem.sv"
`include "path_mem.sv"
`include "map_mem.sv"

//======================================================================
//  TESTBED : wires the synthesizable (logic-only) RRT_TOP core to the
//            external, non-synthesizable memory models and the
//            pattern/checker.
//            External memories:  2x TREEMEM, PATHMEM, MAPMEM.
//======================================================================
module TESTBED;
    //-------- Global --------
    logic clk, rst_n;

    //-------- Core control --------
    logic        in_valid0, out_valid0;
    logic [23:0] rand_num0;
    logic        in_valid1, out_valid1;
    logic [23:0] rand_num1;
    logic        path_found;

    //-------- Core <-> START tree memory --------
    wire        s_read_tree_en;  wire [9:0] s_read_tree_addr;  wire [7:0] s_rd_x, s_rd_y, s_rd_z;
    wire        s_cread_tree_en; wire [9:0] s_cread_tree_addr; wire [7:0] s_cr_x, s_cr_y, s_cr_z;  wire [9:0] s_cread_parent;
    wire        s_write_tree_en; wire [7:0] s_wr_x, s_wr_y, s_wr_z;  wire [9:0] s_write_parent;
    wire [9:0]  s_max_idx;

    //-------- Core <-> GOAL tree memory --------
    wire        g_read_tree_en;  wire [9:0] g_read_tree_addr;  wire [7:0] g_rd_x, g_rd_y, g_rd_z;
    wire        g_cread_tree_en; wire [9:0] g_cread_tree_addr; wire [7:0] g_cr_x, g_cr_y, g_cr_z;  wire [9:0] g_cread_parent;
    wire        g_write_tree_en; wire [7:0] g_wr_x, g_wr_y, g_wr_z;  wire [9:0] g_write_parent;
    wire [9:0]  g_max_idx;

    //-------- Core <-> external map --------
    wire        s_map_en;  wire [7:0] s_map_x, s_map_y, s_map_z;  wire s_obstacle;
    wire        g_map_en;  wire [7:0] g_map_x, g_map_y, g_map_z;  wire g_obstacle;

    //-------- Core -> path memory --------
    wire [1:0]  path_we;  wire [23:0] path_wnode;

    //-------- Pattern <-> map (load / clear / verify) --------
    logic       write_map_en;  logic [7:0] write_x, write_y, write_z;  logic write_obstacle_val;
    logic       global_clear_en;  wire clear_done;
    logic       p_map_en;  logic [7:0] p_map_x, p_map_y, p_map_z;  wire p_obstacle;

    //-------- Pattern <-> path memory read-out --------
    logic       read_path_en;
    logic [9:0] read_path_addr;
    wire [23:0] read_path_node;
    wire [9:0]  path_size;

    //==================================================================
    //  Synthesizable RRT core (LOGIC ONLY : 2 PEs + CC)
    //==================================================================
    RRT_TOP u_TOP (
        .clk(clk), .rst_n(rst_n),
        .in_valid0(in_valid0), .out_valid0(out_valid0), .rand_num0(rand_num0),
        .in_valid1(in_valid1), .out_valid1(out_valid1), .rand_num1(rand_num1),

        // start tree
        .s_read_tree_en(s_read_tree_en), .s_read_tree_addr(s_read_tree_addr),
        .s_read_tree_node_x(s_rd_x), .s_read_tree_node_y(s_rd_y), .s_read_tree_node_z(s_rd_z),
        .s_cread_tree_en(s_cread_tree_en), .s_cread_tree_addr(s_cread_tree_addr),
        .s_cread_tree_node_x(s_cr_x), .s_cread_tree_node_y(s_cr_y), .s_cread_tree_node_z(s_cr_z), .s_cread_tree_parent(s_cread_parent),
        .s_write_tree_en(s_write_tree_en),
        .s_write_tree_node_x(s_wr_x), .s_write_tree_node_y(s_wr_y), .s_write_tree_node_z(s_wr_z), .s_write_tree_parent(s_write_parent),
        .s_max_tree_idx(s_max_idx),

        // goal tree
        .g_read_tree_en(g_read_tree_en), .g_read_tree_addr(g_read_tree_addr),
        .g_read_tree_node_x(g_rd_x), .g_read_tree_node_y(g_rd_y), .g_read_tree_node_z(g_rd_z),
        .g_cread_tree_en(g_cread_tree_en), .g_cread_tree_addr(g_cread_tree_addr),
        .g_cread_tree_node_x(g_cr_x), .g_cread_tree_node_y(g_cr_y), .g_cread_tree_node_z(g_cr_z), .g_cread_tree_parent(g_cread_parent),
        .g_write_tree_en(g_write_tree_en),
        .g_write_tree_node_x(g_wr_x), .g_write_tree_node_y(g_wr_y), .g_write_tree_node_z(g_wr_z), .g_write_tree_parent(g_write_parent),
        .g_max_tree_idx(g_max_idx),

        // external map
        .s_read_map_en(s_map_en), .s_read_x(s_map_x), .s_read_y(s_map_y), .s_read_z(s_map_z), .s_obstacle(s_obstacle),
        .g_read_map_en(g_map_en), .g_read_x(g_map_x), .g_read_y(g_map_y), .g_read_z(g_map_z), .g_obstacle(g_obstacle),

        // path memory write
        .path_we(path_we), .path_wnode(path_wnode),

        .path(path_found)
    );

    //==================================================================
    //  External START tree memory  (root = 0,0,0)
    //==================================================================
    TREEMEM #(.ROOT_X(8'd0), .ROOT_Y(8'd0), .ROOT_Z(8'd0)) u_STARTTREE (
        .clk(clk), .rst_n(rst_n),
        .read_tree_en(s_read_tree_en), .read_tree_addr(s_read_tree_addr),
        .read_tree_node_x(s_rd_x), .read_tree_node_y(s_rd_y), .read_tree_node_z(s_rd_z), .read_tree_parent(),
        .c_read_tree_en(s_cread_tree_en), .c_read_tree_addr(s_cread_tree_addr),
        .c_read_tree_node_x(s_cr_x), .c_read_tree_node_y(s_cr_y), .c_read_tree_node_z(s_cr_z), .c_read_tree_parent(s_cread_parent),
        .write_tree_en(s_write_tree_en),
        .write_node_x(s_wr_x), .write_node_y(s_wr_y), .write_node_z(s_wr_z), .write_parent(s_write_parent),
        .max_tree_idx(s_max_idx)
    );

    //==================================================================
    //  External GOAL tree memory  (root = 255,255,255)
    //==================================================================
    TREEMEM #(.ROOT_X(8'd255), .ROOT_Y(8'd255), .ROOT_Z(8'd255)) u_GOALTREE (
        .clk(clk), .rst_n(rst_n),
        .read_tree_en(g_read_tree_en), .read_tree_addr(g_read_tree_addr),
        .read_tree_node_x(g_rd_x), .read_tree_node_y(g_rd_y), .read_tree_node_z(g_rd_z), .read_tree_parent(),
        .c_read_tree_en(g_cread_tree_en), .c_read_tree_addr(g_cread_tree_addr),
        .c_read_tree_node_x(g_cr_x), .c_read_tree_node_y(g_cr_y), .c_read_tree_node_z(g_cr_z), .c_read_tree_parent(g_cread_parent),
        .write_tree_en(g_write_tree_en),
        .write_node_x(g_wr_x), .write_node_y(g_wr_y), .write_node_z(g_wr_z), .write_parent(g_write_parent),
        .max_tree_idx(g_max_idx)
    );

    //==================================================================
    //  External obstacle map model (256^3)
    //==================================================================
    MAPMEM u_MAPMEM (
        .clk(clk), .rst_n(rst_n),
        .write_map_en(write_map_en), .write_x(write_x), .write_y(write_y), .write_z(write_z),
        .write_obstacle_val(write_obstacle_val),
        .global_clear_en(global_clear_en), .clear_done(clear_done),
        .s_read_map_en(s_map_en), .s_read_x(s_map_x), .s_read_y(s_map_y), .s_read_z(s_map_z), .s_read_obstacle_out(s_obstacle),
        .g_read_map_en(g_map_en), .g_read_x(g_map_x), .g_read_y(g_map_y), .g_read_z(g_map_z), .g_read_obstacle_out(g_obstacle),
        .p_read_map_en(p_map_en), .p_read_x(p_map_x), .p_read_y(p_map_y), .p_read_z(p_map_z), .p_read_obstacle_out(p_obstacle)
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
        .write_map_en(write_map_en), .write_x(write_x), .write_y(write_y), .write_z(write_z),
        .write_obstacle_val(write_obstacle_val),
        .global_clear_en(global_clear_en), .clear_done(clear_done),
        .p_read_map_en(p_map_en), .p_read_x(p_map_x), .p_read_y(p_map_y), .p_read_z(p_map_z), .p_read_obstacle(p_obstacle),
        .read_path_en(read_path_en), .read_path_addr(read_path_addr), .read_path_node(read_path_node), .path_size(path_size),
        .path(path_found)
    );

endmodule
