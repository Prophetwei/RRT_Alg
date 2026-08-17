`timescale 1ns/10ps
`include "PATTERN.sv"
`include "PE.sv"
`include "tree_mem.sv"
`include "map_mem.sv"

module TESTBED;

    // --- Global Signals ---
    wire clk, rst_n;
    wire in_valid, out_valid;
    wire [23:0] rand_num;

    // --- Tree Memory Interface Signals ---
    wire [7:0] read_tree_node_x, read_tree_node_y, read_tree_node_z;
    wire [9:0] read_tree_parent;
    wire [7:0] write_tree_node_x, write_tree_node_y, write_tree_node_z;
    wire [9:0] write_tree_parent;
    wire [9:0] max_tree_idx;
    wire read_tree_en, write_tree_en, check_ans_en;
    wire [9:0] tree_node_addr;
    wire [9:0] check_ans_addr;

    // --- Map Memory Interface Signals ---
    wire [7:0] read_map_x, read_map_y, read_map_z;
    wire write_map_en;
    wire [7:0] write_map_x, write_map_y, write_map_z;
    wire write_obstacle_val;
    wire read_map_en;
    wire obstacle_out;

    

    // --- Waveform Dump Blocks ---
    initial begin
        $dumpfile("PE.vcd");
        $dumpvars(0, TESTBED); // Dumps the whole testbed tree hierarchy
    end

    // --- Processing Engine (PE) Core Instance ---
    PE u_PE (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .out_valid(out_valid),
        .rand_num(rand_num),

        // Connections to catch read data coming out from TREEMEM
        .tree_node_x_read(read_tree_node_x),
        .tree_node_y_read(read_tree_node_y),
        .tree_node_z_read(read_tree_node_z),
        .tree_parent_read(read_tree_parent),

        // Connections to push write updates down into TREEMEM
        .tree_node_x_write(write_tree_node_x),
        .tree_node_y_write(write_tree_node_y),
        .tree_node_z_write(write_tree_node_z),
        .tree_parent_write(write_tree_parent),
        
        // Memory Controller Handshake Pins
        .max_tree_idx(max_tree_idx),
        .read_tree_en(read_tree_en),
        .write_tree_en(write_tree_en),
        .tree_node_addr(tree_node_addr),

        // Collision Avoidance Radar Paths
        .map_x(read_map_x),
        .map_y(read_map_y),
        .map_z(read_map_z),
        .read_map_en(read_map_en),
        .obstacle(obstacle_out)
    );

    // --- Map Memory Matrix Module (MAPMEM) ---
    MAPMEM u_map_mem (
        .clk(clk),
        .rst_n(rst_n),
        
        .write_map_en(write_map_en),
        .write_x(write_map_x),
        .write_y(write_map_y),
        .write_z(write_map_z),
        .write_obstacle_val(write_obstacle_val),

        .read_map_en(read_map_en),
        .read_x(read_map_x),
        .read_y(read_map_y),
        .read_z(read_map_z),
        .obstacle_out(obstacle_out)
    );

    // --- Low-Power Tree Memory Macro (TREEMEM) ---
    TREEMEM u_tree_mem (
        .clk(clk),
        .rst_n(rst_n),

        .read_tree_en(read_tree_en),
        .write_tree_en(write_tree_en),
        .check_ans_en(check_ans_en),
        .tree_node_addr(tree_node_addr),
        .check_ans_addr(check_ans_addr),
        .max_tree_idx(max_tree_idx),

        // Dedicated Synchronous Write Field Inputs (From PE core outputs)
        .write_node_x(write_tree_node_x),
        .write_node_y(write_tree_node_y),
        .write_node_z(write_tree_node_z),
        .write_parent(write_tree_parent),

        // Dedicated Synchronous Read Field Outputs (Forwarded into PE core inputs)
        .read_node_x(read_tree_node_x),
        .read_node_y(read_tree_node_y),
        .read_node_z(read_tree_node_z),
        .read_parent(read_tree_parent)
    );

    // --- Verification Environment Engine Block (PATTERN) ---
    PATTERN u_pattern_gen (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .out_valid(out_valid),
        .rand_num(rand_num),

        .check_ans_en(check_ans_en),
        .check_ans_addr(check_ans_addr),
        .read_node_x(read_tree_node_x),
        .read_node_y(read_tree_node_y),
        .read_node_z(read_tree_node_z),
        .read_parent(read_tree_parent),
        .max_tree_idx(max_tree_idx),
        
        // Map Loading Outputs
        .write_map_en(write_map_en),
        .write_x(write_map_x), 
        .write_y(write_map_y), 
        .write_z(write_map_z),
        .write_obstacle_val(write_obstacle_val)
    );

endmodule