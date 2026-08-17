`timescale 1ns/10ps
//======================================================================
//  TREEMEM : append-only RRT tree store (1024 nodes)
//  Root is hard-loaded at index 0 from parameters:
//     start tree -> (0,0,0)        goal tree -> (255,255,255)
//  Two independent registered read ports:
//     - port A : PE nearest-neighbour scan
//     - port B : CC connection-scan / path reconstruction
//======================================================================
module TREEMEM #(
    parameter [7:0] ROOT_X = 8'd0,
    parameter [7:0] ROOT_Y = 8'd0,
    parameter [7:0] ROOT_Z = 8'd0
)(
    input  logic        clk,
    input  logic        rst_n,

    // ---- port A : PE read ----
    input  logic        read_tree_en,
    input  logic [9:0]  read_tree_addr,
    output logic [7:0]  read_tree_node_x,
    output logic [7:0]  read_tree_node_y,
    output logic [7:0]  read_tree_node_z,
    output logic [9:0]  read_tree_parent,

    // ---- port B : CC read ----
    input  logic        c_read_tree_en,
    input  logic [9:0]  c_read_tree_addr,
    output logic [7:0]  c_read_tree_node_x,
    output logic [7:0]  c_read_tree_node_y,
    output logic [7:0]  c_read_tree_node_z,
    output logic [9:0]  c_read_tree_parent,

    // ---- write (from CC) ----
    input  logic        write_tree_en,
    input  logic [7:0]  write_node_x,
    input  logic [7:0]  write_node_y,
    input  logic [7:0]  write_node_z,
    input  logic [9:0]  write_parent,

    output logic [9:0]  max_tree_idx
);
    localparam [9:0] NONE = 10'h3FF;   // "no parent" marker (root)

    logic [7:0] mem_x      [0:1023];
    logic [7:0] mem_y      [0:1023];
    logic [7:0] mem_z      [0:1023];
    logic [9:0] mem_parent [0:1023];

    logic [9:0] node_count;
    assign max_tree_idx = node_count;

    // ---- write / root init ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            node_count    <= 10'd1;          // index 0 is the root
            mem_x[0]      <= ROOT_X;
            mem_y[0]      <= ROOT_Y;
            mem_z[0]      <= ROOT_Z;
            mem_parent[0] <= NONE;
        end
        else if (write_tree_en && (node_count < 10'd1023)) begin
            mem_x[node_count]      <= write_node_x;
            mem_y[node_count]      <= write_node_y;
            mem_z[node_count]      <= write_node_z;
            mem_parent[node_count] <= write_parent;
            node_count             <= node_count + 1'b1;
        end
    end

    // ---- port A read (registered) ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_tree_node_x <= 0; read_tree_node_y <= 0; read_tree_node_z <= 0; read_tree_parent <= 0;
        end else if (read_tree_en) begin
            read_tree_node_x <= mem_x[read_tree_addr];
            read_tree_node_y <= mem_y[read_tree_addr];
            read_tree_node_z <= mem_z[read_tree_addr];
            read_tree_parent <= mem_parent[read_tree_addr];
        end
    end

    // ---- port B read (registered) ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_read_tree_node_x <= 0; c_read_tree_node_y <= 0; c_read_tree_node_z <= 0; c_read_tree_parent <= 0;
        end else if (c_read_tree_en) begin
            c_read_tree_node_x <= mem_x[c_read_tree_addr];
            c_read_tree_node_y <= mem_y[c_read_tree_addr];
            c_read_tree_node_z <= mem_z[c_read_tree_addr];
            c_read_tree_parent <= mem_parent[c_read_tree_addr];
        end
    end

endmodule
