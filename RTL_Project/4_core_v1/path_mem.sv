`timescale 1ns/10ps
//======================================================================
//  PATHMEM : final START->GOAL path store implemented as a circular deque.
//     we = 2'b01 : front-insert (start half, written connection->root)
//     we = 2'b10 : append       (goal half, written connection->goal)
//  Result order : index 0 = START ... index size-1 = GOAL
//
//  The logical head pointer removes the old O(path_size) shift on every
//  front-insert.  Both write modes now update exactly one memory word.
//======================================================================
module PATHMEM (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [1:0]  we,
    input  logic [23:0] wnode,

    input  logic        read_path_en,
    input  logic [9:0]  read_path_addr,
    output logic [23:0] read_path_node,

    output logic [9:0]  path_size
);
    logic [23:0] mem [0:1023];
    logic [9:0]  head;
    logic [9:0]  size;
    logic [9:0]  front_addr;
    logic [9:0]  append_addr;
    logic [9:0]  physical_read_addr;

    assign path_size = size;
    assign front_addr         = head - 10'd1;
    assign append_addr        = head + size;
    assign physical_read_addr = head + read_path_addr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head <= 10'd0;
            size <= 10'd0;
        end
        else if (we == 2'b01) begin
            mem[front_addr] <= wnode;
            head   <= front_addr;
            size   <= size + 1'b1;
        end
        else if (we == 2'b10) begin
            mem[append_addr] <= wnode;
            size      <= size + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)             read_path_node <= 24'd0;
        else if (read_path_en)  read_path_node <= mem[physical_read_addr];
    end

endmodule
