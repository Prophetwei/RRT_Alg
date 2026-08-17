`timescale 1ns/10ps
//======================================================================
//  PATHMEM : final START->GOAL path store
//     we = 2'b01 : front-insert (start half, written connection->root,
//                  so the array ends up root-first)
//     we = 2'b10 : append       (goal half, written connection->goal)
//  Result order : index 0 = START ... index size-1 = GOAL
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
    logic [9:0]  size;

    assign path_size = size;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            size <= 10'd0;
        end
        else if (we == 2'b01) begin           // front-insert
            for (int i = 1023; i >= 1; i--)
                if (i <= size) mem[i] <= mem[i-1];
            mem[0] <= wnode;
            size   <= size + 1'b1;
        end
        else if (we == 2'b10) begin           // append
            mem[size] <= wnode;
            size      <= size + 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)             read_path_node <= 24'd0;
        else if (read_path_en)  read_path_node <= mem[read_path_addr];
    end

endmodule
