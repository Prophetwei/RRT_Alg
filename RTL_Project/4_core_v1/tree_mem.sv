`timescale 1ns/10ps
//======================================================================
//  TREEMEM : grid-indexed RRT tree store (behavioural, external)
//
//  The 256^3 voxel space is bucketed into a 16x16x16 grid of cells
//  (cell = voxel[7:4]).  Each cell holds up to 8 nodes.  A node record is
//  {node[23:0]={x,y,z}, parent[15:0]}.  A node is addressed by its
//  (grid_x, grid_y, grid_z, slot).  A parent pointer uses the same
//  encoding: {gx[3:0], gy[3:0], gz[3:0], slot[3:0]}; the root's parent is
//  16'hFFFF.
//
//  Two independent registered read ports:
//     - port A : PE nearest-neighbour scan   (shared by 2 PEs via TREECONT)
//     - port B : CC connection-scan / path reconstruction
//  A read also returns grid_size of the addressed cell (occupancy).
//======================================================================
module TREEMEM #(
    parameter [7:0] ROOT_X = 8'd0,
    parameter [7:0] ROOT_Y = 8'd0,
    parameter [7:0] ROOT_Z = 8'd0
)(
    input  logic        clk,
    input  logic        rst_n,

    // ---- port A : PE read ----
    input  logic        a_read_en,
    input  logic [3:0]  a_gx,
    input  logic [3:0]  a_gy,
    input  logic [3:0]  a_gz,
    input  logic [3:0]  a_slot,
    output logic [23:0] a_node,
    output logic [15:0] a_parent,
    output logic [3:0]  a_size,

    // ---- port B : CC read ----
    input  logic        b_read_en,
    input  logic [3:0]  b_gx,
    input  logic [3:0]  b_gy,
    input  logic [3:0]  b_gz,
    input  logic [3:0]  b_slot,
    output logic [23:0] b_node,
    output logic [15:0] b_parent,
    output logic [3:0]  b_size,

    // ---- write (from CC) ----
    input  logic        write_en,
    input  logic [3:0]  write_gx,
    input  logic [3:0]  write_gy,
    input  logic [3:0]  write_gz,
    input  logic [23:0] write_node,
    input  logic [15:0] write_parent,

    output logic [14:0] tree_size          // total node count (debug/limit)
);
    localparam [3:0] RGX = ROOT_X[7:4];
    localparam [3:0] RGY = ROOT_Y[7:4];
    localparam [3:0] RGZ = ROOT_Z[7:4];

    logic [39:0] mem       [0:15][0:15][0:15][0:7];   // {node[23:0],parent[15:0]}
    logic [3:0]  grid_size [0:15][0:15][0:15];         // 0..8 occupancy per cell

    // ---- write / root init ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 16; i++)
                for (int j = 0; j < 16; j++)
                    for (int k = 0; k < 16; k++)
                        grid_size[i][j][k] <= 4'd0;
            mem[RGX][RGY][RGZ][0] <= {ROOT_X, ROOT_Y, ROOT_Z, 16'hFFFF};
            grid_size[RGX][RGY][RGZ] <= 4'd1;
            tree_size <= 15'd1;
        end
        else if (write_en && (grid_size[write_gx][write_gy][write_gz] < 4'd8)) begin
            mem[write_gx][write_gy][write_gz][grid_size[write_gx][write_gy][write_gz]]
                 <= {write_node, write_parent};
            grid_size[write_gx][write_gy][write_gz]
                 <= grid_size[write_gx][write_gy][write_gz] + 4'd1;
            tree_size <= tree_size + 15'd1;
        end
    end

    // ---- port A read (registered) ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_node <= 24'd0; a_parent <= 16'd0; a_size <= 4'd0;
        end else if (a_read_en) begin
            a_node   <= mem[a_gx][a_gy][a_gz][a_slot][39:16];
            a_parent <= mem[a_gx][a_gy][a_gz][a_slot][15:0];
            a_size   <= grid_size[a_gx][a_gy][a_gz];
        end
    end

    // ---- port B read (registered) ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_node <= 24'd0; b_parent <= 16'd0; b_size <= 4'd0;
        end else if (b_read_en) begin
            b_node   <= mem[b_gx][b_gy][b_gz][b_slot][39:16];
            b_parent <= mem[b_gx][b_gy][b_gz][b_slot][15:0];
            b_size   <= grid_size[b_gx][b_gy][b_gz];
        end
    end

endmodule


