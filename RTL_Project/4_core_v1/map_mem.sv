`timescale 1ns/10ps
//======================================================================
//  MAPMEM : 256x256x256 single-bit obstacle grid  (behavioural, external)
//  One write port + shift-register auto-clear (256 cycles after reset).
//  Two registered read ports:
//     pe_* : the 4-PE-shared collision-check port (driven by MAPCONT)
//     p_*  : testbench path verification
//======================================================================
module MAPMEM (
    input  logic        clk,
    input  logic        rst_n,

    // ---- write ----
    input  logic        write_map_en,
    input  logic [7:0]  write_x,
    input  logic [7:0]  write_y,
    input  logic [7:0]  write_z,
    input  logic        write_obstacle_val,

    // ---- global clear ----
    input  logic        global_clear_en,
    output logic        clear_done,

    // ---- shared PE read (via MAPCONT) ----
    input  logic        pe_read_map_en,
    input  logic [7:0]  pe_read_x,
    input  logic [7:0]  pe_read_y,
    input  logic [7:0]  pe_read_z,
    output logic        pe_read_obstacle_out,

    // ---- verify read ----
    input  logic        p_read_map_en,
    input  logic [7:0]  p_read_x,
    input  logic [7:0]  p_read_y,
    input  logic [7:0]  p_read_z,
    output logic        p_read_obstacle_out
);

    logic map_data [0:255][0:255][0:255];

    // shift-register clear : a single '1' walks across the 256 X-plates
    logic [255:0] clear_shift_reg;
    assign clear_done = (clear_shift_reg == 256'd0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)               clear_shift_reg <= 256'd1;
        else if (global_clear_en) clear_shift_reg <= 256'd1;
        else                      clear_shift_reg <= clear_shift_reg << 1;
    end

    // plain `always` (not always_ff): PATTERN back-door-loads map_data too, and
    // VCS forbids a second procedural writer of an always_ff variable (ICPD).
    // MAPMEM is a behavioural, non-synthesized model, so this is functionally
    // identical and keeps VCS happy.
    always @(posedge clk) begin
        if (!clear_done) begin
            for (int i = 0; i < 256; i++) begin
                if (clear_shift_reg[i]) begin
                    for (int yy = 0; yy < 256; yy++)
                        for (int zz = 0; zz < 256; zz++)
                            map_data[i][yy][zz] <= 1'b0;
                end
            end
        end
        else if (write_map_en) begin
            map_data[write_x][write_y][write_z] <= write_obstacle_val;
        end
    end

    // ---- reads (registered) ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)               pe_read_obstacle_out <= 1'b0;
        else if (pe_read_map_en)  pe_read_obstacle_out <= map_data[pe_read_x][pe_read_y][pe_read_z];
        else                      pe_read_obstacle_out <= 1'b0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)               p_read_obstacle_out <= 1'b0;
        else if (p_read_map_en)   p_read_obstacle_out <= map_data[p_read_x][p_read_y][p_read_z];
        else                      p_read_obstacle_out <= 1'b0;
    end

endmodule

