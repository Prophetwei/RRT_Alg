module MAPMEM (
    input  logic        clk,
    input  logic        rst_n,
    
    // --- Port 1: Write Path ---
    input  logic        write_map_en,
    input  logic [7:0]  write_x,
    input  logic [7:0]  write_y,
    input  logic [7:0]  write_z,
    input  logic        write_obstacle_val,
    
    // --- Port 2: Global Clear Command ---
    input  logic        global_clear_en, // Pulse high for 1 cycle to clear everything
    output logic        clear_done,      // 1 when the whole map is clean
    
    // --- Port 3: Read Path ---
    input  logic        read_map_en,
    input  logic [7:0]  read_x,
    input  logic [7:0]  read_y,
    input  logic [7:0]  read_z,
    output logic        obstacle_out
);

    // 16-Megabit Array Matrix
    logic map_data [0:255][0:255][0:255];

    // --- The Reset Shift Register ---
    // A 256-bit chain. Each bit represents one X-plate.
    logic [255:0] clear_shift_reg;

    // The map is done clearing when there are no '1's left in the shift register
    assign clear_done = (clear_shift_reg == 256'd0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // On system power-up, kick off an automatic full clear
            clear_shift_reg <= 256'd1; // Puts a '1' in the very first bit position
        end
        else if (global_clear_en) begin
            // If the driving software requests a fresh clear during operation
            clear_shift_reg <= 256'd1; 
        end
        else begin
            // Shift the '1' to the left by 1 position every clock cycle
            clear_shift_reg <= clear_shift_reg << 1;
        end
    end

    // --- Hardware Memory Manipulation ---
    always_ff @(posedge clk) begin
        if (!clear_done) begin
            // SHIFT REGISTER RESET ACTIVE:
            // We loop through all 256 bits of the shift register.
            // Whichever index 'i' is currently 1, we clear that entire X-plate instantly!
            for (int i = 0; i < 256; i++) begin
                if (clear_shift_reg[i]) begin
                    // Wipes out the entire 2D (Y,Z) matrix for plate 'i' in one cycle
                    for (int y = 0; y < 256; y++) begin
                        for (int z = 0; z < 256; z++) begin
                            map_data[i][y][z] <= 1'b0;
                        end
                    end
                end
            end
        end
        else if (write_map_en) begin
            // NORMAL OPERATION: Write single incoming sensor coordinates
            map_data[write_x][write_y][write_z] <= write_obstacle_val;
        end
    end

    // --- Read Logic ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)          obstacle_out <= 1'b0;
        else if (read_map_en) obstacle_out <= map_data[read_x][read_y][read_z];
        else                 obstacle_out <= 1'b0;
    end

endmodule