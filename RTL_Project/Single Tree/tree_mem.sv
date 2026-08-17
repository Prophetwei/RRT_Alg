module TREEMEM (
    input  logic        clk,
    input  logic        rst_n,

    // --- Control Interface ---
    input  logic        read_tree_en,
    inout  logic        check_ans_en,
    input  logic        write_tree_en,
    input  logic [9:0]  tree_node_addr,
    input  logic [9:0]  check_ans_addr,
    output logic [9:0]  max_tree_idx,

    // --- Dedicated Write Inputs (No Inouts, No Z) ---
    input  logic [7:0]  write_node_x,
    input  logic [7:0]  write_node_y,
    input  logic [7:0]  write_node_z,
    input  logic [9:0]  write_parent,

    // --- Dedicated Read Outputs (No Inouts, No Z) ---
    output logic [7:0]  read_node_x,
    output logic [7:0]  read_node_y,
    output logic [7:0]  read_node_z,
    output logic [9:0]  read_parent
);

    // Memory Arrays (Sized to 256 entries to match your address bounds)
    logic [7:0] mem_x      [0:1023];
    logic [7:0] mem_y      [0:1023];
    logic [7:0] mem_z      [0:1023];
    logic [9:0] mem_parent [0:1023];

    logic [9:0] node_count;
    assign max_tree_idx = node_count;

    // --- Synchronous Write & Control Block ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            node_count <= 10'd1; // Starts at 1 since entry 0 is the root
            
            // Hardcode Root Node into index 0 upon hardware wake-up
            mem_x[0]      <= 8'd0;
            mem_y[0]      <= 8'd0;
            mem_z[0]      <= 8'd0;
            mem_parent[0] <= 10'd1023; // Standard marker for "No Parent" (e.g., all 1s)
        end 
        else if (write_tree_en) begin
            // Safeguard to prevent writing out of memory array bounds
            if (node_count < 10'd1023) begin
                mem_x[node_count]      <= write_node_x;
                mem_y[node_count]      <= write_node_y;
                mem_z[node_count]      <= write_node_z;
                mem_parent[node_count] <= write_parent;
                
                node_count             <= node_count + 1'b1;
            end
        end
    end

    // --- Synchronous Low-Power Read Block ---
    // Registering the read output prevents glitches and downstream combination toggling,
    // which significantly minimizes dynamic power.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_node_x <= 8'd0;
            read_node_y <= 8'd0;
            read_node_z <= 8'd0;
            read_parent <= 10'd0;
        end 
        else if (read_tree_en) begin
            read_node_x <= mem_x[tree_node_addr];
            read_node_y <= mem_y[tree_node_addr];
            read_node_z <= mem_z[tree_node_addr];
            read_parent <= mem_parent[tree_node_addr];
        end 
        else if (check_ans_en) begin
            read_node_x <= mem_x[check_ans_addr];
            read_node_y <= mem_y[check_ans_addr];
            read_node_z <= mem_z[check_ans_addr];
            read_parent <= mem_parent[check_ans_addr];
        end
        else begin
            // LOW POWER: Force data buses strictly to 0 when read is not enabled.
            // This stops downstream logic gates from toggling, cutting dynamic power.
            read_node_x <= 8'd0;
            read_node_y <= 8'd0;
            read_node_z <= 8'd0;
            read_parent <= 10'd0;
        end
    end

endmodule