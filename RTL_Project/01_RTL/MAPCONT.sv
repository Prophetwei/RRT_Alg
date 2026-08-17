`timescale 1ns/10ps
//======================================================================
//  MAPCONT : synthesizable 4->1 rotating-priority arbiter for the map port.
//  Each PE drives a level request (peN_read_en) + address (peN_read_node)
//  and waits for peN_read_ready; on that cycle peN_read_obstacle is valid.
//  A 2-bit selector chooses the first PE, then priority rotates downward with
//  wraparound.  The PE receiving the current response is skipped until its
//  registered request state advances on the following edge.
//
//  Timing: at cycle t the selected PE's address is presented to MAPMEM;
//  MAPMEM registers the read, so at t+1 the data is valid and returned to
//  whichever PE was selected at t (tracked by grant_pe).
//======================================================================
module MAPCONT (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        pe0_read_en,
    input  logic [23:0] pe0_read_node,
    output logic        pe0_read_accept,
    output logic        pe0_read_ready,
    output logic        pe0_read_obstacle_out,

    input  logic        pe1_read_en,
    input  logic [23:0] pe1_read_node,
    output logic        pe1_read_accept,
    output logic        pe1_read_ready,
    output logic        pe1_read_obstacle_out,

    input  logic        pe2_read_en,
    input  logic [23:0] pe2_read_node,
    output logic        pe2_read_accept,
    output logic        pe2_read_ready,
    output logic        pe2_read_obstacle_out,

    input  logic        pe3_read_en,
    input  logic [23:0] pe3_read_node,
    output logic        pe3_read_accept,
    output logic        pe3_read_ready,
    output logic        pe3_read_obstacle_out,

    // ---- to MAPMEM shared read port ----
    output logic        read_map_en,
    output logic [7:0]  read_x,
    output logic [7:0]  read_y,
    output logic [7:0]  read_z,
    input  logic        read_obstacle_in,
    output logic        obstacle_seen
);

    logic [1:0] grant_pe;      // which PE was driven last cycle
    logic       grant_valid;   // ...and it actually issued a read
    logic [1:0] sel;           // first priority: sel, sel-1, sel-2, sel-3

    logic       sel_en;
    logic [23:0] sel_node;
    logic [1:0]  pick;
    logic        pick_valid;
    logic [3:0]  eligible_req;

    // Mask the current response owner, then use the requested casex decoder.
    always_comb begin
        eligible_req = {pe3_read_en, pe2_read_en, pe1_read_en, pe0_read_en};
        if (grant_valid) eligible_req[grant_pe] = 1'b0;

        pick       = 2'b00;
        pick_valid = 1'b0;
        sel_node   = 24'd0;

        casex ({sel, eligible_req})
            // sel=0: PE0 -> PE3 -> PE2 -> PE1
            6'b00xxx1: begin pick=2'b00; pick_valid=1'b1; sel_node=pe0_read_node; end
            6'b001xx0: begin pick=2'b11; pick_valid=1'b1; sel_node=pe3_read_node; end
            6'b0001x0: begin pick=2'b10; pick_valid=1'b1; sel_node=pe2_read_node; end
            6'b000010: begin pick=2'b01; pick_valid=1'b1; sel_node=pe1_read_node; end

            // sel=1: PE1 -> PE0 -> PE3 -> PE2
            6'b01xx1x: begin pick=2'b01; pick_valid=1'b1; sel_node=pe1_read_node; end
            6'b01xx01: begin pick=2'b00; pick_valid=1'b1; sel_node=pe0_read_node; end
            6'b011x00: begin pick=2'b11; pick_valid=1'b1; sel_node=pe3_read_node; end
            6'b010100: begin pick=2'b10; pick_valid=1'b1; sel_node=pe2_read_node; end

            // sel=2: PE2 -> PE1 -> PE0 -> PE3
            6'b10x1xx: begin pick=2'b10; pick_valid=1'b1; sel_node=pe2_read_node; end
            6'b10x01x: begin pick=2'b01; pick_valid=1'b1; sel_node=pe1_read_node; end
            6'b10x001: begin pick=2'b00; pick_valid=1'b1; sel_node=pe0_read_node; end
            6'b101000: begin pick=2'b11; pick_valid=1'b1; sel_node=pe3_read_node; end

            // sel=3: PE3 -> PE2 -> PE1 -> PE0 (original priority)
            6'b111xxx: begin pick=2'b11; pick_valid=1'b1; sel_node=pe3_read_node; end
            6'b1101xx: begin pick=2'b10; pick_valid=1'b1; sel_node=pe2_read_node; end
            6'b11001x: begin pick=2'b01; pick_valid=1'b1; sel_node=pe1_read_node; end
            6'b110001: begin pick=2'b00; pick_valid=1'b1; sel_node=pe0_read_node; end
            default: ;
        endcase

        sel_en = pick_valid;
    end

    assign read_map_en = sel_en;
    assign read_x = sel_node[23:16];
    assign read_y = sel_node[15:8];
    assign read_z = sel_node[7:0];
    assign pe0_read_accept = pick_valid && (pick == 2'b00);
    assign pe1_read_accept = pick_valid && (pick == 2'b01);
    assign pe2_read_accept = pick_valid && (pick == 2'b10);
    assign pe3_read_accept = pick_valid && (pick == 2'b11);

    // Sequential response tag.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel           <= 2'b00;
            grant_pe    <= 2'b00;
            grant_valid <= 1'b0;
            obstacle_seen <= 1'b0;
        end else begin
            if (pick_valid) sel <= pick - 2'd1;
            grant_pe    <= pick;
            grant_valid <= pick_valid;
            if (grant_valid && read_obstacle_in)
                obstacle_seen <= 1'b1;
        end
    end

    // data valid the cycle after the grant
    always_comb begin
        pe0_read_ready = grant_valid && (grant_pe == 2'b00);
        pe1_read_ready = grant_valid && (grant_pe == 2'b01);
        pe2_read_ready = grant_valid && (grant_pe == 2'b10);
        pe3_read_ready = grant_valid && (grant_pe == 2'b11);
    end

    assign pe0_read_obstacle_out = read_obstacle_in;
    assign pe1_read_obstacle_out = read_obstacle_in;
    assign pe2_read_obstacle_out = read_obstacle_in;
    assign pe3_read_obstacle_out = read_obstacle_in;

endmodule
