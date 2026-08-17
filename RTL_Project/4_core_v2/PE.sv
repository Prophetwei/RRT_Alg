`timescale 1ns/10ps
//======================================================================
//  PE : RRT processing element (one tree-grow iteration per in_valid)
//  [ v3 : root-directed half-cube NN + directed-walk fallback ]
//
//  Parameterized so the SAME module serves both trees:
//     - start tree : SEED = 255, STEP_DOWN = -1 (snaps toward GOAL,
//                    walk fallback toward the START root at grid 0,0,0)
//     - goal  tree : SEED =   0, STEP_DOWN = +1 (snaps toward START,
//                    walk fallback toward the GOAL root at grid 15,15,15)
//
//  Nearest-neighbour:
//     Phase A : search the sample cell, then the ten cells in the neighbouring
//               half-cube that are geometrically closer to this tree's root.
//     Phase B : if those cells are empty, walk one grid step at a time toward
//               the root until a non-empty cell is found.
//
//  Tree read shared via TREECONT (req/read_ready); map via MAPCONT
//  (read_map_en/map_ready).  Steer/extend matches 01_RTL.
//======================================================================
module PE #(
    parameter [7:0] SEED      = 8'd255,
    parameter logic signed [4:0] STEP_DOWN = -5'sd1
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        in_valid,
    output logic        out_valid,
    input  logic [23:0] rand_num,

    // ---- nearest-neighbour grid read (to TREECONT) ----
    output logic        read_tree_req,
    output logic [3:0]  read_gx,
    output logic [3:0]  read_gy,
    output logic [3:0]  read_gz,
    output logic [3:0]  read_slot,
    input  logic        read_ready,
    input  logic [23:0] tree_node,
    input  logic [15:0] tree_parent,
    input  logic [3:0]  tree_size,

    // ---- node hand-off to CC ----
    output logic        node_valid,
    output logic [23:0] node_out,        // {x,y,z}
    output logic [15:0] parent_out,      // {gx,gy,gz,slot} of the parent
    input  logic        node_ack,

    // ---- map read (collision check, to MAPCONT) ----
    output logic        read_map_en,
    output logic [23:0] map_node,        // {x,y,z}
    input  logic        map_ready,
    input  logic        obstacle
);

    //  NN is pipelined into three states so no single cycle chains the whole
    //  distance -> compare -> found -> cursor cone (fits a 3.3 ns cycle):
    //    NN_REQ : issue the grid read, latch the returned node/size (short path)
    //    NN_CAL : compute the Manhattan distance and register it (dist_q)
    //    NN_ADV : compare the registered distance, update the running-min, and
    //             advance the scan cursor
    localparam logic [3:0] IDLE=0, RANDOM=1, NN_REQ=2, NN_CAL=3, NN_ADV=4,
                           VECTOR=5, VECTOR1=6, EXTEND=7, INSERT=8, DONE=9;
    logic [3:0] state, n_state;

    localparam logic [2:0] E_IDLE=0, E_NEWNODE=1, E_CHECKMAP=2,
                           E_FETCH=3, E_RETURN=4;
    logic [2:0] ex_state, n_ex_state;


    // -------- sample --------
    logic [7:0]  rand_x_reg, rand_y_reg, rand_z_reg, rand_x, rand_y, rand_z;

    // -------- NN scan state --------
    logic [3:0]  cx_reg, cy_reg, cz_reg;             // sample's centre cell
    logic [3:0]  gx_reg, gy_reg, gz_reg, slot_reg;   // scan cursor
    logic [3:0]  n_gx, n_gy, n_gz, n_slot;
    logic [7:0]  read_x, read_y, read_z;                     // returned node
    logic [3:0]  grid_size;
    logic        empty_cell, empty_cell_reg;
    logic        full_cell, full_cell_reg;
    logic [3:0]  grid_cnt, grid_cnt_reg;
    logic [7:0]  nearest_x, nearest_y, nearest_z, nearest_x_reg, nearest_y_reg, nearest_z_reg;   // NN result
    logic [15:0] parent, parent_reg;   // {gx,gy,gz,slot} of the NN result
    logic [3:0]  dist_x, dist_y, dist_z;
    logic [5:0]  best_dist_n, min_dist, dist_reg, dist_n;


    // -------- steering / extend --------
    logic [8:0]  dx, dy, dz, dx_reg, dy_reg, dz_reg;
    logic signed [8:0] dx_s, dy_s, dz_s;
    logic        x_sign_reg, y_sign_reg, z_sign_reg;
    logic [1:0]  vector_x, vector_y, vector_z;
    logic [1:0]  vector_x_reg, vector_y_reg, vector_z_reg;
    logic [8:0]  new_node_x, new_node_y, new_node_z;
    logic [8:0]  new_node_x_reg, new_node_y_reg, new_node_z_reg;
    logic [8:0]  last_node_x, last_node_y, last_node_z;
    logic [8:0]  last_node_x_reg, last_node_y_reg, last_node_z_reg;
    logic [2:0]  ex_cnt, ex_cnt_reg;
    logic        in_bound_n, in_bound_n_reg;
    logic        obstacle_reg;
    logic        step_clear;

    function automatic [3:0] step_grid(input [3:0] g, input logic away_from_root);
        logic step_high;
        begin
            step_high = (STEP_DOWN > 0) ^ away_from_root;
            if (step_high)
                step_grid = (g == 4'd15) ? 4'd15 : (g + 4'd1);
            else
                step_grid = (g == 4'd0) ? 4'd0 : (g - 4'd1);
        end
    endfunction

    // All NN candidates belong to one grid cell.  The cell-level part of
    // Manhattan distance is therefore common, so only the 4-bit local offset
    // contribution is needed to preserve the exact ordering within the cell.
    function automatic [3:0] local_axis_score(
        input [3:0] grid_coord,
        input [3:0] sample_grid,
        input [3:0] node_offset,
        input [3:0] sample_offset
    );
        begin
            if (grid_coord < sample_grid)
                local_axis_score = 4'hF - node_offset;
            else if (grid_coord > sample_grid)
                local_axis_score = node_offset;
            else if (node_offset >= sample_offset)
                local_axis_score = node_offset - sample_offset;
            else
                local_axis_score = sample_offset - node_offset;
        end
    endfunction

    //==================================================================
    //  Main FSM
    //==================================================================
    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= IDLE;
        else        state <= n_state;
    end

    always@(*) begin
        n_state = state;
        case(state)
            IDLE:    n_state = in_valid ? RANDOM : IDLE;
            RANDOM:  n_state = NN_REQ;
            NN_REQ:  n_state = read_ready ? NN_CAL : NN_REQ;              // wait for read data
            NN_CAL:  n_state = NN_ADV;                                    // register distance
            NN_ADV:  n_state = full_cell_reg ? VECTOR : NN_REQ;
            VECTOR:  n_state = VECTOR1;
            VECTOR1: n_state = EXTEND;
            EXTEND:  n_state = (ex_state == E_RETURN) ? ((ex_cnt_reg != 0) ? INSERT : DONE) : EXTEND;
            INSERT:  n_state = node_ack ? DONE : INSERT;
            DONE:    n_state = IDLE;
            default: n_state = IDLE;
        endcase
    end

    //==================================================================
    //  Extend sub-FSM
    //==================================================================
    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) ex_state <= E_IDLE;
        else        ex_state <= n_ex_state;
    end

    always@(*) begin
        n_ex_state = ex_state;
        case(ex_state)
            E_IDLE:    n_ex_state = (state == EXTEND) ? E_NEWNODE : E_IDLE;
            E_NEWNODE: n_ex_state = (!(vector_x_reg[0] || vector_y_reg[0] || vector_z_reg[0])) ? E_RETURN : E_CHECKMAP;
            E_CHECKMAP:n_ex_state = map_ready ? E_FETCH : E_CHECKMAP;
            E_FETCH:   n_ex_state = (!step_clear || ex_cnt_reg == 3'd7) ? E_RETURN : E_NEWNODE;
            E_RETURN:  n_ex_state = (state == DONE) ? E_IDLE : E_RETURN;
            default:   n_ex_state = E_IDLE;
        endcase
    end

    //==================================================================
    //  Random sample intake (with seed snap toward the OTHER root)
    //==================================================================
    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) {rand_x_reg, rand_y_reg, rand_z_reg} <= 24'd0;
        else        {rand_x_reg, rand_y_reg, rand_z_reg} <= {rand_x, rand_y, rand_z};
    end

    always@(*) begin
        if (state == RANDOM) begin
            if (rand_num[3:0] == 4'h0) begin
                rand_x = SEED;  rand_y = SEED;  rand_z = SEED;
            end else begin
                rand_x = rand_num[7:0];  rand_y = rand_num[15:8];  rand_z = rand_num[23:16];
            end
        end else begin
            rand_x = rand_x_reg;  rand_y = rand_y_reg;  rand_z = rand_z_reg;
        end
    end

    //==================================================================
    //  Nearest-neighbour : root-side half-cube then walk fallback
    //==================================================================
    assign read_tree_req = (state == NN_REQ);
    assign read_gx = gx_reg;
    assign read_gy = gy_reg;
    assign read_gz = gz_reg;
    assign read_slot = slot_reg;

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grid_size <= 4'd0;
            {read_x, read_y, read_z} <= 24'd0;
        end else if (state == NN_REQ && read_ready) begin
            grid_size <= tree_size;
            {read_x, read_y, read_z} <= tree_node;
        end
    end

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nearest_x_reg <= 8'd0;
            nearest_y_reg <= 8'd0;
            nearest_z_reg <= 8'd0;
            parent_reg    <= 16'd0;
            dist_reg      <= 6'd0;
            min_dist      <= 6'h00;
        end else begin
            nearest_x_reg <= nearest_x; nearest_y_reg <= nearest_y; nearest_z_reg <= nearest_z;
            parent_reg <= parent;
            dist_reg <= dist_n;
            min_dist <= best_dist_n;
        end
    end

    assign dist_x = local_axis_score(gx_reg, rand_x_reg[7:4], read_x[3:0], rand_x_reg[3:0]);
    assign dist_y = local_axis_score(gy_reg, rand_y_reg[7:4], read_y[3:0], rand_y_reg[3:0]);
    assign dist_z = local_axis_score(gz_reg, rand_z_reg[7:4], read_z[3:0], rand_z_reg[3:0]);
    assign dist_n = (grid_size != 4'd0) ?
                    ({2'b0, dist_x} + {2'b0, dist_y} + {2'b0, dist_z}) : 6'd0;

    always@(*) begin
        if (state == NN_ADV && !empty_cell && dist_reg < min_dist) begin
            best_dist_n = dist_reg;
            nearest_x = read_x; nearest_y = read_y; nearest_z = read_z;
            parent = {gx_reg, gy_reg, gz_reg, slot_reg};
        end else if (state == IDLE) begin
            best_dist_n = 6'h3F;   // local score max is 45
            nearest_x = 0; nearest_y = 0; nearest_z = 0;
            parent = 0;
        end else begin
            best_dist_n = min_dist;
            nearest_x = nearest_x_reg; nearest_y = nearest_y_reg; nearest_z = nearest_z_reg;
            parent = parent_reg;
        end
    end

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            empty_cell_reg <= 1'b0;
            full_cell_reg  <= 1'b0;
        end else begin
            empty_cell_reg <= empty_cell;
            full_cell_reg  <= full_cell;
        end
    end

    assign empty_cell = (state == IDLE) ? 0 : (state == NN_CAL) ? (grid_size == 0) : empty_cell_reg;
    assign full_cell  = (state == IDLE) ? 0 : (state == NN_CAL) ? (grid_size != 0 && ({1'b0, slot_reg} + 5'd1 >= {1'b0, grid_size})) : full_cell_reg;

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) grid_cnt_reg <= 4'd0;
        else        grid_cnt_reg <= grid_cnt;
    end

    // Counts the ten unique neighbours strictly closer to the tree root
    // (#toward axes > #away axes).  Once that half-cube is exhausted, keep the
    // counter in the default branch while the cursor walks to root.
    // Saturation is required because a 4-bit wrap would restart the local scan.
    assign grid_cnt = (state == IDLE) ? 4'd0 :
                      (state == NN_CAL && grid_cnt_reg < 4'd11) ? (grid_cnt_reg + 4'd1) :
                      grid_cnt_reg;

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx_reg   <= 4'd0; gy_reg <= 4'd0; gz_reg <= 4'd0; slot_reg <= 4'd0;
            cx_reg   <= 4'd0; cy_reg <= 4'd0; cz_reg <= 4'd0;
        end else begin
            gx_reg <= n_gx; gy_reg <= n_gy; gz_reg <= n_gz; slot_reg <= n_slot;
            cx_reg <= rand_x_reg[7:4]; cy_reg <= rand_y_reg[7:4]; cz_reg <= rand_z_reg[7:4];
        end
    end

    always@(*) begin
        n_gx = gx_reg; n_gy = gy_reg; n_gz = gz_reg; n_slot = slot_reg;
        case(state)
        RANDOM: begin
            n_gx = rand_x[7:4]; n_gy = rand_y[7:4]; n_gz = rand_z[7:4]; n_slot = 0;
        end
        NN_ADV: begin
            if (empty_cell_reg) begin
                n_slot = 0;
                case(grid_cnt_reg)
                1: begin 
                    n_gx = step_grid(cx_reg, 1'b0);
                    n_gy = step_grid(cy_reg, 1'b0);
                    n_gz = step_grid(cz_reg, 1'b0);
                end
                2: begin 
                    n_gx = cx_reg;             
                    n_gy = step_grid(cy_reg, 1'b0);
                    n_gz = step_grid(cz_reg, 1'b0);
                end
                3: begin 
                    n_gx = step_grid(cx_reg, 1'b1);
                    n_gy = step_grid(cy_reg, 1'b0);
                    n_gz = step_grid(cz_reg, 1'b0);
                end
                4: begin 
                    n_gx = step_grid(cx_reg, 1'b0);
                    n_gy = cy_reg;             
                    n_gz = step_grid(cz_reg, 1'b0);
                end
                5: begin 
                    n_gx = cx_reg;
                    n_gy = cy_reg;
                    n_gz = step_grid(cz_reg, 1'b0);
                end
                6: begin 
                    n_gx = step_grid(cx_reg, 1'b0);
                    n_gy = step_grid(cy_reg, 1'b1);
                    n_gz = step_grid(cz_reg, 1'b0);
                end
                7: begin 
                    n_gx = step_grid(cx_reg, 1'b0);
                    n_gy = step_grid(cy_reg, 1'b0);
                    n_gz = cz_reg;             
                end
                8: begin 
                    n_gx = cx_reg;             
                    n_gy = step_grid(cy_reg, 1'b0);
                    n_gz = cz_reg;             
                end
                9: begin 
                    n_gx = step_grid(cx_reg, 1'b0);
                    n_gy = cy_reg;             
                    n_gz = cz_reg;             
                end
                10:begin 
                    n_gx = step_grid(cx_reg, 1'b0);
                    n_gy = step_grid(cy_reg, 1'b0);
                    n_gz = step_grid(cz_reg, 1'b1);
                end
                default: begin
                    n_gx = step_grid(gx_reg, 1'b0);
                    n_gy = step_grid(gy_reg, 1'b0);
                    n_gz = step_grid(gz_reg, 1'b0);
                end
                endcase
            end
            else begin
                n_gx = gx_reg;
                n_gy = gy_reg;
                n_gz = gz_reg;
                n_slot = slot_reg + 1;
            end
        end
        default: begin
            n_gx = gx_reg;
            n_gy = gy_reg;
            n_gz = gz_reg;
            n_slot = slot_reg;
        end
        endcase
    end

    //==================================================================
    //  Direction vector  (matches 01_RTL)
    //==================================================================
    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dx_reg <= 9'd0; dy_reg <= 9'd0; dz_reg <= 9'd0;
            x_sign_reg <= 1'b0; y_sign_reg <= 1'b0; z_sign_reg <= 1'b0;
        end else begin
            dx_reg <= dx; dy_reg <= dy; dz_reg <= dz;
            x_sign_reg <= dx_s[8]; y_sign_reg <= dy_s[8]; z_sign_reg <= dz_s[8];
        end
    end

    assign dx_s = $signed({1'b0, rand_x_reg}) - $signed({1'b0, nearest_x_reg});
    assign dy_s = $signed({1'b0, rand_y_reg}) - $signed({1'b0, nearest_y_reg});
    assign dz_s = $signed({1'b0, rand_z_reg}) - $signed({1'b0, nearest_z_reg});
    assign dx = dx_s[8] ? $unsigned(-dx_s) : $unsigned(dx_s);
    assign dy = dy_s[8] ? $unsigned(-dy_s) : $unsigned(dy_s);
    assign dz = dz_s[8] ? $unsigned(-dz_s) : $unsigned(dz_s);

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vector_x_reg <= 2'd0;
            vector_y_reg <= 2'd0;
            vector_z_reg <= 2'd0;
        end else begin
            vector_x_reg <= vector_x;
            vector_y_reg <= vector_y;
            vector_z_reg <= vector_z;
        end
    end

    always@(*) begin
        if (state == VECTOR1) begin
            vector_x = {x_sign_reg, (dx_reg << 1 > dy_reg)};
            vector_y = {y_sign_reg, (dx_reg >> 1 < dy_reg)};
            vector_z = {z_sign_reg, (dz_reg << 1 > dx_reg && dz_reg << 1 > dy_reg)};
        end else begin
            vector_x = vector_x_reg; vector_y = vector_y_reg; vector_z = vector_z_reg;
        end
    end

    //==================================================================
    //  Stepwise extension  (matches 01_RTL, map read handshaked)
    //==================================================================
    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            new_node_x_reg  <= 9'd0;
            new_node_y_reg  <= 9'd0;
            new_node_z_reg  <= 9'd0;
            last_node_x_reg <= 9'd0;
            last_node_y_reg <= 9'd0;
            last_node_z_reg <= 9'd0;
            ex_cnt_reg      <= 3'd0;
        end else begin
            new_node_x_reg <= new_node_x; new_node_y_reg <= new_node_y; new_node_z_reg <= new_node_z;
            last_node_x_reg <= last_node_x; last_node_y_reg <= last_node_y; last_node_z_reg <= last_node_z;
            ex_cnt_reg <= ex_cnt;
        end
    end

    always@(*) begin
        if (ex_state == E_NEWNODE) begin
            casex(vector_x_reg)
                2'bx0:   new_node_x = last_node_x_reg;
                2'b01:   new_node_x = last_node_x_reg + 1;
                2'b11:   new_node_x = last_node_x_reg - 1;
                default: new_node_x = last_node_x_reg;
            endcase
            casex(vector_y_reg)
                2'bx0:   new_node_y = last_node_y_reg;
                2'b01:   new_node_y = last_node_y_reg + 1;
                2'b11:   new_node_y = last_node_y_reg - 1;
                default: new_node_y = last_node_y_reg;
            endcase
            casex(vector_z_reg)
                2'bx0:   new_node_z = last_node_z_reg;
                2'b01:   new_node_z = last_node_z_reg + 1;
                2'b11:   new_node_z = last_node_z_reg - 1;
                default: new_node_z = last_node_z_reg;
            endcase
        end else if (ex_state == E_IDLE) begin
            new_node_x = 0; new_node_y = 0; new_node_z = 0;
        end else begin
            new_node_x = new_node_x_reg; new_node_y = new_node_y_reg; new_node_z = new_node_z_reg;
        end
    end

    always@(*) begin
        if (ex_state == E_IDLE) begin
            last_node_x = nearest_x_reg; last_node_y = nearest_y_reg; last_node_z = nearest_z_reg;
        end else if (ex_state == E_FETCH && step_clear) begin
            last_node_x = new_node_x_reg; last_node_y = new_node_y_reg; last_node_z = new_node_z_reg;
        end else begin
            last_node_x = last_node_x_reg; last_node_y = last_node_y_reg; last_node_z = last_node_z_reg;
        end
    end

    always@(*) begin
        if (ex_state == E_IDLE)
            ex_cnt = 3'd0;
        else if (ex_state == E_FETCH && step_clear)
            ex_cnt = (ex_cnt_reg == 3'd7) ? 3'd7 : (ex_cnt_reg + 3'd1);
        else
            ex_cnt = ex_cnt_reg;
    end

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) in_bound_n_reg <= 1'b0;
        else        in_bound_n_reg <= in_bound_n;
    end

    assign in_bound_n = (new_node_x_reg[8] ^ last_node_x_reg[8]) |
                        (new_node_y_reg[8] ^ last_node_y_reg[8]) |
                        (new_node_z_reg[8] ^ last_node_z_reg[8]);

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) obstacle_reg <= 1'b0;
        else if (ex_state == E_CHECKMAP && map_ready) obstacle_reg <= obstacle;
    end

    assign step_clear = !(obstacle_reg || in_bound_n_reg);
    assign read_map_en = (ex_state == E_CHECKMAP);
    assign map_node    = {new_node_x_reg[7:0], new_node_y_reg[7:0], new_node_z_reg[7:0]};

    //==================================================================
    //  Outputs to CC
    //==================================================================
    assign node_valid = (state == INSERT);
    assign node_out   = {last_node_x_reg[7:0], last_node_y_reg[7:0], last_node_z_reg[7:0]};
    assign parent_out = parent_reg;
    assign out_valid  = (state == DONE);

endmodule
