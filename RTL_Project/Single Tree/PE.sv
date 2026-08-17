`timescale 1ns/10ps

module PE (
    clk,
    rst_n, 
    in_valid, 
    out_valid, 
    rand_num, 

    tree_node_x_read, 
    tree_node_y_read,
    tree_node_z_read,
    tree_parent_read,
    tree_node_x_write, 
    tree_node_y_write,
    tree_node_z_write,
    tree_parent_write,
    max_tree_idx, 
    read_tree_en, 
    write_tree_en, 
    tree_node_addr,

    map_x, 
    map_y, 
    map_z, 
    read_map_en, 
    obstacle
);


    input clk;
    input rst_n;
    input in_valid;
    output logic out_valid;
    input [23:0] rand_num;

    output logic [7:0] tree_node_x_write, tree_node_y_write, tree_node_z_write;
    output logic [9:0] tree_parent_write;
    input [7:0] tree_node_x_read, tree_node_y_read, tree_node_z_read;
    input [9:0] tree_parent_read;
    
    input [9:0] max_tree_idx;
    output logic read_tree_en;
    output logic write_tree_en;
    output logic [9:0] tree_node_addr;

    output logic [7:0] map_x;
    output logic [7:0] map_y;
    output logic [7:0] map_z;
    output logic read_map_en;
    input obstacle;


    parameter START = 0, GOAL = 255;
    parameter IDLE = 0, RANDOM = 1, FETCH = 2, NEAREST1 = 3, NEAREST2 = 4, VECTOR = 5, VECTOR1 = 6, EXTEND = 7, INSERT = 8, DONE = 9;
    logic [3:0] state, n_state;

    parameter E_IDLE = 0, E_NEWNODE = 1, E_CHECKMAP = 2, E_FETCH = 3, E_BRANCH = 4, E_RETURN = 5;
    logic [2:0] ex_state, n_ex_state;

    logic [23:0] rand_num_reg;
    logic [7:0] rand_x_reg, rand_y_reg, rand_z_reg;
    logic [7:0] rand_x, rand_y, rand_z;

    logic [8:0] x_reg, y_reg, z_reg, x, y, z;
    logic [9:0] tree_idx, tree_idx_reg;

    logic [10:0] dist_x_reg, dist_y_reg, dist_z_reg;
    logic [10:0] dist_x, dist_y, dist_z;
    logic [11:0] dist_reg, min_dist;
    logic [9:0] x_1_reg, y_1_reg, z_1_reg, x_1, y_1, z_1;
    logic [10:0] tree_idx_1_reg, tree_idx_1;

    logic [7:0] nearest_x_reg, nearest_y_reg, nearest_z_reg;
    logic [7:0] nearest_x, nearest_y, nearest_z;
    logic [9:0] parent_reg, parent;

    logic [10:0] dx, dy, dz, dx_reg, dy_reg, dz_reg, dx_s, dy_s, dz_s;
    logic x_sign, y_sign, z_sign, x_sign_reg, y_sign_reg, z_sign_reg;

    logic [1:0] vector_x, vector_y, vector_z;
    logic [1:0] vector_x_reg, vector_y_reg, vector_z_reg;
    logic [8:0] new_node_x, new_node_y, new_node_z, new_node_x_reg, new_node_y_reg, new_node_z_reg;

    logic [8:0] last_node_x_reg, last_node_y_reg, last_node_z_reg, last_node_x, last_node_y, last_node_z;
    logic [2:0] ex_cnt, ex_cnt_reg;

    logic in_bound_n, in_bound_n_reg;

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= IDLE;
        else state <= n_state;
    end

    always@(*) begin
        n_state = state;
        case(state)
            IDLE:     if (in_valid) n_state = RANDOM; else n_state = IDLE;
            RANDOM:   n_state = FETCH;
            FETCH:    n_state = NEAREST1;
            NEAREST1: n_state = NEAREST2;
            NEAREST2: if (max_tree_idx == tree_idx_reg) n_state = VECTOR; else n_state = RANDOM;
            VECTOR:   n_state = VECTOR1;
            VECTOR1:  n_state = EXTEND;
            EXTEND:   if (ex_state == E_RETURN) n_state = INSERT; else n_state = EXTEND;
            INSERT:   n_state = DONE;
            DONE:     n_state = IDLE;
            default:  n_state = IDLE;
        endcase
    end

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) ex_state <= 0;
        else ex_state <= n_ex_state;
    end

    always@(*) begin
        n_ex_state = ex_state;
        case(ex_state)
            E_IDLE: begin
                if (state == EXTEND) n_ex_state = E_NEWNODE;
                else n_ex_state = E_IDLE;
            end
            E_NEWNODE: begin
                if (!(vector_x_reg[0] || vector_y_reg[0] || vector_z_reg[0])) n_ex_state = E_RETURN;
                else n_ex_state = E_CHECKMAP;
            end
            E_CHECKMAP: n_ex_state = E_FETCH;
            E_FETCH: begin
                if (obstacle || in_bound_n_reg) n_ex_state = E_RETURN;
                else n_ex_state = E_BRANCH;
            end
            E_BRANCH: n_ex_state = E_NEWNODE;
            E_RETURN: if (state == DONE) n_ex_state = E_IDLE; else n_ex_state = E_RETURN;
        endcase
    end

    //-------- Random Number Intake --------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rand_x_reg <= 0;
            rand_y_reg <= 0;
            rand_z_reg <= 0;
        end else begin
            rand_x_reg <= rand_x;
            rand_y_reg <= rand_y;
            rand_z_reg <= rand_z;
        end
    end

    always @(*) begin
        if (state == RANDOM) begin
            if (rand_num[3:0] == 4'h0) begin
                rand_x = GOAL;
                rand_y = GOAL;
                rand_z = GOAL;
            end else begin
                rand_x = rand_num[7:0];
                rand_y = rand_num[15:8];
                rand_z = rand_num[23:16];
            end
        end else begin
            rand_x = rand_x_reg;
            rand_y = rand_y_reg;
            rand_z = rand_z_reg;
        end
    end

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_reg <= 0;
            y_reg <= 0;
            z_reg <= 0;
            tree_idx_reg <= 0;
        end else begin
            x_reg <= x;
            y_reg <= y;
            z_reg <= z;
            tree_idx_reg <= tree_idx;
        end
    end

    always @(*) begin
        if (state == RANDOM) begin
            read_tree_en = 1;
            tree_node_addr = tree_idx_reg;
        end else begin
            read_tree_en = 0;
            tree_node_addr = 0;
        end
    end

    always @(*) begin
        if (state == FETCH) begin
            x = tree_node_x_read;
            y = tree_node_y_read;
            z = tree_node_z_read;
        end else begin
            x = x_reg;
            y = y_reg;
            z = z_reg;
        end
    end

    assign tree_idx = (state == IDLE) ? 0 : (state == NEAREST1) ? tree_idx_reg + 1 : tree_idx_reg;

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dist_x_reg <= 0;
            dist_y_reg <= 0;
            dist_z_reg <= 0;
            x_1_reg <= 0;
            y_1_reg <= 0;
            z_1_reg <= 0;
            tree_idx_1_reg <= 0;
        end else begin
            dist_x_reg <= dist_x;
            dist_y_reg <= dist_y;
            dist_z_reg <= dist_z;
            x_1_reg <= x_reg;
            y_1_reg <= y_reg;
            z_1_reg <= z_reg;
            tree_idx_1_reg <= tree_idx_reg;
        end
    end

    assign dist_x = (x_reg > rand_x_reg) ? (x_reg - rand_x_reg) : (rand_x_reg - x_reg);
    assign dist_y = (y_reg > rand_y_reg) ? (y_reg - rand_y_reg) : (rand_y_reg - y_reg);
    assign dist_z = (z_reg > rand_z_reg) ? (z_reg - rand_z_reg) : (rand_z_reg - z_reg);

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nearest_x_reg <= 0;
            nearest_y_reg <= 0;
            nearest_z_reg <= 0;
            dist_reg <= 0;
            parent_reg <= 0;
        end else begin
            nearest_x_reg <= nearest_x;
            nearest_y_reg <= nearest_y;
            nearest_z_reg <= nearest_z;
            dist_reg <= min_dist;
            parent_reg <= parent;
        end
    end

    always @(*) begin
        if (state == NEAREST2) begin
            if (dist_x + dist_y + dist_z < dist_reg) begin 
                nearest_x = x_1_reg;
                nearest_y = y_1_reg;
                nearest_z = z_1_reg;
                min_dist = dist_x + dist_y + dist_z;
                parent = tree_idx_1_reg;
            end else begin
                nearest_x = nearest_x_reg;
                nearest_y = nearest_y_reg;
                nearest_z = nearest_z_reg;
                min_dist = dist_reg;
                parent = parent_reg;
            end
        end 
        else if (state == IDLE) begin
            nearest_x = 0;
            nearest_y = 0;
            nearest_z = 0;
            min_dist = 11'h7FF;
            parent = 0;
        end
        else begin
            nearest_x = nearest_x_reg;
            nearest_y = nearest_y_reg;
            nearest_z = nearest_z_reg;
            min_dist = dist_reg;
            parent = parent_reg;
        end
    end

    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dx_reg <= 0; dy_reg <= 0; dz_reg <= 0;
            x_sign_reg <= 0; y_sign_reg <= 0; z_sign_reg <= 0;
        end else begin
            dx_reg <= dx; dy_reg <= dy; dz_reg <= dz;
            x_sign_reg <= dx_s[10]; y_sign_reg <= dy_s[10]; z_sign_reg <= dz_s[10];
        end
    end

    assign dx_s = rand_x_reg - nearest_x_reg;
    assign dy_s = rand_y_reg - nearest_y_reg;
    assign dz_s = rand_z_reg - nearest_z_reg;

    assign dx = (dx_s[10]) ? -dx_s : dx_s;
    assign dy = (dy_s[10]) ? -dy_s : dy_s;
    assign dz = (dz_s[10]) ? -dz_s : dz_s;
    
    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vector_x_reg <= 0;
            vector_y_reg <= 0;
            vector_z_reg <= 0;
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
            vector_x = vector_x_reg;
            vector_y = vector_y_reg;
            vector_z = vector_z_reg;
        end
    end


    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            new_node_x_reg <= 0;
            new_node_y_reg <= 0;
            new_node_z_reg <= 0;
            last_node_x_reg <= 0;
            last_node_y_reg <= 0;
            last_node_z_reg <= 0;
            ex_cnt_reg <= 0;
        end 
        else begin
            new_node_x_reg <= new_node_x;
            new_node_y_reg <= new_node_y;
            new_node_z_reg <= new_node_z;
            last_node_x_reg <= last_node_x;
            last_node_y_reg <= last_node_y;
            last_node_z_reg <= last_node_z;
            ex_cnt_reg <= ex_cnt;
        end
    end

    always@(*) begin
        if (ex_state == E_NEWNODE) begin
            casex(vector_x_reg)
                2'bx0: new_node_x = last_node_x_reg;
                2'b01: new_node_x = last_node_x_reg + 1;
                2'b11: new_node_x = last_node_x_reg - 1;
                default: new_node_x = last_node_x_reg;
            endcase
            casex(vector_y_reg)
                2'bx0: new_node_y = last_node_y_reg;
                2'b01: new_node_y = last_node_y_reg + 1;
                2'b11: new_node_y = last_node_y_reg - 1;
                default: new_node_y = last_node_y_reg;
            endcase
            casex(vector_z_reg)
                2'bx0: new_node_z = last_node_z_reg;
                2'b01: new_node_z = last_node_z_reg + 1;
                2'b11: new_node_z = last_node_z_reg - 1;
                default: new_node_z = last_node_z_reg;
            endcase
        end else if (ex_state == E_IDLE) begin
            new_node_x = 0;
            new_node_y = 0;
            new_node_z = 0;
        end else begin
            new_node_x = new_node_x_reg;
            new_node_y = new_node_y_reg;
            new_node_z = new_node_z_reg;
        end
    end

    always@(*) begin
        if (ex_state == E_IDLE) begin
            last_node_x = nearest_x_reg;
            last_node_y = nearest_y_reg;
            last_node_z = nearest_z_reg;
        end
        else if (ex_state == E_BRANCH) begin
            last_node_x = new_node_x_reg;
            last_node_y = new_node_y_reg;
            last_node_z = new_node_z_reg;
        end
        else begin
            last_node_x = last_node_x_reg;
            last_node_y = last_node_y_reg;
            last_node_z = last_node_z_reg;
        end
    end

    always@(*) begin
        if (ex_state == E_BRANCH) ex_cnt = 1;
        else if (ex_state == E_IDLE) ex_cnt = 0;
        else ex_cnt = ex_cnt_reg;
    end


    always@(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_bound_n_reg <= 0;
        end 
        else begin
            in_bound_n_reg <= in_bound_n;
        end
    end

    assign in_bound_n = (new_node_x_reg[8] ^ last_node_x_reg[8]) | (new_node_y_reg[8] ^ last_node_y_reg[8]) | (new_node_z_reg[8] ^ last_node_z_reg[8]);

    always@(*) begin
        if (ex_state == E_CHECKMAP) begin
            map_x = new_node_x_reg;
            map_y = new_node_y_reg;
            map_z = new_node_z_reg;
            read_map_en = 1;
        end else begin
            map_x = 0;
            map_y = 0;
            map_z = 0;
            read_map_en = 0;
        end
    end


    assign write_tree_en = (state == INSERT && ex_cnt_reg != 0);
    assign tree_node_x_write = (state == INSERT) ? last_node_x_reg[7:0] : 8'b0;
    assign tree_node_y_write = (state == INSERT) ? last_node_y_reg[7:0] : 8'b0;
    assign tree_node_z_write = (state == INSERT) ? last_node_z_reg[7:0] : 8'b0;
    assign tree_parent_write = (state == INSERT) ? parent_reg    : 10'b0;

    assign out_valid = (state == DONE);

endmodule