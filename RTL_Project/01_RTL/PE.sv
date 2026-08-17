`timescale 1ns/10ps
//======================================================================
// PE: one RRT grow context.
//
// Datapath order is fixed:
//   random sample -> nearest neighbour -> vector/branch extend -> CC
//
// The nearest-neighbour scan is a latency-hiding pipeline.  A PE can accept
// another tree response while the previous response is in the distance and
// minimum-update stages.  TREECONT alternates the two contexts of one tree,
// so each tree read port can remain busy without combining the pipeline
// stages into one long timing path.
//======================================================================
module PE #(
    parameter [7:0] SEED = 8'd255,
    parameter logic signed [4:0] STEP_DOWN = -5'sd1
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        in_valid,
    output logic        out_valid,
    input  logic [23:0] rand_num,

    output logic        read_tree_req,
    output logic [3:0]  read_gx,
    output logic [3:0]  read_gy,
    output logic [3:0]  read_gz,
    output logic [3:0]  read_slot,
    input  logic        read_accept,
    input  logic        read_ready,
    input  logic [3:0]  read_rsp_gx,
    input  logic [3:0]  read_rsp_gy,
    input  logic [3:0]  read_rsp_gz,
    input  logic [3:0]  read_rsp_slot,
    input  logic [23:0] tree_node,
    input  logic [15:0] tree_parent,
    input  logic [3:0]  tree_size,

    output logic        node_valid,
    output logic [23:0] node_out,
    output logic [15:0] parent_out,
    input  logic        node_ack,

    output logic        read_map_en,
    output logic [23:0] map_node,
    input  logic        map_accept,
    input  logic        map_ready,
    input  logic        obstacle,
    input  logic        wide_nn
);

    localparam logic [3:0] IDLE=4'd0, RANDOM=4'd1,
                           NN_SCAN=4'd2, NN_DRAIN=4'd3,
                           VECTOR_DELTA=4'd4, VECTOR_QUANT=4'd5,
                           EXTEND=4'd6, INSERT=4'd7, DONE=4'd8;

    localparam logic [2:0] E_IDLE=3'd0, E_STEP=3'd1,
                           E_MAP_REQ=3'd2, E_MAP_EVAL=3'd3,
                           E_RETURN=3'd4;
    localparam logic [3:0] NN_LAST_SLOT = 4'd7;

    logic [3:0] state_q, state_d;
    logic [2:0] ex_state_q, ex_state_d;

    // PATTERN is an external timing boundary.  The core only consumes these
    // registered copies on the cycle after the testbench drives them.
    logic        in_valid_q;
    logic [23:0] rand_num_q;

    logic [7:0] sample_x_q, sample_y_q, sample_z_q;
    logic [7:0] sample_x_d, sample_y_d, sample_z_d;

    logic [3:0] center_gx_q, center_gy_q, center_gz_q;
    logic [3:0] center_gx_d, center_gy_d, center_gz_d;
    logic [3:0] scan_gx_q, scan_gy_q, scan_gz_q, scan_slot_q;
    logic [3:0] scan_gx_d, scan_gy_d, scan_gz_d, scan_slot_d;
    logic [3:0] probe_q, probe_d;
    logic [11:0] prefetch_cell_q, prefetch_cell_d;
    logic        tree_req_active_q, tree_req_active_d;
    logic [3:0]  cell_limit_q, cell_limit_d;

    // Registered tree-memory boundary -> distance -> minimum-update pipeline.
    logic        rsp_valid_q;
    logic [23:0] rsp_node_q;
    logic [15:0] rsp_parent_q;
    logic [3:0]  rsp_size_q;
    logic [15:0] rsp_addr_q;
    logic        dist_valid_q, dist_valid_d;
    logic [5:0]  dist_q, dist_d;
    logic [23:0] dist_node_q, dist_node_d;
    logic [15:0] dist_addr_q, dist_addr_d;

    logic [5:0]  min_dist_q, min_dist_d;
    logic [23:0] nearest_q, nearest_d;
    logic [15:0] parent_q, parent_d;

    logic [8:0] dx_q, dy_q, dz_q;
    logic [8:0] dx_d, dy_d, dz_d;
    logic       x_sign_q, y_sign_q, z_sign_q;
    logic       x_sign_d, y_sign_d, z_sign_d;

    // Vector encoding is {decrement, enable}.
    logic [1:0] base_vx_q, base_vy_q, base_vz_q;
    logic [1:0] base_vx_d, base_vy_d, base_vz_d;
    logic [1:0] active_vx_q, active_vy_q, active_vz_q;
    logic [1:0] active_vx_d, active_vy_d, active_vz_d;
    logic [1:0] retry_q, retry_d;

    logic [8:0] last_x_q, last_y_q, last_z_q;
    logic [8:0] last_x_d, last_y_d, last_z_d;
    logic [8:0] candidate_x_q, candidate_y_q, candidate_z_q;
    logic [8:0] candidate_x_d, candidate_y_d, candidate_z_d;
    logic       candidate_oob_q, candidate_oob_d;
    logic [3:0] step_count_q, step_count_d;

    logic [3:0] dist_x_c, dist_y_c, dist_z_c;
    logic [5:0] dist_sum_c;
    logic [3:0] next_probe_c;
    logic [3:0] root_march_probe_c;
    logic       tree_issue_valid_c;
    logic [3:0] tree_issue_gx_c, tree_issue_gy_c, tree_issue_gz_c, tree_issue_slot_c;
    logic [8:0] next_candidate_x_c, next_candidate_y_c, next_candidate_z_c;
    logic       next_candidate_oob_c;
    logic [8:0] next_candidate_x_q, next_candidate_y_q, next_candidate_z_q;
    logic       next_candidate_oob_q;
    logic       map_issue_valid_c;
    logic [23:0] map_issue_node_c;
    logic       map_rsp_valid_q;
    logic       map_obstacle_q;
    logic fallback_valid_c;
    logic [1:0] fallback_vx_c, fallback_vy_c, fallback_vz_c;
    logic [1:0] fallback_retry_c;

    function automatic [3:0] step_grid(
        input [3:0] g,
        input logic away_from_root
    );
        logic step_high;
        begin
            step_high = (STEP_DOWN > 0) ^ away_from_root;
            if (step_high)
                step_grid = (g == 4'd15) ? 4'd15 : (g + 4'd1);
            else
                step_grid = (g == 4'd0) ? 4'd0 : (g - 4'd1);
        end
    endfunction

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

    // The narrow PE searches the root-side octant.  A wide PE in each tree
    // also checks three lateral cells to preserve exploration around obstacles.
    function automatic [11:0] probe_cell(
        input [3:0] probe,
        input logic wide_search,
        input [3:0] cx,
        input [3:0] cy,
        input [3:0] cz,
        input [3:0] gx,
        input [3:0] gy,
        input [3:0] gz
    );
        begin
            if (wide_search) begin
                case (probe)
                    4'd1:  probe_cell = {step_grid(cx,1'b0), step_grid(cy,1'b0), step_grid(cz,1'b0)};
                    4'd2:  probe_cell = {cx,                   step_grid(cy,1'b0), step_grid(cz,1'b0)};
                    4'd3:  probe_cell = {step_grid(cx,1'b1), step_grid(cy,1'b0), step_grid(cz,1'b0)};
                    4'd4:  probe_cell = {step_grid(cx,1'b0), cy,                   step_grid(cz,1'b0)};
                    4'd5:  probe_cell = {cx,                   cy,                   step_grid(cz,1'b0)};
                    4'd6:  probe_cell = {step_grid(cx,1'b0), step_grid(cy,1'b1), step_grid(cz,1'b0)};
                    4'd7:  probe_cell = {step_grid(cx,1'b0), step_grid(cy,1'b0), cz};
                    4'd8:  probe_cell = {cx,                   step_grid(cy,1'b0), cz};
                    4'd9:  probe_cell = {step_grid(cx,1'b0), cy,                   cz};
                    4'd10: probe_cell = {step_grid(cx,1'b0), step_grid(cy,1'b0), step_grid(cz,1'b1)};
                    default: probe_cell = {step_grid(gx,1'b0), step_grid(gy,1'b0), step_grid(gz,1'b0)};
                endcase
            end else begin
                case (probe)
                    4'd1:  probe_cell = {step_grid(cx,1'b0), step_grid(cy,1'b0), step_grid(cz,1'b0)};
                    4'd2:  probe_cell = {cx,                   step_grid(cy,1'b0), step_grid(cz,1'b0)};
                    4'd3:  probe_cell = {step_grid(cx,1'b0), cy,                   step_grid(cz,1'b0)};
                    4'd4:  probe_cell = {cx,                   cy,                   step_grid(cz,1'b0)};
                    4'd5:  probe_cell = {step_grid(cx,1'b0), step_grid(cy,1'b0), cz};
                    4'd6:  probe_cell = {cx,                   step_grid(cy,1'b0), cz};
                    4'd7:  probe_cell = {step_grid(cx,1'b0), cy,                   cz};
                    default: probe_cell = {step_grid(gx,1'b0), step_grid(gy,1'b0), step_grid(gz,1'b0)};
                endcase
            end
        end
    endfunction

    always @(*) begin
        // rsp_node_q packs x/y/z; select each local nibble explicitly.
        dist_x_c = local_axis_score(rsp_addr_q[15:12], sample_x_q[7:4],
                                    rsp_node_q[19:16], sample_x_q[3:0]);
        dist_y_c = local_axis_score(rsp_addr_q[11:8], sample_y_q[7:4],
                                    rsp_node_q[11:8], sample_y_q[3:0]);
        dist_z_c = local_axis_score(rsp_addr_q[7:4], sample_z_q[7:4],
                                    rsp_node_q[3:0], sample_z_q[3:0]);
        dist_sum_c = {2'b0,dist_x_c} + {2'b0,dist_y_c} + {2'b0,dist_z_c};

        root_march_probe_c = wide_nn ? 4'd11 : 4'd8;
        next_probe_c = (probe_q < root_march_probe_c) ?
                       (probe_q + 4'd1) : root_march_probe_c;

        next_candidate_x_c = !active_vx_q[0] ? candidate_x_q :
                             (active_vx_q[1] ? (candidate_x_q - 9'd1) :
                                                (candidate_x_q + 9'd1));
        next_candidate_y_c = !active_vy_q[0] ? candidate_y_q :
                             (active_vy_q[1] ? (candidate_y_q - 9'd1) :
                                                (candidate_y_q + 9'd1));
        next_candidate_z_c = !active_vz_q[0] ? candidate_z_q :
                             (active_vz_q[1] ? (candidate_z_q - 9'd1) :
                                                (candidate_z_q + 9'd1));
        next_candidate_oob_c =
            (active_vx_q[0] && ((active_vx_q[1] && candidate_x_q == 9'd0) ||
                                (!active_vx_q[1] && candidate_x_q == 9'd255))) ||
            (active_vy_q[0] && ((active_vy_q[1] && candidate_y_q == 9'd0) ||
                                (!active_vy_q[1] && candidate_y_q == 9'd255))) ||
            (active_vz_q[0] && ((active_vz_q[1] && candidate_z_q == 9'd0) ||
                                (!active_vz_q[1] && candidate_z_q == 9'd255)));

        // Retry only changes direction before any step is committed, so every
        // stored edge remains one straight collision-checked segment.
        fallback_valid_c = 1'b0;
        fallback_vx_c = 2'b00;
        fallback_vy_c = 2'b00;
        fallback_vz_c = 2'b00;
        fallback_retry_c = retry_q;
        case (retry_q)
            2'd0: begin
                if (base_vx_q[0] && (base_vy_q[0] || base_vz_q[0])) begin
                    fallback_valid_c = 1'b1;
                    fallback_vx_c = base_vx_q;
                    fallback_retry_c = 2'd1;
                end else if (base_vy_q[0] && base_vz_q[0]) begin
                    fallback_valid_c = 1'b1;
                    fallback_vy_c = base_vy_q;
                    fallback_retry_c = 2'd2;
                end
            end
            2'd1: begin
                if (base_vy_q[0]) begin
                    fallback_valid_c = 1'b1;
                    fallback_vy_c = base_vy_q;
                    fallback_retry_c = 2'd2;
                end else if (base_vz_q[0]) begin
                    fallback_valid_c = 1'b1;
                    fallback_vz_c = base_vz_q;
                    fallback_retry_c = 2'd3;
                end
            end
            2'd2: begin
                if (base_vz_q[0]) begin
                    fallback_valid_c = 1'b1;
                    fallback_vz_c = base_vz_q;
                    fallback_retry_c = 2'd3;
                end
            end
            default: ;
        endcase
    end

    // All combinational control and datapath decisions are kept here.  The
    // sequential block below only owns state storage and reset values.
    always @(*) begin
        state_d = state_q;
        ex_state_d = ex_state_q;

        sample_x_d = sample_x_q;
        sample_y_d = sample_y_q;
        sample_z_d = sample_z_q;
        center_gx_d = center_gx_q;
        center_gy_d = center_gy_q;
        center_gz_d = center_gz_q;
        scan_gx_d = scan_gx_q;
        scan_gy_d = scan_gy_q;
        scan_gz_d = scan_gz_q;
        scan_slot_d = scan_slot_q;
        probe_d = probe_q;
        prefetch_cell_d = prefetch_cell_q;
        tree_req_active_d = tree_req_active_q;
        cell_limit_d = cell_limit_q;

        dist_valid_d = rsp_valid_q && (rsp_size_q != 4'd0);
        dist_d = dist_q;
        dist_node_d = dist_node_q;
        dist_addr_d = dist_addr_q;

        min_dist_d = min_dist_q;
        nearest_d = nearest_q;
        parent_d = parent_q;

        dx_d = dx_q;
        dy_d = dy_q;
        dz_d = dz_q;
        x_sign_d = x_sign_q;
        y_sign_d = y_sign_q;
        z_sign_d = z_sign_q;
        base_vx_d = base_vx_q;
        base_vy_d = base_vy_q;
        base_vz_d = base_vz_q;
        active_vx_d = active_vx_q;
        active_vy_d = active_vy_q;
        active_vz_d = active_vz_q;
        retry_d = retry_q;

        last_x_d = last_x_q;
        last_y_d = last_y_q;
        last_z_d = last_z_q;
        candidate_x_d = candidate_x_q;
        candidate_y_d = candidate_y_q;
        candidate_z_d = candidate_z_q;
        candidate_oob_d = candidate_oob_q;
        step_count_d = step_count_q;

        if (rsp_valid_q && (rsp_size_q != 4'd0)) begin
            dist_d = dist_sum_c;
            dist_node_d = rsp_node_q;
            dist_addr_d = rsp_addr_q;
        end

        if (dist_valid_q && (dist_q < min_dist_q)) begin
            min_dist_d = dist_q;
            nearest_d = dist_node_q;
            parent_d = dist_addr_q;
        end

        case (state_q)
            IDLE: begin
                ex_state_d = E_IDLE;
                dist_valid_d = 1'b0;
                tree_req_active_d = 1'b0;
                cell_limit_d = 4'd0;
                if (in_valid_q)
                    state_d = RANDOM;
            end

            RANDOM: begin
                if (rand_num_q[3:0] == 4'h0) begin
                    sample_x_d = SEED;
                    sample_y_d = SEED;
                    sample_z_d = SEED;
                    center_gx_d = SEED[7:4];
                    center_gy_d = SEED[7:4];
                    center_gz_d = SEED[7:4];
                    scan_gx_d = SEED[7:4];
                    scan_gy_d = SEED[7:4];
                    scan_gz_d = SEED[7:4];
                end else begin
                    sample_x_d = rand_num_q[7:0];
                    sample_y_d = rand_num_q[15:8];
                    sample_z_d = rand_num_q[23:16];
                    center_gx_d = rand_num_q[7:4];
                    center_gy_d = rand_num_q[15:12];
                    center_gz_d = rand_num_q[23:20];
                    scan_gx_d = rand_num_q[7:4];
                    scan_gy_d = rand_num_q[15:12];
                    scan_gz_d = rand_num_q[23:20];
                end
                scan_slot_d = 4'd0;
                probe_d = 4'd0;
                prefetch_cell_d = probe_cell(4'd1, wide_nn,
                                             center_gx_d, center_gy_d, center_gz_d,
                                             scan_gx_d, scan_gy_d, scan_gz_d);
                dist_valid_d = 1'b0;
                tree_req_active_d = !read_accept;
                cell_limit_d = 4'd0;
                min_dist_d = 6'h3F;
                nearest_d = 24'd0;
                parent_d = 16'd0;
                state_d = NN_SCAN;
            end

            NN_SCAN: begin
                // Once the first response reveals the cell occupancy, issue
                // the remaining valid slots as a burst.  read_accept is the
                // arbiter handshake, not memory data, so it can advance this
                // registered request cursor without violating the input rule.
                if (read_accept && tree_req_active_q) begin
                    if (cell_limit_q == 4'd0) begin
                        tree_req_active_d = 1'b0;
                    end else if ((scan_slot_q + 4'd1) < cell_limit_q) begin
                        scan_slot_d = scan_slot_q + 4'd1;
                    end else begin
                        tree_req_active_d = 1'b0;
                    end
                end

                if (rsp_valid_q) begin
                    if (rsp_size_q == 4'd0) begin
                        probe_d = next_probe_c;
                        scan_gx_d = prefetch_cell_q[11:8];
                        scan_gy_d = prefetch_cell_q[7:4];
                        scan_gz_d = prefetch_cell_q[3:0];
                        scan_slot_d = 4'd0;
                        cell_limit_d = 4'd0;
                        tree_req_active_d = !read_accept;
                        prefetch_cell_d = probe_cell(
                            (next_probe_c < root_march_probe_c) ?
                                (next_probe_c + 4'd1) : root_march_probe_c,
                            wide_nn,
                            center_gx_q, center_gy_q, center_gz_q,
                            prefetch_cell_q[11:8], prefetch_cell_q[7:4], prefetch_cell_q[3:0]);
                    end else if (rsp_addr_q[3:0] == 4'd0) begin
                        cell_limit_d = (rsp_size_q > (NN_LAST_SLOT + 4'd1)) ?
                                       (NN_LAST_SLOT + 4'd1) : rsp_size_q;
                        if (rsp_size_q > 4'd1) begin
                            scan_gx_d = rsp_addr_q[15:12];
                            scan_gy_d = rsp_addr_q[11:8];
                            scan_gz_d = rsp_addr_q[7:4];
                            if (read_accept) begin
                                scan_slot_d = 4'd2;
                                tree_req_active_d = (rsp_size_q > 4'd2);
                            end else begin
                                scan_slot_d = 4'd1;
                                tree_req_active_d = 1'b1;
                            end
                        end else begin
                            tree_req_active_d = 1'b0;
                            state_d = NN_DRAIN;
                        end
                    end else if ((rsp_addr_q[3:0] + 4'd1) >= cell_limit_q) begin
                        tree_req_active_d = 1'b0;
                        state_d = NN_DRAIN;
                    end
                end
            end

            NN_DRAIN: begin
                if (!rsp_valid_q && !dist_valid_q)
                    state_d = VECTOR_DELTA;
            end

            VECTOR_DELTA: begin
                if (sample_x_q >= nearest_q[23:16]) begin
                    dx_d = {1'b0,sample_x_q} - {1'b0,nearest_q[23:16]};
                    x_sign_d = 1'b0;
                end else begin
                    dx_d = {1'b0,nearest_q[23:16]} - {1'b0,sample_x_q};
                    x_sign_d = 1'b1;
                end
                if (sample_y_q >= nearest_q[15:8]) begin
                    dy_d = {1'b0,sample_y_q} - {1'b0,nearest_q[15:8]};
                    y_sign_d = 1'b0;
                end else begin
                    dy_d = {1'b0,nearest_q[15:8]} - {1'b0,sample_y_q};
                    y_sign_d = 1'b1;
                end
                if (sample_z_q >= nearest_q[7:0]) begin
                    dz_d = {1'b0,sample_z_q} - {1'b0,nearest_q[7:0]};
                    z_sign_d = 1'b0;
                end else begin
                    dz_d = {1'b0,nearest_q[7:0]} - {1'b0,sample_z_q};
                    z_sign_d = 1'b1;
                end
                state_d = VECTOR_QUANT;
            end

            VECTOR_QUANT: begin
                // These forms are exactly equivalent to 2*dx>dy and 2*dz>d,
                // but avoid the truncating left shifts in the previous RTL.
                base_vx_d = {x_sign_q, (dx_q > (dy_q >> 1))};
                base_vy_d = {y_sign_q, (dy_q > (dx_q >> 1))};
                base_vz_d = {z_sign_q,
                             ((dz_q > (dx_q >> 1)) &&
                              (dz_q > (dy_q >> 1)))};
                active_vx_d = {x_sign_q, (dx_q > (dy_q >> 1))};
                active_vy_d = {y_sign_q, (dy_q > (dx_q >> 1))};
                active_vz_d = {z_sign_q,
                               ((dz_q > (dx_q >> 1)) &&
                                (dz_q > (dy_q >> 1)))};
                retry_d = 2'd0;
                last_x_d = {1'b0,nearest_q[23:16]};
                last_y_d = {1'b0,nearest_q[15:8]};
                last_z_d = {1'b0,nearest_q[7:0]};
                step_count_d = 4'd0;
                if ((dx_q == 9'd0) && (dy_q == 9'd0) && (dz_q == 9'd0)) begin
                    ex_state_d = E_IDLE;
                    state_d = DONE;
                end else begin
                    ex_state_d = E_STEP;
                    state_d = EXTEND;
                end
            end

            EXTEND: begin
                case (ex_state_q)
                    E_STEP: begin
                        candidate_x_d = !active_vx_q[0] ? last_x_q :
                                        (active_vx_q[1] ? (last_x_q - 9'd1) : (last_x_q + 9'd1));
                        candidate_y_d = !active_vy_q[0] ? last_y_q :
                                        (active_vy_q[1] ? (last_y_q - 9'd1) : (last_y_q + 9'd1));
                        candidate_z_d = !active_vz_q[0] ? last_z_q :
                                        (active_vz_q[1] ? (last_z_q - 9'd1) : (last_z_q + 9'd1));
                        candidate_oob_d =
                            (active_vx_q[0] && ((active_vx_q[1] && last_x_q == 9'd0) ||
                                                (!active_vx_q[1] && last_x_q == 9'd255))) ||
                            (active_vy_q[0] && ((active_vy_q[1] && last_y_q == 9'd0) ||
                                                (!active_vy_q[1] && last_y_q == 9'd255))) ||
                            (active_vz_q[0] && ((active_vz_q[1] && last_z_q == 9'd0) ||
                                                (!active_vz_q[1] && last_z_q == 9'd255)));
                        if (candidate_oob_d) begin
                            if ((step_count_q == 4'd0) && fallback_valid_c) begin
                                active_vx_d = fallback_vx_c;
                                active_vy_d = fallback_vy_c;
                                active_vz_d = fallback_vz_c;
                                retry_d = fallback_retry_c;
                                ex_state_d = E_STEP;
                            end else begin
                                state_d = (step_count_q != 4'd0) ? INSERT : DONE;
                                ex_state_d = E_IDLE;
                            end
                        end else begin
                            ex_state_d = E_MAP_REQ;
                        end
                    end

                    E_MAP_REQ: begin
                        if (map_accept)
                            ex_state_d = E_MAP_EVAL;
                    end

                    E_MAP_EVAL: begin
                        if (map_rsp_valid_q) begin
                            if (!map_obstacle_q && !candidate_oob_q) begin
                                last_x_d = candidate_x_q;
                                last_y_d = candidate_y_q;
                                last_z_d = candidate_z_q;
                                step_count_d = step_count_q + 4'd1;
                                if (step_count_q == 4'd7) begin
                                    state_d = INSERT;
                                    ex_state_d = E_IDLE;
                                end else if (next_candidate_oob_q) begin
                                    state_d = INSERT;
                                    ex_state_d = E_IDLE;
                                end else begin
                                    candidate_x_d = next_candidate_x_q;
                                    candidate_y_d = next_candidate_y_q;
                                    candidate_z_d = next_candidate_z_q;
                                    candidate_oob_d = 1'b0;
                                    ex_state_d = map_accept ? E_MAP_EVAL : E_MAP_REQ;
                                end
                            end else if ((step_count_q == 4'd0) && fallback_valid_c) begin
                                active_vx_d = fallback_vx_c;
                                active_vy_d = fallback_vy_c;
                                active_vz_d = fallback_vz_c;
                                retry_d = fallback_retry_c;
                                candidate_oob_d = 1'b0;
                                ex_state_d = E_STEP;
                            end else begin
                                state_d = (step_count_q != 4'd0) ? INSERT : DONE;
                                ex_state_d = E_IDLE;
                            end
                        end
                    end

                    E_RETURN: begin
                        state_d = (step_count_q != 4'd0) ? INSERT : DONE;
                    end

                    default: ex_state_d = E_RETURN;
                endcase
            end

            INSERT: begin
                if (node_ack)
                    state_d = DONE;
            end

            DONE: begin
                state_d = IDLE;
                ex_state_d = E_IDLE;
            end

            default: begin
                state_d = IDLE;
                ex_state_d = E_IDLE;
            end
        endcase
    end

    // External inputs are captured at the PE boundary.  No state transition or
    // datapath operation below reads PATTERN or memory data directly.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_valid_q      <= 1'b0;
            rand_num_q      <= 24'd0;
            rsp_valid_q     <= 1'b0;
            rsp_node_q      <= 24'd0;
            rsp_parent_q    <= 16'd0;
            rsp_size_q      <= 4'd0;
            rsp_addr_q      <= 16'd0;
            map_rsp_valid_q <= 1'b0;
            map_obstacle_q  <= 1'b0;
        end else begin
            in_valid_q <= in_valid;
            if (in_valid)
                rand_num_q <= rand_num;

            rsp_valid_q <= read_ready;
            if (read_ready) begin
                rsp_node_q   <= tree_node;
                rsp_parent_q <= tree_parent;
                rsp_size_q   <= tree_size;
                rsp_addr_q   <= {read_rsp_gx, read_rsp_gy, read_rsp_gz, read_rsp_slot};
            end

            map_rsp_valid_q <= map_ready;
            if (map_ready)
                map_obstacle_q <= obstacle;
        end
    end

    // The next straight-line point is calculated while the current map access
    // is in flight.  This hidden pipeline stage removes three incrementers and
    // boundary checks from the MAPCONT output path without adding a cycle.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            next_candidate_x_q   <= 9'd0;
            next_candidate_y_q   <= 9'd0;
            next_candidate_z_q   <= 9'd0;
            next_candidate_oob_q <= 1'b0;
        end else begin
            next_candidate_x_q   <= next_candidate_x_c;
            next_candidate_y_q   <= next_candidate_y_c;
            next_candidate_z_q   <= next_candidate_z_c;
            next_candidate_oob_q <= next_candidate_oob_c;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q <= IDLE;
            ex_state_q <= E_IDLE;
            sample_x_q <= 8'd0; sample_y_q <= 8'd0; sample_z_q <= 8'd0;
            center_gx_q <= 4'd0; center_gy_q <= 4'd0; center_gz_q <= 4'd0;
            scan_gx_q <= 4'd0; scan_gy_q <= 4'd0; scan_gz_q <= 4'd0; scan_slot_q <= 4'd0;
            probe_q <= 4'd0;
            prefetch_cell_q <= 12'd0;
            tree_req_active_q <= 1'b0;
            cell_limit_q <= 4'd0;
            dist_valid_q <= 1'b0; dist_q <= 6'd0; dist_node_q <= 24'd0; dist_addr_q <= 16'd0;
            min_dist_q <= 6'h3F; nearest_q <= 24'd0; parent_q <= 16'd0;
            dx_q <= 9'd0; dy_q <= 9'd0; dz_q <= 9'd0;
            x_sign_q <= 1'b0; y_sign_q <= 1'b0; z_sign_q <= 1'b0;
            base_vx_q <= 2'd0; base_vy_q <= 2'd0; base_vz_q <= 2'd0;
            active_vx_q <= 2'd0; active_vy_q <= 2'd0; active_vz_q <= 2'd0;
            retry_q <= 2'd0;
            last_x_q <= 9'd0; last_y_q <= 9'd0; last_z_q <= 9'd0;
            candidate_x_q <= 9'd0; candidate_y_q <= 9'd0; candidate_z_q <= 9'd0;
            candidate_oob_q <= 1'b0; step_count_q <= 4'd0;
        end else begin
            state_q <= state_d;
            ex_state_q <= ex_state_d;
            sample_x_q <= sample_x_d; sample_y_q <= sample_y_d; sample_z_q <= sample_z_d;
            center_gx_q <= center_gx_d; center_gy_q <= center_gy_d; center_gz_q <= center_gz_d;
            scan_gx_q <= scan_gx_d; scan_gy_q <= scan_gy_d; scan_gz_q <= scan_gz_d; scan_slot_q <= scan_slot_d;
            probe_q <= probe_d;
            prefetch_cell_q <= prefetch_cell_d;
            tree_req_active_q <= tree_req_active_d;
            cell_limit_q <= cell_limit_d;
            dist_valid_q <= dist_valid_d; dist_q <= dist_d; dist_node_q <= dist_node_d; dist_addr_q <= dist_addr_d;
            min_dist_q <= min_dist_d; nearest_q <= nearest_d; parent_q <= parent_d;
            dx_q <= dx_d; dy_q <= dy_d; dz_q <= dz_d;
            x_sign_q <= x_sign_d; y_sign_q <= y_sign_d; z_sign_q <= z_sign_d;
            base_vx_q <= base_vx_d; base_vy_q <= base_vy_d; base_vz_q <= base_vz_d;
            active_vx_q <= active_vx_d; active_vy_q <= active_vy_d; active_vz_q <= active_vz_d;
            retry_q <= retry_d;
            last_x_q <= last_x_d; last_y_q <= last_y_d; last_z_q <= last_z_d;
            candidate_x_q <= candidate_x_d; candidate_y_q <= candidate_y_d; candidate_z_q <= candidate_z_d;
            candidate_oob_q <= candidate_oob_d; step_count_q <= step_count_d;
        end
    end

    // Registered-response bypass: dependent cell/slot requests can be issued
    // from rsp_*_q in the consume cycle instead of waiting for another cursor
    // register update.  No raw SRAM input participates in this command cone.
    always @(*) begin
        tree_issue_valid_c = (state_q == NN_SCAN) && tree_req_active_q;
        tree_issue_gx_c = scan_gx_q;
        tree_issue_gy_c = scan_gy_q;
        tree_issue_gz_c = scan_gz_q;
        tree_issue_slot_c = scan_slot_q;

        // rand_num_q is already the PATTERN boundary register, so the first
        // slot-zero lookup can be accepted while RANDOM initializes q-state.
        if ((state_q == RANDOM) && !wide_nn) begin
            tree_issue_valid_c = 1'b1;
            if (rand_num_q[3:0] == 4'h0) begin
                tree_issue_gx_c = SEED[7:4];
                tree_issue_gy_c = SEED[7:4];
                tree_issue_gz_c = SEED[7:4];
            end else begin
                tree_issue_gx_c = rand_num_q[7:4];
                tree_issue_gy_c = rand_num_q[15:12];
                tree_issue_gz_c = rand_num_q[23:20];
            end
            tree_issue_slot_c = 4'd0;
        end

        if ((state_q == NN_SCAN) && !tree_req_active_q && rsp_valid_q) begin
            if (rsp_size_q == 4'd0) begin
                tree_issue_valid_c = 1'b1;
                tree_issue_gx_c = prefetch_cell_q[11:8];
                tree_issue_gy_c = prefetch_cell_q[7:4];
                tree_issue_gz_c = prefetch_cell_q[3:0];
                tree_issue_slot_c = 4'd0;
            end else if ((rsp_addr_q[3:0] == 4'd0) && (rsp_size_q > 4'd1)) begin
                tree_issue_valid_c = 1'b1;
                tree_issue_gx_c = rsp_addr_q[15:12];
                tree_issue_gy_c = rsp_addr_q[11:8];
                tree_issue_gz_c = rsp_addr_q[7:4];
                tree_issue_slot_c = 4'd1;
            end
        end
    end

    assign read_tree_req = tree_issue_valid_c;
    assign read_gx = tree_issue_gx_c;
    assign read_gy = tree_issue_gy_c;
    assign read_gz = tree_issue_gz_c;
    assign read_slot = tree_issue_slot_c;

    // A successful registered map response may launch the next straight-line
    // step immediately.  If arbitration does not accept it, E_MAP_REQ retries
    // the same registered candidate on the following cycle.
    always @(*) begin
        map_issue_valid_c = (state_q == EXTEND) && (ex_state_q == E_MAP_REQ);
        map_issue_node_c = {candidate_x_q[7:0],candidate_y_q[7:0],candidate_z_q[7:0]};
        if ((state_q == EXTEND) && (ex_state_q == E_MAP_EVAL) &&
            map_rsp_valid_q && !map_obstacle_q && !candidate_oob_q &&
            (step_count_q != 4'd7) && !next_candidate_oob_q) begin
            map_issue_valid_c = 1'b1;
            map_issue_node_c = {next_candidate_x_q[7:0],
                                next_candidate_y_q[7:0],
                                next_candidate_z_q[7:0]};
        end
    end

    assign read_map_en = map_issue_valid_c;
    assign map_node = map_issue_node_c;

    assign node_valid = (state_q == INSERT);
    assign node_out = {last_x_q[7:0],last_y_q[7:0],last_z_q[7:0]};
    assign parent_out = parent_q;
    // DONE is the only legal state with bit 3 set.  Driving the handshake
    // directly from that state flop avoids a multi-level equality decode whose
    // SDF delay can cross the falling-edge stimulus boundary at 3.3 ns.
    assign out_valid = state_q[3];

    // rsp_parent_q is intentionally not used by NN: parent_out is the address
    // of the selected node, not that node's stored parent payload.
endmodule
