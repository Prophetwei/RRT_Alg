`ifndef CYCLE_TIME
`define CYCLE_TIME 3.3
`endif

`timescale 1ns/10ps

//======================================================================
//  PATTERN : driver + self-checker for the 4-core dual-tree RRT
//
//  Flow:
//    1. clear MAPMEM and load one obstacle map
//       (+NOMAP : skip the load -> empty map, for fast logic bring-up)
//    2. reset core/TREEMEM/PATHMEM for each seed while retaining that map
//    3. drive the four PEs with random samples until `path` is asserted
//    4. verify the produced path:
//         - starts at START (0,0,0), ends at GOAL (255,255,255)
//         - every node is obstacle-free
//         - consecutive nodes form a straight, in-bounds, collision-free
//           segment (every intermediate voxel is checked)
//    5. +BATCH repeats this for MAP_COUNT maps x NSEEDS seeds
//======================================================================
module PATTERN (
    output logic        clk,
    output logic        rst_n,
    output logic        map_rst_n,

    output logic        in_valid0, input logic out_valid0, output logic [23:0] rand_num0,
    output logic        in_valid1, input logic out_valid1, output logic [23:0] rand_num1,
    output logic        in_valid2, input logic out_valid2, output logic [23:0] rand_num2,
    output logic        in_valid3, input logic out_valid3, output logic [23:0] rand_num3,

    // map write (obstacles loaded by back-door poke; ports kept for completeness)
    output logic        write_map_en,
    output logic [7:0]  write_x, write_y, write_z,
    output logic        write_obstacle_val,

    output logic        global_clear_en,
    input  logic        clear_done,

    // map verify read port
    output logic        p_read_map_en,
    output logic [7:0]  p_read_x, p_read_y, p_read_z,
    input  logic        p_read_obstacle,

    // path memory read port
    output logic        read_path_en,
    output logic [9:0]  read_path_addr,
    input  logic [23:0] read_path_node,
    input  logic [9:0]  path_size,

    input  logic        path
);
    localparam integer DEFAULT_TIMEOUT = 40_000_000;

    real CYCLE = `CYCLE_TIME;
    initial begin
        clk = 1'b0;
        #(CYCLE);                     // let asynchronous reset settle first
        forever #(CYCLE/2.0) clk = ~clk;
    end

    integer       latency;
    integer       fd, mx, my, mz, status;
    integer       seed0, seed1, seed2, seed3;
    integer       timeout_cycles;
    integer       batch_mode;
    integer       start_seed, num_seeds, map_count;
    integer       map_index, seed_index, case_seed;
    integer       case_status, map_load_ok;
    integer       total_cases, pass_cases, fail_cases;
    integer       result_fd;
    string        map_file, map_dir, result_file;
    logic [23:0]  pn [0:1023];

    //==================================================================
    //  Main sequence
    //==================================================================
    initial begin
        rst_n=0; map_rst_n=0;
        in_valid0=0; in_valid1=0; in_valid2=0; in_valid3=0;
        rand_num0=0; rand_num1=0; rand_num2=0; rand_num3=0;
        write_map_en=0; write_x=0; write_y=0; write_z=0; write_obstacle_val=0;
        global_clear_en=0; p_read_map_en=0; p_read_x=0; p_read_y=0; p_read_z=0;
        read_path_en=0; read_path_addr=0; latency=0;

        batch_mode = $test$plusargs("BATCH");
        if (!$value$plusargs("TIMEOUT=%d", timeout_cycles))
            timeout_cycles = DEFAULT_TIMEOUT;
        if (timeout_cycles < 1) begin
            $display("ERROR: TIMEOUT must be positive");
            $finish;
        end

        result_fd = 0;
        if ($value$plusargs("OUT=%s", result_file)) begin
            result_fd = $fopen(result_file, "w");
            if (result_fd == 0) begin
                $display("ERROR: cannot open result file %s", result_file);
                $finish;
            end
            $fdisplay(result_fd,
                      "# map seed success status cycles time_ms start_nodes goal_nodes path_nodes");
            $fdisplay(result_fd,
                      "# status: 0=PASS 1=TIMEOUT 2=X_PROPAGATION 3=PATH_CHECK_FAIL");
        end

        // MAPMEM receives a separate reset so each map is loaded only once;
        // core/TREEMEM/PATHMEM can then be reset independently for every seed.
        repeat (3) @(negedge clk);
        map_rst_n=1;
        wait (clear_done);
        @(negedge clk);

        total_cases=0; pass_cases=0; fail_cases=0;
        if (batch_mode) begin
            if (!$value$plusargs("MAPDIR=%s", map_dir)) map_dir = "../map/";
            if (!$value$plusargs("START_SEED=%d", start_seed)) start_seed = 32'd42;
            if (!$value$plusargs("NSEEDS=%d", num_seeds)) num_seeds = 50;
            if (!$value$plusargs("MAP_COUNT=%d", map_count)) map_count = 10;
            if (num_seeds < 1 || map_count < 1) begin
                $display("ERROR: NSEEDS and MAP_COUNT must be positive");
                $finish;
            end

            $display("========================================================");
            $display(" RTL batch: maps=%0d seeds=%0d..%0d timeout=%0d cycles",
                     map_count, start_seed, start_seed+num_seeds-1,
                     timeout_cycles);
            $display(" MAPDIR: %s", map_dir);
            $display("========================================================");

            for (map_index=0; map_index<map_count; map_index=map_index+1) begin
                if (map_index != 0) CLEAR_MAP;
                map_file = $sformatf("%s/map_%0d.txt", map_dir, map_index);
                LOAD_MAP(map_load_ok);
                if (!map_load_ok) begin
                    if (result_fd != 0) $fclose(result_fd);
                    $finish;
                end

                for (seed_index=0; seed_index<num_seeds; seed_index=seed_index+1) begin
                    case_seed = start_seed + seed_index;
                    RESET_CORE;
                    RUN_CASE(case_seed, case_status);
                    REPORT_RESULT(map_index, case_seed, case_status);
                    total_cases = total_cases + 1;
                    if (case_status == 0) pass_cases = pass_cases + 1;
                    else                  fail_cases = fail_cases + 1;
                end
            end

            $display("========================================================");
            $display(" RTL_BATCH_SUMMARY cases=%0d pass=%0d fail=%0d",
                     total_cases, pass_cases, fail_cases);
            $display("========================================================");
        end
        else begin
            map_index = 0;
            if (!$test$plusargs("NOMAP")) begin
                if (!$value$plusargs("MAP=%s", map_file)) map_file = "../map/map.txt";
                LOAD_MAP(map_load_ok);
                if (!map_load_ok) begin
                    if (result_fd != 0) $fclose(result_fd);
                    $finish;
                end
            end
            else begin
                $display("------------- NOMAP : empty obstacle map -------------");
            end

            if (!$value$plusargs("SEED=%d", case_seed)) case_seed = 32'd42;
            RESET_CORE;
            RUN_CASE(case_seed, case_status);
            REPORT_RESULT(map_index, case_seed, case_status);
            if (case_status == 0) DISPLAY_PASS;
        end

        if (result_fd != 0) $fclose(result_fd);
        $finish;
    end

    //==================================================================
    //  Per-case reset and execution
    //==================================================================
    task RESET_CORE;
        begin
            // Assert and release only the core/tree/path reset.  MAPMEM keeps
            // the current obstacle map across all seeds for this map index.
            @(negedge clk);
            rst_n=0;
            in_valid0=0; in_valid1=0; in_valid2=0; in_valid3=0;
            rand_num0=0; rand_num1=0; rand_num2=0; rand_num3=0;
            read_path_en=0; read_path_addr=0;
            repeat (3) @(negedge clk);
            rst_n=1;
        end
    endtask

    task CLEAR_MAP;
        begin
            // Keep the core idle while MAPMEM clears one x-plane per cycle.
            @(negedge clk);
            rst_n=0;
            in_valid0=0; in_valid1=0; in_valid2=0; in_valid3=0;
            read_path_en=0; read_path_addr=0;
            global_clear_en=1;
            @(negedge clk);
            global_clear_en=0;
            wait (clear_done);
            @(negedge clk);
        end
    endtask

    task RUN_CASE;
        input  integer requested_seed;
        output integer run_status;
        integer check_ok;
        begin
            latency = 0;
            run_status = 0;
            seed0 = requested_seed;
            seed1 = requested_seed ^ 32'h0BAD_F00D;
            seed2 = requested_seed ^ 32'hDEAD_BEEF;
            seed3 = requested_seed ^ 32'hFEED_FACE;

            $display("------------- Running map=%0d seed=%0d -------------",
                     map_index, requested_seed);

            // Sample out_valid on clk so SDF propagation cannot move a new
            // request across the following falling-edge stimulus boundary.
            fork : pe_drivers
                begin : drv0
                    @(negedge clk); in_valid0=1; rand_num0=$random(seed0);
                    @(negedge clk); in_valid0=0;
                    forever begin
                        @(posedge clk);
                        while (out_valid0 !== 1'b1) @(posedge clk);
                        @(negedge clk); in_valid0=1; rand_num0=$random(seed0);
                        @(negedge clk); in_valid0=0;
                    end
                end
                begin : drv1
                    @(negedge clk); in_valid1=1; rand_num1=$random(seed1);
                    @(negedge clk); in_valid1=0;
                    forever begin
                        @(posedge clk);
                        while (out_valid1 !== 1'b1) @(posedge clk);
                        @(negedge clk); in_valid1=1; rand_num1=$random(seed1);
                        @(negedge clk); in_valid1=0;
                    end
                end
                begin : drv2
                    @(negedge clk); in_valid2=1; rand_num2=$random(seed2);
                    @(negedge clk); in_valid2=0;
                    forever begin
                        @(posedge clk);
                        while (out_valid2 !== 1'b1) @(posedge clk);
                        @(negedge clk); in_valid2=1; rand_num2=$random(seed2);
                        @(negedge clk); in_valid2=0;
                    end
                end
                begin : drv3
                    @(negedge clk); in_valid3=1; rand_num3=$random(seed3);
                    @(negedge clk); in_valid3=0;
                    forever begin
                        @(posedge clk);
                        while (out_valid3 !== 1'b1) @(posedge clk);
                        @(negedge clk); in_valid3=1; rand_num3=$random(seed3);
                        @(negedge clk); in_valid3=0;
                    end
                end
            join_none

            while (path !== 1'b1 && latency < timeout_cycles && run_status == 0) begin
                @(negedge clk);
                latency = latency + 1;
                if ($isunknown(path)) run_status = 2;
                if (latency % 50000 == 0)
                    $display("  [progress] map=%0d seed=%0d cyc=%0d start_tree=%0d goal_tree=%0d",
                             map_index, requested_seed, latency,
                             TESTBED.s_tree_size, TESTBED.g_tree_size);
            end

            disable pe_drivers;
            in_valid0=0; in_valid1=0; in_valid2=0; in_valid3=0;

            if (run_status == 2) begin
                $display("FAIL: path became X at cycle %0d; check timing violations", latency);
            end
            else if (path !== 1'b1) begin
                run_status = 1;
                $display("FAIL: no connection after %0d cycles", latency);
            end
            else begin
                @(negedge clk);
                CHECK_PATH(check_ok);
                if (!check_ok) run_status = 3;
            end
        end
    endtask

    task REPORT_RESULT;
        input integer result_map;
        input integer result_seed;
        input integer result_status;
        integer result_success;
        begin
            result_success = (result_status == 0);
            $display("RTL_RESULT map=%0d seed=%0d success=%0d status=%0d cycles=%0d time_ms=%0.6f start_nodes=%0d goal_nodes=%0d path_nodes=%0d",
                     result_map, result_seed, result_success, result_status,
                     latency, latency*CYCLE/1000000.0,
                     TESTBED.s_tree_size, TESTBED.g_tree_size,
                     TESTBED.u_PATHMEM.size);
            if (result_fd != 0)
                $fdisplay(result_fd, "%0d %0d %0d %0d %0d %0.9f %0d %0d %0d",
                          result_map, result_seed, result_success, result_status,
                          latency, latency*CYCLE/1000000.0,
                          TESTBED.s_tree_size, TESTBED.g_tree_size,
                          TESTBED.u_PATHMEM.size);
        end
    endtask

    //==================================================================
    //  Back-door map load
    //==================================================================
    task LOAD_MAP;
        output integer load_ok;
        begin
            load_ok = 0;
            fd = $fopen(map_file, "r");
            if (fd == 0) begin
                $display("ERROR: cannot open %s", map_file);
            end
            else begin
                $display("------------- Loading obstacle map: %s -------------", map_file);
                while (!$feof(fd)) begin
                    status = $fscanf(fd, "%d %d %d\n", mx, my, mz);
                    if (status == 3 && mx>=0 && mx<256 && my>=0 && my<256 && mz>=0 && mz<256)
                        TESTBED.u_MAPMEM.map_data[mx][my][mz] = 1'b1;
                end
                $fclose(fd);
                load_ok = 1;
                $display("------------- Map load complete -------------");
            end
        end
    endtask

    //==================================================================
    //  Path verification
    //==================================================================
    task CHECK_PATH;
        output integer check_ok;
        integer i, k, psize;
        integer ax, ay, az, bx, by, bz, dx, dy, dz, m, stx, sty, stz, cx, cy, cz;
        reg fail;
        begin
            check_ok = 0;
            fail  = 0;
            psize = TESTBED.u_PATHMEM.size;
            $display("------------- Verifying path (size = %0d) -------------", psize);

            if (psize < 2) begin
                $display("FAIL: path is empty / too short (size=%0d)", psize);
                disable CHECK_PATH;
            end

            // Read through PATHMEM's logical-address port so the checker is
            // independent of the memory's physical (circular) layout.
            @(negedge clk);
            read_path_en   = 1'b1;
            read_path_addr = 10'd0;
            for (i = 0; i < psize; i = i + 1) begin
                @(negedge clk);
                pn[i] = read_path_node;
                read_path_addr = i + 1;
            end
            read_path_en = 1'b0;

            // ---- endpoints ----
            if (!(pn[0][23:16]==8'd0 && pn[0][15:8]==8'd0 && pn[0][7:0]==8'd0)) begin
                $display("FAIL: path[0] is not START, got (%0d,%0d,%0d)",
                          pn[0][23:16], pn[0][15:8], pn[0][7:0]); fail=1;
            end
            if (!(pn[psize-1][23:16]==8'd255 && pn[psize-1][15:8]==8'd255 && pn[psize-1][7:0]==8'd255)) begin
                $display("FAIL: path[last] is not GOAL, got (%0d,%0d,%0d)",
                          pn[psize-1][23:16], pn[psize-1][15:8], pn[psize-1][7:0]); fail=1;
            end

            // ---- every node obstacle-free ----
            for (i = 0; i < psize; i = i + 1) begin
                if (TESTBED.u_MAPMEM.map_data[pn[i][23:16]][pn[i][15:8]][pn[i][7:0]] !== 1'b0) begin
                    $display("FAIL: node %0d (%0d,%0d,%0d) is inside an obstacle",
                              i, pn[i][23:16], pn[i][15:8], pn[i][7:0]); fail=1;
                end
            end

            // ---- connectivity + every intermediate voxel ----
            for (i = 0; i < psize-1; i = i + 1) begin
                ax=pn[i][23:16];   ay=pn[i][15:8];   az=pn[i][7:0];
                bx=pn[i+1][23:16]; by=pn[i+1][15:8]; bz=pn[i+1][7:0];
                dx=(bx>=ax)?bx-ax:ax-bx;
                dy=(by>=ay)?by-ay:ay-by;
                dz=(bz>=az)?bz-az:az-bz;
                m=dx; if(dy>m) m=dy; if(dz>m) m=dz;

                if (m != 0) begin
                    if (!((dx==0||dx==m)&&(dy==0||dy==m)&&(dz==0||dz==m))) begin
                        $display("FAIL: seg %0d not a straight move (%0d,%0d,%0d)->(%0d,%0d,%0d)",
                                  i, ax,ay,az, bx,by,bz); fail=1;
                    end
                    if (m > 8) begin
                        $display("FAIL: seg %0d too long (%0d voxels)", i, m); fail=1;
                    end
                    stx=(bx>ax)?1:((bx<ax)?-1:0);
                    sty=(by>ay)?1:((by<ay)?-1:0);
                    stz=(bz>az)?1:((bz<az)?-1:0);
                    for (k=1; k<=m; k=k+1) begin
                        cx=ax+stx*k; cy=ay+sty*k; cz=az+stz*k;
                        if (cx<0||cx>255||cy<0||cy>255||cz<0||cz>255) begin
                            $display("FAIL: seg %0d leaves the map at step %0d", i, k); fail=1;
                        end else if (TESTBED.u_MAPMEM.map_data[cx][cy][cz] !== 1'b0) begin
                            $display("FAIL: seg %0d collides at (%0d,%0d,%0d)", i, cx,cy,cz); fail=1;
                        end
                    end
                end
            end

            if (fail) begin
                $display("====================  PATH CHECK FAILED  ====================");
                check_ok = 0;
            end
            else begin
                check_ok = 1;
                $display("====================  PATH CHECK PASSED  ====================");
                $display("  Path length : %0d nodes", psize);
                $display("  Start tree  : %0d nodes", TESTBED.s_tree_size);
                $display("  Goal  tree  : %0d nodes", TESTBED.g_tree_size);
                $display("  Total cycles: %0d", latency);
                $display("  Total time  : %0.3f ms", latency*CYCLE/1000000.0);
            end
        end
    endtask

    task DISPLAY_PASS;
        begin
            $display("");
            $display("  *********************************************************");
            $display("  *   Congratulations! 4-core dual-tree RRT found a path *");
            $display("  *   from START (0,0,0) to GOAL (255,255,255).          *");
            $display("  *********************************************************");
        end
    endtask

endmodule
