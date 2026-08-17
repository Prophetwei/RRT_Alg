`define CYCLE_TIME 10.0

//======================================================================
//  PATTERN : driver + self-checker for the dual-tree RRT
//
//  Flow:
//    1. reset, wait for MAPMEM auto-clear, then load obstacles
//    2. drive BOTH PEs with random samples until `path` is asserted
//    3. verify the produced path:
//         - starts at START (0,0,0), ends at GOAL (255,255,255)
//         - every node is obstacle-free
//         - consecutive nodes form a straight, in-bounds, collision-free
//           segment (every intermediate voxel is checked)
//======================================================================
module PATTERN (
    output logic        clk,
    output logic        rst_n,

    output logic        in_valid0,
    input  logic        out_valid0,
    output logic [23:0] rand_num0,

    output logic        in_valid1,
    input  logic        out_valid1,
    output logic [23:0] rand_num1,

    // map write (unused in this TB - map is loaded by back-door poke)
    output logic        write_map_en,
    output logic [7:0]  write_x, write_y, write_z,
    output logic        write_obstacle_val,

    output logic        global_clear_en,
    input  logic        clear_done,

    // map verify read port (declared for completeness)
    output logic        p_read_map_en,
    output logic [7:0]  p_read_x, p_read_y, p_read_z,
    input  logic        p_read_obstacle,

    // path memory read port (declared for completeness)
    output logic        read_path_en,
    output logic [9:0]  read_path_addr,
    input  logic [23:0] read_path_node,
    input  logic [9:0]  path_size,

    input  logic        path
);
    // ---- generous limits so a probabilistic search never false-fails ----
    localparam integer TIMEOUT = 40_000_000;

    real CYCLE = `CYCLE_TIME;
    initial clk = 0;
    always #(CYCLE/2.0) clk = ~clk;

    integer       latency;
    integer       fd, mx, my, mz, status;
    integer       seed0, seed1;
    logic [23:0]  pn [0:1023];

    //==================================================================
    //  Main sequence
    //==================================================================
    initial begin
        rst_n=1; in_valid0=0; in_valid1=0; rand_num0=0; rand_num1=0;
        write_map_en=0; write_x=0; write_y=0; write_z=0; write_obstacle_val=0;
        global_clear_en=0; p_read_map_en=0; p_read_x=0; p_read_y=0; p_read_z=0;
        read_path_en=0; read_path_addr=0; latency=0;

        // ---- random seed (override with +SEED=<n>) ----
        if (!$value$plusargs("SEED=%d", seed0)) seed0 = 32'h1234_5678;
        seed1 = seed0 ^ 32'h0BAD_F00D;

        // ---- reset ----
        @(negedge clk); rst_n=0;
        repeat (3) @(negedge clk); rst_n=1;

        // ---- wait for the map to finish auto-clearing, then load it ----
        wait (clear_done);
        @(negedge clk);
        LOAD_MAP;

        // ---- launch the two PE drivers ----
        // out_valid pulses during DONE, one cycle BEFORE the PE is back in
        // IDLE, so after each completion we wait for IDLE before re-feeding.
        fork
            begin : drv0
                @(negedge clk); in_valid0=1; rand_num0=$random(seed0);   // first sample
                @(negedge clk); in_valid0=0;
                forever begin
                    @(posedge out_valid0);
                    @(negedge clk);                                      // DONE -> IDLE
                    @(negedge clk); in_valid0=1; rand_num0=$random(seed0);
                    @(negedge clk); in_valid0=0;
                end
            end
            begin : drv1
                @(negedge clk); in_valid1=1; rand_num1=$random(seed1);   // first sample
                @(negedge clk); in_valid1=0;
                forever begin
                    @(posedge out_valid1);
                    @(negedge clk);                                      // DONE -> IDLE
                    @(negedge clk); in_valid1=1; rand_num1=$random(seed1);
                    @(negedge clk); in_valid1=0;
                end
            end
        join_none

        // ---- wait until the two trees connect ----
        while (!path && latency < TIMEOUT) begin
            @(negedge clk); latency = latency + 1;
            if (latency % 50000 == 0) begin
                $display("  [progress] cyc=%0d  start_tree=%0d  goal_tree=%0d",
                          latency, TESTBED.s_max_idx, TESTBED.g_max_idx);
            end
        end

        if (!path) begin
            $display("--------------------------------------------------------");
            $display("                 OUTPUT TIMEOUT FAIL!                   ");
            $display("   No connection after %0d cycles.                      ", latency);
            $display("--------------------------------------------------------");
            $finish;
        end

        // give PATHMEM one cycle of settle margin
        @(negedge clk);
        CHECK_PATH;
        DISPLAY_PASS;
        $finish;
    end

    //==================================================================
    //  Back-door map load (1.16M voxels -> done in zero sim time)
    //==================================================================
    task LOAD_MAP;
        begin
            fd = $fopen("../PE_test/map.txt", "r");
            if (fd == 0) begin $display("ERROR: cannot open ../PE_test/map.txt"); $finish; end
            $display("------------- Loading obstacle map -------------");
            while (!$feof(fd)) begin
                status = $fscanf(fd, "%d %d %d\n", mx, my, mz);
                if (status == 3 && mx>=0 && mx<256 && my>=0 && my<256 && mz>=0 && mz<256)
                    TESTBED.u_MAPMEM.map_data[mx][my][mz] = 1'b1;
            end
            $fclose(fd);
            $display("------------- Map load complete -------------");
        end
    endtask

    //==================================================================
    //  Path verification
    //==================================================================
    task CHECK_PATH;
        integer i, k, psize;
        integer ax, ay, az, bx, by, bz, dx, dy, dz, m, stx, sty, stz, cx, cy, cz;
        reg fail;
        begin
            fail  = 0;
            psize = TESTBED.u_PATHMEM.size;
            $display("------------- Verifying path (size = %0d) -------------", psize);

            if (psize < 2) begin
                $display("FAIL: path is empty / too short (size=%0d)", psize);
                $finish;
            end

            for (i = 0; i < psize; i = i + 1)
                pn[i] = TESTBED.u_PATHMEM.mem[i];

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
                $finish;
            end
            $display("====================  PATH CHECK PASSED  ====================");
            $display("  Path length : %0d nodes", psize);
            $display("  Start tree  : %0d nodes", TESTBED.s_max_idx);
            $display("  Goal  tree  : %0d nodes", TESTBED.g_max_idx);
            $display("  Total cycles: %0d", latency);
        end
    endtask

    task DISPLAY_PASS;
        begin
            $display("");
            $display("  *********************************************************");
            $display("  *   Congratulations! Dual-tree RRT found a valid path  *");
            $display("  *   from START (0,0,0) to GOAL (255,255,255).          *");
            $display("  *********************************************************");
        end
    endtask

endmodule
