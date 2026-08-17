`define CYCLE_TIME 10.0

module PATTERN (
    clk,
    rst_n, 
    in_valid, 
    out_valid, 
    rand_num, 

    // --- Connections to Control & Read your Internal TREEMEM module ---
    check_ans_en,
    check_ans_addr,
    read_node_x,
    read_node_y,
    read_node_z,
    read_parent,
    max_tree_idx, // Monitors current tree size directly from hardware

    // --- Map Initialization Lines ---
    write_map_en,
    write_x,
    write_y,
    write_z,
    write_obstacle_val
);

    output logic clk, rst_n, in_valid;
    input  logic out_valid;
    output logic [23:0] rand_num;

    // Pattern outputs that drive the read ports of your TREEMEM module
    output logic        check_ans_en;
    output logic [9:0]  check_ans_addr;
    
    // Pattern inputs that catch the results coming back from TREEMEM
    input  logic [7:0]  read_node_x;
    input  logic [7:0]  read_node_y;
    input  logic [7:0]  read_node_z;
    input  logic [9:0]  read_parent;
    input  logic [9:0]  max_tree_idx;

    output logic write_map_en;
    output logic [7:0] write_x, write_y, write_z;
    output logic write_obstacle_val;

    real CYCLE = `CYCLE_TIME;
    always #(CYCLE/2.0) clk = ~clk;
    initial clk = 0;

    integer file_rand, file_map, file_gold;
    integer status;

    // Pattern & Latency Trackers
    integer patcount, latency;
    integer file_line_ptr; 
    integer total_latency;
    integer rand_color; 

    // Control trackers for verifying if a tree node was appended
    logic [9:0]  current_tree_size;
    logic [9:0]  prev_tree_size; 

    initial begin
        RESET_TASK;
        #(CYCLE*300);
        UPDATE_MAP;

        file_rand = $fopen("../PE_test/random.txt", "r");
        if (file_rand == 0) begin
            $display("ERROR: Open random.txt failed!");
            $finish;
        end

        patcount = 0;
        total_latency = 0;
        file_line_ptr = 1; // Row 0 is root; checks start at Row 1

        while (!$feof(file_rand)) begin
            status = $fscanf(file_rand, "%d\n", rand_num);
            if (status == 1) begin
                patcount = patcount + 1;
                prev_tree_size = max_tree_idx; // Read baseline tree size directly from hardware

                @(negedge clk);
                in_valid = 1'b1;
                @(negedge clk);
                in_valid = 1'b0; // Exactly 1 Cycle
                
                WAIT_OUT_VALID;
                
                #(CYCLE);
            end
        end
        $fclose(file_rand);

        DISPLAY_PASS_BANNER;
        $finish;
    end

    task RESET_TASK; begin
        rst_n    = 1'b1;
        in_valid = 1'b0;
        rand_num = 0;
        
        check_ans_en   = 1'b0;
        check_ans_addr = 10'd0;

        write_map_en       = 1'b0;
        write_x            = 8'd0;
        write_y            = 8'd0;
        write_z            = 8'd0;
        write_obstacle_val = 1'b0;
        
        force clk = 0;
        #(0.5*CYCLE); rst_n = 1'b0; 
        #(3.0*CYCLE); rst_n = 1'b1; 
        #(5.0*CYCLE); release clk;
    end endtask

    task UPDATE_MAP; begin
        integer r_x, r_y, r_z;
        integer status;
        
        file_map = $fopen("../PE_test/map.txt", "r");
        if (file_map == 0) begin
            $display("ERROR: Can't open map.txt from pattern!");
            $finish;
        end
        
        $display("------------- Initializing Map Memory From File -------------");
        
        while (!$feof(file_map)) begin
            status = $fscanf(file_map, "%d %d %d\n", r_x, r_y, r_z);
            if (status == 3) begin
                if (r_x < 256 && r_y < 256 && r_z < 256) begin
                    @(negedge clk);
                    write_map_en       = 1'b1;
                    write_x            = r_x[7:0];
                    write_y            = r_y[7:0];
                    write_z            = r_z[7:0];
                    write_obstacle_val = 1'b1;
                end
            end
        end
        
        @(negedge clk);
        write_map_en       = 1'b0;
        write_x            = 8'd0;
        write_y            = 8'd0;
        write_z            = 8'd0;
        write_obstacle_val = 1'b0;
        
        $fclose(file_map);
        $display("------------- Map Memory Load Phase Complete -------------");
    end endtask

    assign current_tree_size = max_tree_idx; // Continuously monitor tree size directly from hardware
    task WAIT_OUT_VALID; begin
        latency = 0; 
        while(out_valid !== 1'b1) begin
            @(negedge clk); 
            latency += 1; 
            if(latency >= 10000) begin
                $display ("----------------------------------------------------------------------------------------");
                $display ("                                   OUTPUT TIMEOUT FAIL!                                 ");
                $display ("                      Latency should not be longer than 10000 cycles                     ");
                $display ("----------------------------------------------------------------------------------------");
                #(10*CYCLE);
                $finish;
            end
        end
        
        total_latency     = total_latency + latency;
        rand_color        = 31 + (patcount % 7); 
        
        if (current_tree_size > prev_tree_size) begin
            // A node was successfully added! Query the internal memory to check its content
            CHECK_ANS_TASK;
            file_line_ptr = file_line_ptr + 1;
        end else begin
            $display ("\033[%0dmPassed Pattern NO. %4d (No node added due to obstacle collision) [Latency: %2d cycles]\033[m", rand_color, patcount, latency);
        end
    end endtask

    task CHECK_ANS_TASK; begin
        logic [7:0] g_x, g_y, g_z;
        logic [9:0] g_parent;
        integer match_status;
        logic [9:0] target_addr;
        
        // 1. Issue a Synchronous Memory Read Command to TREEMEM
        // Target index is the last node added: (current_tree_size - 1)
        target_addr = current_tree_size - 1;
        
        @(negedge clk);
        check_ans_en   = 1'b1;
        check_ans_addr = target_addr;
        
        // 2. Wait 1 Cycle for the synchronous memory block to register 
        // the address, sample its array matrix, and update output lines
        @(negedge clk);
        check_ans_en   = 1'b0; // Immediately turn off check pin to preserve low-power standards
        
        // 3. Parse the Golden Reference file up to the matching entry row
        file_gold = $fopen("../PE_test/tree.txt", "r");
        if (file_gold == 0) begin
            $display("ERROR: Cannot open golden file tree.txt!");
            $finish;
        end

        for (int i = 0; i <= file_line_ptr; i++) begin
            if ($feof(file_gold)) begin
                $display("ERROR: Golden tree.txt ends prematurely at line pointer index %d!", file_line_ptr);
                $fclose(file_gold);
                $finish;
            end
            match_status = $fscanf(file_gold, "%d %d %d %d\n", g_x, g_y, g_z, g_parent);
        end
        $fclose(file_gold);

        // 4. Perform direct pin-to-pin validation against the hardware memory output
        if (match_status == 4) begin
            // Coordinate Validation Check
            if ((read_node_x !== g_x) || (read_node_y !== g_y) || (read_node_z !== g_z)) begin
                $display ("-------------------------------------------------------------------");
                $display ("* PATTERN NO.%4d                        ", patcount);
                $display ("* WRONG HARDWARE MEMORY NODE DETECTED  ");
                $display ("  Checked Memory Address Index: %d", target_addr);
                $display ("  Golden tree.txt Row Match   : %d", file_line_ptr);
                $display ("  answer should be : (%3d, %3d, %3d, %4d)", g_x, g_y, g_z, g_parent);
                $display ("  your hardware is : (%3d, %3d, %3d, %4d)", read_node_x, read_node_y, read_node_z, read_parent);
                $display ("-------------------------------------------------------------------");
                #(10*CYCLE);
                $finish;
            end
            
            // Parent Pointer Validation Check
            if (read_parent !== g_parent) begin
                $display ("-------------------------------------------------------------------");
                $display ("* PATTERN NO.%4d                        ", patcount);
                $display ("* WRONG HARDWARE MEMORY PARENT DETECTED ");
                $display ("  Checked Memory Address Index: %d", target_addr);
                $display ("  Golden tree.txt Row Match   : %d", file_line_ptr);
                $display ("  answer should be : %4d , your hardware is : %4d", g_parent, read_parent);
                $display ("-------------------------------------------------------------------");
                #(10*CYCLE);
                $finish;
            end
            
            $display ("\033[%0dmPassed Pattern NO. %4d (Tree Size: %3d, Verified Hardware Row: %3d) [Latency: %2d cycles]\033[m", 
                      rand_color, patcount, current_tree_size, file_line_ptr, latency);
        end
    end endtask

    // Styled Green Congratulations Pass Character Art Task
    task DISPLAY_PASS_BANNER; begin
        $display("\033[37m                                                                                                                                          ");        
        $display("\033[37m                                                                                \033[32m      :BBQvi.                                              ");        
        $display("\033[37m                                                              .i7ssrvs7         \033[32m     BBBBBBBBQi                                           ");        
        $display("\033[37m                        .:r7rrrr:::.        .::::::...   .i7vr:.      .B:       \033[32m    :BBBP :7BBBB.                                         ");        
        $display("\033[37m                      .Kv.........:rrvYr7v7rr:.....:rrirJr.   .rgBBBBg  Bi      \033[32m    BBBB     BBBB                                         ");        
        $display("\033[37m                     7Q  :rubEPUri:.       ..:irrii:..    :bBBBBBBBBBBB  B      \033[32m   iBBBv     BBBB       vBr                               ");        
        $display("\033[37m                    7B  BBBBBBBBBBBBBBB::BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB :R     \033[32m   BBBBBKrirBBBB.     :BBBBBB:                            ");        
        $display("\033[37m                   Jd .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB: Bi    \033[32m  rBBBBBBBBBBBR.    .BBBM:BBB                             ");        
        $display("\033[37m                  uZ .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB .B    \033[32m  BBBB   .::.      EBBBi :BBU                             ");        
        $display("\033[37m                 7B .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  B    \033[32m MBBBr           vBBBu   BBB.                             ");        
        $display("\033[37m                .B  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB: JJ   \033[32m i7PB          iBBBBB.  iBBB                              ");        
        $display("\033[37m                B. BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  Lu             \033[32m  vBBBBPBBBBPBBB7       .7QBB5i                ");        
        $display("\033[37m               Y1 KBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBi XBBBBBBBi :B            \033[32m :RBBB.  .rBBBBB.      rBBBBBBBB7              ");        
        $display("\033[37m              :B .BBBBBBBBBBBBBsRBBBBBBBBBBBrQBBBBB. UBBBRrBBBBBBr 1BBBBBBBBB  B.          \033[32m    .       BBBB       BBBB  :BBBB             ");        
        $display("\033[37m              Bi BBBBBBBBBBBBBi :BBBBBBBBBBE .BBK.  .  .   QBBBBBBBBBBBBBBBBBB  Bi         \033[32m           rBBBr       BBBB    BBBU            ");        
        $display("\033[37m             .B .BBBBBBBBBBBBBBQBBBBBBBBBBBB       \033[38;2;242;172;172mBBv \033[37m.LBBBBBBBBBBBBBBBBBBBBBB. B7.:ii:   \033[32m           vBBB        .BBBB   :7i.            ");        
        $display("\033[37m            .B  PBBBBBBBBBBBBBBBBBBBBBBBBBBBBbYQB. \033[38;2;242;172;172mBB: \033[37mBBBBBBBBBBBBBBBBBBBBBBBBB  Jr:::rK7 \033[32m             .7  BBB7   iBBBg                  ");        
        $display("\033[37m           7M  PBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  \033[38;2;242;172;172mBB. \033[37mBBBBBBBBBBBBBBBBBBBBBBB..i   .   v1                  \033[32mdBBB.   5BBBr                 ");        
        $display("\033[37m          sZ .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  \033[38;2;242;172;172mBB. \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBBB iD2BBQL.                 \033[32m ZBBBr  EBBBv     YBBBBQi     ");        
        $display("\033[37m  .7YYUSIX5 .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  \033[38;2;242;172;172mBB. \033[37mBBBBBBBBBBBBBBBBBBBBBBBBY.:.      :B                 \033[32m  iBBBBBBBBD     BBBBBBBBB.   ");        
        $display("\033[37m LB.        ..BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB. \033[38;2;242;172;172mBB: \033[37mBBBBBBBBBBBBBBBBBBBBBBBBMBBB. BP17si                 \033[32m    :LBBBr      vBBBi  5BBB   ");        
        $display("\033[37m  KvJPBBB :BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB: \033[38;2;242;172;172mZB: \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBsiJr .i7ssr:                \033[32m          ...   :BBB:   BBBu  ");        
        $display("\033[37m i7ii:.   ::BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBj \033[38;2;242;172;172muBi \033[37mQBBBBBBBBBBBBBBBBBBBBBBBBi.ir      iB                \033[32m         .BBBi   BBBB   iMBu  ");        
        $display("\033[37mDB    .  vBdBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBg \033[38;2;242;172;172m7Bi \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBBBBB rBrXPv.                \033[32m          BBBX   :BBBr        ");        
        $display("\033[37m :vQBBB. BQBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBQ \033[38;2;242;172;172miB: \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBBBBB .L:ii::irrrrrrrr7jIr   \033[32m          .BBBv  :BBBQ        ");        
        $display("\033[37m :7:.   .. 5BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  \033[38;2;242;172;172mBr \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBBBB:            ..... ..YB. \033[32m           .BBBBBBBBB:        ");        
        $display("\033[37mBU  .:. BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  \033[38;2;242;172;172mB7 \033[37mgBBBBBBBBBBBBBBBBBBBBBBBBBB. gBBBBBBBBBBBBBBBBBB. BL \033[32m             rBBBBB1.         ");        
        $display("\033[37m rY7iB: BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB: \033[38;2;242;172;172mB7 \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBB. QBBBBBBBBBBBBBBBBBi  v5                                ");        
        $display("\033[37m     us EBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB \033[38;2;242;172;172mIr \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBgu7i.:BBBBBBBr Bu                                 ");        
        $display("\033[37m      B  7BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB.\033[38;2;242;172;172m:i \033[37mBBBBBBBBBBBBBBBBBBBBBBBBBBBv:.  .. :::  .rr    rB                                  ");        
        $display("\033[37m      us  .BBBBBBBBBBBBBQLXBBBBBBBBBBBBBBBBBBBBBBBBq  .BBBBBBBBBBBBBBBBBBBBBBBBBv  :iJ7vri:::1Jr..isJYr                                   ");        
        $display("\033[37m      B  BBBBBBB  MBBBM      qBBBBBBBBBBBBBBBBBBBBBB: BBBBBBBBBBBBBBBBBBBBBBBBBB  B:           iir:                                       ");        
        $display("\033[37m     iB iBBBBBBBL       BBBP. :BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  B.                                                       ");        
        $display("\033[37m     P: BBBBBBBBBBB5v7gBBBBBB  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB: Br                                                        ");        
        $display("\033[37m     B  BBBs 7BBBBBBBBBBBBBB7 :BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB .B                                                         ");        
        $display("\033[37m    .B :BBBB.  EBBBBBQBBBBBJ .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB. B.                                                         ");        
        $display("\033[37m    ij qBBBBBg          ..  .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB .B                                                          ");        
        $display("\033[37m    UY QBBBBBBBBSUSPDQL...iBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBK EL                                                          ");        
        $display("\033[37m    B7 BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB: B:                                                          ");        
        $display("\033[37m    B  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBYrBB vBBBBBBBBBBBBBBBBBBBBBBBB. Ls                                                          ");        
        $display("\033[37m    B  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBi_  /UBBBBBBBBBBBBBBBBBBBBBBBBB. :B:                                                        ");        
        $display("\033[37m   rM .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  ..IBBBBBBBBBBBBBBBBQBBBBBBBBBB  B                                                        ");        
        $display("\033[37m   B  BBBBBBBBBdZBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBPBBBBBBBBBBBBEji:..     sBBBBBBBr Br                                                       ");        
        $display("\033[37m  7B 7BBBBBBBr     .:vXQBBBBBBBBBBBBBBBBBBBBBBBBBQqui::..  ...i:i7777vi  BBBBBBr Bi                                                       ");        
        $display("\033[37m  Ki BBBBBBB  rY7vr:i....  .............:.....  ...:rii7vrr7r:..      7B  BBBBB  Bi                                                       ");        
        $display("\033[37m  B. BBBBBB  B:    .::ir77rrYLvvriiiiiiirvvY7rr77ri:..                 bU  iQBB:..rI                                                      ");        
        $display("\033[37m.S: 7BBBBP  B.                                                          vI7.  .:.  B.                                                     ");        
        $display("\033[37mB: ir:.   :B.                                                             :rvsUjUgU.                                                      ");        
        $display("\033[37mrMvrrirJKur                                                                                                                               \033[m");
        report_pass_message;
    end endtask

    task report_pass_message; begin
        $display ("----------------------------------------------------------------------------------------------------------------------");
        $display ("                                                  Congratulations!                                                    ");
        $display ("                                           You have passed all patterns!                                              ");
        $display ("                                          Total latency is \033[0;35m%7d\033[0m cycles                                ", total_latency);
        $display ("                                              Total time is \033[0;35m%7d\033[0m ms                                   ", total_latency * CYCLE / 1000000.0);
        $display ("----------------------------------------------------------------------------------------------------------------------");
    end endtask

endmodule