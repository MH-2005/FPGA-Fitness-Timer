`timescale 1ns / 1ps

module TopModule_tb;

    // Clock and reset
    reg clk_40MHz;
    reg btn_reset, btn_start, btn_skip;

    // User inputs
    reg [2:0] W;
    reg [1:0] Cal;
    reg [1:0] MET;
    reg G;

    // Outputs
    wire [7:0] SEG_DATA;
    wire [4:0] SEG_SEL;
    wire buzzer;
    wire LCD_RS, LCD_E, LCD_RW;
    wire [7:0] LCD_D;

    // Test parameters
    parameter SIMULATION = 1'b1;  // Enable fast simulation

    // DUT instantiation
    TopModule #(
        .SIM_SPEEDUP(SIMULATION),
        .SEG_ACTIVE_HIGH(1'b1),
        .SEL_ACTIVE_HIGH(1'b1),
        .BTN_ACTIVE_LOW(1'b1)
    ) dut (
        .clk_40MHz(clk_40MHz),
        .btn_reset(btn_reset),
        .btn_start(btn_start), 
        .btn_skip(btn_skip),
        .W(W),
        .Cal(Cal),
        .MET(MET),
        .G(G),
        .SEG_DATA(SEG_DATA),
        .SEG_SEL(SEG_SEL),
        .buzzer(buzzer),
        .LCD_RS(LCD_RS),
        .LCD_E(LCD_E),
        .LCD_RW(LCD_RW),
        .LCD_D(LCD_D)
    );

    // Clock generation (40 MHz)
    initial begin
        clk_40MHz = 0;
        forever #12.5 clk_40MHz = ~clk_40MHz;  // 25ns period = 40MHz
    end

    // Test stimulus
    initial begin
        $display("=== TopModule Testbench Starting ===");
        
        // Initialize signals
        btn_reset = 1;  // Not pressed (assuming active low from parameter)
        btn_start = 1;  // Not pressed
        btn_skip = 1;   // Not pressed
        W = 3'b010;     // 70kg
        Cal = 2'b01;    // 100 calories
        MET = 2'b00;    // MET level 1
        G = 1'b0;       // Male

        // Test Vector 1: Basic Reset Test
        $display("\n--- Test 1: Reset Behavior ---");
        apply_reset();
        wait_cycles(1000);
        check_idle_state("After Reset");

        // Test Vector 2: Invalid Configuration (should not start)
        $display("\n--- Test 2: Invalid Configuration ---");
        W = 3'b000; Cal = 2'b00; MET = 2'b11; G = 1'b0;  // Should result in 0 exercises
        wait_cycles(100);
        press_button(btn_start, "START with invalid config");
        wait_cycles(500);
        check_idle_state("Should remain in IDLE");

        // Test Vector 3: Valid Configuration - Male, Light Workout
        $display("\n--- Test 3: Male Light Workout ---");
        W = 3'b010; Cal = 2'b01; MET = 2'b00; G = 1'b0;  // 70kg, 100cal, MET1, Male
        wait_cycles(100);
        press_button(btn_start, "START valid config");
        wait_cycles(500);
        check_workout_started("Male workout");

        // Test Vector 4: Skip Exercise
        $display("\n--- Test 4: Skip Exercise ---");
        wait_cycles(1000);
        press_button(btn_skip, "SKIP exercise");
        wait_cycles(500);
        check_exercise_incremented("After skip");

        // Test Vector 5: Let Work Timer Expire  
        $display("\n--- Test 5: Work Timer Expiration ---");
        wait_for_work_completion();
        check_rest_state("After work completion");

        // Test Vector 6: Skip During Rest
        $display("\n--- Test 6: Skip During Rest ---");
        wait_cycles(500);
        press_button(btn_skip, "SKIP during rest");
        wait_cycles(500);
        check_next_exercise("After rest skip");

        // Test Vector 7: Female Configuration
        $display("\n--- Test 7: Female Configuration ---");
        apply_reset();
        wait_cycles(500);
        W = 3'b011; Cal = 2'b10; MET = 2'b01; G = 1'b1;  // 80kg, 150cal, MET2, Female
        wait_cycles(100);
        press_button(btn_start, "Female workout start");
        wait_cycles(500);
        check_workout_started("Female workout");

        // Test Vector 8: High MET Level
        $display("\n--- Test 8: High MET Level ---");
        apply_reset();
        wait_cycles(500);
        W = 3'b100; Cal = 2'b11; MET = 2'b10; G = 1'b0;  // 90kg, 200cal, MET4, Male
        wait_cycles(100);
        press_button(btn_start, "High MET workout");
        wait_cycles(500);
        check_workout_started("High MET workout");

        // Test Vector 9: Maximum Configuration
        $display("\n--- Test 9: Maximum Configuration ---");
        apply_reset();
        wait_cycles(500);
        W = 3'b111; Cal = 2'b11; MET = 2'b00; G = 1'b1;  // 120kg, 200cal, MET1, Female
        wait_cycles(100);
        press_button(btn_start, "Maximum config");
        wait_cycles(500);
        check_workout_started("Maximum config");

        // Test Vector 10: Reset During Workout
        $display("\n--- Test 10: Reset During Workout ---");
        wait_cycles(1000);
        press_button(btn_reset, "RESET during workout");
        wait_cycles(1000);
        check_idle_state("After reset during workout");

        // Test completion
        $display("\n=== All Tests Completed ===");
        wait_cycles(2000);
        $finish;
    end

    // Helper tasks
    task apply_reset;
        begin
            $display("Applying reset...");
            btn_reset = 0;  // Assert reset (active low)
            wait_cycles(50);
            btn_reset = 1;  // Release reset
            wait_cycles(100);
        end
    endtask

    // CORRECTED: changed 'input string' to 'input [255:0]'
    task press_button;
        inout button_signal;
        input [255:0] button_name;
        begin
            $display("Pressing %s button", button_name);
            button_signal = 0;  // Press (active low)
            wait_cycles(10);
            button_signal = 1;  // Release
            wait_cycles(50);    // Debounce time
        end
    endtask

    task wait_cycles;
        input integer num_cycles;
        integer i;
        begin
            for (i = 0; i < num_cycles; i = i + 1) begin
                @(posedge clk_40MHz);
            end
        end
    endtask

    task check_idle_state;
        input [255:0] test_name;
        begin
            if (dut.workout_state == 2'b00) begin
                $display("✓ %s: System in IDLE state", test_name);
            end else begin
                $display("✗ %s: Expected IDLE state, got %b", test_name, dut.workout_state);
            end
        end
    endtask

    task check_workout_started;
        input [255:0] test_name;
        begin
            if (dut.workout_state == 2'b01 && dut.current_exercise_num > 0) begin
                $display("✓ %s: Workout started (Ex: %d, State: %b)", 
                         test_name, dut.current_exercise_num, dut.workout_state);
            end else begin
                $display("✗ %s: Workout not started (Ex: %d, State: %b)", 
                         test_name, dut.current_exercise_num, dut.workout_state);
            end
        end
    endtask

    task check_exercise_incremented;
        input [255:0] test_name;
        reg [8:0] prev_exercise;
        begin
            prev_exercise = dut.current_exercise_num;
            wait_cycles(10);
            if (dut.current_exercise_num > prev_exercise || dut.workout_state == 2'b00) begin
                $display("✓ %s: Exercise advanced (was %d, now %d)", 
                         test_name, prev_exercise, dut.current_exercise_num);
            end else begin
                $display("✗ %s: Exercise not advanced (still %d)", 
                         test_name, dut.current_exercise_num);
            end
        end
    endtask

    task check_rest_state;
        input [255:0] test_name;
        begin
            if (dut.workout_state == 2'b10) begin
                $display("✓ %s: Entered REST state", test_name);
            end else begin
                $display("✗ %s: Expected REST state, got %b", test_name, dut.workout_state);
            end
        end
    endtask

    task check_next_exercise;
        input [255:0] test_name;
        begin
            if (dut.workout_state == 2'b01) begin
                $display("✓ %s: Moved to next exercise (Ex: %d)", test_name, dut.current_exercise_num);
            end else begin
                $display("✗ %s: Not in next exercise state %b", test_name, dut.workout_state);
            end
        end
    endtask

    task wait_for_work_completion;
        integer timeout;
        begin
            $display("Waiting for work timer to complete...");
            timeout = 0;
            while (dut.workout_state == 2'b01 && timeout < 10000) begin
                wait_cycles(10);
                timeout = timeout + 1;
            end
            if (timeout >= 10000) begin
                $display("✗ Timeout waiting for work completion");
            end else begin
                $display("✓ Work phase completed");
            end
        end
    endtask

    // Monitor key signals
    initial begin
        $monitor("Time: %t | State: %b | Exercise: %d | Timer: %d | Buzzer: %b | 7Seg: %h %h", 
                 $time, dut.workout_state, dut.current_exercise_num, 
                 dut.countdown_seconds, buzzer, SEG_DATA, SEG_SEL);
    end

    // Generate VCD file for waveform viewing
    initial begin
        $dumpfile("topmodule_tb.vcd");
        $dumpvars(0, TopModule_tb);
    end

    // Test timeout
    initial begin
        #50000000;  // 50ms timeout
        $display("✗ TEST TIMEOUT");
        $finish;
    end

endmodule

