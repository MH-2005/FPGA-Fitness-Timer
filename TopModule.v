`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    00:35:49 08/31/2025 
// Design Name: 
// Module Name:    EX_p 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 1.03 - Fixed non-synthesizable modulo in LCDTextGenerator
// Additional Comments: 
//
//  Logic Lab Project (Spring 1404) - REFACTORED & CORRECTED VERSION
//  TopModule - Fitness Workout Timer System
//  - Implemented 5-state FSM as per design documents.
//  - Corrected buzzer logic to trigger only from FSM events.
//  - Implemented different frequencies for normal and final beeps.
// ==========================================================


module TopModule #(
    // Simulation Parameters
    parameter SIM_SPEEDUP       = 1'b0,
    
    // Display Parameters   
    parameter SEG_ACTIVE_HIGH   = 1'b1,
    parameter SEL_ACTIVE_HIGH   = 1'b1,
    parameter BTN_ACTIVE_LOW    = 1'b1,
    
    // Timing Parameters
    parameter WORK_TIME_SEC     = 8'd45,
    parameter REST_TIME_SEC     = 8'd15,
    parameter DEBOUNCE_MS       = 5,
    parameter RESET_HOLD_MS     = 100,
    parameter POWERUP_DELAY_MS  = 80
)(
    // Clock & Reset
    input  clk_40MHz,
    input  btn_reset, 
    input  btn_start, 
    input  btn_skip,    // Active-Low (board pull-ups)
    
    // User Inputs
    input  [2:0] W,       // Weight selection
    input  [1:0] Cal,     // Calorie target 
    input  [1:0] MET,     // MET level
    input        G,       // Gender
    
    // Display Outputs
    output [7:0] SEG_DATA,
    output [4:0] SEG_SEL, 
    output       buzzer,
    
    // LCD 16x2 Interface (8-bit: DB7..DB0)
    output       LCD_RS,
    output       LCD_E,
    output       LCD_RW,
    output [7:0] LCD_D
);


    // Workout state definitions for display mapping
    localparam [2:0] FSM_STATE_IDLE              = 3'b000;
    localparam [2:0] FSM_STATE_EXERCISING        = 3'b001;
    localparam [2:0] FSM_STATE_RESTING           = 3'b010;
    localparam [2:0] FSM_STATE_BEEP_AFTER_REST   = 3'b011;
    localparam [2:0] FSM_STATE_FINAL_BEEP        = 3'b100;

    // ============ CLOCK GENERATION ============
    localparam integer FREQ_40MHZ = 40_000_000;
    localparam integer DIV_1HZ_TOGGLE = SIM_SPEEDUP ? 20000 : (FREQ_40MHZ / 2); // 1 Hz toggle
    localparam integer DIV_1KHZ_TOGGLE = SIM_SPEEDUP ? 2000 : (FREQ_40MHZ / 2000); // 1 kHz toggle   
    wire clk_1Hz, clk_1kHz;
    wire system_reset;

    ClockDivToggle #(.TOGGLE_COUNT(DIV_1HZ_TOGGLE))
      clock_div_1hz (.clk(clk_40MHz), .rst(system_reset), .clk_out(clk_1Hz));

    ClockDivToggle #(.TOGGLE_COUNT(DIV_1KHZ_TOGGLE))
      clock_div_1khz (.clk(clk_40MHz), .rst(system_reset), .clk_out(clk_1kHz));

    // Generate 1Hz tick pulse in 40MHz domain
    reg clk_1Hz_prev;   
    always @(posedge clk_40MHz) clk_1Hz_prev <= clk_1Hz;
    wire tick_1Hz = clk_1Hz & ~clk_1Hz_prev;

    // ============ BUTTON DEBOUNCING ============
    localparam integer DEBOUNCE_CYCLES = SIM_SPEEDUP ? 2000 : (FREQ_40MHZ / 1000 * DEBOUNCE_MS);

    wire start_btn_pressed, skip_btn_pressed, reset_btn_pressed;
    
    ButtonConditioner #(.ACTIVE_LOW(BTN_ACTIVE_LOW), .STABLE_COUNT(DEBOUNCE_CYCLES))
      btn_start_cond (.clk(clk_40MHz), .rst(system_reset), .btn_in(btn_start), .press_pulse(start_btn_pressed));
      
    ButtonConditioner #(.ACTIVE_LOW(BTN_ACTIVE_LOW), .STABLE_COUNT(DEBOUNCE_CYCLES))
      btn_skip_cond  (.clk(clk_40MHz), .rst(system_reset), .btn_in(btn_skip),  .press_pulse(skip_btn_pressed));
      
    ButtonConditioner #(.ACTIVE_LOW(BTN_ACTIVE_LOW), .STABLE_COUNT(DEBOUNCE_CYCLES))
      btn_reset_cond (.clk(clk_40MHz), .rst(system_reset), .btn_in(btn_reset), .press_pulse(reset_btn_pressed));

    // ============ RESET GENERATION ============
    localparam integer POR_CYCLES = SIM_SPEEDUP ? 32'd40000 : (FREQ_40MHZ / 1000 * POWERUP_DELAY_MS);
    localparam integer RST_HOLD_CYCLES = SIM_SPEEDUP ? 32'd400000 : (FREQ_40MHZ / 1000 * RESET_HOLD_MS);
    
    wire power_on_reset;
    PowerOnReset #(.CYCLES(POR_CYCLES)) 
      por_gen (.clk(clk_40MHz), .rst(power_on_reset));

    ResetController #(.HOLD_CYCLES(RST_HOLD_CYCLES))
      reset_ctrl (.clk(clk_40MHz), .por(power_on_reset), .trigger(reset_btn_pressed), .rst_out(system_reset));

    // ============ EXERCISE CALCULATION ============
    wire [8:0] total_exercises_preview;
    ExerciseCalculator calc_unit (
        .weight_sel(W), 
        .calorie_sel(Cal), 
        .gender(G), 
        .met_sel(MET), 
        .total_exercises(total_exercises_preview)
    );

    // Only start if calculated exercises > 0
    wire start_workout_cmd = start_btn_pressed & (total_exercises_preview != 9'd0);

    // Latch user inputs when workout starts
    reg [2:0] weight_latched;    
    reg [1:0] calorie_latched, met_latched; 
    reg gender_latched; 
    reg [8:0] total_exercises_latched;
    
    always @(posedge clk_40MHz or posedge system_reset) begin
        if (system_reset) begin 
            weight_latched <= 3'd0; 
            calorie_latched <= 2'd0; 
            met_latched <= 2'd0; 
            gender_latched <= 1'b0; 
            total_exercises_latched <= 9'd0; 
        end
        else if (start_btn_pressed) begin // Latch on any start press, cmd is for FSM
            weight_latched <= W; 
            calorie_latched <= Cal; 
            met_latched <= MET; 
            gender_latched <= G; 
            total_exercises_latched <= total_exercises_preview; 
        end
    end

    // ============ WORKOUT STATE MACHINE (5-STATE VERSION) ============
    wire [8:0] current_exercise_num;
    wire [7:0] countdown_seconds;
    wire beep_after_rest_pulse, workout_finished_pulse;
    wire [2:0] workout_state_3bit;
    
    WorkoutStateMachine #(
        .WORK_DURATION(WORK_TIME_SEC),
        .REST_DURATION(REST_TIME_SEC)
    ) fsm_unit (
        .reset(system_reset), 
        .skip_button(skip_btn_pressed), 
        .start_button(start_btn_pressed), // Use raw button press here
        .clk(clk_40MHz), 
        .tick_1Hz(tick_1Hz),
        .total_exercises(total_exercises_latched),
        .current_exercise(current_exercise_num), 
        .countdown_timer(countdown_seconds),
        .beep_after_rest(beep_after_rest_pulse),
        .workout_complete(workout_finished_pulse), 
        .state(workout_state_3bit)
    );

    // ============ 1KHZ DOMAIN SYNCHRONIZATION ============
    reg system_reset_sync1, system_reset_sync2;
    always @(posedge clk_1kHz) begin 
        system_reset_sync1 <= system_reset; 
        system_reset_sync2 <= system_reset_sync1; 
    end
    wire reset_1khz_domain = system_reset_sync2;

    // Synchronize display data to 1kHz domain
    reg [7:0] seconds_display, exercise_display; 
    reg idle_state_display;
    reg [1:0] workout_state_display; // 2-bit state for LCD
    
    always @(posedge clk_1kHz) begin
        seconds_display <= countdown_seconds;
        exercise_display <= (current_exercise_num > 9'd99) ? 8'd99 : current_exercise_num[7:0];
        
        // Map the 5 FSM states to the 3 relevant display states (Idle, Work, Rest)
        case (workout_state_3bit)
            FSM_STATE_IDLE, FSM_STATE_FINAL_BEEP: begin
                 idle_state_display <= 1'b1;
                 workout_state_display <= 2'b00; // IDLE display mode
            end
            FSM_STATE_EXERCISING, FSM_STATE_BEEP_AFTER_REST: begin
                 idle_state_display <= 1'b0;
                 workout_state_display <= 2'b01; // WORK display mode
            end
            FSM_STATE_RESTING: begin
                 idle_state_display <= 1'b0;
                 workout_state_display <= 2'b10; // REST display mode
            end
            default: begin
                 idle_state_display <= 1'b1;
                 workout_state_display <= 2'b00;
            end
        endcase
    end

    // ============ 7-SEGMENT DISPLAY ============
    wire [7:0] exercise_ones_seg, exercise_tens_seg, timer_ones_seg, timer_tens_seg;
    BCDToSevenSegment exercise_decoder (.bcd_value(exercise_display), 
                                         .ones_segments(exercise_ones_seg), 
                                         .tens_segments(exercise_tens_seg));
    BCDToSevenSegment timer_decoder (.bcd_value(seconds_display), 
                                       .ones_segments(timer_ones_seg), 
                                       .tens_segments(timer_tens_seg));

    localparam [7:0] SEG_UNDERSCORE = 8'b00001000;
    wire [7:0] digit0_pattern = idle_state_display ? SEG_UNDERSCORE : timer_ones_seg;
    wire [7:0] digit1_pattern = idle_state_display ? SEG_UNDERSCORE : timer_tens_seg;
    wire [7:0] digit2_pattern = idle_state_display ? SEG_UNDERSCORE : exercise_ones_seg;
    wire [7:0] digit3_pattern = idle_state_display ? SEG_UNDERSCORE : exercise_tens_seg;
    
    SevenSegmentScanner4Digit #(
        .SEG_ACTIVE_HIGH(SEG_ACTIVE_HIGH),
        .SEL_ACTIVE_HIGH(SEL_ACTIVE_HIGH)
    ) seg_scanner (
        .clk_scan(clk_1kHz), 
        .rst(reset_1khz_domain), 
        .blank_all(1'b0), 
        .digit0(digit0_pattern), 
        .digit1(digit1_pattern), 
        .digit2(digit2_pattern), 
        .digit3(digit3_pattern),
        .segment_outputs(SEG_DATA), 
        .select_outputs(SEG_SEL)
    );

    // ============ LCD DISPLAY ============
    wire [127:0] lcd_line1_text, lcd_line2_text;
    
    LCDTextGenerator lcd_text_gen (
        .clk(clk_1kHz),
        .rst(reset_1khz_domain),
        .idle_mode(idle_state_display),
        .weight_sel(W),
        .calorie_sel(Cal), 
        .met_sel(MET),
        .gender(G),
        .exercises_preview(total_exercises_preview),
        .weight_latched(weight_latched),
        .gender_latched(gender_latched),
        .total_exercises(total_exercises_latched),
        .current_exercise(current_exercise_num),
        .countdown_time(countdown_seconds),
        .workout_state(workout_state_display),
        .line1_text(lcd_line1_text),
        .line2_text(lcd_line2_text)
    );

    LCD1602Controller lcd_controller (
        .clk(clk_1kHz), 
        .rst(reset_1khz_domain),
        .line1_data(lcd_line1_text), 
        .line2_data(lcd_line2_text),
        .lcd_rs(LCD_RS), 
        .lcd_e(LCD_E), 
        .lcd_rw(LCD_RW), 
        .lcd_data(LCD_D)
    );

    // ============ AUDIO FEEDBACK - CORRECTED ============
    // CORRECTED: Triggers now come ONLY from the FSM state transitions.
    // Buttons no longer directly trigger the buzzer.
    wire short_beep_trigger = beep_after_rest_pulse;
    wire long_beep_trigger  = workout_finished_pulse;
                               
    BuzzerController buzzer_ctrl (
        .clk(clk_40MHz), 
        .rst(system_reset),
        .short_beep_trigger(short_beep_trigger), 
        .long_beep_trigger(long_beep_trigger),
        .buzzer_output(buzzer)
    );

endmodule

// ============================================================================
//                               SUPPORTING MODULES
// ============================================================================


// ===================== Exercise Calculator ==================
module ExerciseCalculator(
    input [2:0] weight_sel,
    input [1:0] calorie_sel, 
    input gender,
    input [1:0] met_sel,
    output [8:0] total_exercises
);
    
    reg [8:0] base_exercises;
    always @(*) begin
        case ({weight_sel, calorie_sel})
            5'b00000: base_exercises = 9'd60;   5'b00001: base_exercises = 9'd120;  
            5'b00010: base_exercises = 9'd180;  5'b00011: base_exercises = 9'd240;
            5'b00100: base_exercises = 9'd50;   5'b00101: base_exercises = 9'd100;  
            5'b00110: base_exercises = 9'd150;  5'b00111: base_exercises = 9'd200;
            5'b01000: base_exercises = 9'd42;   5'b01001: base_exercises = 9'd85;   
            5'b01010: base_exercises = 9'd128;  5'b01011: base_exercises = 9'd171;
            5'b01100: base_exercises = 9'd37;   5'b01101: base_exercises = 9'd75;   
            5'b01110: base_exercises = 9'd112;  5'b01111: base_exercises = 9'd150;
            5'b10000: base_exercises = 9'd33;   5'b10001: base_exercises = 9'd66;   
            5'b10010: base_exercises = 9'd100;  5'b10011: base_exercises = 9'd133;
            5'b10100: base_exercises = 9'd30;   5'b10101: base_exercises = 9'd60;   
            5'b10110: base_exercises = 9'd90;   5'b10111: base_exercises = 9'd120;
            5'b11000: base_exercises = 9'd27;   5'b11001: base_exercises = 9'd54;   
            5'b11010: base_exercises = 9'd81;   5'b11011: base_exercises = 9'd109;
            5'b11100: base_exercises = 9'd25;   5'b11101: base_exercises = 9'd50;   
            5'b11110: base_exercises = 9'd75;   5'b11111: base_exercises = 9'd100;
            default:  base_exercises = 9'd50;
        endcase
    end
    
    wire [9:0] gender_adjusted_wide = gender ? (base_exercises + (base_exercises >> 3)) : base_exercises;
    wire [8:0] gender_adjusted = (gender_adjusted_wide > 9'd511) ? 9'd511 : gender_adjusted_wide[8:0];
    
    assign total_exercises = (met_sel == 2'b00) ? gender_adjusted :
                             (met_sel == 2'b01) ? (gender_adjusted >> 1) :
                             (met_sel == 2'b10) ? (gender_adjusted >> 2) :
                                                  (gender_adjusted >> 3);

endmodule


// ===================== Workout State Machine - 5-STATE FSM REVISED ==================
module WorkoutStateMachine #(
    parameter [7:0] WORK_DURATION = 8'd45,
    parameter [7:0] REST_DURATION = 8'd15
)(
    input reset, 
    input skip_button, 
    input start_button, 
    input clk, 
    input tick_1Hz,
    input  [8:0] total_exercises,

    output reg [8:0] current_exercise, 
    output reg [7:0] countdown_timer,
    output reg beep_after_rest,   // Pulse signal for short beep
    output reg workout_complete,  // Pulse signal for long beep
    output [2:0] state
);

    // 5-State FSM definition as per project documentation
    localparam [2:0] STATE_IDLE              = 3'b000;
    localparam [2:0] STATE_EXERCISING        = 3'b001;
    localparam [2:0] STATE_RESTING           = 3'b010;
    localparam [2:0] STATE_BEEP_AFTER_REST   = 3'b011;
    localparam [2:0] STATE_FINAL_BEEP        = 3'b100;

    reg [2:0] current_state, next_state;
    reg [7:0] work_counter; 
    reg [7:0] rest_counter;
    
    // Sequential block for state transitions (runs at main clock speed)
    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= STATE_IDLE;
        else
            current_state <= next_state;
    end
    
    // Sequential block for counters (updates based on state and 1Hz tick)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            work_counter <= WORK_DURATION;
            rest_counter <= REST_DURATION;
            current_exercise <= 9'd1;
        end else begin
            // Decrement counters when in the correct state and a 1-second tick occurs
            case(current_state)
                STATE_EXERCISING: begin
                    if (tick_1Hz && work_counter > 0)
                        work_counter <= work_counter - 1;
                end
                STATE_RESTING: begin
                    if (tick_1Hz && rest_counter > 0)
                        rest_counter <= rest_counter - 1;
                end
            endcase

            // Logic to reset counters and increment exercise count upon state changes
            if (next_state != current_state) begin
                case (next_state)
                    STATE_IDLE: begin
                        current_exercise <= 9'd1;
                        work_counter <= WORK_DURATION;
                        rest_counter <= REST_DURATION;
                    end
                    STATE_EXERCISING: begin
                        work_counter <= WORK_DURATION;
                    end
                    STATE_RESTING: begin
                        rest_counter <= REST_DURATION;
                    end
                    STATE_BEEP_AFTER_REST: begin
                        if (current_exercise < total_exercises)
                           current_exercise <= current_exercise + 1;
                    end
                endcase
            end
        end
    end

    // Combinational block for next state logic and outputs
    always @(*) begin
        // Default assignments for combinational outputs
        next_state = current_state;
        beep_after_rest = 1'b0;
        workout_complete = 1'b0;
        countdown_timer = 8'd0;

        case (current_state)
            STATE_IDLE: begin
                countdown_timer = (total_exercises > 9'd99) ? 8'd99 : total_exercises[7:0];
                if (start_button) begin
                    if (total_exercises > 0)
                        next_state = STATE_EXERCISING;
                    else
                        next_state = STATE_FINAL_BEEP;
                end
            end

            STATE_EXERCISING: begin
                countdown_timer = work_counter;
                if (skip_button || work_counter == 0) begin
                    if (current_exercise >= total_exercises)
                        next_state = STATE_FINAL_BEEP;
                    else
                        next_state = STATE_RESTING;
                end
            end

            STATE_RESTING: begin
                countdown_timer = rest_counter;
                 if (skip_button || rest_counter == 0)
                    next_state = STATE_BEEP_AFTER_REST;
            end
            
            STATE_BEEP_AFTER_REST: begin
                beep_after_rest = 1'b1; // Generate a single main-clock-cycle pulse
                // This is a transitional state, it moves on immediately in the next clock cycle
                next_state = STATE_EXERCISING;
            end

            STATE_FINAL_BEEP: begin
                workout_complete = 1'b1; // Generate a single main-clock-cycle pulse
                // This state holds until start or reset. A new start press transitions to IDLE.
                if (start_button)
                    next_state = STATE_IDLE;
            end
            
            default: next_state = STATE_IDLE;
        endcase
    end
    
    assign state = current_state; // Output the current state for display/debug

endmodule


// ===================== LCD Text Generator - OPTIMIZED & FIXED ==================
module LCDTextGenerator (
    input clk,
    input rst,
    input idle_mode,
    input [2:0] weight_sel,
    input [1:0] calorie_sel, 
    input [1:0] met_sel,
    input gender,
    input [8:0] exercises_preview,
    input [2:0] weight_latched,
    input gender_latched,
    input [8:0] total_exercises,
    input [8:0] current_exercise,
    input [7:0] countdown_time,
    input [1:0] workout_state, // 2-bit state for display (Idle/Work/Rest)
    output [127:0] line1_text,
    output [127:0] line2_text
);
    
    localparam [1:0] WORKOUT_STATE_WORK = 2'b01;

    function [15:0] weight_to_ascii;
        input [2:0] sel;
        case (sel)
            3'b000: weight_to_ascii = "50";   3'b001: weight_to_ascii = "60";
            3'b010: weight_to_ascii = "70";   3'b011: weight_to_ascii = "80";
            3'b100: weight_to_ascii = "90";   3'b101: weight_to_ascii = "100";
            3'b110: weight_to_ascii = "110";  3'b111: weight_to_ascii = "120";
            default: weight_to_ascii = "70";
        endcase
    endfunction

    function [23:0] calorie_to_ascii;
        input [1:0] sel;
        case (sel)
            2'b00: calorie_to_ascii = " 50";   2'b01: calorie_to_ascii = "100";
            2'b10: calorie_to_ascii = "150";   2'b11: calorie_to_ascii = "200";
            default: calorie_to_ascii = "100";
        endcase
    endfunction

    function [7:0] met_to_ascii;
        input [1:0] sel;
        case (sel) 
            2'b00: met_to_ascii = "1";  2'b01: met_to_ascii = "2"; 
            2'b10: met_to_ascii = "4";  2'b11: met_to_ascii = "8";
            default: met_to_ascii = "1";
        endcase
    endfunction

    function [11:0] bin_to_bcd3;
        input [8:0] bin_in;
        reg [3:0] d2, d1, d0;
        integer i;
        reg [15:0] temp;
    begin
        temp = bin_in;
        d2=0; d1=0; d0=0;
        for (i=0; i<9; i=i+1) begin
            if (d2 >= 5) d2 = d2 + 3;
            if (d1 >= 5) d1 = d1 + 3;
            d2 = {d2[2:0], d1[3]};
            d1 = {d1[2:0], d0[3]};
            d0 = {d0[2:0], temp[8-i]};
        end
        bin_to_bcd3 = {d2, d1, d0};
    end
    endfunction
    
    function [23:0] format_3digit_bcd;
        input [8:0] value;
        reg [11:0] bcd_digits;
        reg [3:0] hundreds, tens, ones;
    begin
        bcd_digits = bin_to_bcd3(value);
        hundreds = bcd_digits[11:8];
        tens = bcd_digits[7:4];
        ones = bcd_digits[3:0];
        format_3digit_bcd = {8'd48 + hundreds, 8'd48 + tens, 8'd48 + ones};
    end
    endfunction

    function [15:0] format_2digit_bcd;
        input [7:0] value;
        reg [3:0] tens, ones;
    begin
        if (value >= 90) begin tens = 9; ones = value - 90; end
        else if (value >= 80) begin tens = 8; ones = value - 80; end
        else if (value >= 70) begin tens = 7; ones = value - 70; end
        else if (value >= 60) begin tens = 6; ones = value - 60; end
        else if (value >= 50) begin tens = 5; ones = value - 50; end
        else if (value >= 40) begin tens = 4; ones = value - 40; end
        else if (value >= 30) begin tens = 3; ones = value - 30; end
        else if (value >= 20) begin tens = 2; ones = value - 20; end
        else if (value >= 10) begin tens = 1; ones = value - 10; end
        else begin tens = 0; ones = value; end
        format_2digit_bcd = {8'd48 + tens, 8'd48 + ones};
    end
    endfunction

    function [127:0] get_exercise_name;
        input [3:0] index;
        case (index)
            4'd0: get_exercise_name = "JUMP JACKS      ";
            4'd1: get_exercise_name = "SIT UPS         ";
            4'd2: get_exercise_name = "PUSH UPS        ";
            4'd3: get_exercise_name = "SQUATS          ";
            4'd4: get_exercise_name = "PLANK           ";
            4'd5: get_exercise_name = "LUNGES          ";
            4'd6: get_exercise_name = "TRICEPS DIPS    ";
            4'd7: get_exercise_name = "WALL SIT        ";
            4'd8: get_exercise_name = "HIGH KNEES      ";
            4'd9: get_exercise_name = "BURPEES         ";
            default: get_exercise_name = "EXERCISE        ";
        endcase
    endfunction

    reg [127:0] line1_reg, line2_reg;
    assign line1_text = line1_reg;
    assign line2_text = line2_reg;

    wire [15:0] weight_ascii_val = weight_to_ascii(idle_mode ? weight_sel : weight_latched);
    wire [23:0] calorie_ascii_val = calorie_to_ascii(calorie_sel);
    wire [7:0]  met_ascii_val = met_to_ascii(met_sel);
    wire [23:0] exercises_ascii = format_3digit_bcd(idle_mode ? exercises_preview : total_exercises);
    wire [15:0] current_ex_ascii = format_2digit_bcd(current_exercise[7:0]);
    wire [15:0] timer_ascii = format_2digit_bcd(countdown_time);
    
    // CORRECTED: Replaced non-synthesizable modulo operator
    wire [8:0] ex_minus_1 = (current_exercise == 0) ? 9'd0 : (current_exercise - 1);
    wire [11:0] ex_bcd = bin_to_bcd3(ex_minus_1);
    wire [3:0] name_index = ex_bcd[3:0]; // The 'ones' digit is the result of modulo 10
    
    wire [127:0] current_exercise_name = get_exercise_name(name_index);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            line1_reg <= "FITNESS TIMER   ";
            line2_reg <= "SET PARAMETERS  ";
        end else begin
            if (idle_mode) begin
                line1_reg <= {"G:", (gender ? "F" : "M"), " W", weight_ascii_val, " M", met_ascii_val, " C", calorie_ascii_val};
                line2_reg <= {"T=", exercises_ascii, " READY         "};
            end else begin
                line1_reg <= current_exercise_name;
                line2_reg <= {"E", current_ex_ascii[7:0], "/", exercises_ascii[15:8], " ", 
                             (workout_state == WORKOUT_STATE_WORK) ? "W" : "R", ":", timer_ascii, " G:", 
                             (gender_latched ? "F" : "M")};
            end
        end
    end
endmodule


// ===================== BCD to 7-Segment Converter (FIXED) ==================
module BCDToSevenSegment (
    input  [7:0] bcd_value,
    output reg [7:0] ones_segments,
    output reg [7:0] tens_segments
);
    
    reg [3:0] tens_digit;
    reg [3:0] ones_digit;

    // CORRECTED: Replaced non-synthesizable operators with synthesizable logic
    always @(*) begin
        // Default assignment
        tens_digit = 0;
        ones_digit = bcd_value; // Assume ones digit is the value itself initially

        if (bcd_value >= 90) begin tens_digit = 9; ones_digit = bcd_value - 90; end
        else if (bcd_value >= 80) begin tens_digit = 8; ones_digit = bcd_value - 80; end
        else if (bcd_value >= 70) begin tens_digit = 7; ones_digit = bcd_value - 70; end
        else if (bcd_value >= 60) begin tens_digit = 6; ones_digit = bcd_value - 60; end
        else if (bcd_value >= 50) begin tens_digit = 5; ones_digit = bcd_value - 50; end
        else if (bcd_value >= 40) begin tens_digit = 4; ones_digit = bcd_value - 40; end
        else if (bcd_value >= 30) begin tens_digit = 3; ones_digit = bcd_value - 30; end
        else if (bcd_value >= 20) begin tens_digit = 2; ones_digit = bcd_value - 20; end
        else if (bcd_value >= 10) begin tens_digit = 1; ones_digit = bcd_value - 10; end
    end

    always @(*) begin
        case (ones_digit)
            4'd0: ones_segments = 8'b00111111;
            4'd1: ones_segments = 8'b00000110;
            4'd2: ones_segments = 8'b01011011;
            4'd3: ones_segments = 8'b01001111;
            4'd4: ones_segments = 8'b01100110;
            4'd5: ones_segments = 8'b01101101;
            4'd6: ones_segments = 8'b01111101;
            4'd7: ones_segments = 8'b00000111;
            4'd8: ones_segments = 8'b01111111;
            4'd9: ones_segments = 8'b01101111;
            default: ones_segments = 8'b00000000;
        endcase

        case (tens_digit)
            4'd0: tens_segments = 8'b00111111;
            4'd1: tens_segments = 8'b00000110;
            4'd2: tens_segments = 8'b01011011;
            4'd3: tens_segments = 8'b01001111;
            4'd4: tens_segments = 8'b01100110;
            4'd5: tens_segments = 8'b01101101;
            4'd6: tens_segments = 8'b01111101;
            4'd7: tens_segments = 8'b00000111;
            4'd8: tens_segments = 8'b01111111;
            4'd9: tens_segments = 8'b01101111;
            default: tens_segments = 8'b00000000;
        endcase
    end
endmodule


// ===================== 7-Segment Scanner - SIMPLIFIED ==================
module SevenSegmentScanner4Digit #(
    parameter SEG_ACTIVE_HIGH    = 1'b1,
    parameter SEL_ACTIVE_HIGH    = 1'b1
)(
    input clk_scan, 
    input rst, 
    input blank_all, 
    input  [7:0] digit0, 
    input  [7:0] digit1, 
    input  [7:0] digit2, 
    input  [7:0] digit3,
    output [7:0] segment_outputs, 
    output [4:0] select_outputs
);
    
    reg [1:0] scan_index; 
    
    always @(posedge clk_scan or posedge rst) begin
        if (rst) 
            scan_index <= 2'd0; 
        else
            scan_index <= scan_index + 2'd1;
    end
    
    reg [7:0] current_segments; 
    reg [4:0] current_selects;
    always @(*) begin
        case (scan_index)
            2'd0: begin current_segments = digit0; current_selects = 5'b00001; end
            2'd1: begin current_segments = digit1; current_selects = 5'b00010; end  
            2'd2: begin current_segments = digit2; current_selects = 5'b00100; end
            2'd3: begin current_segments = digit3; current_selects = 5'b01000; end
            default: begin current_segments = 8'h00; current_selects = 5'b00000; end
        endcase
    end

    wire [7:0] final_segments = blank_all ? 8'b00000000 : current_segments;
    wire [4:0] final_selects  = blank_all ? 5'b00000    : current_selects;

    assign segment_outputs      = SEG_ACTIVE_HIGH ? final_segments : ~final_segments;
    assign select_outputs[3:0]  = SEL_ACTIVE_HIGH ? final_selects[3:0] : ~final_selects[3:0];
    assign select_outputs[4]    = SEL_ACTIVE_HIGH ? 1'b0 : 1'b1;
endmodule


// ===================== Button Conditioner ==================
module ButtonConditioner #(
    parameter ACTIVE_LOW = 1, 
    parameter integer STABLE_COUNT = 200000
)(
    input  clk, 
    input rst, 
    input btn_in,
    output reg press_pulse
);
    
    wire stable_level;
    DebounceFilter #(.STABLE_COUNT(STABLE_COUNT))
      debounce_filter (.clk(clk), .rst(rst), .signal_in(btn_in), .signal_out(stable_level));

    wire normalized_level = ACTIVE_LOW ? ~stable_level : stable_level;
    reg previous_state;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            previous_state <= 1'b0; 
            press_pulse <= 1'b0; 
        end else begin 
            press_pulse <= (normalized_level & ~previous_state); 
            previous_state <= normalized_level; 
        end
    end
endmodule


// ===================== Debounce Filter ==================   
module DebounceFilter #(
    parameter integer STABLE_COUNT = 200000
)(
    input clk, 
    input rst, 
    input signal_in, 
    output reg signal_out
);
    
    reg [31:0] stability_counter; 
    reg stable_state;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            stability_counter <= 0; 
            stable_state <= 1'b1; 
            signal_out <= 1'b1; 
        end else begin
            if (signal_in == stable_state) 
                stability_counter <= 0;
            else if (stability_counter >= STABLE_COUNT) begin 
                stable_state <= signal_in; 
                signal_out <= signal_in; 
                stability_counter <= 0; 
            end else 
                stability_counter <= stability_counter + 1;
        end
    end
endmodule


// ===================== Power-On Reset ==================
module PowerOnReset #(
    parameter integer CYCLES = 32'd3200000
)(
    input clk, 
    output rst
);
    
    reg [31:0] counter = 0; 
    reg reset_state = 1'b1;
    
    always @(posedge clk) begin
        if (counter >= CYCLES) 
            reset_state <= 1'b0; 
        else begin 
            counter <= counter + 1; 
            reset_state <= 1'b1; 
        end
    end
    
    assign rst = reset_state;
endmodule


// ===================== Reset Controller ==================
module ResetController #(
    parameter integer HOLD_CYCLES = 32'd4000000
)(
    input clk, 
    input por, 
    input trigger, 
    output rst_out
);
    
    reg [31:0] hold_counter = 0; 
    reg reset_active = 1'b1;
    
    always @(posedge clk) begin
        if (por) begin 
            reset_active <= 1'b1; 
            hold_counter <= HOLD_CYCLES; 
        end else if (trigger) begin 
            reset_active <= 1'b1; 
            hold_counter <= HOLD_CYCLES; 
        end else if (reset_active) begin 
            if (hold_counter == 0) 
                reset_active <= 1'b0; 
            else 
                hold_counter <= hold_counter - 1; 
        end
    end
    
    assign rst_out = reset_active;
endmodule


// ===================== Clock Divider ==================
module ClockDivToggle #(
    parameter integer TOGGLE_COUNT = 20000000
)(
    input clk, 
    input rst, 
    output reg clk_out
);
    
    reg [31:0] counter;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            counter <= 0; 
            clk_out <= 1'b0; 
        end else if (counter >= TOGGLE_COUNT - 1) begin 
            counter <= 0; 
            clk_out <= ~clk_out; 
        end else 
            counter <= counter + 1;
    end
endmodule


// ===================== Buzzer Controller - REVISED ==================
module BuzzerController(
    input clk, 
    input rst, 
    input short_beep_trigger, 
    input long_beep_trigger, 
    output buzzer_output
);
    
    // Timing parameters for proper audio feedback
    localparam [23:0] SHORT_BEEP_DURATION = 24'd4000000;    // 0.1 seconds at 40MHz
    localparam [23:0] LONG_BEEP_DURATION  = 24'd20000000;   // 0.5 seconds at 40MHz  
    
    // Frequencies for different beeps
    localparam [15:0] NORMAL_BEEP_FREQ  = 16'd45454;      // ~880 Hz (A5 note)
    localparam [15:0] FINAL_BEEP_FREQ   = 16'd75835;      // ~523 Hz (C5 note, a different tone)

    reg beep_active; 
    reg [23:0] duration_counter; 
    reg [15:0] frequency_counter; 
    reg square_wave;
    reg is_long_beep; // 0=short, 1=long
    
    // Edge detection for triggers to prevent continuous activation
    reg short_trigger_prev, long_trigger_prev;
    wire short_trigger_edge = short_beep_trigger & ~short_trigger_prev;
    wire long_trigger_edge = long_beep_trigger & ~long_trigger_prev;
    
    // CORRECTED: Moved wire declaration outside of always block
    wire [15:0] current_freq_limit;
    assign current_freq_limit = is_long_beep ? FINAL_BEEP_FREQ : NORMAL_BEEP_FREQ;

    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            beep_active <= 1'b0; 
            duration_counter <= 24'd0; 
            frequency_counter <= 16'd0; 
            square_wave <= 1'b0; 
            is_long_beep <= 1'b0;
            short_trigger_prev <= 1'b0;
            long_trigger_prev <= 1'b0;
        end else begin
            short_trigger_prev <= short_beep_trigger;
            long_trigger_prev <= long_beep_trigger;
            
            if (!beep_active) begin
                if (long_trigger_edge) begin 
                    beep_active <= 1'b1; 
                    duration_counter <= LONG_BEEP_DURATION; 
                    is_long_beep <= 1'b1;
                    frequency_counter <= 16'd0;
                end else if (short_trigger_edge) begin 
                    beep_active <= 1'b1; 
                    duration_counter <= SHORT_BEEP_DURATION; 
                    is_long_beep <= 1'b0;
                    frequency_counter <= 16'd0;
                end
            end else begin // beep is active
                if (duration_counter > 0)
                    duration_counter <= duration_counter - 1;
                else
                    beep_active <= 1'b0;
                
                if (frequency_counter >= current_freq_limit) begin 
                    frequency_counter <= 16'd0; 
                    square_wave <= ~square_wave; 
                end else 
                    frequency_counter <= frequency_counter + 1;
            end
        end
    end
    
    assign buzzer_output = beep_active ? square_wave : 1'b0;
endmodule


// ===================== LCD Controller - FIXED VERSION ==================
module LCD1602Controller (
    input  wire        clk,
    input  wire        rst,
    input  wire [127:0] line1_data,
    input  wire [127:0] line2_data,
    output reg         lcd_rs,
    output reg         lcd_e,
    output wire        lcd_rw,
    output reg  [7:0]  lcd_data
);
    
    assign lcd_rw = 1'b0;

    reg [7:0] delay_counter;
    reg [7:0] current_byte;
    reg       is_data_mode;
    reg [4:0] character_index;

    reg [127:0] line1_prev, line2_prev;
    wire data_changed;
    always @(posedge clk) begin
        line1_prev <= line1_data;
        line2_prev <= line2_data;
    end
    assign data_changed = (line1_data != line1_prev) || (line2_data != line2_prev);

    function [7:0] extract_character;
        input [127:0] line_data; 
        input [3:0] char_index;
        extract_character = line_data >> (8 * (15 - char_index));
    endfunction

    localparam [4:0] LCD_POWER_WAIT    = 5'd0,  LCD_FUNCTION_SET = 5'd1,  LCD_DISPLAY_OFF = 5'd2,
                     LCD_CLEAR_DISPLAY = 5'd3,  LCD_ENTRY_MODE   = 5'd4,  LCD_DISPLAY_ON  = 5'd5,
                     LCD_SET_LINE1     = 5'd6,  LCD_WRITE_LINE1  = 5'd7,  LCD_NEXT_LINE1  = 5'd8,
                     LCD_SET_LINE2     = 5'd9,  LCD_WRITE_LINE2  = 5'd10, LCD_NEXT_LINE2  = 5'd11,
                     LCD_SEND_BYTE     = 5'd12, LCD_E_HIGH       = 5'd13, LCD_E_LOW       = 5'd14, 
                     LCD_DELAY         = 5'd15, LCD_IDLE         = 5'd16;
                     
    reg [4:0] current_state, return_state;
    reg initialization_done;
    reg [7:0] post_delay;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lcd_rs <= 1'b0; 
            lcd_e <= 1'b0; 
            lcd_data <= 8'h00;
            delay_counter <= 8'd20;
            current_state <= LCD_POWER_WAIT; 
            character_index <= 5'd0; 
            is_data_mode <= 1'b0; 
            post_delay <= 8'd0; 
            current_byte <= 8'h00;
            initialization_done <= 1'b0;
        end else begin
            
            if (delay_counter > 0) begin
                delay_counter <= delay_counter - 8'd1;
            end else begin
                case (current_state)
                    LCD_POWER_WAIT:    begin post_delay <= 8'd20; return_state <= LCD_FUNCTION_SET; current_state <= LCD_DELAY; end
                    LCD_FUNCTION_SET:  begin current_byte <= 8'h38; is_data_mode <= 1'b0; post_delay <= 8'd5; return_state <= LCD_DISPLAY_OFF; current_state <= LCD_SEND_BYTE; end
                    LCD_DISPLAY_OFF:   begin current_byte <= 8'h08; is_data_mode <= 1'b0; post_delay <= 8'd2; return_state <= LCD_CLEAR_DISPLAY; current_state <= LCD_SEND_BYTE; end
                    LCD_CLEAR_DISPLAY: begin current_byte <= 8'h01; is_data_mode <= 1'b0; post_delay <= 8'd2; return_state <= LCD_ENTRY_MODE; current_state <= LCD_SEND_BYTE; end
                    LCD_ENTRY_MODE:    begin current_byte <= 8'h06; is_data_mode <= 1'b0; post_delay <= 8'd2; return_state <= LCD_DISPLAY_ON; current_state <= LCD_SEND_BYTE; end
                    LCD_DISPLAY_ON:    begin current_byte <= 8'h0C; is_data_mode <= 1'b0; post_delay <= 8'd2; return_state <= LCD_SET_LINE1; initialization_done <= 1'b1; current_state <= LCD_SEND_BYTE; end
                    LCD_SET_LINE1:     begin current_byte <= 8'h80; is_data_mode <= 1'b0; post_delay <= 8'd1; character_index <= 5'd0; return_state <= LCD_WRITE_LINE1; current_state <= LCD_SEND_BYTE; end
                    LCD_WRITE_LINE1:   begin current_byte <= extract_character(line1_data, character_index[3:0]); is_data_mode <= 1'b1; post_delay <= 8'd1; return_state <= LCD_NEXT_LINE1; current_state <= LCD_SEND_BYTE; end
                    LCD_NEXT_LINE1:    begin if (character_index == 5'd15) current_state <= LCD_SET_LINE2; else begin character_index <= character_index + 5'd1; current_state <= LCD_WRITE_LINE1; end end
                    LCD_SET_LINE2:     begin current_byte <= 8'hC0; is_data_mode <= 1'b0; post_delay <= 8'd1; character_index <= 5'd0; return_state <= LCD_WRITE_LINE2; current_state <= LCD_SEND_BYTE; end
                    LCD_WRITE_LINE2:   begin current_byte <= extract_character(line2_data, character_index[3:0]); is_data_mode <= 1'b1; post_delay <= 8'd1; return_state <= LCD_NEXT_LINE2; current_state <= LCD_SEND_BYTE; end
                    LCD_NEXT_LINE2:    begin if (character_index == 5'd15) current_state <= LCD_IDLE; else begin character_index <= character_index + 5'd1; current_state <= LCD_WRITE_LINE2; end end
                    LCD_IDLE:          begin if (data_changed && initialization_done) current_state <= LCD_SET_LINE1; end
                    LCD_SEND_BYTE:     begin lcd_rs <= is_data_mode; lcd_data <= current_byte; lcd_e <= 1'b0; delay_counter <= 8'd1; current_state <= LCD_E_HIGH; end
                    LCD_E_HIGH:        begin lcd_e <= 1'b1; delay_counter <= 8'd1; current_state <= LCD_E_LOW; end
                    LCD_E_LOW:         begin lcd_e <= 1'b0; delay_counter <= post_delay; current_state <= LCD_DELAY; end
                    LCD_DELAY:         current_state <= return_state;
                    default:           current_state <= LCD_POWER_WAIT;
                endcase
            end
        end
    end
endmodule


