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
// Revision 0.01 - File Created
// Additional Comments: 
//
//  Logic Lab Project (Spring 1404) - REFACTORED VERSION
//  TopModule - Fitness Workout Timer System
//  - Debounce + one-shot on 40MHz
//  - 1Hz tick, 1kHz scan domain  
//  - 7-segment frame lock + re-frame blank
//  - LCD1602 eight-bit driver with robust timing/state
//  - Buzzer: 1kHz short, 2kHz long beeps
// ==========================================================

module TopModule #(
    // Simulation Parameters
    parameter SIM_SPEEDUP      = 1'b0,
    
    // Display Parameters  
    parameter SEG_ACTIVE_HIGH  = 1'b1,
    parameter SEL_ACTIVE_HIGH  = 1'b1,
    parameter BTN_ACTIVE_LOW   = 1'b1,
    
    // Timing Parameters
    parameter WORK_TIME_SEC    = 8'd45,
    parameter REST_TIME_SEC    = 8'd15,
    parameter DEBOUNCE_MS      = 5,
    parameter RESET_HOLD_MS    = 100,
    parameter POWERUP_DELAY_MS = 80
)(
    // Clock & Reset
    input  clk_40MHz,
    input  btn_reset, 
    input  btn_start, 
    input  btn_skip,    // Active-Low (board pull-ups)
    
    // User Inputs
    input  [2:0] W,      // Weight selection
    input  [1:0] Cal,    // Calorie target 
    input  [1:0] MET,    // MET level
    input        G,      // Gender
    
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

    // Workout state definitions - MOVED INSIDE MODULE
    localparam [1:0] WORKOUT_STATE_IDLE = 2'b00;
    localparam [1:0] WORKOUT_STATE_WORK = 2'b01; 
    localparam [1:0] WORKOUT_STATE_REST = 2'b10;

    // ============ CLOCK GENERATION ============
    localparam integer FREQ_40MHZ = 40_000_000;
    localparam integer DIV_1HZ_TOGGLE = SIM_SPEEDUP ? 20000 : (FREQ_40MHZ / 2); // 1 Hz toggle
    localparam integer DIV_1KHZ_TOGGLE = SIM_SPEEDUP ? 2000 : (FREQ_40MHZ / 2000); // 1 kHz toggle    
    wire clk_1Hz, clk_1kHz;
    wire system_reset; // Forward declaration

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

    wire buzzer_reset = power_on_reset;

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
        else if (start_workout_cmd) begin 
            weight_latched <= W; 
            calorie_latched <= Cal; 
            met_latched <= MET; 
            gender_latched <= G; 
            total_exercises_latched <= total_exercises_preview; 
        end
    end

    while (dut.system_reset) begin
        @(posedge clk_40MHz);
    end

    // ============ WORKOUT STATE MACHINE ============
    wire [8:0] current_exercise_num;
    wire [7:0] countdown_seconds;
    wire rest_period_entered, exercise_completed, workout_finished;
    wire [1:0] workout_state;
    
    WorkoutStateMachine #(
        .WORK_DURATION(WORK_TIME_SEC),
        .REST_DURATION(REST_TIME_SEC)
    ) fsm_unit (
        .reset(system_reset), 
        .skip_button(skip_btn_pressed), 
        .start_button(start_workout_cmd),
        .clk(clk_40MHz), 
        .tick_1Hz(tick_1Hz),
        .total_exercises(total_exercises_latched),
        .current_exercise(current_exercise_num), 
        .countdown_timer(countdown_seconds),
        .rest_entered(rest_period_entered), 
        .exercise_done(exercise_completed),
        .workout_complete(workout_finished), 
        .state(workout_state)
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
    
    always @(posedge clk_1kHz) begin
        seconds_display <= countdown_seconds;
        exercise_display <= (current_exercise_num > 9'd99) ? 8'd99 : current_exercise_num[7:0];
        idle_state_display <= (workout_state == WORKOUT_STATE_IDLE);
    end

    // Power-on blank timing
    reg [2:0] startup_blank_ms;
    always @(posedge clk_1kHz) begin
        if (reset_1khz_domain) 
            startup_blank_ms <= 3'd5;
        else if (startup_blank_ms != 0) 
            startup_blank_ms <= startup_blank_ms - 3'd1;
    end

    // ============ 7-SEGMENT DISPLAY ============
    
    // Frame sync for smooth transitions
    reg dot_blink_sync1, dot_blink_sync2;  
    always @(posedge clk_1kHz) begin 
        if (reset_1khz_domain) begin 
            dot_blink_sync1 <= 1'b1; 
            dot_blink_sync2 <= 1'b1; 
        end else begin 
            dot_blink_sync1 <= clk_1Hz; 
            dot_blink_sync2 <= dot_blink_sync1; 
        end 
    end
    wire dot_blink_signal = dot_blink_sync2;

    // Generate 7-segment patterns
    wire [7:0] exercise_ones_seg, exercise_tens_seg, timer_ones_seg, timer_tens_seg;
    BCDToSevenSegment exercise_decoder (.bcd_value(exercise_display), 
                                       .ones_segments(exercise_ones_seg), 
                                       .tens_segments(exercise_tens_seg));
    BCDToSevenSegment timer_decoder (.bcd_value(seconds_display), 
                                     .ones_segments(timer_ones_seg), 
                                     .tens_segments(timer_tens_seg));

    // Display patterns
    localparam [7:0] SEG_UNDERSCORE = 8'b00001000;
    wire [7:0] digit0_pattern = idle_state_display ? SEG_UNDERSCORE : timer_ones_seg;
    wire [7:0] digit1_pattern = idle_state_display ? SEG_UNDERSCORE : timer_tens_seg;
    wire [7:0] digit2_pattern = idle_state_display ? SEG_UNDERSCORE : exercise_ones_seg;
    wire [7:0] digit3_pattern = idle_state_display ? SEG_UNDERSCORE : exercise_tens_seg;

    // Frame transition detection for smooth updates
    reg idle_state_prev; 
    always @(posedge clk_1kHz) idle_state_prev <= idle_state_display;
    wire display_layout_changed = (idle_state_prev ^ idle_state_display);

    wire frame_strobe;
    reg [1:0] reframe_state;
    always @(posedge clk_1kHz or posedge reset_1khz_domain) begin
        if (reset_1khz_domain) 
            reframe_state <= 2'd0;
        else begin
            if (display_layout_changed && reframe_state == 2'd0) 
                reframe_state <= 2'd1;
            else if (reframe_state == 2'd1 && frame_strobe) 
                reframe_state <= 2'd2;
            else if (reframe_state == 2'd2 && frame_strobe) 
                reframe_state <= 2'd0;
        end
    end

    wire blank_display = (startup_blank_ms != 0) | (reframe_state == 2'd2);

    SevenSegmentScanner4Digit #(
        .SEG_ACTIVE_HIGH(SEG_ACTIVE_HIGH),
        .SEL_ACTIVE_HIGH(SEL_ACTIVE_HIGH),
        .DOT_DIGIT_INDEX(2)
    ) seg_scanner (
        .clk_scan(clk_1kHz), 
        .rst(reset_1khz_domain), 
        .blank_all(blank_display), 
        .dot_blink(dot_blink_signal),
        .digit0(digit0_pattern), 
        .digit1(digit1_pattern), 
        .digit2(digit2_pattern), 
        .digit3(digit3_pattern),
        .segment_outputs(SEG_DATA), 
        .select_outputs(SEG_SEL), 
        .frame_strobe(frame_strobe)
    );

    // ============ LCD DISPLAY ============
    
    wire [127:0] lcd_line1_text, lcd_line2_text;
    
    LCDTextGenerator lcd_text_gen (
        .clk(clk_1kHz),
        .rst(reset_1khz_domain),
        .idle_mode(idle_state_display),
        
        // Current inputs (for idle display)
        .weight_sel(W),
        .calorie_sel(Cal), 
        .met_sel(MET),
        .gender(G),
        .exercises_preview(total_exercises_preview),
        
        // Latched values (for workout display)
        .weight_latched(weight_latched),
        .gender_latched(gender_latched),
        .total_exercises(total_exercises_latched),
        .current_exercise(current_exercise_num),
        .countdown_time(countdown_seconds),
        .workout_state(workout_state),
        
        // Output text lines
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

    // ============ AUDIO FEEDBACK ============
    wire short_beep_trigger = start_btn_pressed | skip_btn_pressed | reset_btn_pressed | 
                              rest_period_entered | exercise_completed;
                              
    BuzzerController buzzer_ctrl (
        .clk(clk_40MHz), 
        .rst(buzzer_reset),
        .short_beep_trigger(short_beep_trigger), 
        .long_beep_trigger(workout_finished),
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
    
    // Weight mapping (kg)
    function [6:0] get_weight_kg;
        input [2:0] sel;
        case (sel)
            3'b000: get_weight_kg = 7'd50;   3'b001: get_weight_kg = 7'd60;
            3'b010: get_weight_kg = 7'd70;   3'b011: get_weight_kg = 7'd80;
            3'b100: get_weight_kg = 7'd90;   3'b101: get_weight_kg = 7'd100;
            3'b110: get_weight_kg = 7'd110;  3'b111: get_weight_kg = 7'd120;
            default: get_weight_kg = 7'd70;
        endcase
    endfunction

    // Calorie target mapping
    function [8:0] get_calorie_target;
        input [1:0] sel;
        case (sel)
            2'b00: get_calorie_target = 9'd50;   2'b01: get_calorie_target = 9'd100;
            2'b10: get_calorie_target = 9'd150;  2'b11: get_calorie_target = 9'd200;
            default: get_calorie_target = 9'd100;
        endcase
    endfunction

    // MET multiplier mapping
    function [3:0] get_met_multiplier;
        input [1:0] sel;
        case (sel) 
            2'b00: get_met_multiplier = 4'd1;    2'b01: get_met_multiplier = 4'd2; 
            2'b10: get_met_multiplier = 4'd4;    2'b11: get_met_multiplier = 4'd8;
            default: get_met_multiplier = 4'd1;
        endcase
    endfunction

    // Lookup table for base exercises (weight + calorie combination)
    reg [7:0] base_exercises;
    always @(*) begin
        case ({weight_sel, calorie_sel})
            5'b00000: base_exercises = 8'd60;   5'b00001: base_exercises = 8'd120;  
            5'b00010: base_exercises = 8'd180;  5'b00011: base_exercises = 8'd240;
            5'b00100: base_exercises = 8'd50;   5'b00101: base_exercises = 8'd100;  
            5'b00110: base_exercises = 8'd150;  5'b00111: base_exercises = 8'd200;
            5'b01000: base_exercises = 8'd42;   5'b01001: base_exercises = 8'd85;   
            5'b01010: base_exercises = 8'd128;  5'b01011: base_exercises = 8'd171;
            5'b01100: base_exercises = 8'd37;   5'b01101: base_exercises = 8'd75;   
            5'b01110: base_exercises = 8'd112;  5'b01111: base_exercises = 8'd150;
            5'b10000: base_exercises = 8'd33;   5'b10001: base_exercises = 8'd66;   
            5'b10010: base_exercises = 8'd100;  5'b10011: base_exercises = 8'd133;
            5'b10100: base_exercises = 8'd30;   5'b10101: base_exercises = 8'd60;   
            5'b10110: base_exercises = 8'd90;   5'b10111: base_exercises = 8'd120;
            5'b11000: base_exercises = 8'd27;   5'b11001: base_exercises = 8'd54;   
            5'b11010: base_exercises = 8'd81;   5'b11011: base_exercises = 8'd109;
            5'b11100: base_exercises = 8'd25;   5'b11101: base_exercises = 8'd50;   
            5'b11110: base_exercises = 8'd75;   5'b11111: base_exercises = 8'd100;
            default:  base_exercises = 8'd50;
        endcase
    end
    
    // Gender adjustment (female gets +25% more exercises)
    wire [8:0] gender_adjusted = gender ? 
        ({1'b0, base_exercises} + {4'b0000, base_exercises[7:3]}) : // +25% for female
        {1'b0, base_exercises};  // Male: no adjustment
    
    // MET level adjustment (higher MET = fewer exercises)
    assign total_exercises = (met_sel == 2'b00) ? gender_adjusted :                // MET 1: no change
                             (met_sel == 2'b01) ? {1'b0, gender_adjusted[8:1]} : // MET 2: /2
                             (met_sel == 2'b10) ? {2'b00, gender_adjusted[8:2]} : // MET 4: /4  
                                                  {3'b000, gender_adjusted[8:3]};   // MET 8: /8

endmodule

// ===================== Workout State Machine ==================
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
    output reg rest_entered, 
    output reg exercise_done, 
    output reg workout_complete,
    output [1:0] state
);

    // Workout state definitions - also defined locally
    localparam [1:0] WORKOUT_STATE_IDLE = 2'b00;
    localparam [1:0] WORKOUT_STATE_WORK = 2'b01; 
    localparam [1:0] WORKOUT_STATE_REST = 2'b10;

    reg [1:0] workout_state; 
    reg [7:0] work_counter; 
    reg [7:0] rest_counter;
    
    assign state = workout_state;

    always @(posedge clk) begin
        if (reset) begin
            workout_state <= WORKOUT_STATE_IDLE; 
            current_exercise <= 9'd0;
            countdown_timer <= (total_exercises > 9'd99) ? 8'd99 : total_exercises[7:0];
            rest_entered <= 1'b0; 
            exercise_done <= 1'b0; 
            workout_complete <= 1'b0;
            work_counter <= WORK_DURATION; 
            rest_counter <= REST_DURATION;
        end else begin
            // Clear pulse signals
            rest_entered <= 1'b0; 
            exercise_done <= 1'b0; 
            workout_complete <= 1'b0;
            
            case (workout_state)
                WORKOUT_STATE_IDLE: begin 
                    countdown_timer <= (total_exercises > 9'd99) ? 8'd99 : total_exercises[7:0];
                    current_exercise <= 9'd0;
                    if (start_button) begin
                        workout_state <= WORKOUT_STATE_WORK; 
                        current_exercise <= 9'd1; 
                        work_counter <= WORK_DURATION; 
                        rest_counter <= REST_DURATION;
                    end
                end
                
                WORKOUT_STATE_WORK: begin 
                    countdown_timer <= work_counter;
                    if (skip_button) begin
                        if (current_exercise < total_exercises) begin
                            current_exercise <= current_exercise + 9'd1; 
                            work_counter <= WORK_DURATION; 
                            workout_state <= WORKOUT_STATE_WORK;
                        end else begin
                            workout_complete <= 1'b1; 
                            workout_state <= WORKOUT_STATE_IDLE;
                        end
                    end else if (work_counter == 0) begin
                        rest_counter <= REST_DURATION; 
                        rest_entered <= 1'b1; 
                        workout_state <= WORKOUT_STATE_REST;
                    end else if (tick_1Hz && work_counter > 0) begin
                        work_counter <= work_counter - 8'd1;
                    end
                end
                
                WORKOUT_STATE_REST: begin 
                    countdown_timer <= rest_counter;
                    if (skip_button) begin
                        if (current_exercise < total_exercises) begin
                            current_exercise <= current_exercise + 9'd1; 
                            work_counter <= WORK_DURATION; 
                            workout_state <= WORKOUT_STATE_WORK;
                        end else begin
                            workout_complete <= 1'b1; 
                            workout_state <= WORKOUT_STATE_IDLE;
                        end
                    end else if (rest_counter == 0) begin
                        if (current_exercise < total_exercises) begin
                            exercise_done <= 1'b1; 
                            current_exercise <= current_exercise + 9'd1; 
                            work_counter <= WORK_DURATION; 
                            workout_state <= WORKOUT_STATE_WORK;  
                        end else begin
                            workout_complete <= 1'b1; 
                            workout_state <= WORKOUT_STATE_IDLE;
                        end
                    end else if (tick_1Hz && rest_counter > 0) begin
                        rest_counter <= rest_counter - 8'd1;
                    end
                end
                
                default: workout_state <= WORKOUT_STATE_IDLE;
            endcase
        end
    end
endmodule

// ===================== LCD Text Generator ==================
module LCDTextGenerator (
    input clk,
    input rst,
    input idle_mode,
    
    // Current inputs (for idle display)
    input [2:0] weight_sel,
    input [1:0] calorie_sel, 
    input [1:0] met_sel,
    input gender,
    input [8:0] exercises_preview,
    
    // Latched values (for workout display)  
    input [2:0] weight_latched,
    input gender_latched,
    input [8:0] total_exercises,
    input [8:0] current_exercise,
    input [7:0] countdown_time,
    input [1:0] workout_state,
    
    // Output text lines
    output [127:0] line1_text,
    output [127:0] line2_text
);
    // Workout state definitions needed for line 2 text
    localparam [1:0] WORKOUT_STATE_WORK = 2'b01;

    // Helper functions
    function [7:0] weight_to_kg;
        input [2:0] sel;
        case (sel)
            3'b000: weight_to_kg = 8'd50;   3'b001: weight_to_kg = 8'd60;
            3'b010: weight_to_kg = 8'd70;   3'b011: weight_to_kg = 8'd80;
            3'b100: weight_to_kg = 8'd90;   3'b101: weight_to_kg = 8'd100;
            3'b110: weight_to_kg = 8'd110;  3'b111: weight_to_kg = 8'd120;
            default: weight_to_kg = 8'd70;
        endcase
    endfunction

    function [8:0] calorie_target;
        input [1:0] sel;
        case (sel)
            2'b00: calorie_target = 9'd50;   2'b01: calorie_target = 9'd100;
            2'b10: calorie_target = 9'd150;  2'b11: calorie_target = 9'd200;
            default: calorie_target = 9'd100;
        endcase
    endfunction

    function [3:0] met_level;
        input [1:0] sel;
        case (sel) 
            2'b00: met_level = 4'd1;  2'b01: met_level = 4'd2; 
            2'b10: met_level = 4'd4;  2'b11: met_level = 4'd8;
            default: met_level = 4'd1;
        endcase
    endfunction

    // BCD conversion for 9-bit values
    function [11:0] binary_to_bcd_9bit;
        input [8:0] binary_val;
        integer i;
        reg [11:0] bcd_result;
        reg [8:0] shift_reg;
        begin
            bcd_result = 12'd0;
            shift_reg = binary_val;
            for(i = 0; i < 9; i = i + 1) begin
                if(bcd_result[3:0] >= 5)    bcd_result[3:0] = bcd_result[3:0] + 4'd3;
                if(bcd_result[7:4] >= 5)    bcd_result[7:4] = bcd_result[7:4] + 4'd3;
                if(bcd_result[11:8] >= 5)   bcd_result[11:8] = bcd_result[11:8] + 4'd3;
                bcd_result = {bcd_result[10:0], shift_reg[8]};
                shift_reg = {shift_reg[7:0], 1'b0};
            end
            binary_to_bcd_9bit = bcd_result;
        end
    endfunction

    // Split 8-bit value into tens and ones
    function [7:0] split_tens_ones;
        input [7:0] value;
        reg [3:0] tens, ones;
        begin
            if      (value < 10)  begin tens = 4'd0; ones = value[3:0]; end
            else if (value < 20)  begin tens = 4'd1; ones = value - 8'd10; end
            else if (value < 30)  begin tens = 4'd2; ones = value - 8'd20; end
            else if (value < 40)  begin tens = 4'd3; ones = value - 8'd30; end
            else if (value < 50)  begin tens = 4'd4; ones = value - 8'd40; end
            else if (value < 60)  begin tens = 4'd5; ones = value - 8'd50; end
            else if (value < 70)  begin tens = 4'd6; ones = value - 8'd60; end
            else if (value < 80)  begin tens = 4'd7; ones = value - 8'd70; end
            else if (value < 90)  begin tens = 4'd8; ones = value - 8'd80; end
            else                  begin tens = 4'd9; ones = value - 8'd90; end
            split_tens_ones = {tens, ones};
        end
    endfunction

    // Modulo 10 for exercise name indexing
    function [3:0] mod10_9bit;
        input [8:0] value;
        reg [8:0] remainder;
        begin
            remainder = value;
            if (remainder >= 9'd200) remainder = remainder - 9'd200;
            if (remainder >= 9'd100) remainder = remainder - 9'd100;
            if (remainder >= 9'd50 ) remainder = remainder - 9'd50;
            if (remainder >= 9'd40 ) remainder = remainder - 9'd40;
            if (remainder >= 9'd30 ) remainder = remainder - 9'd30;
            if (remainder >= 9'd20 ) remainder = remainder - 9'd20;
            if (remainder >= 9'd10 ) remainder = remainder - 9'd10;
            mod10_9bit = remainder[3:0];
        end
    endfunction

    // Convert digit to ASCII
    function [7:0] digit_to_ascii;
        input [3:0] digit;
        digit_to_ascii = (digit <= 9) ? (8'd48 + digit) : 8'd48;
    endfunction

    // LCD line registers
    reg [7:0] line1_chars [0:15];
    reg [7:0] line2_chars [0:15];

    // Pack character arrays into output vectors
    assign line1_text = {line1_chars[0], line1_chars[1], line1_chars[2], line1_chars[3],
                         line1_chars[4], line1_chars[5], line1_chars[6], line1_chars[7],
                         line1_chars[8], line1_chars[9], line1_chars[10], line1_chars[11],
                         line1_chars[12], line1_chars[13], line1_chars[14], line1_chars[15]};
                         
    assign line2_text = {line2_chars[0], line2_chars[1], line2_chars[2], line2_chars[3],
                         line2_chars[4], line2_chars[5], line2_chars[6], line2_chars[7],
                         line2_chars[8], line2_chars[9], line2_chars[10], line2_chars[11],
                         line2_chars[12], line2_chars[13], line2_chars[14], line2_chars[15]};

    // Exercise names as wires instead of packed array
    wire [127:0] exercise_name_0 = "JUMP JACKS      ";
    wire [127:0] exercise_name_1 = "SIT UPS         ";
    wire [127:0] exercise_name_2 = "PUSH UPS        ";
    wire [127:0] exercise_name_3 = "SQUATS          ";
    wire [127:0] exercise_name_4 = "PLANK           ";
    wire [127:0] exercise_name_5 = "LUNGES          ";
    wire [127:0] exercise_name_6 = "TRICEPS DIPS    ";
    wire [127:0] exercise_name_7 = "WALL SIT        ";
    wire [127:0] exercise_name_8 = "HIGH KNEES      ";
    wire [127:0] exercise_name_9 = "BURPEES         ";

    // **FIX 1: Moved declarations out of the always block**
    // Wires for workout mode calculations
    wire [8:0] exercise_minus_1;
    wire [3:0] exercise_name_index;

    // Registers for text generation logic
    reg [11:0] total_bcd;
    reg [3:0] total_hundreds, total_tens, total_ones;
    reg [7:0] exercise_split, timer_split;
    reg [3:0] ex_tens, ex_ones, tm_tens, tm_ones;
    reg [127:0] selected_exercise_name;
    
    // Registers for idle mode calculations (previously declared inside 'if')
    reg [8:0] weight_val, calorie_val;
    reg [3:0] met_val;
    reg [11:0] weight_bcd, calorie_bcd, exercises_bcd;
    reg [3:0] w_hundreds, w_tens, w_ones;
    reg [3:0] c_hundreds, c_tens, c_ones; 
    reg [3:0] e_hundreds, e_tens, e_ones;

    // **FIX 2: Moved continuous assignments out of the always block**
    assign exercise_minus_1 = (current_exercise == 0) ? 9'd0 : (current_exercise - 9'd1);
    assign exercise_name_index = mod10_9bit(exercise_minus_1);

    // Text generation logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Initialize with startup message
            {line1_chars[0], line1_chars[1], line1_chars[2], line1_chars[3],
             line1_chars[4], line1_chars[5], line1_chars[6], line1_chars[7],
             line1_chars[8], line1_chars[9], line1_chars[10], line1_chars[11],
             line1_chars[12], line1_chars[13], line1_chars[14], line1_chars[15]} <= "SET W/CAL/MET   ";
             
            {line2_chars[0], line2_chars[1], line2_chars[2], line2_chars[3],
             line2_chars[4], line2_chars[5], line2_chars[6], line2_chars[7],
             line2_chars[8], line2_chars[9], line2_chars[10], line2_chars[11],
             line2_chars[12], line2_chars[13], line2_chars[14], line2_chars[15]} <= "T=000 READY     ";
        end else begin
            if (idle_mode) begin
                // IDLE MODE: Show current input settings
                
                weight_val = {2'b00, weight_to_kg(weight_sel)};
                calorie_val = calorie_target(calorie_sel);
                met_val = met_level(met_sel);
                
                weight_bcd = binary_to_bcd_9bit(weight_val);
                calorie_bcd = binary_to_bcd_9bit(calorie_val);  
                exercises_bcd = binary_to_bcd_9bit(exercises_preview);
                
                w_hundreds = weight_bcd[11:8]; w_tens = weight_bcd[7:4]; w_ones = weight_bcd[3:0];
                c_hundreds = calorie_bcd[11:8]; c_tens = calorie_bcd[7:4]; c_ones = calorie_bcd[3:0];
                e_hundreds = exercises_bcd[11:8]; e_tens = exercises_bcd[7:4]; e_ones = exercises_bcd[3:0];

                // Line 1: "G:M W070 M1 C100" or "G:F W070 M1 C100"
                line1_chars[0] <= "G"; 
                line1_chars[1] <= ":"; 
                line1_chars[2] <= gender ? "F" : "M"; 
                line1_chars[3] <= " ";
                line1_chars[4] <= "W"; 
                line1_chars[5] <= (w_hundreds == 0) ? " " : digit_to_ascii(w_hundreds);
                line1_chars[6] <= digit_to_ascii(w_tens); 
                line1_chars[7] <= digit_to_ascii(w_ones);
                line1_chars[8] <= " "; 
                line1_chars[9] <= "M"; 
                line1_chars[10] <= digit_to_ascii(met_val);
                line1_chars[11] <= " ";
                line1_chars[12] <= "C"; 
                line1_chars[13] <= (c_hundreds == 0) ? " " : digit_to_ascii(c_hundreds);
                line1_chars[14] <= digit_to_ascii(c_tens); 
                line1_chars[15] <= digit_to_ascii(c_ones);

                // Line 2: "T=052 READY     "
                line2_chars[0] <= "T"; 
                line2_chars[1] <= "="; 
                line2_chars[2] <= digit_to_ascii(e_hundreds); 
                line2_chars[3] <= digit_to_ascii(e_tens); 
                line2_chars[4] <= digit_to_ascii(e_ones);
                line2_chars[5] <= " "; 
                line2_chars[6] <= "R"; 
                line2_chars[7] <= "E"; 
                line2_chars[8] <= "A"; 
                line2_chars[9] <= "D"; 
                line2_chars[10] <= "Y";
                line2_chars[11] <= " "; 
                line2_chars[12] <= " "; 
                line2_chars[13] <= " "; 
                line2_chars[14] <= " "; 
                line2_chars[15] <= " ";
                
            end else begin
                // WORKOUT MODE: Show exercise name and progress
                
                total_bcd = binary_to_bcd_9bit(total_exercises);
                total_hundreds = total_bcd[11:8]; total_tens = total_bcd[7:4]; total_ones = total_bcd[3:0];
                
                exercise_split = split_tens_ones((current_exercise > 9'd99) ? 8'd99 : current_exercise[7:0]);
                timer_split = split_tens_ones(countdown_time);
                ex_tens = exercise_split[7:4]; ex_ones = exercise_split[3:0];
                tm_tens = timer_split[7:4]; tm_ones = timer_split[3:0];

                // Select exercise name
                case (exercise_name_index)
                    4'd0: selected_exercise_name = exercise_name_0;
                    4'd1: selected_exercise_name = exercise_name_1;
                    4'd2: selected_exercise_name = exercise_name_2;
                    4'd3: selected_exercise_name = exercise_name_3;
                    4'd4: selected_exercise_name = exercise_name_4;
                    4'd5: selected_exercise_name = exercise_name_5;
                    4'd6: selected_exercise_name = exercise_name_6;
                    4'd7: selected_exercise_name = exercise_name_7;
                    4'd8: selected_exercise_name = exercise_name_8;
                    default: selected_exercise_name = exercise_name_9;
                endcase

                // Line 1: Exercise name (16 characters)
                {line1_chars[0], line1_chars[1], line1_chars[2], line1_chars[3],
                 line1_chars[4], line1_chars[5], line1_chars[6], line1_chars[7],
                 line1_chars[8], line1_chars[9], line1_chars[10], line1_chars[11],
                 line1_chars[12], line1_chars[13], line1_chars[14], line1_chars[15]} <= selected_exercise_name;

                // Line 2: "E05/052 W:45 G:M" or "E05/052 R:15 G:M"
                line2_chars[0]  <= "E";  
                line2_chars[1]  <= digit_to_ascii(ex_tens); 
                line2_chars[2]  <= digit_to_ascii(ex_ones); 
                line2_chars[3]  <= "/";
                line2_chars[4]  <= digit_to_ascii(total_hundreds);     
                line2_chars[5]  <= digit_to_ascii(total_tens);     
                line2_chars[6]  <= digit_to_ascii(total_ones);     
                line2_chars[7]  <= " ";
                line2_chars[8]  <= (workout_state == WORKOUT_STATE_WORK) ? "W" : "R"; 
                line2_chars[9]  <= ":";
                line2_chars[10] <= digit_to_ascii(tm_tens); 
                line2_chars[11] <= digit_to_ascii(tm_ones); 
                line2_chars[12] <= " ";
                line2_chars[13] <= "G"; 
                line2_chars[14] <= ":"; 
                line2_chars[15] <= gender_latched ? "F" : "M";
            end
        end
    end
endmodule

// ===================== BCD to 7-Segment Converter ==================
module BCDToSevenSegment (
    input  [7:0] bcd_value,          // 0-99
    output reg [7:0] ones_segments,  // 7-segment pattern for ones digit
    output reg [7:0] tens_segments   // 7-segment pattern for tens digit
);
    
    reg [3:0] tens_digit, ones_digit;
    
    // Extract BCD digits
    always @(*) begin
        if      (bcd_value < 10)  begin tens_digit = 4'd0; ones_digit = bcd_value[3:0]; end
        else if (bcd_value < 20)  begin tens_digit = 4'd1; ones_digit = bcd_value - 8'd10; end
        else if (bcd_value < 30)  begin tens_digit = 4'd2; ones_digit = bcd_value - 8'd20; end
        else if (bcd_value < 40)  begin tens_digit = 4'd3; ones_digit = bcd_value - 8'd30; end
        else if (bcd_value < 50)  begin tens_digit = 4'd4; ones_digit = bcd_value - 8'd40; end
        else if (bcd_value < 60)  begin tens_digit = 4'd5; ones_digit = bcd_value - 8'd50; end
        else if (bcd_value < 70)  begin tens_digit = 4'd6; ones_digit = bcd_value - 8'd60; end
        else if (bcd_value < 80)  begin tens_digit = 4'd7; ones_digit = bcd_value - 8'd70; end
        else if (bcd_value < 90)  begin tens_digit = 4'd8; ones_digit = bcd_value - 8'd80; end
        else                      begin tens_digit = 4'd9; ones_digit = bcd_value - 8'd90; end
    end

    // 7-segment patterns (segments: DP G F E D C B A)
    always @(*) begin
        case (ones_digit)
            4'd0: ones_segments = 8'b00111111;  // 0
            4'd1: ones_segments = 8'b00000110;  // 1  
            4'd2: ones_segments = 8'b01011011;  // 2
            4'd3: ones_segments = 8'b01001111;  // 3
            4'd4: ones_segments = 8'b01100110;  // 4
            4'd5: ones_segments = 8'b01101101;  // 5
            4'd6: ones_segments = 8'b01111101;  // 6
            4'd7: ones_segments = 8'b00000111;  // 7
            4'd8: ones_segments = 8'b01111111;  // 8
            4'd9: ones_segments = 8'b01101111;  // 9
            default: ones_segments = 8'b00000000; // blank
        endcase

        case (tens_digit)
            4'd0: tens_segments = 8'b00111111;  // 0
            4'd1: tens_segments = 8'b00000110;  // 1
            4'd2: tens_segments = 8'b01011011;  // 2  
            4'd3: tens_segments = 8'b01001111;  // 3
            4'd4: tens_segments = 8'b01100110;  // 4
            4'd5: tens_segments = 8'b01101101;  // 5
            4'd6: tens_segments = 8'b01111101;  // 6
            4'd7: tens_segments = 8'b00000111;  // 7
            4'd8: tens_segments = 8'b01111111;  // 8
            4'd9: tens_segments = 8'b01101111;  // 9
            default: tens_segments = 8'b00000000; // blank
        endcase
    end
endmodule

// ===================== 7-Segment Scanner ==================
module SevenSegmentScanner4Digit #(
    parameter SEG_ACTIVE_HIGH    = 1'b1,
    parameter SEL_ACTIVE_HIGH    = 1'b1,
    parameter integer DOT_DIGIT_INDEX = 2
)(
    input clk_scan, 
    input rst, 
    input blank_all, 
    input dot_blink,
    input  [7:0] digit0, 
    input  [7:0] digit1, 
    input  [7:0] digit2, 
    input  [7:0] digit3,
    output [7:0] segment_outputs, 
    output [4:0] select_outputs, 
    output frame_strobe
);
    
    reg [1:0] scan_index; 
    reg [7:0] current_segments; 
    reg [4:0] current_selects;
    reg dot_state_latched; 
    reg frame_strobe_reg;

    // Remove dot from input digits (will be added separately)
    wire [7:0] digit0_no_dot = {1'b0, digit0[6:0]};
    wire [7:0] digit1_no_dot = {1'b0, digit1[6:0]};
    wire [7:0] digit2_no_dot = {1'b0, digit2[6:0]};
    wire [7:0] digit3_no_dot = {1'b0, digit3[6:0]};

    localparam [1:0] DOT_INDEX = (DOT_DIGIT_INDEX == 0) ? 2'd0 :
                                 (DOT_DIGIT_INDEX == 1) ? 2'd1 :
                                 (DOT_DIGIT_INDEX == 2) ? 2'd2 : 2'd3;
                                 
    always @(posedge clk_scan or posedge rst) begin
        if (rst) begin 
            scan_index <= 2'd0; 
            dot_state_latched <= 1'b0; 
            frame_strobe_reg <= 1'b0; 
        end else begin
            frame_strobe_reg <= (scan_index == 2'd0);
            if (scan_index == 2'd0) 
                dot_state_latched <= dot_blink;
            scan_index <= scan_index + 2'd1;
        end
    end
    
    assign frame_strobe = frame_strobe_reg;

    // Multiplex digits and selects
    always @(*) begin
        case (scan_index)
            2'd0: begin current_segments = digit0_no_dot; current_selects = 5'b00001; end
            2'd1: begin current_segments = digit1_no_dot; current_selects = 5'b00010; end  
            2'd2: begin current_segments = digit2_no_dot; current_selects = 5'b00100; end
            2'd3: begin current_segments = digit3_no_dot; current_selects = 5'b01000; end
        endcase
        
        // Add dot to designated digit
        if (scan_index == DOT_INDEX) 
            current_segments[7] = dot_state_latched;
    end

    // Apply blanking and polarity
    wire [7:0] final_segments = blank_all ? 8'b00000000 : current_segments;
    wire [4:0] final_selects  = blank_all ? 5'b00000    : current_selects;

    assign segment_outputs      = SEG_ACTIVE_HIGH ? final_segments : ~final_segments;
    assign select_outputs[3:0]  = SEL_ACTIVE_HIGH ? final_selects[3:0] : ~final_selects[3:0];
    assign select_outputs[4]    = SEL_ACTIVE_HIGH ? 1'b0 : 1'b1;  // Unused digit
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
            else if (stability_counter == STABLE_COUNT) begin 
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
        end else 
            reset_active <= 1'b0;
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
        end else if (counter == TOGGLE_COUNT - 1) begin 
            counter <= 0; 
            clk_out <= ~clk_out; 
        end else 
            counter <= counter + 1;
    end
endmodule

// ===================== Buzzer Controller ==================
module BuzzerController(
    input clk, 
    input rst, 
    input short_beep_trigger, 
    input long_beep_trigger, 
    output buzzer_output
);
    
    // Timing parameters
    localparam [25:0] SHORT_BEEP_DURATION = 26'd8000000;  // ~0.20 s
    localparam [25:0] LONG_BEEP_DURATION  = 26'd24000000; // ~0.60 s
    localparam [15:0] SHORT_BEEP_PERIOD   = 16'd19999;    // ~1 kHz
    localparam [15:0] LONG_BEEP_PERIOD    = 16'd9999;     // ~2 kHz

    reg beep_active; 
    reg [25:0] duration_counter; 
    reg [15:0] frequency_counter, frequency_period; 
    reg square_wave;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            beep_active <= 0; 
            duration_counter <= 0; 
            frequency_counter <= 0; 
            frequency_period <= SHORT_BEEP_PERIOD; 
            square_wave <= 0; 
        end else begin
            // Start beep on trigger
            if (long_beep_trigger) begin 
                beep_active <= 1; 
                duration_counter <= LONG_BEEP_DURATION; 
                frequency_period <= LONG_BEEP_PERIOD; 
            end else if (short_beep_trigger) begin 
                beep_active <= 1; 
                duration_counter <= SHORT_BEEP_DURATION; 
                frequency_period <= SHORT_BEEP_PERIOD; 
            end else if (duration_counter != 0) begin 
                duration_counter <= duration_counter - 1; 
                beep_active <= 1; 
            end else begin 
                beep_active <= 0; 
            end

            // Generate square wave when active
            if (!beep_active) begin 
                frequency_counter <= 0; 
                square_wave <= 0; 
            end else if (frequency_counter == frequency_period) begin 
                frequency_counter <= 0; 
                square_wave <= ~square_wave; 
            end else 
                frequency_counter <= frequency_counter + 1;
        end
    end
    
    assign buzzer_output = square_wave;
endmodule

// ===================== LCD Controller ==================
module LCD1602Controller (
    input  wire       clk,        // 1 kHz tick (1 ms resolution)
    input  wire       rst,
    input  wire [127:0] line1_data,
    input  wire [127:0] line2_data,
    output reg        lcd_rs,
    output reg        lcd_e,
    output wire       lcd_rw,     // always write mode
    output reg  [7:0] lcd_data
);
    
    assign lcd_rw = 1'b0;  // Always in write mode

    // Timing and control registers
    reg [7:0] delay_ms, post_delay_ms, current_byte;
    reg       is_data_mode;
    reg [4:0] character_index;

    // Extract character from 16-character line (MSB first)
    function [7:0] extract_character;
        input [127:0] line_data; 
        input [3:0] char_index;
        case(char_index)
            4'd0:  extract_character = line_data[127:120]; 4'd1:  extract_character = line_data[119:112];
            4'd2:  extract_character = line_data[111:104]; 4'd3:  extract_character = line_data[103:96];
            4'd4:  extract_character = line_data[95:88];   4'd5:  extract_character = line_data[87:80];
            4'd6:  extract_character = line_data[79:72];   4'd7:  extract_character = line_data[71:64];
            4'd8:  extract_character = line_data[63:56];   4'd9:  extract_character = line_data[55:48];
            4'd10: extract_character = line_data[47:40];   4'd11: extract_character = line_data[39:32];
            4'd12: extract_character = line_data[31:24];   4'd13: extract_character = line_data[23:16];
            4'd14: extract_character = line_data[15:8];    4'd15: extract_character = line_data[7:0];
            default: extract_character = 8'h20;  // space
        endcase
    endfunction

    // LCD state machine states
    localparam [5:0] LCD_POWER_WAIT    = 6'd0,  LCD_FUNCTION_SET = 6'd1,  LCD_DISPLAY_OFF = 6'd2,
                     LCD_CLEAR_DISPLAY = 6'd3,  LCD_ENTRY_MODE   = 6'd4,  LCD_DISPLAY_ON  = 6'd5,
                     LCD_SET_LINE1     = 6'd6,  LCD_WRITE_LINE1  = 6'd7,  LCD_NEXT_LINE1  = 6'd8,
                     LCD_SET_LINE2     = 6'd9,  LCD_WRITE_LINE2  = 6'd10, LCD_NEXT_LINE2  = 6'd11,
                     LCD_SEND_BYTE     = 6'd12, LCD_E_HIGH       = 6'd13, LCD_E_HOLD      = 6'd14, 
                     LCD_POST_WAIT     = 6'd15;
                     
    reg [5:0] current_state, return_state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lcd_rs <= 0; 
            lcd_e <= 0; 
            lcd_data <= 8'h00;
            delay_ms <= 8'd50; 
            current_state <= LCD_POWER_WAIT; 
            character_index <= 5'd0; 
            is_data_mode <= 0; 
            post_delay_ms <= 0; 
            current_byte <= 8'h00;
        end else if (delay_ms != 0) begin
            delay_ms <= delay_ms - 8'd1;
            if (current_state != LCD_E_HIGH && current_state != LCD_E_HOLD) 
                lcd_e <= 1'b0;
        end else begin
            case (current_state)
                LCD_POWER_WAIT: current_state <= LCD_FUNCTION_SET;

                // LCD Initialization sequence
                LCD_FUNCTION_SET: begin 
                    current_byte <= 8'h38; is_data_mode <= 0; post_delay_ms <= 8'd2; 
                    return_state <= LCD_DISPLAY_OFF; current_state <= LCD_SEND_BYTE; 
                end
                LCD_DISPLAY_OFF: begin 
                    current_byte <= 8'h08; is_data_mode <= 0; post_delay_ms <= 8'd2; 
                    return_state <= LCD_CLEAR_DISPLAY; current_state <= LCD_SEND_BYTE; 
                end
                LCD_CLEAR_DISPLAY: begin 
                    current_byte <= 8'h01; is_data_mode <= 0; post_delay_ms <= 8'd3; 
                    return_state <= LCD_ENTRY_MODE; current_state <= LCD_SEND_BYTE; 
                end
                LCD_ENTRY_MODE: begin 
                    current_byte <= 8'h06; is_data_mode <= 0; post_delay_ms <= 8'd2; 
                    return_state <= LCD_DISPLAY_ON; current_state <= LCD_SEND_BYTE; 
                end
                LCD_DISPLAY_ON: begin 
                    current_byte <= 8'h0C; is_data_mode <= 0; post_delay_ms <= 8'd2; 
                    return_state <= LCD_SET_LINE1; current_state <= LCD_SEND_BYTE; 
                end

                // Line 1 operations
                LCD_SET_LINE1: begin 
                    current_byte <= 8'h80; is_data_mode <= 0; post_delay_ms <= 8'd2; 
                    character_index <= 0; return_state <= LCD_WRITE_LINE1; current_state <= LCD_SEND_BYTE; 
                end
                LCD_WRITE_LINE1: begin 
                    current_byte <= extract_character(line1_data, character_index[3:0]); 
                    is_data_mode <= 1; post_delay_ms <= 8'd1; 
                    return_state <= LCD_NEXT_LINE1; current_state <= LCD_SEND_BYTE; 
                end
                LCD_NEXT_LINE1: begin 
                    if (character_index == 5'd15) 
                        current_state <= LCD_SET_LINE2; 
                    else begin 
                        character_index <= character_index + 1; 
                        current_state <= LCD_WRITE_LINE1; 
                    end 
                end

                // Line 2 operations  
                LCD_SET_LINE2: begin 
                    current_byte <= 8'hC0; is_data_mode <= 0; post_delay_ms <= 8'd2; 
                    character_index <= 0; return_state <= LCD_WRITE_LINE2; current_state <= LCD_SEND_BYTE; 
                end
                LCD_WRITE_LINE2: begin 
                    current_byte <= extract_character(line2_data, character_index[3:0]); 
                    is_data_mode <= 1; post_delay_ms <= 8'd1; 
                    return_state <= LCD_NEXT_LINE2; current_state <= LCD_SEND_BYTE; 
                end
                LCD_NEXT_LINE2: begin 
                    if (character_index == 5'd15) 
                        current_state <= LCD_SET_LINE1; 
                    else begin 
                        character_index <= character_index + 1; 
                        current_state <= LCD_WRITE_LINE2; 
                    end 
                end

                // Generic byte send sequence
                LCD_SEND_BYTE: begin 
                    lcd_rs <= is_data_mode; 
                    lcd_data <= current_byte; 
                    lcd_e <= 1; 
                    delay_ms <= 8'd1; 
                    current_state <= LCD_E_HIGH; 
                end
                LCD_E_HIGH: begin 
                    lcd_e <= 0; 
                    delay_ms <= 8'd1; 
                    current_state <= LCD_E_HOLD; 
                end
                LCD_E_HOLD: begin 
                    delay_ms <= post_delay_ms; 
                    current_state <= LCD_POST_WAIT; 
                end
                LCD_POST_WAIT: current_state <= return_state;

                default: current_state <= LCD_POWER_WAIT;
            endcase
        end
    end
endmodule

