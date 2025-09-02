`timescale 1ns / 1ps

module TopModule_TB();

    // ============== Signal Declarations ==============
    reg clk_40MHz;
    reg btn_reset, btn_start, btn_skip;
    reg [2:0] W;
    reg [1:0] Cal;
    reg [1:0] MET;
    reg       G;

    // DUT Outputs
    wire [7:0] SEG_DATA;
    wire [4:0] SEG_SEL;
    wire       buzzer;
    wire       LCD_RS, LCD_E, LCD_RW;
    wire [7:0] LCD_D;

    // ============== Testbench Specific Variables ==============
    integer input_file, output_file;
    reg [7:0] test_vector;
    integer test_count = 0;

    // ============== UUT Instantiation ==============
    TopModule #(.SIM_SPEEDUP(1'b1)) uut (
        .clk_40MHz(clk_40MHz), .btn_reset(btn_reset), .btn_start(btn_start),
        .btn_skip(btn_skip), .W(W), .Cal(Cal), .MET(MET), .G(G),
        .SEG_DATA(SEG_DATA), .SEG_SEL(SEG_SEL), .buzzer(buzzer),
        .LCD_RS(LCD_RS), .LCD_E(LCD_E), .LCD_RW(LCD_RW), .LCD_D(LCD_D)
    );

    // ============== Helper Functions for Readable Output ==============
    // This function converts the 3-bit weight code to a readable string
    function [8*8:1] weight_to_string;
        input [2:0] w_code;
        case (w_code)
            3'b000: weight_to_string = "W= 50kg";
            3'b001: weight_to_string = "W= 60kg";
            3'b010: weight_to_string = "W= 70kg";
            3'b011: weight_to_string = "W= 80kg";
            3'b100: weight_to_string = "W= 90kg";
            3'b101: weight_to_string = "W=100kg";
            3'b110: weight_to_string = "W=110kg";
            3'b111: weight_to_string = "W=120kg";
            default: weight_to_string = "W= UNK ";
        endcase
    endfunction

    // This function converts the 2-bit calorie code to a readable string
    function [8*8:1] cal_to_string;
        input [1:0] cal_code;
        case (cal_code)
            2'b00: cal_to_string = "Cal= 50";
            2'b01: cal_to_string = "Cal=100";
            2'b10: cal_to_string = "Cal=150";
            2'b11: cal_to_string = "Cal=200";
            default: cal_to_string = "Cal=UNK";
        endcase
    endfunction
    
    // This function converts the 2-bit MET code to a readable string
    function [8*7:1] met_to_string;
        input [1:0] met_code;
        case (met_code)
            2'b00: met_to_string = "MET=1";
            2'b01: met_to_string = "MET=2";
            2'b10: met_to_string = "MET=4";
            2'b11: met_to_string = "MET=8";
            default: met_to_string = "MET=UNK";
        endcase
    endfunction

    // This function converts the 1-bit Gender code to a readable string
    function [8*10:1] g_to_string;
        input g_code;
        if (g_code == 1'b1)
            g_to_string = "G=1.125";
        else
            g_to_string = "G=1";
    endfunction


    // ============== Clock Generation ==============
    initial begin
        clk_40MHz = 0;
        forever #12.5 clk_40MHz = ~clk_40MHz; // 40MHz clock
    end

    // ============== Main Test Sequence ==============
    initial begin
        $display("Starting Testbench with Readable File Output...");

        // Initialize inputs
        btn_reset = 1; btn_start = 1; btn_skip  = 1;
        {W, Cal, MET, G} = 8'b0;

        // Open files
        input_file = $fopen("inputs.txt", "r");
        output_file = $fopen("output.txt", "w");

        if (input_file == 0) begin
            $display("ERROR: Could not open inputs.txt.");
            $finish;
        end

        // Write header to the output file
        $fdisplay(output_file, "======= Test Results (Readable Format) =======");
        $fdisplay(output_file, "Inputs                               | Outputs");
        $fdisplay(output_file, "-------------------------------------+-----------------");

        #1000; // Wait for initial reset

        // Loop through the input file
        while (!$feof(input_file)) begin
            $fscanf(input_file, "%b\n", test_vector);
            {W, Cal, MET, G} = test_vector;
            #100; // Wait for logic to settle

            // Write the formatted, readable result to the output file
            $fdisplay(output_file, "%s, %s, %s, %s | => T = %d min",
                      weight_to_string(W),
                      cal_to_string(Cal),
                      met_to_string(MET),
                      g_to_string(G),
                      uut.total_exercises_preview);
        end

        $display("Test finished. Results saved in output.txt.");

        // Clean up and finish
        $fclose(input_file);
        $fclose(output_file);
        $finish;
    end

endmodule