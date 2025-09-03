`timescale 1ns/1ps

module ExerciseCalculator_tb;

    reg  [2:0] W;
    reg  [1:0] Cal;
    reg  [1:0] MET;
    reg        G;
    wire [8:0] T_rtl;

    ExerciseCalculator dut (
        .calorie_sel(Cal),
        .weight_sel(W),
        .gender(G),
        .met_sel(MET),
        .total_exercises(T_rtl)
    );

    integer fin, fout_txt, fout_bin;
    integer line_num;
    reg [7:0] din8;
    integer rv;
    integer pass_cnt, fail_cnt;

    function integer map_W_kg;
        input [2:0] Wc;
        begin
            case (Wc)
                3'b000: map_W_kg=50;
                3'b001: map_W_kg=60;
                3'b010: map_W_kg=70;
                3'b011: map_W_kg=80;
                3'b100: map_W_kg=90;
                3'b101: map_W_kg=100;
                3'b110: map_W_kg=110;
                default: map_W_kg=120;
            endcase
        end
    endfunction

    function integer map_Cal;
        input [1:0] Cc;
        begin
            case (Cc)
                2'b00: map_Cal=50;
                2'b01: map_Cal=100;
                2'b10: map_Cal=150;
                default: map_Cal=200;
            endcase
        end
    endfunction

    function integer map_MET;
        input [1:0] Mc;
        begin
            case (Mc)
                2'b00: map_MET=1;
                2'b01: map_MET=2;
                2'b10: map_MET=4;
                default: map_MET=8;
            endcase
        end
    endfunction

    function integer ref_T_min;
        input [2:0] Wc;
        input [1:0] Cc;
        input [1:0] Mc;
        input Gbit;
        integer Wkg, CalVal, base, gendered, shift;
        begin
            Wkg    = map_W_kg(Wc);
            CalVal = map_Cal(Cc);
            base   = (CalVal*60)/Wkg;
            if (Gbit==1)
                gendered = base + (base>>3);
            else
                gendered = base;

            case (Mc)
                2'b00: shift=0;
                2'b01: shift=1;
                2'b10: shift=2;
                default: shift=3;
            endcase
            ref_T_min = (gendered >> shift);
        end
    endfunction

    initial begin
        pass_cnt=0; fail_cnt=0; line_num=0;

        fin      = $fopen("inputs.txt", "r");
        fout_txt = $fopen("output.txt", "w");
        fout_bin = $fopen("output_bin.txt", "w");

        if (fin==0) begin
            $display("ERROR: inputs.txt not found. Simulation cannot continue.");
            $finish;
        end

        $fdisplay(fout_txt, "Line |  W  | Cal | MET |  G  | T_ref | T_rtl | Result");
        $fdisplay(fout_txt, "-----+-----+-----+-----+-----+-------+-------+--------");

        while (!$feof(fin)) begin
            line_num = line_num + 1;

            rv = $fscanf(fin, "%8b\n", din8);
            if (rv==1) begin
                W   = din8[7:5];
                Cal = din8[4:3];
                MET = din8[2:1];
                G   = din8[0];

                #1;

                begin : SCORE
                    integer Wkg, CalVal, METval, T_ref;
                    Wkg    = map_W_kg(W);
                    CalVal = map_Cal(Cal);
                    METval = map_MET(MET);
                    T_ref  = ref_T_min(W,Cal,MET,G);

                    $fdisplay(fout_txt, "%4d | %3d | %3d | %3d |  %1s  |  %3d  |  %3d  | %s",
                                      line_num, Wkg, CalVal, METval, (G?"F":"M"),
                                      T_ref, T_rtl, (T_ref==T_rtl)?"PASS":"FAIL");

                    $fdisplay(fout_bin, "%9b", T_rtl);

                    if (T_ref==T_rtl) pass_cnt = pass_cnt + 1; else fail_cnt = fail_cnt + 1;
                end
            end
        end

        $display("Lines=%0d | PASS=%0d | FAIL=%0d", line_num-1, pass_cnt, fail_cnt);
        $fclose(fin);
        $fclose(fout_txt);
        $fclose(fout_bin);
        $finish;
    end
endmodule