`timescale 1ns / 1ps

module tb_symbol_template_matcher;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg glyph_valid = 1'b0;
    reg [63:0] glyph = 64'd0;
    wire busy;
    wire result_valid;
    wire recognized;
    wire [5:0] class_id;
    wire [7:0] ascii_value;
    wire [6:0] best_distance;
    wire [6:0] second_distance;

    reg [5:0] rom_addr;
    wire [63:0] rom_bitmap;
    wire [7:0] rom_ascii;
    integer n;
    integer failures = 0;
    integer noisy_rejects = 0;
    reg [63:0] stimulus;

    always #5 clk = ~clk;

    symbol_template_rom u_rom (
        .addr_i(rom_addr), .bitmap_o(rom_bitmap), .ascii_o(rom_ascii)
    );

    symbol_template_matcher #(
        .MAX_DISTANCE(7'd12), .MIN_MARGIN(7'd2)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .glyph_valid_i(glyph_valid), .glyph_i(glyph),
        .busy_o(busy), .result_valid_o(result_valid),
        .recognized_o(recognized), .class_id_o(class_id),
        .ascii_o(ascii_value), .best_distance_o(best_distance),
        .second_distance_o(second_distance)
    );

    task run_case;
        input [63:0] bitmap;
        input [5:0] expected_class;
        input [6:0] expected_distance;
        begin
            while (busy) @(negedge clk);
            @(negedge clk);
            glyph       = bitmap;
            glyph_valid = 1'b1;
            @(negedge clk);
            glyph_valid = 1'b0;
            while (!result_valid) @(negedge clk);
            if (!recognized || class_id != expected_class ||
                best_distance != expected_distance) begin
                $display("FAIL expected=%0d got=%0d rec=%0d dist=%0d second=%0d",
                         expected_class, class_id, recognized,
                         best_distance, second_distance);
                failures = failures + 1;
            end
        end
    endtask

    task run_noisy_case;
        input [63:0] bitmap;
        input [5:0] expected_class;
        begin
            while (busy) @(negedge clk);
            @(negedge clk);
            glyph       = bitmap;
            glyph_valid = 1'b1;
            @(negedge clk);
            glyph_valid = 1'b0;
            while (!result_valid) @(negedge clk);
            // Ambiguous noise may be rejected, but must never be reported as
            // a different class.
            if (recognized && class_id != expected_class) begin
                $display("FAIL noisy expected=%0d got=%0d dist=%0d second=%0d",
                         expected_class, class_id,
                         best_distance, second_distance);
                failures = failures + 1;
            end else if (!recognized) begin
                noisy_rejects = noisy_rejects + 1;
            end
        end
    endtask

    initial begin
        @(negedge clk);
        @(negedge clk);
        @(negedge clk);
        @(negedge clk);
        rst_n = 1'b1;

        // Every exact template must recognize as its own class.
        for (n = 0; n < 50; n = n + 1) begin
            rom_addr = n[5:0];
            #1 stimulus = rom_bitmap;
            run_case(stimulus, n + 1, 0);
        end

        // Every class must tolerate two deterministic flipped cells.
        for (n = 0; n < 50; n = n + 1) begin
            rom_addr = n[5:0];
            #1 stimulus = rom_bitmap ^ (64'h1 << (n % 64)) ^
                                      (64'h1 << ((n + 17) % 64));
            run_noisy_case(stimulus, n + 1);
        end

        // Empty input must be rejected rather than guessed as a line.
        while (busy) @(negedge clk);
        @(negedge clk);
        glyph = 64'd0;
        glyph_valid = 1'b1;
        @(negedge clk);
        glyph_valid = 1'b0;
        while (!result_valid) @(negedge clk);
        if (recognized || class_id != 0) begin
            $display("FAIL empty glyph was accepted as class %0d", class_id);
            failures = failures + 1;
        end

        if (failures == 0)
            $display("SYMBOL_MATCHER PASS: 101 cases, noisy rejects=%0d",
                     noisy_rejects);
        else
            $display("SYMBOL_MATCHER FAIL: %0d cases", failures);
        $finish;
    end
endmodule
