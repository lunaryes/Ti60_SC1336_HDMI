`timescale 1ns / 1ps

// Regression for the combined VF-Ti60F225 key controller.
// K1 cycles smoothing 0/1/2; K2 cycles threshold 4..48 in steps of 4.
module tb_canny_thr_key;
    localparam integer CLK_HZ = 8000;
    localparam integer DB_MS  = 1;
    localparam integer DB_CYC = 8;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst_n = 1'b0;
    reg  [1:0] key_data = 2'b11;
    reg        uart_thr_valid = 1'b0;
    reg  [7:0] uart_thr = 8'd0;
    reg        uart_smooth_valid = 1'b0;
    reg  [1:0] uart_smooth = 2'd0;
    wire [7:0] threshold;
    wire [7:0] threshold_low;
    wire [1:0] smooth_mode;
    wire [1:0] key_press;

    integer errors = 0;
    integer i;

    canny_threshold_key_ctrl #(
        .CLK_FREQ_HZ(CLK_HZ), .DEBOUNCE_MS(DB_MS),
        .THR_DEFAULT(8'd12), .THR_MIN(8'd4),
        .THR_MAX(8'd48), .THR_STEP(8'd4)
    ) dut (
        .clk(clk), .rst_n(rst_n), .key_data(key_data),
        .uart_thr_valid_i(uart_thr_valid), .uart_thr_i(uart_thr),
        .uart_smooth_valid_i(uart_smooth_valid),
        .uart_smooth_i(uart_smooth),
        .threshold_o(threshold), .threshold_low_o(threshold_low),
        .smooth_mode_o(smooth_mode), .key_press_o(key_press)
    );

    task check;
        input [255:0] label;
        input [7:0] got;
        input [7:0] expected;
        begin
            if (got !== expected) begin
                errors = errors + 1;
                $display("FAIL %0s: got=%0d expected=%0d", label, got, expected);
            end
        end
    endtask

    task press_key;
        input integer index;
        begin
            key_data[index] = 1'b0;
            repeat (DB_CYC + 6) @(posedge clk);
            key_data[index] = 1'b1;
            repeat (DB_CYC + 6) @(posedge clk);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        check("reset threshold", threshold, 8'd12);
        check("reset smoothing", {6'd0, smooth_mode}, 8'd1);

        // A short contact bounce must not create an event.
        key_data[0] = 1'b0; repeat (DB_CYC / 2) @(posedge clk);
        key_data[0] = 1'b1; repeat (DB_CYC + 6) @(posedge clk);
        check("K1 bounce ignored", {6'd0, smooth_mode}, 8'd1);

        // K1: weak -> bypass -> standard -> weak.
        press_key(0); check("K1 weak to bypass", {6'd0, smooth_mode}, 8'd2);
        press_key(0); check("K1 bypass to standard", {6'd0, smooth_mode}, 8'd0);
        press_key(0); check("K1 standard to weak", {6'd0, smooth_mode}, 8'd1);
        check("K1 leaves threshold", threshold, 8'd12);

        // K2: 12 -> 16 and wraps 48 -> 4.
        press_key(1); check("K2 increments", threshold, 8'd16);
        for (i = 0; i < 8; i = i + 1) press_key(1); // 20..48
        check("K2 reaches max", threshold, 8'd48);
        press_key(1); check("K2 wraps max to min", threshold, 8'd4);
        check("threshold low tracks high", threshold_low, 8'd2);
        check("K2 leaves smoothing", {6'd0, smooth_mode}, 8'd1);

        // Holding a key generates only one press event.
        key_data[1] = 1'b0; repeat (6 * DB_CYC) @(posedge clk);
        key_data[1] = 1'b1; repeat (DB_CYC + 6) @(posedge clk);
        check("held K2 advances once", threshold, 8'd8);

        // UART commands keep priority and the next key press continues cycling.
        uart_thr = 8'd48; uart_thr_valid = 1'b1; @(posedge clk);
        uart_thr_valid = 1'b0; repeat (2) @(posedge clk);
        press_key(1); check("UART threshold then K2 wrap", threshold, 8'd4);

        uart_smooth = 2'd0; uart_smooth_valid = 1'b1; @(posedge clk);
        uart_smooth_valid = 1'b0; repeat (2) @(posedge clk);
        check("UART smoothing", {6'd0, smooth_mode}, 8'd0);
        press_key(0); check("UART smoothing then K1", {6'd0, smooth_mode}, 8'd1);

        if (errors == 0) $display("CANNY_THR_KEY PASS");
        else             $display("CANNY_THR_KEY FAIL: %0d errors", errors);
        $finish;
    end
endmodule
