`timescale 1ns/1ps
// Host-side serial waveforms verify OCR toggles do not affect Canny controls.
module tb_uart_ocr_enable;
    localparam integer CLK_HZ = 74250000;
    localparam integer BAUD = 460800;
    localparam integer DIV = (CLK_HZ + BAUD / 2) / BAUD;
    localparam real TCLK = 1000000000.0 / CLK_HZ;
    localparam real TBIT = 1000000000.0 / BAUD;
    reg clk = 0, rst_n = 0, rx = 1;
    always #(TCLK / 2.0) clk = ~clk;
    wire tx, thr_valid, smooth_valid, ocr_enable, hist_req, tx_busy;
    wire [7:0] thr, gain_r, gain_b;
    wire [1:0] view, mode, smooth;
    integer errors = 0, checks = 0, thr_writes = 0, smooth_writes = 0;

    uart_cmd #(.CLK_FREQ(CLK_HZ), .BAUD(BAUD), .THR_DEF(8'd12),
               .VIEW_DEF(2'd2), .MODE_DEF(2'd3)) dut (
        .clk(clk), .rst_n(rst_n), .rx_i(rx), .tx_o(tx),
        .thr_read_i(thr), .smooth_read_i(smooth),
        .thr_o(thr), .thr_valid_o(thr_valid), .view_o(view), .mode_o(mode),
        .smooth_o(smooth), .smooth_valid_o(smooth_valid),
        .gain_r_o(gain_r), .gain_b_o(gain_b), .ocr_enable_o(ocr_enable),
        .hist_req_o(hist_req), .tx_busy_o(tx_busy)
    );

    always @(posedge clk) begin
        if (rst_n && thr_valid) thr_writes = thr_writes + 1;
        if (rst_n && smooth_valid) smooth_writes = smooth_writes + 1;
    end

    task check8(input string label_text, input [7:0] got, input [7:0] expected);
        begin
            checks = checks + 1;
            if (got !== expected) begin
                errors = errors + 1;
                $display("FAIL %s got=%02h expected=%02h", label_text, got, expected);
            end
        end
    endtask

    task send_byte(input [7:0] data_byte);
        integer bit_number;
        begin
            rx = 0; #(TBIT);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                rx = data_byte[bit_number]; #(TBIT);
            end
            rx = 1; #(TBIT);
        end
    endtask

    task send_command(input [7:0] command_byte, input [7:0] data_byte);
        begin
            send_byte(command_byte);
            send_byte(data_byte);
            repeat (DIV) @(posedge clk);
        end
    endtask

    task check_controls(input [7:0] expected_thr, input [1:0] expected_smooth,
                        input [1:0] expected_view, input [1:0] expected_mode);
        begin
            check8("Canny threshold", thr, expected_thr);
            check8("Canny smoothing", {6'b0,smooth}, {6'b0,expected_smooth});
            check8("Canny view", {6'b0,view}, {6'b0,expected_view});
            check8("display mode", {6'b0,mode}, {6'b0,expected_mode});
            check8("red gain", gain_r, 8'd112);
            check8("blue gain", gain_b, 8'd96);
        end
    endtask

    // Start listening before the DUT can reply in the command stop bit.
    task request_status;
        integer bit_number;
        reg [7:0] request_byte;
        begin
            request_byte = 8'h52;
            rx = 0; #(TBIT);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                rx = request_byte[bit_number]; #(TBIT);
            end
            rx = 1;
        end
    endtask

    task receive_byte(output [7:0] data_byte);
        integer timeout_cycles, bit_number;
        begin
            timeout_cycles = 0;
            while (tx !== 0 && timeout_cycles < 20 * DIV) begin
                @(posedge clk);
                timeout_cycles = timeout_cycles + 1;
            end
            if (tx !== 0) begin
                errors = errors + 1;
                data_byte = 8'hxx;
                $display("FAIL status response timeout");
            end else begin
                repeat (DIV + DIV / 2 - 1) @(posedge clk);
                for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                    data_byte[bit_number] = tx;
                    repeat (DIV) @(posedge clk);
                end
            end
        end
    endtask

    reg [7:0] received [0:6];
    integer byte_number, guard_cycle;
    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);
        check8("OCR enabled after reset", {7'b0,ocr_enable}, 8'd1);
        check_controls(8'd12, 2'd1, 2'd2, 2'd3);

        send_command(8'h4f, 8'h00);
        check8("4F 00 disables OCR", {7'b0,ocr_enable}, 8'd0);
        check_controls(8'd12, 2'd1, 2'd2, 2'd3);
        check8("OCR command creates no threshold pulse", thr_writes[7:0], 0);
        check8("OCR command creates no smoothing pulse", smooth_writes[7:0], 0);

        send_command(8'h54, 8'd13);
        send_command(8'h53, 8'd2);
        send_command(8'h56, 8'd1);
        send_command(8'h4d, 8'd2);
        check8("Canny commands leave OCR disabled", {7'b0,ocr_enable}, 8'd0);
        check_controls(8'd13, 2'd2, 2'd1, 2'd2);

        send_command(8'h4f, 8'h01);
        check8("4F 01 enables OCR", {7'b0,ocr_enable}, 8'd1);
        check_controls(8'd13, 2'd2, 2'd1, 2'd2);

        send_command(8'h4f, 8'h00);
        check8("OCR disables again", {7'b0,ocr_enable}, 8'd0);
        check_controls(8'd13, 2'd2, 2'd1, 2'd2);
        request_status;
        for (byte_number = 0; byte_number < 7; byte_number = byte_number + 1)
            receive_byte(received[byte_number]);
        check8("R byte0 threshold", received[0], 8'd13);
        check8("R byte1 view", received[1], 8'd1);
        check8("R byte2 mode", received[2], 8'd2);
        check8("R byte3 red gain", received[3], 8'd112);
        check8("R byte4 blue gain", received[4], 8'd96);
        check8("R byte5 smoothing", received[5], 8'd2);
        check8("R byte6 newline", received[6], 8'h0a);
        repeat (2 * DIV) @(posedge clk);
        for (guard_cycle = 0; guard_cycle < 20 * DIV; guard_cycle = guard_cycle + 1) begin
            @(posedge clk);
            if (tx !== 1'b1 || tx_busy !== 1'b0) begin
                errors = errors + 1;
                $display("FAIL R sent data beyond its original seven bytes");
                guard_cycle = 20 * DIV;
            end
        end
        check8("R leaves OCR disabled", {7'b0,ocr_enable}, 8'd0);
        check8("only T command wrote threshold", thr_writes[7:0], 8'd1);
        check8("only S command wrote smoothing", smooth_writes[7:0], 8'd1);
        if (errors != 0) $fatal(1, "UART_OCR_ENABLE FAIL checks=%0d errors=%0d", checks, errors);
        $display("UART_OCR_ENABLE PASS checks=%0d errors=0", checks);
        $finish;
    end

    initial begin
        #10000000;
        $fatal(1, "UART_OCR_ENABLE FAIL timeout");
    end
endmodule
