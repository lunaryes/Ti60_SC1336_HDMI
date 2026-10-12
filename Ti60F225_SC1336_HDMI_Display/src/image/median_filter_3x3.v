`timescale 1ns / 1ps

// Edge-preserving 3x3 median filter.
//
// A reduced median network first sorts each row, then obtains the median of
// nine pixels from the row minima, medians and maxima. Compared with a full
// nine-phase sort this has a shorter combinational path and uses fewer
// compare/select operations.
//
// With IMPULSE_ONLY=1, the centre pixel is replaced only when it is a local
// extreme and differs sufficiently from the neighbourhood median. This
// removes salt-and-pepper noise without blurring every valid edge pixel.
module median_filter_3x3 #(
    parameter integer IMPULSE_ONLY  = 1,
    parameter [7:0]   IMPULSE_DELTA = 8'd24
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vsync_i,
    input  wire       de_i,
    input  wire [7:0] p11,
    input  wire [7:0] p12,
    input  wire [7:0] p13,
    input  wire [7:0] p21,
    input  wire [7:0] p22,
    input  wire [7:0] p23,
    input  wire [7:0] p31,
    input  wire [7:0] p32,
    input  wire [7:0] p33,

    output reg        vsync_o,
    output reg        de_o,
    output reg  [7:0] gray_o
);

    function [7:0] min3;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        reg   [7:0] ab_min;
        begin
            ab_min = (a < b) ? a : b;
            min3   = (ab_min < c) ? ab_min : c;
        end
    endfunction

    function [7:0] max3;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        reg   [7:0] ab_max;
        begin
            ab_max = (a > b) ? a : b;
            max3   = (ab_max > c) ? ab_max : c;
        end
    endfunction

    function [7:0] median3;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        begin
            if (a > b)
                median3 = (b > c) ? b : ((a > c) ? c : a);
            else
                median3 = (a > c) ? a : ((b > c) ? c : b);
        end
    endfunction

    wire [7:0] row1_min = min3(p11, p12, p13);
    wire [7:0] row2_min = min3(p21, p22, p23);
    wire [7:0] row3_min = min3(p31, p32, p33);
    wire [7:0] row1_mid = median3(p11, p12, p13);
    wire [7:0] row2_mid = median3(p21, p22, p23);
    wire [7:0] row3_mid = median3(p31, p32, p33);
    wire [7:0] row1_max = max3(p11, p12, p13);
    wire [7:0] row2_max = max3(p21, p22, p23);
    wire [7:0] row3_max = max3(p31, p32, p33);

    // Median-of-nine identity for three independently sorted rows.
    wire [7:0] lower_candidate = max3(row1_min, row2_min, row3_min);
    wire [7:0] middle_candidate = median3(row1_mid, row2_mid, row3_mid);
    wire [7:0] upper_candidate = min3(row1_max, row2_max, row3_max);
    wire [7:0] median_value = median3(lower_candidate,
                                      middle_candidate,
                                      upper_candidate);

    wire [7:0] local_min = min3(row1_min, row2_min, row3_min);
    wire [7:0] local_max = max3(row1_max, row2_max, row3_max);
    wire [7:0] centre_delta = (p22 >= median_value)
                            ? (p22 - median_value)
                            : (median_value - p22);
    wire centre_is_impulse = (((p22 == local_min) && (p22 < median_value)) ||
                              ((p22 == local_max) && (p22 > median_value))) &&
                             (centre_delta >= IMPULSE_DELTA);
    wire [7:0] filtered_pixel = (!IMPULSE_ONLY || centre_is_impulse)
                              ? median_value : p22;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            gray_o  <= 8'd0;
        end else begin
            vsync_o <= vsync_i;
            de_o    <= de_i;
            gray_o  <= de_i ? filtered_pixel : 8'd0;
        end
    end

endmodule
