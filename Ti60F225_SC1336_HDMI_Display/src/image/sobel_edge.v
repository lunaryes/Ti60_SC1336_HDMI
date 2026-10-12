`timescale 1ns / 1ps

// 3x3 Sobel edge detector.
// The nine gray pixels are supplied by pix_window_3x3.  The module uses
// max(|Gx|, |Gy|) + min(|Gx|, |Gy|)/2 as a low-cost magnitude estimate.
// gray_edge_o overlays white edges on a contrast-enhanced grayscale image.
module sobel_edge #(
    // 0 keeps the original contrast, 1 doubles it, 2 quadruples it, etc.
    parameter integer CONTRAST_SHIFT = 1
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
    input  wire [7:0] thr_i,

    output reg        vsync_o,
    output reg        de_o,
    output reg  [7:0] mag_o,
    output reg        edge_o,
    output reg  [7:0] gray_edge_o
);

    // Extend each unsigned eight-bit pixel to a signed value before
    // subtraction.  Twelve bits cover the full Sobel range: -1020 to 1020.
    wire signed [11:0] p11_s = $signed({1'b0, p11});
    wire signed [11:0] p12_s = $signed({1'b0, p12});
    wire signed [11:0] p13_s = $signed({1'b0, p13});
    wire signed [11:0] p21_s = $signed({1'b0, p21});
    wire signed [11:0] p23_s = $signed({1'b0, p23});
    wire signed [11:0] p31_s = $signed({1'b0, p31});
    wire signed [11:0] p32_s = $signed({1'b0, p32});
    wire signed [11:0] p33_s = $signed({1'b0, p33});

    // p22 has a zero coefficient in both Sobel kernels.
    wire signed [11:0] gx =
          (p13_s - p11_s)
        + ((p23_s - p21_s) <<< 1)
        +  (p33_s - p31_s);

    wire signed [11:0] gy =
          (p11_s - p31_s)
        + ((p12_s - p32_s) <<< 1)
        +  (p13_s - p33_s);

    wire [11:0] abs_gx = gx[11] ? -gx : gx;
    wire [11:0] abs_gy = gy[11] ? -gy : gy;
    wire [11:0] grad_max = (abs_gx >= abs_gy) ? abs_gx : abs_gy;
    wire [11:0] grad_min = (abs_gx >= abs_gy) ? abs_gy : abs_gx;
    wire [12:0] mag_approx = {1'b0, grad_max} +
                             ({1'b0, grad_min} >> 1);

    // Keep an eight-bit result by clamping every value above 255 to 255.
    wire [7:0] mag_sat = (mag_approx > 13'd255) ? 8'hff
                                                : mag_approx[7:0];
    wire       edge_comb = (mag_sat > thr_i);

    // Contrast stretch around middle gray (128).  CONTRAST_SHIFT=1 gives:
    //   gray < 128 : 128 - 2 * (128 - gray)
    //   gray >=128 : 128 + 2 * (gray - 128)
    // Both sides are saturated to the legal 0..255 range.
    wire [15:0] gray_ext = {8'd0, p22};
    wire [15:0] contrast_delta = (p22 >= 8'd128)
                               ? (gray_ext - 16'd128)
                               : (16'd128 - gray_ext);
    wire [15:0] contrast_scaled = contrast_delta << CONTRAST_SHIFT;
    wire [15:0] contrast_high = 16'd128 + contrast_scaled;
    wire [7:0] contrast_gray = (p22 >= 8'd128)
                              ? ((contrast_high > 16'd255)
                                 ? 8'hff : contrast_high[7:0])
                              : ((contrast_scaled >= 16'd128)
                                 ? 8'h00
                                 : (8'd128 - contrast_scaled[7:0]));

    // A detected edge is always white; other pixels retain enhanced gray.
    wire [7:0] gray_edge_comb = edge_comb ? 8'hff : contrast_gray;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            mag_o   <= 8'd0;
            edge_o  <= 1'b0;
            gray_edge_o <= 8'd0;
        end else begin
            vsync_o <= vsync_i;
            de_o    <= de_i;
            mag_o   <= de_i ? mag_sat : 8'd0;
            edge_o  <= de_i ? edge_comb : 1'b0;
            gray_edge_o <= de_i ? gray_edge_comb : 8'd0;
        end
    end

endmodule
