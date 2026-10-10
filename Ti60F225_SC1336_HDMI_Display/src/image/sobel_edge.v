`timescale 1ns / 1ps

// 3x3 Sobel edge detector.
// The nine gray pixels are supplied by pix_window_3x3.  The module uses
// |Gx| + |Gy| as the gradient magnitude and saturates it to eight bits.
module sobel_edge (
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
    output reg        edge_o
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
    wire [12:0] mag_sum = {1'b0, abs_gx} + {1'b0, abs_gy};

    // The exact Sobel magnitude can reach 2040.  Keep an eight-bit result
    // by clamping every value above 255 to 255.
    wire [7:0] mag_sat = (mag_sum[12:8] != 5'd0) ? 8'hff : mag_sum[7:0];
    wire       edge_comb = (mag_sat > thr_i);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            mag_o   <= 8'd0;
            edge_o  <= 1'b0;
        end else begin
            vsync_o <= vsync_i;
            de_o    <= de_i;
            mag_o   <= de_i ? mag_sat : 8'd0;
            edge_o  <= de_i ? edge_comb : 1'b0;
        end
    end

endmodule
