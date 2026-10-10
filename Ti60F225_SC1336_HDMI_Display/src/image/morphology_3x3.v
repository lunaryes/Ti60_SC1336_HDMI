`timescale 1ns / 1ps

// 3 x 3 binary morphology.
//
// p11..p33 are the 3 x 3 window supplied by pix_window_3x3.  Every pixel is
// made binary by comparing it with threshold_i.  operation_i selects:
//   1'b0 : erosion  -- output is 1 only when all nine input pixels are 1
//   1'b1 : dilation -- output is 1 when any one input pixel is 1
//
// For a Sobel edge image represented as 8'h00 / 8'hff, use threshold_i = 8'd128.
module morphology_3x3 (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vsync_i,
    input  wire       de_i,
    input  wire       operation_i,
    input  wire [7:0] threshold_i,

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
    output reg        binary_o
);

    // Convert the grayscale window to nine binary pixels.
    wire b11 = (p11 >= threshold_i);
    wire b12 = (p12 >= threshold_i);
    wire b13 = (p13 >= threshold_i);
    wire b21 = (p21 >= threshold_i);
    wire b22 = (p22 >= threshold_i);
    wire b23 = (p23 >= threshold_i);
    wire b31 = (p31 >= threshold_i);
    wire b32 = (p32 >= threshold_i);
    wire b33 = (p33 >= threshold_i);

    wire any_set = b11 | b12 | b13 |
                   b21 | b22 | b23 |
                   b31 | b32 | b33;

    wire all_set = b11 & b12 & b13 &
                   b21 & b22 & b23 &
                   b31 & b32 & b33;

    wire morphology_result = operation_i ? any_set : all_set;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_o  <= 1'b0;
            de_o     <= 1'b0;
            binary_o <= 1'b0;
        end else begin
            // Keep control timing aligned with the registered result.
            vsync_o  <= vsync_i;
            de_o     <= de_i;
            binary_o <= de_i ? morphology_result : 1'b0;
        end
    end

endmodule
