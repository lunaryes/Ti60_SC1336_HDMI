`timescale 1ns / 1ps

module rgb_to_gray (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        vs_i,
    input  wire        hs_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,

    output reg         vs_o,
    output reg         hs_o,
    output reg         de_o,
    output reg  [23:0] rgb_o
);

    wire [7:0] r_i = rgb_i[23:16];
    wire [7:0] g_i = rgb_i[15:8];
    wire [7:0] b_i = rgb_i[7:0];

    wire [15:0] y_sum =
        (r_i * 8'd77) +
        (g_i * 8'd150) +
        (b_i * 8'd29);

    wire [7:0] y = y_sum[15:8];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_o  <= 1'b0;
            hs_o  <= 1'b0;
            de_o  <= 1'b0;
            rgb_o <= 24'h000000;
        end else begin
            vs_o  <= vs_i;
            hs_o  <= hs_i;
            de_o  <= de_i;
            rgb_o <= de_i ? {y, y, y} : 24'h000000;
        end
    end

endmodule
