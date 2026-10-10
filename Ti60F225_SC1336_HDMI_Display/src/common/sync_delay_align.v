`timescale 1ns / 1ps
//=====================================================================
// sync_delay_align  —— 通用延迟对齐
//---------------------------------------------------------------------
// 把 vsync / de / 数据整体延迟 DEPTH 个 clk，用于把旁路（原图）路径与
// 算法路径打齐，保证进 edge_display_mix 的 orig_rgb_i 与 edge_rgb_i 严格同拍。
//
// 【为什么需要它】
//   旁路路径与算法路径的延迟天然不同（算法路径多了 Sobel 的 4 级流水），
//   直接用一个 mode MUX 选通会得到错位的画面。正确做法是让两条路径的延迟
//   完全相同，再进 MUX。
//
// 【用法】
//   DEPTH = 算法路径相对旁路路径多出的拍数。
//   本工程中 DEPTH = sobel_edge 的 LATENCY = 4。
//
// 【注意】只做寄存器延迟，不引入新的时钟域。
//=====================================================================
module sync_delay_align #(
    parameter DEPTH = 4,
    parameter DW    = 24
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          vsync_i,
    input  wire          de_i,
    input  wire [DW-1:0] data_i,

    output wire          vsync_o,
    output wire          de_o,
    output wire [DW-1:0] data_o
);

    reg [DW-1:0] sr_data [0:DEPTH-1];
    reg          sr_vs   [0:DEPTH-1];
    reg          sr_de   [0:DEPTH-1];

    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < DEPTH; k = k + 1) begin
                sr_data[k] <= {DW{1'b0}};
                sr_vs[k]   <= 1'b0;
                sr_de[k]   <= 1'b0;
            end
        end else begin
            sr_data[0] <= data_i;
            sr_vs[0]   <= vsync_i;
            sr_de[0]   <= de_i;

            for (k = 1; k < DEPTH; k = k + 1) begin
                sr_data[k] <= sr_data[k-1];
                sr_vs[k]   <= sr_vs[k-1];
                sr_de[k]   <= sr_de[k-1];
            end
        end
    end

    assign data_o  = sr_data[DEPTH-1];
    assign vsync_o = sr_vs[DEPTH-1];
    assign de_o    = sr_de[DEPTH-1];

endmodule
