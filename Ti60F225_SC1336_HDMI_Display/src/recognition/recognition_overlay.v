`timescale 1ns / 1ps

// Draws the fixed ROI, recognized 8x8 symbol (scaled 4x), and confidence bar.
// This is a pure pixel overlay and adds no cycle of latency.
module recognition_overlay #(
    parameter integer ROI_X0 = 576,
    parameter integer ROI_Y0 = 296,
    parameter integer ROI_W  = 128,
    parameter integer ROI_H  = 128,
    parameter integer TEXT_X = 24,
    parameter integer TEXT_Y = 24,
    parameter integer SCALE  = 4
)(
    input  wire [10:0] x_i,
    input  wire [9:0]  y_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    input  wire        recognized_i,
    input  wire [5:0]  class_id_i,
    input  wire [6:0]  best_distance_i,
    output reg  [23:0] rgb_o
);
    wire [5:0] rom_addr = (class_id_i == 0) ? 6'd0 : class_id_i - 6'd1;
    wire [63:0] class_bitmap;
    wire [7:0]  unused_ascii;
    symbol_template_rom u_overlay_rom (
        .addr_i   (rom_addr),
        .bitmap_o (class_bitmap),
        .ascii_o  (unused_ascii)
    );

    wire roi_border = recognized_i &&
        ((((x_i == ROI_X0) || (x_i == ROI_X0 + ROI_W - 1)) &&
          (y_i >= ROI_Y0) && (y_i < ROI_Y0 + ROI_H)) ||
         (((y_i == ROI_Y0) || (y_i == ROI_Y0 + ROI_H - 1)) &&
          (x_i >= ROI_X0) && (x_i < ROI_X0 + ROI_W)));

    wire in_icon = recognized_i &&
                   (x_i >= TEXT_X) && (x_i < TEXT_X + 8*SCALE) &&
                   (y_i >= TEXT_Y) && (y_i < TEXT_Y + 8*SCALE);
    wire [2:0] icon_x = (x_i - TEXT_X) / SCALE;
    wire [2:0] icon_y = (y_i - TEXT_Y) / SCALE;
    wire [6:0] icon_bit_index = 7'd63 - ({4'd0, icon_y, 3'b000} + icon_x);
    wire icon_pixel = in_icon && class_bitmap[icon_bit_index];

    // Distance 0 produces a full 64-pixel bar. Distances >=32 show no bar.
    wire [6:0] quality = (best_distance_i >= 7'd32) ? 7'd0 :
                         (7'd64 - {best_distance_i[5:0], 1'b0});
    wire confidence_bar = recognized_i &&
                          (x_i >= TEXT_X) && (x_i < TEXT_X + quality) &&
                          (y_i >= TEXT_Y + 8*SCALE + 4) &&
                          (y_i <  TEXT_Y + 8*SCALE + 8);

    always @(*) begin
        rgb_o = de_i ? rgb_i : 24'h000000;
        if (de_i) begin
            if (roi_border)
                rgb_o = 24'h00FF00;
            if (in_icon)
                rgb_o = icon_pixel ? 24'hFFFFFF : 24'h202020;
            if (confidence_bar)
                rgb_o = 24'h00FF00;
        end
    end
endmodule
