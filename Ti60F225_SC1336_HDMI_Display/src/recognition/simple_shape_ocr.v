`timescale 1ns / 1ps

// Standalone wrapper: fixed-ROI sampler + 50-class matcher.
// Connect foreground_i to a thresholded grayscale stream for OCR/silhouettes;
// Canny edge_o may also be used for outline-only templates after retuning.
module simple_shape_ocr #(
    parameter integer IMG_W         = 1280,
    parameter integer ROI_X0        = 576,
    parameter integer ROI_Y0        = 296,
    parameter integer ROI_W         = 128,
    parameter integer ROI_H         = 128,
    parameter integer CELL_HITS_MIN = 4,
    parameter [6:0]   MAX_DISTANCE  = 7'd12,
    parameter [6:0]   MIN_MARGIN    = 7'd2,
    // 35 = digits 0-9 plus uppercase A-Z only.
    parameter [5:0]   LAST_TEMPLATE = 6'd35
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire        foreground_i,

    output wire [10:0] x_o,
    output wire [9:0]  y_o,
    output wire        result_valid_o,
    output wire        recognized_o,
    output wire [5:0]  class_id_o,
    output wire [7:0]  ascii_o,
    output wire [6:0]  best_distance_o,
    output wire [6:0]  second_distance_o
);
    wire [63:0] glyph;
    wire        glyph_valid;
    wire        matcher_busy;

    fixed_roi_glyph_sampler #(
        .IMG_W         (IMG_W),
        .ROI_X0        (ROI_X0),
        .ROI_Y0        (ROI_Y0),
        .ROI_W         (ROI_W),
        .ROI_H         (ROI_H),
        .CELL_HITS_MIN (CELL_HITS_MIN)
    ) u_sampler (
        .clk          (clk),
        .rst_n        (rst_n),
        .vsync_i      (vsync_i),
        .de_i         (de_i),
        .foreground_i (foreground_i),
        .glyph_o      (glyph),
        .glyph_valid_o(glyph_valid),
        .x_o          (x_o),
        .y_o          (y_o)
    );

    symbol_template_matcher #(
        .MAX_DISTANCE (MAX_DISTANCE),
        .MIN_MARGIN   (MIN_MARGIN),
        .LAST_TEMPLATE(LAST_TEMPLATE)
    ) u_matcher (
        .clk               (clk),
        .rst_n             (rst_n),
        .glyph_valid_i     (glyph_valid),
        .glyph_i           (glyph),
        .busy_o            (matcher_busy),
        .result_valid_o    (result_valid_o),
        .recognized_o      (recognized_o),
        .class_id_o        (class_id_o),
        .ascii_o           (ascii_o),
        .best_distance_o   (best_distance_o),
        .second_distance_o (second_distance_o)
    );
endmodule
