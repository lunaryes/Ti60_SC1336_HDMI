`timescale 1ns / 1ps

// Adaptive camera-oriented single-glyph sampler. It uses streaming frames:
// estimate background/polarity, locate a bounding box, then normalize it.
module adaptive_roi_glyph_sampler #(
    parameter integer IMG_W = 1280,
    parameter integer ROI_X0 = 544,
    parameter integer ROI_Y0 = 248,
    parameter integer ROI_W = 192,
    parameter integer ROI_H = 192,
    parameter integer CORNER_SIZE = 16,
    parameter integer MIN_CONTRAST = 32,
    parameter integer MIN_GLYPH_W = 16,
    parameter integer MIN_GLYPH_H = 24,
    parameter integer BORDER_GUARD = 3
)(
    input wire clk, input wire rst_n, input wire vsync_i, input wire de_i,
    input wire [7:0] gray_i,
    output reg [63:0] glyph_o, output reg glyph_valid_o,
    output reg [10:0] x_o, output reg [9:0] y_o,
    output reg bbox_valid_o,
    output reg [10:0] bbox_xmin_o, output reg [10:0] bbox_xmax_o,
    output reg [9:0] bbox_ymin_o, output reg [9:0] bbox_ymax_o,
    output reg [7:0] threshold_o, output reg dark_on_light_o,
    output reg [7:0] contrast_o
);
    localparam integer CORNER_PIXELS = 4 * CORNER_SIZE * CORNER_SIZE;
    localparam integer CORNER_SHIFT = $clog2(CORNER_PIXELS);

    reg [10:0] x_cnt; reg [9:0] y_cnt; reg vsync_d; reg de_d;
    reg [17:0] corner_sum; reg [7:0] roi_min; reg [7:0] roi_max;
    reg threshold_valid;
    reg bbox_found;
    reg [10:0] bbox_xmin_acc, bbox_xmax_acc;
    reg [9:0] bbox_ymin_acc, bbox_ymax_acc;
    reg bbox_active_valid;
    reg [10:0] bbox_xmin_active, bbox_xmax_active;
    reg [9:0] bbox_ymin_active, bbox_ymax_active;
    reg [11:0] cell_hits [0:63];
    reg [11:0] cell_total [0:63];
    integer i;

    wire frame_start = vsync_i && !vsync_d;
    wire line_end = de_d && !de_i;
    wire inside_roi = de_i && x_cnt >= ROI_X0 && x_cnt < ROI_X0 + ROI_W &&
                      y_cnt >= ROI_Y0 && y_cnt < ROI_Y0 + ROI_H;
    wire in_corner = inside_roi &&
        ((x_cnt < ROI_X0 + CORNER_SIZE || x_cnt >= ROI_X0 + ROI_W - CORNER_SIZE) &&
         (y_cnt < ROI_Y0 + CORNER_SIZE || y_cnt >= ROI_Y0 + ROI_H - CORNER_SIZE));

    wire [7:0] background_mean = corner_sum >> CORNER_SHIFT;
    wire [8:0] dark_span_w = background_mean > roi_min ? background_mean - roi_min : 9'd0;
    wire [8:0] bright_span_w = roi_max > background_mean ? roi_max - background_mean : 9'd0;
    wire next_dark = dark_span_w >= bright_span_w;
    wire [8:0] next_contrast_w = next_dark ? dark_span_w : bright_span_w;
    wire [7:0] half_contrast = next_contrast_w[8:1] < 8'd12 ? 8'd12 : next_contrast_w[8:1];
    wire [8:0] dark_thr_w = {1'b0,background_mean} - {1'b0,half_contrast};
    wire [8:0] bright_thr_w = {1'b0,background_mean} + {1'b0,half_contrast};
    wire [7:0] next_threshold = next_dark ? (dark_thr_w[8] ? 8'd0 : dark_thr_w[7:0]) :
                                           (bright_thr_w[8] ? 8'd255 : bright_thr_w[7:0]);
    wire foreground_now = inside_roi && threshold_valid &&
                          (dark_on_light_o ? gray_i < threshold_o : gray_i > threshold_o);
    wire bbox_guard_area = foreground_now &&
        x_cnt >= ROI_X0 + BORDER_GUARD && x_cnt < ROI_X0 + ROI_W - BORDER_GUARD &&
        y_cnt >= ROI_Y0 + BORDER_GUARD && y_cnt < ROI_Y0 + ROI_H - BORDER_GUARD;

    wire [10:0] bbox_width_acc = bbox_xmax_acc - bbox_xmin_acc + 1'b1;
    wire [9:0] bbox_height_acc = bbox_ymax_acc - bbox_ymin_acc + 1'b1;
    wire bbox_acc_valid = bbox_found && bbox_width_acc >= MIN_GLYPH_W &&
        bbox_height_acc >= MIN_GLYPH_H &&
        bbox_xmin_acc > ROI_X0 + BORDER_GUARD &&
        bbox_xmax_acc < ROI_X0 + ROI_W - BORDER_GUARD - 1 &&
        bbox_ymin_acc > ROI_Y0 + BORDER_GUARD &&
        bbox_ymax_acc < ROI_Y0 + ROI_H - BORDER_GUARD - 1;

    wire inside_active_bbox = de_i && bbox_active_valid &&
        x_cnt >= bbox_xmin_active && x_cnt <= bbox_xmax_active &&
        y_cnt >= bbox_ymin_active && y_cnt <= bbox_ymax_active;
    wire [10:0] active_width = bbox_xmax_active - bbox_xmin_active + 1'b1;
    wire [9:0] active_height = bbox_ymax_active - bbox_ymin_active + 1'b1;
    wire [10:0] active_x_pos = x_cnt - bbox_xmin_active;
    wire [9:0] active_y_pos = y_cnt - bbox_ymin_active;

    function [2:0] canonical_x;
        input [10:0] pos; input [10:0] span; reg [13:0] scaled;
        begin
            scaled = pos * 6;
            if (scaled < span) canonical_x=1; else if (scaled < span*2) canonical_x=2;
            else if (scaled < span*3) canonical_x=3; else if (scaled < span*4) canonical_x=4;
            else if (scaled < span*5) canonical_x=5; else canonical_x=6;
        end
    endfunction
    function [2:0] canonical_y;
        input [9:0] pos; input [9:0] span; reg [12:0] scaled;
        begin
            scaled = pos * 7;
            if (scaled < span) canonical_y=0; else if (scaled < span*2) canonical_y=1;
            else if (scaled < span*3) canonical_y=2; else if (scaled < span*4) canonical_y=3;
            else if (scaled < span*5) canonical_y=4; else if (scaled < span*6) canonical_y=5;
            else canonical_y=6;
        end
    endfunction
    wire [2:0] normalized_x = canonical_x(active_x_pos, active_width);
    wire [2:0] normalized_y = canonical_y(active_y_pos, active_height);
    wire [5:0] normalized_index = {normalized_y,3'b000} + normalized_x;

`ifndef SYNTHESIS
    initial if (CORNER_PIXELS != (1 << CORNER_SHIFT))
        $error("adaptive_roi_glyph_sampler: corner sample count must be power-of-two");
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt<=0; y_cnt<=0; x_o<=0; y_o<=0; vsync_d<=0; de_d<=0;
            corner_sum<=0; roi_min<=8'hff; roi_max<=0; threshold_o<=96;
            dark_on_light_o<=1; contrast_o<=0; threshold_valid<=0;
            bbox_found<=0; bbox_xmin_acc<=11'h7ff; bbox_xmax_acc<=0;
            bbox_ymin_acc<=10'h3ff; bbox_ymax_acc<=0;
            bbox_active_valid<=0; bbox_xmin_active<=0; bbox_xmax_active<=0;
            bbox_ymin_active<=0; bbox_ymax_active<=0;
            bbox_valid_o<=0; bbox_xmin_o<=0; bbox_xmax_o<=0;
            bbox_ymin_o<=0; bbox_ymax_o<=0; glyph_o<=0; glyph_valid_o<=0;
            for (i=0;i<64;i=i+1) begin cell_hits[i]<=0; cell_total[i]<=0; end
        end else begin
            vsync_d<=vsync_i; de_d<=de_i; glyph_valid_o<=0; x_o<=x_cnt; y_o<=y_cnt;
            if (de_i) x_cnt <= (x_cnt==IMG_W-1) ? 0 : x_cnt+1'b1; else x_cnt<=0;
            if (line_end) y_cnt<=y_cnt+1'b1;
            if (inside_roi) begin
                if (gray_i<roi_min) roi_min<=gray_i;
                if (gray_i>roi_max) roi_max<=gray_i;
                if (in_corner) corner_sum<=corner_sum+gray_i;
            end
            if (bbox_guard_area) begin
                if (!bbox_found) begin
                    bbox_found<=1; bbox_xmin_acc<=x_cnt; bbox_xmax_acc<=x_cnt;
                    bbox_ymin_acc<=y_cnt; bbox_ymax_acc<=y_cnt;
                end else begin
                    if (x_cnt<bbox_xmin_acc) bbox_xmin_acc<=x_cnt;
                    if (x_cnt>bbox_xmax_acc) bbox_xmax_acc<=x_cnt;
                    if (y_cnt<bbox_ymin_acc) bbox_ymin_acc<=y_cnt;
                    if (y_cnt>bbox_ymax_acc) bbox_ymax_acc<=y_cnt;
                end
            end
            if (inside_active_bbox) begin
                if (cell_total[normalized_index]!=12'hfff)
                    cell_total[normalized_index]<=cell_total[normalized_index]+1'b1;
                if (foreground_now && cell_hits[normalized_index]!=12'hfff)
                    cell_hits[normalized_index]<=cell_hits[normalized_index]+1'b1;
            end
            if (frame_start) begin
                for (i=0;i<64;i=i+1) begin
                    glyph_o[63-i] <= cell_total[i]!=0 && ((cell_hits[i]<<3)>=cell_total[i]);
                    cell_hits[i]<=0; cell_total[i]<=0;
                end
                glyph_valid_o<=bbox_active_valid; bbox_valid_o<=bbox_active_valid;
                bbox_xmin_o<=bbox_xmin_active; bbox_xmax_o<=bbox_xmax_active;
                bbox_ymin_o<=bbox_ymin_active; bbox_ymax_o<=bbox_ymax_active;
                bbox_active_valid<=bbox_acc_valid;
                bbox_xmin_active<=bbox_xmin_acc; bbox_xmax_active<=bbox_xmax_acc;
                bbox_ymin_active<=bbox_ymin_acc; bbox_ymax_active<=bbox_ymax_acc;
                threshold_o<=next_threshold; dark_on_light_o<=next_dark;
                contrast_o<=next_contrast_w[8] ? 8'hff : next_contrast_w[7:0];
                threshold_valid<=next_contrast_w>=MIN_CONTRAST;
                corner_sum<=0; roi_min<=8'hff; roi_max<=0; bbox_found<=0;
                bbox_xmin_acc<=11'h7ff; bbox_xmax_acc<=0;
                bbox_ymin_acc<=10'h3ff; bbox_ymax_acc<=0; x_cnt<=0; y_cnt<=0;
            end
        end
    end
endmodule
