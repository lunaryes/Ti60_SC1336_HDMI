`timescale 1ns / 1ps

// Converts a fixed rectangular binary ROI into an 8x8 occupancy bitmap.
// foreground_i must be 1 for the symbol/shape and 0 for the background.
// ROI_W and ROI_H must be positive multiples of 8.
module fixed_roi_glyph_sampler #(
    parameter integer IMG_W         = 1280,
    parameter integer ROI_X0        = 576,
    parameter integer ROI_Y0        = 296,
    parameter integer ROI_W         = 128,
    parameter integer ROI_H         = 128,
    parameter integer CELL_HITS_MIN = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync_i,
    input  wire        de_i,
    input  wire        foreground_i,

    output reg  [63:0] glyph_o,
    output reg         glyph_valid_o,
    output reg  [10:0] x_o,
    output reg  [9:0]  y_o
);
    localparam integer CELL_W = ROI_W / 8;
    localparam integer CELL_H = ROI_H / 8;
    localparam integer CELL_PIXELS = CELL_W * CELL_H;
    localparam integer HIT_W = (CELL_PIXELS <= 1) ? 1 : $clog2(CELL_PIXELS + 1);

    reg [HIT_W-1:0] cell_hits [0:63];
    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    reg        vsync_d;
    reg        de_d;
    reg        snapshot_pending;
    integer i;

    wire frame_start = vsync_i && !vsync_d;
    wire line_end    = de_d && !de_i;
    wire inside_roi  = de_i &&
                       (x_cnt >= ROI_X0) && (x_cnt < ROI_X0 + ROI_W) &&
                       (y_cnt >= ROI_Y0) && (y_cnt < ROI_Y0 + ROI_H);
    wire [10:0] roi_x = x_cnt - ROI_X0;
    wire [9:0]  roi_y = y_cnt - ROI_Y0;
    wire [2:0]  cell_x = roi_x / CELL_W;
    wire [2:0]  cell_y = roi_y / CELL_H;
    wire [5:0]  cell_index = {cell_y, 3'b000} + cell_x;

`ifndef SYNTHESIS
    initial begin
        if ((ROI_W <= 0) || (ROI_H <= 0) ||
            ((ROI_W % 8) != 0) || ((ROI_H % 8) != 0)) begin
            $error("fixed_roi_glyph_sampler: ROI_W/ROI_H must be positive multiples of 8");
        end
    end
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt            <= 11'd0;
            y_cnt            <= 10'd0;
            x_o              <= 11'd0;
            y_o              <= 10'd0;
            vsync_d          <= 1'b0;
            de_d             <= 1'b0;
            glyph_o          <= 64'd0;
            glyph_valid_o    <= 1'b0;
            snapshot_pending <= 1'b0;
            for (i = 0; i < 64; i = i + 1)
                cell_hits[i] <= {HIT_W{1'b0}};
        end else begin
            vsync_d       <= vsync_i;
            de_d          <= de_i;
            glyph_valid_o <= snapshot_pending;
            snapshot_pending <= 1'b0;

            x_o <= x_cnt;
            y_o <= y_cnt;

            // The project uses vsync_i as a frame-start pulse, not as a
            // frame-active level. Keep counting after that pulse and reset
            // the vertical coordinate only on the next frame_start.
            if (de_i) begin
                if (x_cnt == IMG_W-1)
                    x_cnt <= 11'd0;
                else
                    x_cnt <= x_cnt + 11'd1;
            end else begin
                x_cnt <= 11'd0;
            end

            if (line_end)
                y_cnt <= y_cnt + 10'd1;

            if (inside_roi && foreground_i &&
                (cell_hits[cell_index] != {HIT_W{1'b1}}))
                cell_hits[cell_index] <= cell_hits[cell_index] + 1'b1;

            // Snapshot the completed frame and clear counters for the new one.
            // glyph_valid_o is deliberately delayed one cycle so consumers see
            // the complete newly registered bitmap.
            if (frame_start) begin
                for (i = 0; i < 64; i = i + 1) begin
                    glyph_o[63-i] <= (cell_hits[i] >= CELL_HITS_MIN);
                    cell_hits[i]  <= {HIT_W{1'b0}};
                end
                snapshot_pending <= 1'b1;
                x_cnt <= 11'd0;
                y_cnt <= 10'd0;
            end
        end
    end
endmodule
