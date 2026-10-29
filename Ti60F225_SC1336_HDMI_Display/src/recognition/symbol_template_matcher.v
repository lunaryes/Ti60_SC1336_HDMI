`timescale 1ns / 1ps

// Sequential 50-class Hamming-distance matcher.
// One recognition takes 51 clocks and uses one shared popcount datapath.
module symbol_template_matcher #(
    parameter [6:0] MAX_DISTANCE = 7'd12,
    parameter [6:0] MIN_MARGIN   = 7'd2,
    // ROM address of the final class to scan. 49 keeps all 50 classes;
    // 35 limits matching to digits 0-9 and uppercase letters A-Z.
    parameter [5:0] LAST_TEMPLATE = 6'd49
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        glyph_valid_i,
    input  wire [63:0] glyph_i,

    output reg         busy_o,
    output reg         result_valid_o,
    output reg         recognized_o,
    output reg  [5:0]  class_id_o,
    output reg  [7:0]  ascii_o,
    output reg  [6:0]  best_distance_o,
    output reg  [6:0]  second_distance_o
);
    reg [63:0] glyph_latched;
    reg [5:0]  scan_addr;
    reg [5:0]  best_addr;
    reg [6:0]  best_dist;
    reg [6:0]  second_dist;

    wire [63:0] template_bitmap;
    wire [7:0]  template_ascii;
    wire [63:0] difference = glyph_latched ^ template_bitmap;

    symbol_template_rom u_template_rom (
        .addr_i   (scan_addr),
        .bitmap_o (template_bitmap),
        .ascii_o  (template_ascii)
    );

    function [6:0] popcount64;
        input [63:0] value;
        integer k;
        begin
            popcount64 = 7'd0;
            for (k = 0; k < 64; k = k + 1)
                popcount64 = popcount64 + value[k];
        end
    endfunction

    wire [6:0] current_dist = popcount64(difference);
    wire       current_best = (current_dist < best_dist);
    wire [6:0] final_best_dist = current_best ? current_dist : best_dist;
    wire [5:0] final_best_addr = current_best ? scan_addr : best_addr;
    wire [6:0] final_second_dist = current_best ? best_dist :
                                          ((current_dist < second_dist) ? current_dist : second_dist);
    wire [6:0] final_margin = final_second_dist - final_best_dist;

    // A second ROM lookup provides the ASCII/tag for the winning address.
    wire [63:0] unused_winner_bitmap;
    wire [7:0]  winner_ascii;
    symbol_template_rom u_winner_rom (
        .addr_i   (final_best_addr),
        .bitmap_o (unused_winner_bitmap),
        .ascii_o  (winner_ascii)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            glyph_latched    <= 64'd0;
            scan_addr        <= 6'd0;
            best_addr        <= 6'd0;
            best_dist        <= 7'd64;
            second_dist      <= 7'd64;
            busy_o           <= 1'b0;
            result_valid_o   <= 1'b0;
            recognized_o     <= 1'b0;
            class_id_o       <= 6'd0;
            ascii_o          <= 8'h00;
            best_distance_o  <= 7'd64;
            second_distance_o <= 7'd64;
        end else begin
            result_valid_o <= 1'b0;

            if (glyph_valid_i && !busy_o) begin
                glyph_latched <= glyph_i;
                scan_addr     <= 6'd0;
                best_addr     <= 6'd0;
                best_dist     <= 7'd64;
                second_dist   <= 7'd64;
                busy_o        <= 1'b1;
            end else if (busy_o) begin
                if (current_best) begin
                    second_dist <= best_dist;
                    best_dist   <= current_dist;
                    best_addr   <= scan_addr;
                end else if (current_dist < second_dist) begin
                    second_dist <= current_dist;
                end

                if (scan_addr == LAST_TEMPLATE) begin
                    busy_o            <= 1'b0;
                    result_valid_o    <= 1'b1;
                    best_distance_o   <= final_best_dist;
                    second_distance_o <= final_second_dist;
                    recognized_o      <= (final_best_dist <= MAX_DISTANCE) &&
                                         (final_margin >= MIN_MARGIN);
                    class_id_o        <= ((final_best_dist <= MAX_DISTANCE) &&
                                          (final_margin >= MIN_MARGIN))
                                         ? (final_best_addr + 6'd1) : 6'd0;
                    ascii_o           <= ((final_best_dist <= MAX_DISTANCE) &&
                                          (final_margin >= MIN_MARGIN))
                                         ? winner_ascii : "?";
                end else begin
                    scan_addr <= scan_addr + 6'd1;
                end
            end
        end
    end
endmodule
