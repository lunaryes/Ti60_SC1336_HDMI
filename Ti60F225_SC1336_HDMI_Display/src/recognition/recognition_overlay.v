`timescale 1ns/1ps
// Pixel-only annotations. All unannotated pixels, including blanking, are
// bit-for-bit passthrough. Neither this block nor OCR controls video timing.
module recognition_overlay #(
    parameter integer ROI_X0=320,ROI_Y0=120,ROI_W=640,ROI_H=480,
    parameter integer TEXT_X=24,TEXT_Y=24,SCALE=4,SHOW_LEGACY=1
)(
    input wire [10:0] x_i,input wire [9:0] y_i,input wire de_i,enable_i,
    input wire [23:0] rgb_i,
    input wire recognized_i,input wire [5:0] class_id_i,
    input wire [8:0] best_distance_i,
    input wire [255:0] glyph_i,template_i,
    input wire bbox_valid_i,
    input wire [10:0] bbox_xmin_i,bbox_xmax_i,
    input wire [9:0] bbox_ymin_i,bbox_ymax_i,
    input wire [7:0] canny_threshold_i,ocr_threshold_i,
    input wire [1:0] canny_smooth_i,
    input wire [3:0] reject_reason_i,
    output reg [23:0] rgb_o
);
    localparam integer TEMPLATE_X=TEXT_X+16*SCALE+12;
    wire valid_result=recognized_i && bbox_valid_i;
    wire roi_border=(((x_i==ROI_X0||x_i==ROI_X0+ROI_W-1)&&y_i>=ROI_Y0&&y_i<ROI_Y0+ROI_H)||
        ((y_i==ROI_Y0||y_i==ROI_Y0+ROI_H-1)&&x_i>=ROI_X0&&x_i<ROI_X0+ROI_W));
    wire bbox_border=bbox_valid_i &&
        (((x_i==bbox_xmin_i||x_i==bbox_xmax_i)&&y_i>=bbox_ymin_i&&y_i<=bbox_ymax_i)||
         ((y_i==bbox_ymin_i||y_i==bbox_ymax_i)&&x_i>=bbox_xmin_i&&x_i<=bbox_xmax_i));
    wire in_sample=x_i>=TEXT_X&&x_i<TEXT_X+16*SCALE&&y_i>=TEXT_Y&&y_i<TEXT_Y+16*SCALE;
    wire [3:0] sx=(x_i-TEXT_X)/SCALE,sy=(y_i-TEXT_Y)/SCALE;
    wire [7:0] sample_bit=8'd255-{sy,sx};
    wire in_template=x_i>=TEMPLATE_X&&x_i<TEMPLATE_X+16*SCALE&&y_i>=TEXT_Y&&y_i<TEXT_Y+16*SCALE;
    wire [3:0] tx=(x_i-TEMPLATE_X)/SCALE,ty=(y_i-TEXT_Y)/SCALE;
    wire [7:0] template_bit=8'd255-{ty,tx};
    // Two compact status rows. First row: Canny threshold (3 digits), smooth
    // mode (1 digit); second: OCR threshold (3 digits), reject code (1 digit).
    wire in_status=x_i>=TEXT_X&&x_i<TEXT_X+64&&y_i>=TEXT_Y+16*SCALE+4&&y_i<TEXT_Y+16*SCALE+24;
    wire [2:0] digit_position=(x_i-TEXT_X)/16;
    wire second_line=y_i>=TEXT_Y+16*SCALE+14;
    reg [7:0] status_value;
    reg [3:0] digit;
    always @(*) begin
        status_value=second_line?ocr_threshold_i:canny_threshold_i;
        case(digit_position)
            0: digit=status_value/100;
            1: digit=(status_value/10)%10;
            2: digit=status_value%10;
            default: digit=second_line?reject_reason_i:{2'b0,canny_smooth_i};
        endcase
    end
    wire [63:0] digit_bitmap;
    wire [7:0] unused_ascii;
    symbol_template_rom u_digits(.addr_i({2'b0,digit}),.bitmap_o(digit_bitmap),.ascii_o(unused_ascii));
    wire [2:0] dx=((x_i-TEXT_X)%16)/2;
    wire [2:0] dy=second_line ? y_i-(TEXT_Y+16*SCALE+14) : y_i-(TEXT_Y+16*SCALE+4);
    wire digit_row_valid=second_line ? y_i<TEXT_Y+16*SCALE+22 : y_i<TEXT_Y+16*SCALE+12;
    wire [5:0] digit_bit=6'd63-{dy,dx};
    always @(*) begin
        rgb_o=rgb_i;
        if(enable_i && de_i) begin
            if(SHOW_LEGACY&&roi_border) rgb_o=24'hff8000;
            if(SHOW_LEGACY&&bbox_border) rgb_o=valid_result?24'h00ff00:24'hffff00;
            if(SHOW_LEGACY&&in_sample) rgb_o=bbox_valid_i&&glyph_i[sample_bit]?24'hffffff:24'h202020;
            if(SHOW_LEGACY&&in_template) rgb_o=valid_result&&template_i[template_bit]?24'h00ff00:24'h202020;
            if(in_status) rgb_o=digit_row_valid&&digit_bitmap[digit_bit]?(second_line?24'hffff00:24'h00ffff):24'h202020;
        end
    end
endmodule
