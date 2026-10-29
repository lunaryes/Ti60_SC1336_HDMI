`timescale 1ns/1ps
// V3: explicit invalid-frame clearing, 16x16 camera descriptor, multi-font
// different-class matching and three-frame confirmation. Canny is independent.
module simple_shape_ocr #(
    parameter integer IMG_W=1280,ROI_X0=320,ROI_Y0=120,ROI_W=640,ROI_H=480
)(
    input wire clk,rst_n,vsync_i,de_i,
    input wire [7:0] gray_i,
    output reg result_valid_o,recognized_o,
    output reg [5:0] class_id_o,
    output reg [7:0] ascii_o,
    output wire [8:0] best_distance_o,second_distance_o,
    output wire [255:0] glyph_o,
    output reg [255:0] template_o,
    output wire bbox_valid_o,
    output wire [10:0] bbox_xmin_o,bbox_xmax_o,
    output wire [9:0] bbox_ymin_o,bbox_ymax_o,
    output wire [7:0] threshold_o,
    output wire dark_on_light_o,
    output wire [3:0] reject_reason_o
);
    wire frame_valid,candidate_valid,raw_valid,raw_recognized;
    wire [10:0] width;
    wire [9:0] height;
    wire [5:0] raw_class;
    wire [7:0] raw_ascii;
    wire [255:0] raw_template;
    assign bbox_valid_o=candidate_valid;
    ocr_sampler16 #(.IMG_W(IMG_W),.ROI_X0(ROI_X0),.ROI_Y0(ROI_Y0),.ROI_W(ROI_W),.ROI_H(ROI_H)) u_sampler(
        .clk(clk),.rst_n(rst_n),.vsync_i(vsync_i),.de_i(de_i),.gray_i(gray_i),
        .frame_valid_o(frame_valid),.candidate_valid_o(candidate_valid),.glyph_o(glyph_o),
        .bbox_xmin_o(bbox_xmin_o),.bbox_xmax_o(bbox_xmax_o),.bbox_ymin_o(bbox_ymin_o),
        .bbox_ymax_o(bbox_ymax_o),.width_o(width),.height_o(height),
        .threshold_o(threshold_o),.dark_on_light_o(dark_on_light_o),.reject_reason_o(reject_reason_o));
    ocr_matcher16 u_matcher(.clk(clk),.rst_n(rst_n),.start_i(frame_valid),
        .candidate_valid_i(candidate_valid),.glyph_i(glyph_o),.width_i(width),.height_i(height),
        .result_valid_o(raw_valid),.recognized_o(raw_recognized),.class_id_o(raw_class),
        .ascii_o(raw_ascii),.best_distance_o(best_distance_o),.second_distance_o(second_distance_o),
        .template_o(raw_template));
    reg [7:0] pending_ascii;
    reg [1:0] confirmations;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pending_ascii<=0;confirmations<=0;recognized_o<=0;result_valid_o<=0;
            class_id_o<=0;ascii_o<="?";template_o<=0;
        end else begin
            result_valid_o<=0;
            if(frame_valid && !candidate_valid) begin
                pending_ascii<=0;confirmations<=0;recognized_o<=0;
                class_id_o<=0;ascii_o<="?";template_o<=0;
            end else if(raw_valid) begin
                result_valid_o<=1;
                if(!raw_recognized || !candidate_valid) begin
                    pending_ascii<=0;confirmations<=0;recognized_o<=0;
                    class_id_o<=0;ascii_o<="?";template_o<=0;
                end else if(raw_ascii!=pending_ascii) begin
                    pending_ascii<=raw_ascii;confirmations<=1;recognized_o<=0;
                    class_id_o<=0;ascii_o<="?";template_o<=0;
                end else begin
                    if(confirmations<3) confirmations<=confirmations+1'b1;
                    if(confirmations>=2) begin
                        recognized_o<=1;class_id_o<=raw_class;ascii_o<=raw_ascii;template_o<=raw_template;
                    end
                end
            end
        end
    end
endmodule
