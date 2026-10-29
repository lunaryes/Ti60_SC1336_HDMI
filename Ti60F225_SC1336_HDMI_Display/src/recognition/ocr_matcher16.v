`timescale 1ns/1ps
// One 16-bit popcount reused for 256-bit templates. The second score is
// explicitly a DIFFERENT CLASS, not another font of the winning class.
module ocr_matcher16 #(
    parameter [8:0] MAX_DISTANCE=9'd40, MIN_MARGIN=9'd10
)(
    input wire clk,rst_n,start_i,candidate_valid_i,
    input wire [255:0] glyph_i,
    input wire [10:0] width_i,
    input wire [9:0] height_i,
    output reg result_valid_o,recognized_o,
    output reg [5:0] class_id_o,
    output reg [7:0] ascii_o,
    output reg [8:0] best_distance_o,second_distance_o,
    output reg [255:0] template_o
);
    localparam IDLE=0,SCAN=1,FINISH_TEMPLATE=2,FINISH_CLASS=3,DONE=4;
    reg [2:0] state;
    reg [8:0] address;
    reg [3:0] word_index;
    wire [255:0] rom_bitmap;
    wire [5:0] rom_class;
    wire [7:0] rom_ascii,rom_aspect;
    ocr_font16_rom u_rom(.addr_i(address),.bitmap_o(rom_bitmap),.class_o(rom_class),
                         .ascii_o(rom_ascii),.aspect_o(rom_aspect));
    reg [255:0] glyph;
    reg [10:0] width;
    reg [9:0] height;
    reg [8:0] distance,intersection_count,union_count;
    reg [8:0] class_best,best,second;
    reg [255:0] class_template,best_template;
    reg [5:0] best_class;
    reg [7:0] best_ascii;
    wire [255:0] diff=glyph^rom_bitmap;
    wire [255:0] both=glyph&rom_bitmap;
    wire [255:0] either=glyph|rom_bitmap;
    function [4:0] pop16;
        input [15:0] value;
        integer b;
        begin pop16=0;for(b=0;b<16;b=b+1) pop16=pop16+value[b];end
    endfunction
    wire [18:0] aspect_observed={width,7'b0};
    wire [18:0] aspect_reference=rom_aspect*height;
    wire aspect_ok=(aspect_observed*3>=aspect_reference*2) &&
                   (aspect_observed*2<=aspect_reference*3);
    wire overlap_ok=({1'b0,intersection_count}<<1)>={1'b0,union_count};
    wire [8:0] template_distance=aspect_ok&&overlap_ok ? distance:9'd256;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;address<=0;word_index<=0;glyph<=0;width<=0;height<=0;
            distance<=0;intersection_count<=0;union_count<=0;
            class_best<=256;best<=256;second<=256;class_template<=0;best_template<=0;
            best_class<=0;best_ascii<=0;template_o<=0;
            result_valid_o<=0;recognized_o<=0;class_id_o<=0;ascii_o<="?";
            best_distance_o<=256;second_distance_o<=256;
        end else begin
            result_valid_o<=0;
            // start_i arrives on every completed frame. Invalid input cancels
            // work/results, avoiding late old matches after a glyph disappears.
            if(start_i) begin
                recognized_o<=0;class_id_o<=0;ascii_o<="?";template_o<=0;
                if(!candidate_valid_i) begin
                    state<=IDLE;result_valid_o<=1;best_distance_o<=256;second_distance_o<=256;
                end else begin
                    glyph<=glyph_i;width<=width_i;height<=height_i;
                    address<=0;word_index<=0;distance<=0;intersection_count<=0;union_count<=0;
                    class_best<=256;best<=256;second<=256;state<=SCAN;
                end
            end else case(state)
                SCAN: begin
                    distance<=distance+pop16(diff[word_index*16 +: 16]);
                    intersection_count<=intersection_count+pop16(both[word_index*16 +: 16]);
                    union_count<=union_count+pop16(either[word_index*16 +: 16]);
                    if(word_index==15) state<=FINISH_TEMPLATE;else word_index<=word_index+1'b1;
                end
                FINISH_TEMPLATE: begin
                    if(template_distance<class_best) begin
                        class_best<=template_distance;class_template<=rom_bitmap;
                    end
                    if(address[2:0]==7) state<=FINISH_CLASS;
                    else begin
                        address<=address+1'b1;word_index<=0;distance<=0;
                        intersection_count<=0;union_count<=0;state<=SCAN;
                    end
                end
                FINISH_CLASS: begin
                    if(class_best<best) begin
                        second<=best;best<=class_best;best_class<=rom_class;
                        best_ascii<=rom_ascii;best_template<=class_template;
                    end else if(class_best<second) second<=class_best;
                    if(address==287) state<=DONE;
                    else begin
                        address<=address+1'b1;word_index<=0;distance<=0;
                        intersection_count<=0;union_count<=0;class_best<=256;state<=SCAN;
                    end
                end
                DONE: begin
                    result_valid_o<=1;best_distance_o<=best;second_distance_o<=second;
                    recognized_o<=best<=MAX_DISTANCE && second-best>=MIN_MARGIN;
                    class_id_o<=(best<=MAX_DISTANCE && second-best>=MIN_MARGIN)?best_class:0;
                    ascii_o<=(best<=MAX_DISTANCE && second-best>=MIN_MARGIN)?best_ascii:"?";
                    template_o<=best_template;state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
