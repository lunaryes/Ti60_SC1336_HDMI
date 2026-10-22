`timescale 1ns/1ps
// Single printed glyph on a reasonably uniform background. All pixel work is
// a read-only branch; no ready/stall or control signal returns to the video path.
module ocr_sampler16 #(
    parameter integer IMG_W=1280, ROI_X0=320, ROI_Y0=120, ROI_W=640, ROI_H=480
)(
    input wire clk, rst_n, vsync_i, de_i,
    input wire [7:0] gray_i,
    output reg frame_valid_o, candidate_valid_o,
    output reg [255:0] glyph_o,
    output reg [10:0] bbox_xmin_o, bbox_xmax_o, width_o,
    output reg [9:0] bbox_ymin_o, bbox_ymax_o, height_o,
    output reg [7:0] threshold_o,
    output reg dark_on_light_o,
    output reg [3:0] reject_reason_o
);
    localparam integer PIXELS=ROI_W*ROI_H;
    localparam integer TAIL_PIXELS=(PIXELS/2048 < 4) ? 4 : PIXELS/2048;
    reg [10:0] x;
    reg [9:0] y;
    reg vs_d, de_d;
    wire fs=vsync_i && !vs_d;
    wire in_roi=de_i && x>=ROI_X0 && x<ROI_X0+ROI_W && y>=ROI_Y0 && y<ROI_Y0+ROI_H;
    reg [18:0] histogram[0:15];
    reg [15:0] corner_sum[0:3];
    wire corner_left=x<ROI_X0+16;
    wire corner_right=x>=ROI_X0+ROI_W-16;
    wire corner_top=y<ROI_Y0+16;
    wire corner_bottom=y>=ROI_Y0+ROI_H-16;
    wire in_corner=in_roi && (corner_left||corner_right) && (corner_top||corner_bottom);
    wire [1:0] corner_idx={corner_bottom,corner_right};
    wire [7:0] c0=corner_sum[0][15:8], c1=corner_sum[1][15:8];
    wire [7:0] c2=corner_sum[2][15:8], c3=corner_sum[3][15:8];
    wire [7:0] cmin0=c0<c1?c0:c1, cmin1=c2<c3?c2:c3;
    wire [7:0] cmax0=c0>c1?c0:c1, cmax1=c2>c3?c2:c3;
    wire [7:0] corner_min=cmin0<cmin1?cmin0:cmin1;
    wire [7:0] corner_max=cmax0>cmax1?cmax0:cmax1;
    wire [9:0] corner_mean_sum={2'b0,c0}+{2'b0,c1}+{2'b0,c2}+{2'b0,c3};
    reg [7:0] saved_bg, saved_cmin, saved_cmax;
    reg [1:0] cal_state;
    reg [3:0] hist_idx;
    reg [19:0] cumulative;
    reg [7:0] low_level,high_level;
    reg low_found,high_found;
    reg calibration_valid;
    wire [19:0] next_cumulative=cumulative+{1'b0,histogram[hist_idx]};
    wire [8:0] level_sum={1'b0,low_level}+{1'b0,high_level};
    wire [7:0] next_threshold=level_sum[8:1];
    wire next_dark=saved_bg>=next_threshold;
    wire next_calibration_valid=(high_level>low_level) &&
        (high_level-low_level>=40) && (saved_cmax-saved_cmin<=48) &&
        (next_dark ? saved_cmin>next_threshold : saved_cmax<next_threshold);

    wire fg=in_roi && calibration_valid &&
        (dark_on_light_o ? gray_i<threshold_o : gray_i>threshold_o);
    reg [1:0] fg_run;
    reg found;
    reg [10:0] xmin,xmax;
    reg [9:0] ymin,ymax;
    reg [19:0] foreground_count;
    reg [7:0] border_count;
    reg [7:0] row_runs,max_row_runs;
    wire border_pixel=in_roi && (x<ROI_X0+8 || x>=ROI_X0+ROI_W-8 ||
                                 y<ROI_Y0+8 || y>=ROI_Y0+ROI_H-8);
    wire [10:0] box_w=xmax-xmin+11'd1;
    wire [9:0] box_h=ymax-ymin+10'd1;
    wire [20:0] box_area=box_w*box_h;
    wire box_size_ok=found && box_w>=3 && box_h>=24 &&
        box_w<=ROI_W-24 && box_h<=ROI_H-24 &&
        ({4'b0,box_w}<<4)>={5'b0,box_h} &&
        {1'b0,box_w}<({2'b0,box_h}<<1);
    wire border_ok=border_count<=8 && xmin>ROI_X0+8 && xmax<ROI_X0+ROI_W-9 &&
        ymin>ROI_Y0+8 && ymax<ROI_Y0+ROI_H-9;
    // Reject solid regions, sparse scenery and highly fragmented scan lines.
    wire density_ok=({4'b0,foreground_count}<<3)>=box_area &&
        ((({4'b0,foreground_count}<<2)<(box_area*3)) ||
         (({2'b0,box_w}<<2)<{3'b0,box_h})) && max_row_runs<=6;
    wire current_box_ok=calibration_valid && box_size_ok && border_ok && density_ok;
    reg active_valid;
    reg [10:0] ax0,ax1,aw;
    reg [9:0] ay0,ay1,ah;
    function near11;
        input [10:0] a,b;
        begin near11=(a>b ? a-b:b-a)<=11'd8; end
    endfunction
    wire motion_ok=near11(xmin,ax0)&&near11(xmax,ax1)&&
                   near11({1'b0,ymin},{1'b0,ay0})&&near11({1'b0,ymax},{1'b0,ay1});
    wire sample_ok=active_valid && current_box_ok && motion_ok;

    reg [10:0] xedge[1:15];
    reg [9:0] yedge[1:15];
    reg [3:0] bin_x,bin_y;
    integer b;
    always @(*) begin
        bin_x=0;bin_y=0;
        for(b=1;b<16;b=b+1) begin
            if(x>=xedge[b]) bin_x=b;
            if(y>=yedge[b]) bin_y=b;
        end
    end
    reg sample_de,sample_fg;
    reg [7:0] sample_index;
    reg [11:0] hits[0:255];
    reg [11:0] totals[0:255];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;vs_d<=0;de_d<=0;
            frame_valid_o<=0;candidate_valid_o<=0;glyph_o<=0;
            threshold_o<=128;dark_on_light_o<=1;reject_reason_o<=0;
            cal_state<=0;hist_idx<=0;cumulative<=0;low_level<=0;high_level<=0;
            low_found<=0;high_found<=0;calibration_valid<=0;
            saved_bg<=0;saved_cmin<=0;saved_cmax<=0;
            fg_run<=0;found<=0;xmin<=2047;xmax<=0;ymin<=1023;ymax<=0;
            foreground_count<=0;border_count<=0;row_runs<=0;max_row_runs<=0;
            active_valid<=0;ax0<=0;ax1<=0;ay0<=0;ay1<=0;aw<=0;ah<=0;
            bbox_xmin_o<=0;bbox_xmax_o<=0;bbox_ymin_o<=0;bbox_ymax_o<=0;
            width_o<=0;height_o<=0;sample_de<=0;sample_fg<=0;sample_index<=0;
            for(i=0;i<16;i=i+1) histogram[i]<=0;
            for(i=0;i<4;i=i+1) corner_sum[i]<=0;
            for(i=1;i<16;i=i+1) begin xedge[i]<=0;yedge[i]<=0;end
            for(i=0;i<256;i=i+1) begin hits[i]<=0;totals[i]<=0;end
        end else begin
            vs_d<=vsync_i;de_d<=de_i;frame_valid_o<=0;
            if(de_i) x<=(x==IMG_W-1)?11'd0:x+1'b1;else x<=0;
            if(de_d&&!de_i) begin
                y<=y+1'b1;row_runs<=0;
                if(row_runs>max_row_runs) max_row_runs<=row_runs;
            end
            if(in_roi) histogram[gray_i[7:4]]<=histogram[gray_i[7:4]]+1'b1;
            if(in_corner) corner_sum[corner_idx]<=corner_sum[corner_idx]+gray_i;
            if(fg) begin
                foreground_count<=foreground_count+1'b1;
                if(border_pixel && border_count!=255) border_count<=border_count+1'b1;
                if(fg_run!=3) fg_run<=fg_run+1'b1;
                // A three-pixel horizontal run rejects isolated hot pixels.
                if(fg_run==2 && row_runs!=255) row_runs<=row_runs+1'b1;
                if(fg_run>=2) begin
                    found<=1;
                    if(x-11'd2<xmin) xmin<=x-11'd2;
                    if(x>xmax) xmax<=x;
                    if(y<ymin) ymin<=y;
                    if(y>ymax) ymax<=y;
                end
            end else fg_run<=0;
            sample_de<=de_i && active_valid && x>=ax0 && x<=ax1 && y>=ay0 && y<=ay1;
            sample_fg<=fg;sample_index<={bin_y,bin_x};
            if(sample_de) begin
                if(totals[sample_index]!=12'hfff) totals[sample_index]<=totals[sample_index]+1'b1;
                if(sample_fg && hits[sample_index]!=12'hfff) hits[sample_index]<=hits[sample_index]+1'b1;
            end
            if(cal_state==1) begin
                cumulative<=next_cumulative;
                histogram[hist_idx]<=0;
                if(!low_found && next_cumulative>=TAIL_PIXELS) begin low_found<=1;low_level<={hist_idx,4'h8};end
                if(!high_found && next_cumulative>=PIXELS-TAIL_PIXELS) begin high_found<=1;high_level<={hist_idx,4'h8};end
                if(hist_idx==15) cal_state<=2;else hist_idx<=hist_idx+1'b1;
            end else if(cal_state==2) begin
                threshold_o<=next_threshold;dark_on_light_o<=next_dark;
                calibration_valid<=next_calibration_valid && low_found && high_found;
                cal_state<=0;
            end
            if(fs) begin
                x<=0;y<=0;sample_de<=0;
                // Always report a completed frame, including invalid/empty
                // frames, so a removed glyph cannot leave a stale result.
                frame_valid_o<=1;candidate_valid_o<=sample_ok;
                bbox_xmin_o<=ax0;bbox_xmax_o<=ax1;bbox_ymin_o<=ay0;bbox_ymax_o<=ay1;
                width_o<=aw;height_o<=ah;
                if(!calibration_valid) reject_reason_o<=1;
                else if(!border_ok) reject_reason_o<=2;
                else if(!box_size_ok) reject_reason_o<=3;
                else if(!density_ok) reject_reason_o<=4;
                else if(!motion_ok||!active_valid) reject_reason_o<=5;
                else reject_reason_o<=0;
                for(i=0;i<256;i=i+1) begin
                    // Explicit extension prevents the V2 left-shift overflow.
                    // A narrow solid sans-serif I has fewer than 16 source
                    // columns. Its continuous rectangle upscales to all ones;
                    // retain the original width/height for class rejection.
                    if(aw<16 && ({4'b0,foreground_count}<<3)>=box_area*7)
                        glyph_o[255-i]<=1'b1;
                    else
                        glyph_o[255-i]<=totals[i]!=0 && ({hits[i],2'b00}>={2'b0,totals[i]});
                    hits[i]<=0;totals[i]<=0;
                end
                active_valid<=current_box_ok;
                ax0<=xmin;ax1<=xmax;ay0<=ymin;ay1<=ymax;aw<=box_w;ah<=box_h;
                for(i=1;i<16;i=i+1) begin
                    xedge[i]<=xmin+((box_w*i+15)>>4);
                    yedge[i]<=ymin+((box_h*i+15)>>4);
                end
                found<=0;xmin<=2047;xmax<=0;ymin<=1023;ymax<=0;
                foreground_count<=0;border_count<=0;row_runs<=0;max_row_runs<=0;fg_run<=0;
                saved_bg<=corner_mean_sum[9:2];saved_cmin<=corner_min;saved_cmax<=corner_max;
                for(i=0;i<4;i=i+1) corner_sum[i]<=0;
                cal_state<=1;hist_idx<=0;cumulative<=0;low_found<=0;high_found<=0;
            end
        end
    end
endmodule
