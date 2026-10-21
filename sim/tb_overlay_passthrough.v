`timescale 1ns / 1ps

// Exhaustive 1280x720 overlay isolation regression.
// The overlay has no clocks/sync ports: only RGB may be annotated.
// Compile with recognition_overlay.v; use vvp to execute the result.
module tb_overlay_passthrough;
    localparam integer W=1280, H=720;
    localparam integer RX=320, RY=120, RW=640, RH=480;
    localparam integer TX=24, TY=24, SCALE=4;
    reg [10:0] x;
    reg [9:0] y;
    reg de, enable, recognized, bbox_valid;
    reg [23:0] rgb;
    wire [23:0] out_rgb;
    reg [255:0] glyph, template_bitmap;
    reg [8:0] best_distance;
    reg [10:0] xmin, xmax;
    reg [9:0] ymin, ymax;
    integer frame, xx, yy, blank;
    integer total_pixels, protected_pixels, changed_pixels;
    reg [31:0] rng;

    recognition_overlay #(
        .ROI_X0(RX), .ROI_Y0(RY), .ROI_W(RW), .ROI_H(RH),
        .TEXT_X(TX), .TEXT_Y(TY), .SCALE(SCALE)
    ) dut (
        .x_i(x), .y_i(y), .de_i(de), .enable_i(enable), .rgb_i(rgb),
        .recognized_i(recognized), .class_id_i(6'd5), .best_distance_i(best_distance),
        .glyph_i(glyph), .template_i(template_bitmap),
        .bbox_valid_i(bbox_valid),
        .bbox_xmin_i(xmin), .bbox_xmax_i(xmax),
        .bbox_ymin_i(ymin), .bbox_ymax_i(ymax),
        .canny_threshold_i(8'd12), .canny_smooth_i(2'd1),
        .ocr_threshold_i(8'd96), .reject_reason_i(4'd0),
        .rgb_o(out_rgb)
    );

    function annotation_allowed;
        input integer px, py;
        reg roi_border, candidate_border, sample_icon, template_icon, status_area;
        begin
            roi_border = (((px==RX || px==RX+RW-1) && py>=RY && py<RY+RH) ||
                          ((py==RY || py==RY+RH-1) && px>=RX && px<RX+RW));
            candidate_border = bbox_valid &&
                (((px==xmin || px==xmax) && py>=ymin && py<=ymax) ||
                 ((py==ymin || py==ymax) && px>=xmin && px<=xmax));
            sample_icon = px>=TX && px<TX+64 && py>=TY && py<TY+64;
            template_icon = px>=TX+76 && px<TX+140 && py>=TY && py<TY+64;
            status_area = px>=TX && px<TX+160 && py>=TY+64 && py<TY+96;
            annotation_allowed = roi_border || candidate_border || sample_icon ||
                                 template_icon || status_area;
        end
    endfunction

    task randomize_rgb;
        begin
            rng = {rng[30:0], rng[31]^rng[21]^rng[1]^rng[0]};
            rgb = rng[23:0];
        end
    endtask

    task expect_equal;
        begin
            if (out_rgb !== rgb) begin
                $display("FAIL frame=%0d x=%0d y=%0d de=%b enable=%b expected=%h actual=%h",
                         frame,x,y,de,enable,rgb,out_rgb);
                $fatal(1,"Overlay modified a protected pixel");
            end
            protected_pixels = protected_pixels+1;
        end
    endtask

    initial begin
        x=0; y=0; de=0; enable=0; rgb=0; rng=32'h3279cb51;
        recognized=0; bbox_valid=0; best_distance=9'd256;
        glyph={16{16'h0f0f}};
        template_bitmap=~glyph;
        xmin=11'd420; xmax=11'd700; ymin=10'd170; ymax=10'd540;
        total_pixels=0; protected_pixels=0; changed_pixels=0;

        // 0: no candidate, 1: rejected candidate, 2: accepted candidate.
        for(frame=0;frame<3;frame=frame+1) begin
            bbox_valid=(frame!=0);
            recognized=(frame==2);
            best_distance=frame==2 ? 9'd10 : 9'd200;
            for(yy=0;yy<H;yy=yy+1) begin
                y=yy;
                for(xx=0;xx<W;xx=xx+1) begin
                    x=xx; de=1; enable=0; randomize_rgb;
                    #1;
                    expect_equal;
                    enable=1;
                    #1;
                    if(!annotation_allowed(xx,yy)) expect_equal;
                    else if(out_rgb !== rgb) changed_pixels=changed_pixels+1;
                    if(^out_rgb===1'bx) $fatal(1,"Unknown RGB output");
                    total_pixels=total_pixels+1;
                end
                // Valid RGB values during blanking must also pass unchanged,
                // including coordinates inside every annotation region.
                for(blank=0;blank<4;blank=blank+1) begin
                    de=0; enable=blank[0];
                    case(blank)
                        0: begin x=RX; y=RY; end
                        1: begin x=TX; y=TY; end
                        2: begin x=xmin; y=ymin; end
                        3: begin x=TX; y=TY+80; end
                    endcase
                    randomize_rgb;
                    #1;
                    expect_equal;
                end
                y=yy;
            end
        end
        // A stale classifier result must not draw a green template after the
        // sampler has invalidated the character bounding box.
        bbox_valid=0; recognized=1; template_bitmap={256{1'b1}};
        de=1; enable=1; x=TX+76; y=TY; rgb=24'h123456;
        #1;
        if(out_rgb!==24'h202020) $fatal(1,"Stale recognized result still rendered");
        x=xmin; y=ymin;
        #1;
        expect_equal;
        if(changed_pixels<1000) $fatal(1,"Vacuous pass: annotations not exercised");
        $display("OVERLAY PASSTHROUGH PASS: frames=3 active_pixels=%0d protected_checks=%0d annotated_pixels=%0d",
                 total_pixels,protected_pixels,changed_pixels);
        $finish;
    end
endmodule
