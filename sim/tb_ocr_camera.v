`timescale 1ns/1ps
module tb_ocr_camera;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,vs=0,de=0;reg[7:0] gray=255;
    wire recognized,bbox_valid,result_valid;
    wire[7:0] ascii;wire[8:0] best,second;
    wire[3:0] reject_reason;
    wire[7:0] threshold;
    wire dark;
    simple_shape_ocr #(.IMG_W(256),.ROI_X0(32),.ROI_Y0(16),.ROI_W(192),.ROI_H(160)) dut(
        .clk(clk),.rst_n(rst_n),.vsync_i(vs),.de_i(de),.gray_i(gray),
        .result_valid_o(result_valid),.recognized_o(recognized),.ascii_o(ascii),
        .best_distance_o(best),.second_distance_o(second),.bbox_valid_o(bbox_valid),
        .threshold_o(threshold),.dark_on_light_o(dark),.reject_reason_o(reject_reason));
    reg[7:0] patch[0:16383];
    reg[2047:0] filename;
    integer expected=52,bg=255,frame,row,col,offsetx=64,offsety=32;
    task tick;
        input dv;input vv;input[7:0] pixel;
        begin @(negedge clk);de=dv;vs=vv;gray=pixel;end
    endtask
    task send_frame;
        input blank;
        begin
            tick(0,1,bg);
            repeat(6000) tick(0,1,bg);
            for(row=0;row<192;row=row+1) begin
                for(col=0;col<256;col=col+1) begin
                    if(!blank && col>=offsetx&&col<offsetx+128&&row>=offsety&&row<offsety+128)
                        tick(1,1,patch[(row-offsety)*128+col-offsetx]);
                    else tick(1,1,bg);
                end
                repeat(4) tick(0,1,bg);
            end
            repeat(8) tick(0,0,bg);
        end
    endtask
    initial begin
        if(!$value$plusargs("STIM=%s",filename)) $fatal(1,"STIM required");
        if($value$plusargs("EXPECT=%d",expected)) begin end
        if($value$plusargs("BG=%d",bg)) begin end
        if($value$plusargs("X=%d",offsetx)) begin end
        $readmemh(filename,patch);
        repeat(4) @(negedge clk);rst_n=1;
        for(frame=0;frame<8;frame=frame+1) send_frame(0);
        tick(0,1,bg);repeat(6000) tick(0,1,bg);
        if(expected!=0 && (!recognized||ascii!=expected)) begin
            $display("OCR FAIL expected=%c actual=%c recognized=%b bbox=%b dist=%0d second=%0d thr=%0d dark=%b reason=%0d",
                expected,ascii,recognized,bbox_valid,best,second,threshold,dark,reject_reason);
            $display("glyph=%064h",dut.glyph_o);$fatal(1,"positive camera fixture");
        end
        if(expected==0 && recognized) $fatal(1,"negative scene accepted as %c",ascii);
        // Removing a recognized glyph MUST clear its label on invalid input.
        tick(0,0,bg);
        send_frame(1);send_frame(1);send_frame(1);
        if(recognized||ascii!="?") $fatal(1,"stale recognition after removal");
        $display("OCR CAMERA PASS expected=%0d; removal invalidation PASS",expected);$finish;
    end
endmodule
