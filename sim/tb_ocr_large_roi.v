`timescale 1ns/1ps
// Exercise the production 1280x720 / 640x480 ROI and >512 hits per cell.
// A 16x16 bitmap expanded 25x26 yields 650 pixels in each full cell.
module tb_ocr_large_roi;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0,vs=0,de=0;reg[7:0] gray=255;
    wire frame_valid,candidate_valid;
    wire[255:0] glyph;
    wire[255:0] expected;
    ocr_font16_rom font(.addr_i(9'd32),.bitmap_o(expected));
    ocr_sampler16 dut(.clk(clk),.rst_n(rst_n),.vsync_i(vs),.de_i(de),.gray_i(gray),
        .frame_valid_o(frame_valid),.candidate_valid_o(candidate_valid),.glyph_o(glyph));
    integer f,r,c,bit_index;
    task tick;
        input dv,vv;input[7:0] g;
        begin @(negedge clk);de=dv;vs=vv;gray=g;end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        for(f=0;f<4;f=f+1) begin
            tick(0,1,255);repeat(32) tick(0,1,255);
            for(r=0;r<720;r=r+1) begin
                for(c=0;c<1280;c=c+1) begin
                    if(c>=440&&c<840&&r>=152&&r<568) begin
                        bit_index=255-((r-152)/26)*16-(c-440)/25;
                        tick(1,1,expected[bit_index]?8'd0:8'd255);
                    end else tick(1,1,255);
                end
                repeat(4) tick(0,1,255);
            end
            repeat(4) tick(0,0,255);
        end
        tick(0,1,255);repeat(4) tick(0,1,255);
        if(!candidate_valid) $fatal(1,"large ROI invalid candidate");
        if(glyph!==expected) begin $display("got %h expected %h",glyph,expected);$fatal(1,"large-cell overflow/normalization");end
        $display("OCR LARGE ROI PASS: 1280x720,640x480 search,650 pixels/cell no overflow");$finish;
    end
endmodule
