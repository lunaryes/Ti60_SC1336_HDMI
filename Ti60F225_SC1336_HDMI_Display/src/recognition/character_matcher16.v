`timescale 1ns/1ps
// 实心/描边双原型、对称一像素容差；次优类别不经接受门限预筛。
module character_matcher16 #(
 parameter [10:0] MAX_DISTANCE=11'd128, MIN_MARGIN=11'd10
)(
 input wire clk,rst_n,start_i,candidate_valid_i,
 input wire [255:0] glyph_i,input wire [10:0] width_i,input wire [9:0] height_i,
 output reg result_valid_o,recognized_o,output reg [5:0] class_id_o,
 output reg [7:0] ascii_o,output reg [8:0] best_distance_o,second_distance_o,
 output reg [255:0] template_o
);
 localparam IDLE=0,SCAN=1,TEMPLATE=2,CLASS=3,DONE=4;
 reg [2:0] state;reg [9:0] address;reg [3:0] row;reg outline;
 wire [255:0] bitmap;wire [5:0] rom_class;wire [7:0] rom_ascii,aspect;
 character_font16_rom rom(address,bitmap,rom_class,rom_ascii,aspect);
 reg [255:0] glyph;reg [10:0] width;reg [9:0] height;
 reg [10:0] distance,class_best,best,second;
 reg [8:0] unmatched,raw_error,class_unmatched,class_error,best_unmatched,best_error;
 reg [7:0] best_ascii;reg [5:0] best_class;
 reg [255:0] class_template,best_template;
 function [4:0] pop16;input [15:0] v;integer k;
 begin pop16=0;for(k=0;k<16;k=k+1) pop16=pop16+v[k];end endfunction
 function [4:0] leading;input [15:0] v;integer k;reg found;
 begin leading=16;found=0;for(k=15;k>=0;k=k-1) if(v[k]&&!found) begin leading=15-k;found=1;end end endfunction
 function [4:0] trailing;input [15:0] v;integer k;reg found;
 begin trailing=16;found=0;for(k=0;k<16;k=k+1) if(v[k]&&!found) begin trailing=k;found=1;end end endfunction
 function [15:0] getrow;input [255:0] v;input integer r;
 begin getrow=(r<0||r>15)?16'd0:v[(15-r)*16+:16];end endfunction
 function [15:0] prototype;input [255:0] v;input integer r;input edge_mode;
 reg [15:0] c;
 begin c=getrow(v,r);prototype=edge_mode ?
 c & ~(getrow(v,r-1)&getrow(v,r+1)&(c<<1)&(c>>1)):c;end endfunction
 wire [15:0] g=getrow(glyph,{28'd0,row});
 wire [15:0] t=prototype(bitmap,{28'd0,row},outline);
 wire [15:0] gn=g|getrow(glyph,{28'd0,row}-1)|getrow(glyph,{28'd0,row}+1);
 wire [15:0] tn=t|prototype(bitmap,{28'd0,row}-1,outline)|prototype(bitmap,{28'd0,row}+1,outline);
 wire [5:0] missed={1'b0,pop16(g&~(tn|(tn<<1)|(tn>>1)))}+
                         {1'b0,pop16(t&~(gn|(gn<<1)|(gn>>1)))};
 wire [4:0] error=pop16(g^t);
 wire [4:0] gl=leading(g),tl=leading(t),gr=trailing(g),tr=trailing(t);
 wire [5:0] profile=(gl>tl?gl-tl:tl-gl)+(gr>tr?gr-tr:tr-gr);
 wire [18:0] observed={width,7'd0},reference=aspect*height;
 wire aspect_ok=observed*3>=reference*2 && observed*2<=reference*3;
 wire [10:0] score=aspect_ok?distance:11'd1023;
 // 近乎精确的字模允许较小的绝对差；较差的匹配仍保持严格类别间隔。
 wire margin_ok=second>=best+MIN_MARGIN ||
                (best<=32 && best_unmatched<=3 && second>=best+6);
 // 0/O 必须保留中心孔洞；实心圆/椭圆不能仅凭外轮廓相似而成为数字0。
 wire hole_ok=(best_ascii!="0"&&best_ascii!="O") ||
              pop16({glyph[150+:4],glyph[134+:4],glyph[118+:4],glyph[102+:4]})<12;
 wire accept=best<=MAX_DISTANCE && best_unmatched<=12 && best_error<=80 && margin_ok && best_ascii!="a" && hole_ok;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;address<=0;row<=0;outline<=0;glyph<=0;width<=0;height<=0;
   distance<=0;unmatched<=0;raw_error<=0;class_best<=1023;best<=1023;second<=1023;
   class_unmatched<=511;class_error<=511;best_unmatched<=511;best_error<=511;
   best_ascii<=0;best_class<=0;class_template<=0;best_template<=0;
   result_valid_o<=0;recognized_o<=0;class_id_o<=0;ascii_o<="?";
   best_distance_o<=511;second_distance_o<=511;template_o<=0;
  end else begin
   result_valid_o<=0;
   if(start_i) begin
    recognized_o<=0;class_id_o<=0;ascii_o<="?";template_o<=0;
    best_distance_o<=511;second_distance_o<=511;
    if(!candidate_valid_i) begin state<=IDLE;result_valid_o<=1;end
    else begin
     glyph<=glyph_i;width<=width_i;height<=height_i;address<=0;row<=0;outline<=0;
     distance<=0;unmatched<=0;raw_error<=0;class_best<=1023;best<=1023;second<=1023;state<=SCAN;
    end
   end else case(state)
    SCAN:begin
     distance<=distance+error+profile+({5'd0,missed}<<2);
     unmatched<=unmatched+missed;raw_error<=raw_error+error;
     if(row==15) state<=TEMPLATE;else row<=row+1'b1;
    end
    TEMPLATE:begin
     if(score<class_best) begin
      class_best<=score;class_unmatched<=unmatched;class_error<=raw_error;class_template<=bitmap;
     end
     row<=0;distance<=0;unmatched<=0;raw_error<=0;
     if(!outline) begin outline<=1;state<=SCAN;end
     else begin
      outline<=0;
      if(address[3:0]==15) state<=CLASS;
      else begin address<=address+1'b1;state<=SCAN;end
     end
    end
    CLASS:begin
     if(class_best<best) begin
      second<=best;best<=class_best;best_ascii<=rom_ascii;best_class<=rom_class;
      best_template<=class_template;best_unmatched<=class_unmatched;best_error<=class_error;
     end else if(class_best<second) second<=class_best;
     class_best<=1023;
     if(address==591) state<=DONE;
     else begin address<=address+1'b1;state<=SCAN;end
    end
    DONE:begin
     result_valid_o<=1;recognized_o<=accept;
     class_id_o<=accept?best_class:0;ascii_o<=accept?best_ascii:"?";
     best_distance_o<=best>511?511:best;second_distance_o<=second>511?511:second;
     template_o<=best_template;state<=IDLE;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
