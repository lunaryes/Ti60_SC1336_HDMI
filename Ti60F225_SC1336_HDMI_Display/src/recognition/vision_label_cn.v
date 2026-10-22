`timescale 1ns/1ps
// 每字32x32：中文放大两倍，数字/英文字母保留原字符。
module vision_label_cn(
 input wire character_i,unknown_i,input wire [7:0] id_i,
 input wire [6:0] dx_i,input wire [4:0] dy_i,output wire ink_o
);
 wire [1:0] cell_index=dx_i[6:5];reg [4:0] cn_id;
 wire latin=character_i&&!unknown_i&&cell_index==2;
 wire [255:0] chinese_bitmap;wire [63:0] latin_bitmap;wire [7:0] unused_ascii;
 wire [5:0] latin_addr=id_i>="0"&&id_i<="9"?id_i-"0":id_i>="A"&&id_i<="Z"?id_i-"A"+10:63;
 vision_cn_font font_cn(cn_id,chinese_bitmap);
 symbol_template_rom font_en(latin_addr,latin_bitmap,unused_ascii);
 always @(*) begin
  cn_id=0;
  if(unknown_i) case(cell_index) 0:cn_id=17;1:cn_id=18;2:cn_id=19;default:cn_id=0;endcase
  else if(character_i) begin
   if(cell_index==0) cn_id=id_i<="9"?15:13;
   if(cell_index==1) cn_id=id_i<="9"?13:16;
  end else case(id_i)
   1:case(cell_index) 0:cn_id=1;1:cn_id=2;2:cn_id=3;default:cn_id=0;endcase
   2:case(cell_index) 0:cn_id=4;1:cn_id=5;2:cn_id=3;default:cn_id=0;endcase
   3:case(cell_index) 0:cn_id=6;1:cn_id=5;2:cn_id=3;default:cn_id=0;endcase
   4:case(cell_index) 0:cn_id=7;1:cn_id=3;default:cn_id=0;endcase
   5:case(cell_index) 0:cn_id=8;1:cn_id=7;default:cn_id=0;endcase
   6:case(cell_index) 0:cn_id=9;1:cn_id=10;2:cn_id=3;default:cn_id=0;endcase
   8:case(cell_index) 0:cn_id=12;1:cn_id=13;2:cn_id=3;default:cn_id=0;endcase
   9:case(cell_index) 0:cn_id=14;1:cn_id=10;2:cn_id=3;default:cn_id=0;endcase
   default:cn_id=0;
  endcase
 end
 assign ink_o=latin?latin_bitmap[6'd63-{dy_i[4:2],dx_i[4:2]}]:chinese_bitmap[8'd255-{dy_i[4:1],dx_i[4:1]}];
endmodule
