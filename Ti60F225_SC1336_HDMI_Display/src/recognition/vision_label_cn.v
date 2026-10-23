`timescale 1ns/1ps
// Shape names only, using 32x32 Chinese glyphs.
module vision_label_cn(
 input wire [7:0] id_i,
 input wire [6:0] dx_i,input wire [4:0] dy_i,output wire ink_o
);
 wire [1:0] cell_index=dx_i[6:5];reg [4:0] cn_id;
 wire [255:0] chinese_bitmap;
 vision_cn_font font_cn(cn_id,chinese_bitmap);
 always @(*) begin
  cn_id=0;
  case(id_i)
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
 assign ink_o=chinese_bitmap[8'd255-{dy_i[4:1],dx_i[4:1]}];
endmodule
