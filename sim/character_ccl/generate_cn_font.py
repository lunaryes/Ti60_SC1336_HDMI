"""生成界面必需的中文字模，不分发系统字体文件。"""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;OUT=HERE.parent/'src'
if not OUT.exists():OUT=HERE.parents[1]/'Ti60F225_SC1336_HDMI_Display'/'src'/'recognition'
chars='三角形正方长圆椭五边六十字四数母未识别'
font=ImageFont.truetype('C:/Windows/Fonts/msyhbd.ttc',16)
lines=['`timescale 1ns/1ps','// 自动生成：16x16 中文标注字模，最高位对应左上角。',
 'module vision_cn_font(input wire [4:0] id_i,output reg [255:0] bitmap_o);',
 'always @(*) begin bitmap_o=0;case(id_i)']
sheet=Image.new('RGB',(len(chars)*32,48),'white');sd=ImageDraw.Draw(sheet)
for i,ch in enumerate(chars,1):
 im=Image.new('1',(16,16));d=ImageDraw.Draw(im);box=font.getbbox(ch)
 d.text(((16-(box[2]-box[0]))//2-box[0],(16-(box[3]-box[1]))//2-box[1]),ch,font=font,fill=1)
 bits=0
 for v in im.get_flattened_data():bits=(bits<<1)|int(v!=0)
 lines.append(f"5'd{i}:bitmap_o=256'h{bits:064x}; // {ch}")
 sheet.paste(im.resize((32,32)).convert('RGB'),((i-1)*32,0));sd.text(((i-1)*32,34),str(i),fill='black')
lines+=['default:bitmap_o=0;endcase end','endmodule','']
(OUT/'vision_cn_font.v').write_text('\n'.join(lines),encoding='utf-8')
sheet.save(HERE/'chinese_font_preview.png')
print('生成',len(chars),'个中文字模')
