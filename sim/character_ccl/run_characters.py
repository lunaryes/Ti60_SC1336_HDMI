"""无卡片、连通域独立裁剪的可复现字符测试；不宣称摄像头精度。"""
import pathlib,subprocess,re,sys,json,hashlib
from concurrent.futures import ThreadPoolExecutor,as_completed
from PIL import Image,ImageFont,ImageDraw,ImageOps
HERE=pathlib.Path(__file__).resolve().parent
SRC=HERE.parent/'src'
if not (SRC/'character_ccl_engine.v').exists():SRC=HERE.parents[1]/'Ti60F225_SC1336_HDMI_Display'/'src'/'recognition'
OLD=SRC if (SRC/'ocr_font16_rom.v').exists() else pathlib.Path(r'E:\fpga\Ti60_SC1336_HDMI\Ti60F225_SC1336_HDMI_Display\src\recognition')
TOOL=pathlib.Path(r'F:\codex\tools\iverilog\bin')
sources=[OLD/'shape_sdp_ram.v',OLD/'ocr_font16_rom.v',SRC/'character_matcher16.v',SRC/'character_ccl_engine.v',HERE/'tb_character_file.v']
stamp=hashlib.sha256(b''.join(p.read_bytes() for p in sources)).hexdigest()[:12]
OUT=HERE/'generated'/stamp;OUT.mkdir(parents=True,exist_ok=True)
subprocess.run([str(TOOL/'iverilog.exe'),'-g2012','-s','tb_character_file','-o',str(OUT/'characters.vvp'),*map(str,sources)],check=True)
def glyph(text,fontfile='arialbd.ttf',size=144):
 font=ImageFont.truetype('C:/Windows/Fonts/'+fontfile,size);box=font.getbbox(text)
 im=Image.new('L',(box[2]-box[0]+12,box[3]-box[1]+12),255)
 ImageDraw.Draw(im).text((6-box[0],6-box[1]),text,font=font,fill=0)
 return im
cases=[]
for text in ('4','A','B','7','8','Z','AB','42'):
 im=Image.new('L',(1280,720),255);g=glyph(text);im.paste(g,(240,200));cases.append((text,im,text))
im=Image.new('L',(1280,720),255);g=glyph('A');im.paste(g,(800,380));cases.append(('outside_old_roi',im,'A'))
im=Image.new('L',(1280,720),0);g=ImageOps.invert(glyph('4'));im.paste(g,(500,200));cases.append(('bright4',im,'4'))
cases.append(('empty',Image.new('L',(1280,720),255),''))
im=Image.new('L',(1280,720),255);ImageDraw.Draw(im).ellipse((280,180,620,520),fill=0);cases.append(('solid_circle_reject',im,''))
if '--extended' in sys.argv:
 for text in '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ':
  im=Image.new('L',(1280,720),255);im.paste(glyph(text,size=160),(360,240));cases.append(('full_'+text,im,text))
 for name,shape in [('triangle',[(360,200),(680,200),(520,500)]),('rectangle',[(360,200),(600,200),(600,500),(360,500)])]:
  im=Image.new('L',(1280,720),255);ImageDraw.Draw(im).polygon(shape,outline=0,width=5);cases.append(('reject_'+name,im,''))
selected=[a for a in sys.argv[1:] if not a.startswith('--')]
tasks=[]
for name,im,want in cases:
 if selected and name not in selected:continue
 pixels=im.load();values=[]
 for y in range(180):
  for x in range(320):
   count=sum(pixels[x*4+dx,y*4+dy]<128 for dy in range(4) for dx in range(4))
   values.append(int(count>=8)|(int(count<=8)<<1))
 path=OUT/(name+'.hex');path.write_text(''.join(f'{v:x}\n' for v in values),encoding='ascii')
 if name=='4':(OUT/'end_to_end_4.hex').write_text(''.join(f'{v:02x}\n' for v in im.get_flattened_data()),encoding='ascii')
 tasks.append((name,path,want))
def run_one(task):
 name,path,want=task
 res=subprocess.run([str(TOOL/'vvp.exe'),'characters.vvp','+FILE='+path.name],cwd=OUT,capture_output=True,text=True,check=True,timeout=300).stdout
 m=re.search(r'RESULT valid=(\w+) overflow=(\d) cycles=(\d+) ascii=(.*)',res)
 if not m:raise RuntimeError(res)
 got=m[4].strip();ok=sorted(want)==sorted(got) and m[2]=='0'
 return dict(name=name,expected=want,actual=got,passed=ok,overflow=int(m[2]),cycles=int(m[3]))
fail=[];results=[]
with ThreadPoolExecutor(max_workers=3) as pool:
 for result in as_completed([pool.submit(run_one,t) for t in tasks]):
  r=result.result();results.append(r)
  print(('PASS' if r['passed'] else 'FAIL'),r['name'],'expected',repr(r['expected']),'got',repr(r['actual']),'cycles',r['cycles'],flush=True)
  if not r['passed']:fail.append(r['name'])
(OUT/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
if fail:raise SystemExit('FAILED: '+','.join(fail))
print('ALL',len(results),'CHARACTER TESTS PASSED')
