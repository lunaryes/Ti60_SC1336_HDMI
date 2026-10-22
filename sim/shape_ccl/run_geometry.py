"""Deterministic mask tests, NOT an assertion of real-camera accuracy."""
import math, pathlib, subprocess, re, json
from PIL import Image, ImageDraw
HERE=pathlib.Path(__file__).resolve().parent
SRC=HERE.parents[1]/'src'
if not (SRC/'shape_geometry_engine.v').exists():
    SRC=HERE.parents[1]/'Ti60F225_SC1336_HDMI_Display'/'src'/'recognition'
IVER=pathlib.Path(r'F:\codex\tools\iverilog\bin')
OUT=HERE/'generated';OUT.mkdir(exist_ok=True)
subprocess.run([str(IVER/'iverilog.exe'),'-g2012','-s','tb_geometry_file','-o',str(OUT/'geometry.vvp'),str(SRC/'shape_sdp_ram.v'),str(SRC/'shape_geometry_engine.v'),str(HERE/'tb_geometry_file.v')],check=True)
def polygon(n,angle=0):
    return [(80+30*math.cos(angle+2*math.pi*i/n),45+30*math.sin(angle+2*math.pi*i/n)) for i in range(n)]
cases=[]
def add(name,kind,coords,want,solid=False):
    im=Image.new('1',(160,90));d=ImageDraw.Draw(im)
    if kind=='ellipse':d.ellipse(coords,outline=1,fill=1 if solid else None)
    elif kind=='line':d.line(coords,fill=1,width=1)
    else:d.polygon(coords,outline=1,fill=1 if solid else None)
    cases.append((name,im,want))
cases.append(('empty',Image.new('1',(160,90)),[]))
for solid in (False,True):
    suffix='_solid' if solid else '_hollow'
    for n,want in ((3,1),(4,2),(5,6),(6,7)):
        add(str(n)+suffix,'poly',polygon(n,math.pi/2),[] if n==6 else [want],solid)
    add('rectangle'+suffix,'poly',[(40,25),(120,25),(120,65),(40,65)],[3],solid)
    add('circle'+suffix,'ellipse',(50,15,110,75),[4],solid)
    add('ellipse'+suffix,'ellipse',(35,25,125,65),[5],solid)
add('inverted_triangle','poly',[(45,20),(115,20),(80,75)],[1])
add('open_line','line',[(40,20),(100,65)],[])
add('plus','poly',[(70,15),(90,15),(90,35),(110,35),(110,55),(90,55),(90,75),(70,75),(70,55),(50,55),(50,35),(70,35)],[8])
im=Image.new('1',(160,90));d=ImageDraw.Draw(im)
d.rectangle((10,15,50,55),outline=1);d.polygon([(90,15),(125,55),(60,55)],outline=1)
cases.append(('two_components',im,[2,1]))
fail=[];results=[]
for solid in (False,True):
 for radius in (12,18,24,32):
  for angle in (0,math.pi/6,math.pi/12):
   coords=[(80+radius*math.cos(angle+2*math.pi*i/6),45+radius*math.sin(angle+2*math.pi*i/6)) for i in range(6)]
   add(f'hex_reject_{solid}_{radius}_{angle:.2f}','poly',coords,[],solid)
for radius in (10,14,18,24):
 add(f'small_circle_{radius}','ellipse',(80-radius,45-radius,80+radius,45+radius),[4])
im=Image.new('1',(160,90));d=ImageDraw.Draw(im)
for yy in (8,48):
 for xx in (8,45,82,119):d.rectangle((xx,yy,xx+24,yy+24),outline=1)
cases.append(('eight_shapes',im,[2]*8))
for name,im,want in cases:
    pixels=getattr(im,'get_flattened_data',im.getdata)()
    file=OUT/(name+'.hex');file.write_text(''.join(f'{int(v!=0):x}\n' for v in pixels),encoding='ascii')
    result=subprocess.run([str(IVER/'vvp.exe'),'geometry.vvp','+FILE='+file.name],cwd=OUT,text=True,capture_output=True,check=True).stdout
    m=re.search(r'RESULT valid=([0-9a-f]+) objects=([0-9a-f]+) rejected=(\d+) cycles=(\d+)',result)
    if not m:raise RuntimeError(result)
    valid,packed=int(m[1],16),int(m[2],16)
    got=[(packed>>(36*i+32))&15 for i in range(8) if valid&(1<<i)]
    ok=sorted(got)==sorted(want)
    results.append(dict(name=name,expected=want,actual=got,passed=ok,cycles=int(m[4])))
    print(('PASS' if ok else 'FAIL'),name,'expected',want,'got',got,'cycles',m[4],flush=True)
    if not ok:fail.append(name)
(OUT/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
if fail:raise SystemExit('FAILED: '+', '.join(fail))
print('ALL',len(cases),'MASK TESTS PASSED')
