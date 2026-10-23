from pathlib import Path
import subprocess, os, shutil
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
IVER=Path(os.environ.get('IVERILOG_BIN', str(Path(shutil.which('iverilog')).parent) if shutil.which('iverilog') else 'F:/codex/tools/iverilog/bin'))
EXE='.exe' if os.name=='nt' else ''
src=ROOT/'Ti60F225_SC1336_HDMI_Display/src/recognition'
subprocess.run([str(IVER/('iverilog'+EXE)),'-g2012','-s','tb_geometry','-o',str(HERE/'geometry.vvp'),str(HERE/'tb_geometry.sv'),str(src/'shape_geometry_engine.v'),str(src/'shape_sdp_ram.v')],check=True)
def outline(inside):
 return {(x,y) for x,y in inside if any((x+dx,y+dy) not in inside for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)])}
cases=[]
for ox,oy in [(15,12),(95,42)]:
 cases += [('square_'+str(ox),outline({(x,y) for y in range(oy,oy+25) for x in range(ox,ox+25)}),2),('rectangle_'+str(ox),outline({(x,y) for y in range(oy,oy+20) for x in range(ox,ox+36)}),3)]
for name,rx,ry,cls in [('circle',15,15,4),('ellipse',23,13,5)]:
 cases.append((name,outline({(x,y) for y in range(15,65) for x in range(20,95) if ((x-55)/rx)**2+((y-40)/ry)**2<=1}),cls))
cases += [('blank',set(),0),('noise',{(10+i*3,12+i%8*4) for i in range(35)},0),('line',{(x,20) for x in range(20,70)},0),('open_U',{(x,y) for x in range(25,56) for y in range(20,51) if x in (25,55) or y==50},0)]
def polygon(vertices):
 def inside(x,y):
  signs=[]
  for (ax,ay),(bx,by) in zip(vertices,vertices[1:]+vertices[:1]):
   signs.append((bx-ax)*(y-ay)-(by-ay)*(x-ax))
  return all(v>=0 for v in signs) or all(v<=0 for v in signs)
 return {(x,y) for y in range(90) for x in range(160) if inside(x,y)}
for name,vertices,cls in [
 ('triangle_up',[(40,18),(20,58),(60,58)],1),
 ('triangle_down',[(20,18),(60,18),(40,58)],1),
 ('triangle_right',[(20,18),(60,38),(20,58)],1),
 ('pentagon',[(40,15),(62,31),(54,57),(26,57),(18,31)],6),
 ('diamond',[(40,15),(60,35),(40,55),(20,35)],9)]:
 region=polygon(vertices)
 cases.extend([(name,outline(region),cls),(name+'_solid',region,cls)])
plus={(x,y) for y in range(15,54) for x in range(20,59) if 33<=x<=45 or 28<=y<=40}
cases.extend([('plus',outline(plus),8),('plus_solid',plus,8)])
cases.append(('solid_square',{(x,y) for y in range(20,45) for x in range(20,45)},2))
cases.append(('dense',{(x,y) for y in range(5,85) for x in range(5,155)},0))
outer=outline({(x,y) for y in range(10,65) for x in range(20,75)})
inner=outline({(x,y) for y in range(28,45) for x in range(38,55)})
cases.append(('nested_squares',outer|inner,2,2))
cases.append(('two_squares',outline({(x,y) for y in range(12,37) for x in range(15,40)})|outline({(x,y) for y in range(42,67) for x in range(95,120)}),2,2))
circle=next(points for name,points,cls in cases[:10] if name=='circle')
cases.append(('circle_inner_noise',circle|{(55,40),(51,36),(59,44)},4))
fail=[]
for case in cases:
 name,points,expected=case[:3]
 count=case[3] if len(case)>3 else int(expected!=0)
 f=HERE/(name+'.mem');f.write_text('\n'.join('1' if (x,y) in points else '0' for y in range(90) for x in range(160)))
 r=subprocess.run([str(IVER/('vvp'+EXE)),str(HERE/'geometry.vvp'),'+MEM='+f.as_posix(),'+EXPECT='+str(expected),'+COUNT='+str(count)],text=True,capture_output=True)
 print(name, 'PASS' if r.returncode==0 else r.stdout.strip(),flush=True)
 if r.returncode:fail.append(name)
if fail:raise SystemExit('FAILED: '+', '.join(fail))
for name,modules in [('capture',['shape_frame_capture','shape_sdp_ram']),('tracker',['recognition_object_tracker'])]:
 out=HERE/(name+'.vvp')
 subprocess.run([str(IVER/('iverilog'+EXE)),'-g2012','-s','tb_'+name,'-o',str(out),str(HERE/('tb_'+name+'.sv'))]+[str(src/(m+'.v')) for m in modules],check=True)
 subprocess.run([str(IVER/('vvp'+EXE)),str(out)],check=True)
print('PASS',len(cases),'geometry cases plus capture and tracker')
