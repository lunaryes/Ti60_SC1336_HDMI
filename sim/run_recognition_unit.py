from pathlib import Path
import subprocess,json
root=Path(__file__).resolve().parents[1];src=root/'src'
if not (src/'character_detection_top.v').exists():src=root/'Ti60F225_SC1336_HDMI_Display'/'src'/'recognition'
out=root/'sim'/'character_ccl'/'generated'/'unit';out.mkdir(parents=True,exist_ok=True)
tool=Path(r'F:\codex\tools\iverilog\bin')
tests=['character_ccl/tb_character_stability.v','shape_ccl/tb_shape_stability.v',
       'character_ccl/tb_object_fusion.v','character_ccl/tb_character_io.v','character_ccl/tb_character_lighting.v',
       'shape_ccl/tb_shape_io.v','shape_ccl/tb_geometry.v']
results=[]
for test in tests:
 p=root/'sim'/test;target=out/(p.stem+'.vvp')
 build=subprocess.run([str(tool/'iverilog.exe'),'-g2012','-s',p.stem,'-o',str(target),*map(str,src.glob('*.v')),str(p)],capture_output=True,text=True)
 if build.returncode:raise RuntimeError(build.stderr)
 run=subprocess.run([str(tool/'vvp.exe'),str(target)],capture_output=True,text=True,timeout=180)
 print(test,run.stdout,run.stderr,flush=True)
 results.append(dict(test=test,passed=run.returncode==0,output=run.stdout,build_warnings=build.stderr))
(out/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
if any(not r['passed'] for r in results):raise SystemExit(1)
