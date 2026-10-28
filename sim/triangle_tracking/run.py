from pathlib import Path
import os,subprocess
root=Path(__file__).resolve().parents[2]
os.environ["TEMP"]=os.environ["TMP"]="F:/codex/tmp"
rtl=root/"Ti60F225_SC1336_HDMI_Display/src/pwm_tracker"
for top in ("tb","tb_pwm","tb_follow_link","tb_blend"):
    output=Path(__file__).parent/(top+".vvp")
    sources=[rtl/(n+".v") for n in ("triangle_follow_ctrl","triangle_target_select","axis_cdc","triangle_tracking_top")]
    sources.append(root/"Ti60F225_SC1336_HDMI_Display/src/display/image_mix.v")
    subprocess.run(["F:/codex/tools/iverilog/bin/iverilog.exe","-g2005","-s",top,"-o",str(output),str(Path(__file__).parent/(top+".v"))]+list(map(str,sources)),check=True)
    subprocess.run(["F:/codex/tools/iverilog/bin/vvp.exe","-i",str(output)],check=True)
