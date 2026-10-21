"""从用户录像左侧灰度显示区提取诊断裁片，不宣称其为摄像头原始帧。"""
import argparse,pathlib
from PIL import Image
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('frame',type=pathlib.Path)
p.add_argument('--output-dir',type=pathlib.Path,required=True)
a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
photo=Image.open(a.frame).convert('L')
for text,box in [('1',(292,143,343,225)),('2',(387,141,459,228)),('3',(511,140,582,230))]:
 photo.crop(box).resize(((box[2]-box[0])*2,(box[3]-box[1])*2)).save(a.output_dir/f'camera_crop_{text}.png')
