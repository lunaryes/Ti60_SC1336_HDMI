#!/usr/bin/env python3
"""DDR frame controls and post-median gray recursive IIR status/counters.
No histogram, ROI image capture, diagnostic frame RAM or image UART transfer.
"""
import argparse
import json
import time

MODES={'live':0,'freeze':1,'replay':2,'compare':3,'difference':4}


def read_reply(s, command):
    s.write(command)
    reply=s.read(8)
    if len(reply)!=8 or reply[-1]!=10:
        raise RuntimeError(f'{command!r}: invalid reply {reply.hex()}')
    return reply


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',default='COM6')
    ap.add_argument('--mode',choices=MODES)
    ap.add_argument('--tnr',choices=['on','off'])
    ap.add_argument('--age',type=int,choices=range(16))
    ap.add_argument('--reference',type=int,choices=range(16))
    ap.add_argument('--replay-div',type=int)
    ap.add_argument('--layout',choices=['original','edge','split','overlay'])
    ap.add_argument('--view',choices=['edge','gray'])
    ap.add_argument('--iir-param',nargs=2,action='append',metavar=('INDEX','VALUE'),
                    help='Set indexed IIR parameter, e.g. --iir-param 0x06 0x10')
    ap.add_argument('--iir-defaults',action='store_true')
    ap.add_argument('--iir-tuning',action='store_true',help='Read all eight IIR tuning bytes')
    args=ap.parse_args()
    params=[]
    for index,value in args.iir_param or []:
        try:index,value=int(index,0),int(value,0)
        except ValueError:ap.error('IIR index/value must be integers; use 0x prefix for HEX')
        if not 0<=index<8 or not 0<=value<256:ap.error('IIR index must be 0..7, value 0..255')
        params.append((index,value))
    if args.replay_div is not None and not 1<=args.replay_div<=255:ap.error('replay-div must be 1..255')
    import serial
    with serial.Serial(args.port,460800,timeout=.5,write_timeout=1) as s:
        s.reset_input_buffer()
        q=read_reply(s,bytes([0x51]))
        if q[0]!=ord('V'):raise RuntimeError('Application frame-store firmware is not loaded')
        tuning=None
        if params or args.iir_defaults or args.iir_tuning:
            probe=read_reply(s,bytes([0x5B]))
            if probe[:3]!=bytes([0x5B,0x49,2]):raise RuntimeError('Runtime IIR tuning firmware is not loaded')
            previous_index=probe[3]
            if args.iir_defaults:s.write(bytes([0x5D]))
            for index,value in params:s.write(bytes([0x59,index,0x5A,value]))
            tuning=[]
            for index in range(8):
                s.write(bytes([0x59,index]))
                reply=read_reply(s,bytes([0x5B]))
                if reply[:4]!=bytes([0x5B,0x49,2,index]) or reply[6]!=0:raise RuntimeError('Invalid tuning reply')
                tuning.append(reply[4])
            s.write(bytes([0x59,previous_index]))
        updates=[(0x4A,args.age),(0x4B,args.reference),(0x4F,args.replay_div),
                 (0x4E,None if args.tnr is None else int(args.tnr=='on')),
                 (0x4D,None if args.layout is None else {'original':0,'edge':1,'split':2,'overlay':3}[args.layout]),
                 (0x56,None if args.view is None else (3 if args.view=='gray' else 0)),
                 (0x50,None if args.mode is None else MODES[args.mode])]
        for command,value in updates:
            if value is not None:s.write(bytes([command,value]))
        time.sleep(.15)
        q=read_reply(s,bytes([0x51]));d=read_reply(s,bytes([0x44]));r=read_reply(s,bytes([0x52]))
        if d[:3]!=b'DV\x01':raise RuntimeError('Invalid application status protocol')
        status=int.from_bytes(d[3:7],'little')
        history=read_reply(s,bytes([0x49]))
        if history[:3]!=bytes([0x49,0x49,1]):raise RuntimeError('Gray recursive IIR firmware is not loaded')
        hs=int.from_bytes(history[3:7],'little')
        counts=[]
        for code in [0x55,0x57,0x58]:
            reply=read_reply(s,bytes([code]))
            if reply[:3]!=bytes([code,0x49,1]):raise RuntimeError('Invalid IIR counter reply')
            counts.append(int.from_bytes(reply[3:7],'little'))
        print(json.dumps(dict(requested_mode=q[1],tnr=bool(q[2]),age=q[3],reference_age=q[4],
                              replay_display_frames=q[5],active_mode=(status>>16)&7,
                              retained_frames=(status>>19)&31,display_source_id=status&65535,
                              error_flags=status>>24,threshold=r[0],view=r[1],layout=r[2],
                              smooth=r[5],link=r[6],iir_operation=['bypass','initialize','recursive','cached'][(hs>>1)&3],
                              history_valid=bool(hs&8),history_source_id=(hs>>8)&65535,
                              history_errors=hs>>24,strong_pixels=counts[0],weak_pixels=counts[1],
                              bypass_pixels=counts[2],count_total=sum(counts),iir_tuning=tuning),indent=2))


if __name__=='__main__':main()
