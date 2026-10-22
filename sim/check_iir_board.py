"""Exercise volatile UART settings and save gray-IIR board evidence; end live/on."""
import argparse
import json
import time
from pathlib import Path
import serial
from video_control import read_reply


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM6')
    ap.add_argument('--output', default='artifacts/gray_iir/board_validation.json')
    args = ap.parse_args()
    records = []
    with serial.Serial(args.port, 460800, timeout=.5, write_timeout=1) as s:
        s.reset_input_buffer()

        def write(*values):
            s.write(bytes(values))

        def sample(label):
            q = read_reply(s, bytes([0x51]))
            d = read_reply(s, bytes([0x44]))
            h = read_reply(s, bytes([0x49]))
            assert q[0] == 0x56 and d[:3] == bytes([0x44, 0x56, 1])
            assert h[:3] == bytes([0x49, 0x49, 1]), 'IIR firmware not loaded'
            a = int.from_bytes(d[3:7], 'little')
            b = int.from_bytes(h[3:7], 'little')
            counters = []
            for code in [0x55, 0x57, 0x58]:
                reply = read_reply(s, bytes([code]))
                assert reply[:3] == bytes([code, 0x49, 1])
                counters.append(int.from_bytes(reply[3:7], 'little'))
            x = dict(label=label, mode=q[1], tnr=q[2], source=a & 65535,
                     active_mode=(a >> 16) & 7, retained=(a >> 19) & 31,
                     errors=a >> 24, history_errors=b >> 24,
                     history_source=(b >> 8) & 65535, operation=(b >> 1) & 3,
                     history_valid=bool(b & 8), counts=counters)
            records.append(x)
            assert x['errors'] == 0 and x['history_errors'] == 0, x
            return x

        try:
            write(0x50, 0x00, 0x4E, 0x01)
            time.sleep(.8)
            a = sample('live_on');time.sleep(.3);b = sample('live_advances')
            assert a['source'] != b['source'] and b['history_valid']
            write(0x4A, 0x00, 0x50, 0x01);time.sleep(.35)
            a = sample('freeze_cached');time.sleep(.8);b = sample('freeze_stable')
            assert a['source'] == b['source'] == b['history_source']
            assert b['operation'] == 3 and a['counts'] == b['counts']
            assert sum(b['counts']) == 1280 * 720
            write(0x4A, 0x03);time.sleep(.3)
            c = sample('seek_reset_and_cache')
            assert c['source'] != b['source'] and c['history_source'] == c['source']
            write(0x4F, 0x04, 0x50, 0x02);time.sleep(.25)
            a = sample('replay');time.sleep(.2);b = sample('replay_advances')
            assert a['source'] != b['source']
            for mode in [3, 4]:
                write(0x50, mode);time.sleep(.25)
                x = sample('compare' if mode == 3 else 'difference')
                assert x['active_mode'] == mode and x['operation'] == 0
            write(0x50, 0x00, 0x4E, 0x00);time.sleep(.3)
            assert sample('disabled')['operation'] == 0
            write(0x4A, 0x00, 0x4B, 0x01, 0x4F, 0x02, 0x4E, 0x01)
            time.sleep(.6)
            x = sample('restored_live_on')
            assert x['history_valid'] and sum(x['counts']) == 1280 * 720
        finally:
            write(0x50, 0x00, 0x4E, 0x01, 0x4A, 0x00, 0x4B, 0x01, 0x4F, 0x02)
            path = Path(args.output);path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(records, indent=2), encoding='utf-8')
    print(json.dumps(records, indent=2))
    print('GRAY_IIR_BOARD_PASS')


if __name__ == '__main__':
    main()
