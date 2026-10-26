"""Current Ti60 binary UART protocol; no device access on import."""
import time
SETTINGS = {0x54:32, 0x56:3, 0x4D:3, 0x45:1, 0x41:255, 0x42:255,
            0x53:2, 0x50:4, 0x4A:15, 0x4B:15, 0x4F:255, 0x46:1}
QUERIES = {0x51, 0x52, 0x44}
MODES = ['实时', '冻结', '回放', '两帧对比', '两帧差分']
LAYOUTS = ['原图', '算法视图', '左灰度 / 右边缘', '左灰度 / 右原图＋黄色边缘']
VIEWS = ['边缘', '灰度']
SMOOTH = ['标准高斯', '弱平滑', '旁路']

def command(code, value):
    if code not in SETTINGS or not 0 <= value <= SETTINGS[code]:
        raise ValueError('参数超出范围')
    if code == 0x54 and value < 4:
        raise ValueError('阈值必须为 4～32')
    if code == 0x4F and value == 0:
        raise ValueError('回放间隔必须为 1～255')
    return bytes([code, value])

def parse_hex(text):
    try:
        data = bytes.fromhex(text)
    except ValueError as exc:
        raise ValueError('请填写 HEX 字节，例如 4D 03；不要填写命令文字') from exc
    if not data:
        raise ValueError('HEX 内容为空')
    frames = []
    i = 0
    while i < len(data):
        code = data[i]
        if code in QUERIES:
            frames.append(bytes([code])); i += 1
        elif code in SETTINGS:
            if i + 1 == len(data):
                raise ValueError('参数命令缺少一个参数字节，本次未发送')
            frames.append(command(code, data[i+1])); i += 2
        else:
            raise ValueError(f'当前固件不支持命令 {code:02X}，本次未发送')
    return frames

def validate_reply(code, data):
    if len(data) != 8 or data[-1] != 10:
        raise ValueError(f'{code:02X} 回复长度/结束标志无效：{data.hex(" ")}')
    if code == 0x51 and (data[0] != 0x56 or data[1] > 4):
        raise ValueError('帧配置回复无效，请确认固件版本和波特率')
    if code == 0x44 and data[:3] != bytes([0x44,0x56,1]):
        raise ValueError('DDR 状态回复无效')
    if code == 0x52 and (data[1] > 3 or data[2] > 3 or data[5] > 2):
        raise ValueError('图像配置回复无效')
    return data

def query(port, code):
    port.write(bytes([code]))
    deadline = time.monotonic() + 1.0
    data = bytearray()
    while len(data) < 8 and time.monotonic() < deadline:
        data.extend(port.read(8-len(data)))
    result = validate_reply(code, bytes(data))
    time.sleep(.01)
    return result

def snapshot(port, log=lambda text: None):
    packets = {}
    for code in [0x51, 0x52, 0x44]:
        reply = query(port, code)
        log(f'TX {code:02X}  RX {reply.hex(" ").upper()}')
        packets[code] = reply
    q,r,d = packets[0x51],packets[0x52],packets[0x44]
    status = int.from_bytes(d[3:7], 'little')
    return dict(requested_mode=q[1], age=q[3], reference=q[4], divider=q[5],
                active_mode=(status>>16)&7, retained=(status>>19)&31,
                source=status&65535, errors=status>>24, threshold=r[0],
                view=r[1], layout=r[2], gain_r=r[3], gain_b=r[4], smooth=r[5])
