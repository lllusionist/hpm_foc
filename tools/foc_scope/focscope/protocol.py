# -*- coding: utf-8 -*-
"""与固件的通讯协议: 5 字节命令帧 + VOFA Justfloat 波形帧.

- 命令帧: [cmd_id(1B)] + [float32 小端(4B)]           共 5 字节
- 波形帧: N 个 float32 小端 + 帧尾 00 00 80 7F         Justfloat
"""
import math
import struct
import time

# 命令字 (与固件 usb_driver.c / 旧上位机一致)
CMD = {
    "opmode": 0x20, "enable": 0x21, "disable": 0x22, "clear_fault": 0x26,
    "torque": 0x23, "velocity": 0x24, "position": 0x25,
    "calib": 0x27, "anticog": 0x28, "heartbeat": 0x29,
    "set_anticog": 0x2A, "set_ff": 0x2B, "anticog_sweep": 0x2E,
           "dump_cog": 0x2C, "home": 0x2D, "save": 0x3F, "reset": 0x3B,
    "getcfg": 0x3E,
    "curkp": 0x40, "curki": 0x41, "velkp": 0x42, "velki": 0x43, "poskp": 0x44,
    "curbw": 0x58, "motor_profile": 0x5F,
    "uv": 0x56, "ov": 0x57,
    "step": 0x60, "step_base": 0x61,
}

TAIL = b"\x00\x00\x80\x7f"
CFG_MAGIC = b"$FOCFG,"
NCH = 22
HEARTBEAT_INTERVAL_MS = 100
FLOAT32_MAX = float.fromhex("0x1.fffffep127")
STEP_MAGIC = b"$STEP,"
STEP_MAX_HEADER = 128
STEP_MAX_PAYLOAD = 256 * 1024

# 运行模式
MODES = ["电流", "速度", "位置", "轨迹"]


def pack_cmd(cid, val=None):
    """打包一条 5 字节命令帧."""
    b = bytearray(5)
    b[0] = cid & 0xFF
    if val is not None:
        b[1:5] = struct.pack("<f", float(val))
    return bytes(b)


def read_step_reply(port, cancelled, timeout=3.0):
    """读取一个完整的阶跃回复，允许前面有波形数据或分段到达。"""
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline and not cancelled():
        buf.extend(port.read(512))
        start = buf.find(STEP_MAGIC)
        if start < 0:
            del buf[:-(len(STEP_MAGIC) - 1)]
            continue
        del buf[:start]
        end = buf.find(b"\n")
        if end >= 0:
            if end >= STEP_MAX_HEADER:
                raise ValueError("阶跃数据头过长")
            fs, n, ch, pre = _parse_step_header(buf[len(STEP_MAGIC):end])
            del buf[:end + 1]
            break
        if len(buf) >= STEP_MAX_HEADER:
            raise ValueError("阶跃数据头过长")
    else:
        if cancelled():
            raise InterruptedError("阶跃已取消")
        raise TimeoutError("未收到阶跃数据，请检查使能和固件故障")

    needed = n * ch * 4
    while len(buf) < needed and time.monotonic() < deadline and not cancelled():
        buf.extend(port.read(min(2048, needed - len(buf))))
    if cancelled():
        raise InterruptedError("阶跃已取消")
    if len(buf) < needed:
        raise TimeoutError(f"阶跃数据不完整（{len(buf)}/{needed} 字节）")
    values = struct.unpack_from(f"<{n * ch}f", buf)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("阶跃数据包含无效数值")
    return fs, n, ch, pre, values


def _parse_step_header(header):
    try:
        fields = dict(part.split("=", 1)
                      for part in header.decode("ascii").split(","))
        fs, n = float(fields["FS"]), int(fields["N"])
        ch, pre = int(fields["CH"]), int(fields["PRE"])
    except (KeyError, ValueError) as exc:
        raise ValueError("阶跃数据头格式错误") from exc
    if not (math.isfinite(fs) and fs > 0 and 0 < pre < n and ch >= 2
            and n * ch * 4 <= STEP_MAX_PAYLOAD):
        raise ValueError("阶跃采样率、点数或通道数无效")
    return fs, n, ch, pre


class JustfloatParser:
    """增量解析 Justfloat 流, 每次 feed 返回本批解析出的样本列表(每个样本为 float 元组)."""

    def __init__(self, max_backlog=8192):
        self._buf = bytearray()
        self._max = max_backlog

    def feed(self, data):
        out = []
        self._buf += data
        while True:
            i = self._buf.find(TAIL)
            if i < 0:
                break
            payload = bytes(self._buf[:i])
            del self._buf[:i + 4]
            if i > 0 and i % 4 == 0:
                out.append(struct.unpack("<%df" % (i // 4), payload))
        if len(self._buf) > self._max:
            del self._buf[:-4]
        return out

    def reset(self):
        self._buf = bytearray()
