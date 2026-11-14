# -*- coding: utf-8 -*-
"""与固件的通讯协议: 5 字节命令帧 + VOFA Justfloat 波形帧.

- 命令帧: [cmd_id(1B)] + [float32 小端(4B)]           共 5 字节
- 波形帧: N 个 float32 小端 + 帧尾 00 00 80 7F         Justfloat
"""
import struct

# 命令字 (与固件 usb_driver.c / 旧上位机一致)
CMD = {
    "opmode": 0x20, "enable": 0x21, "disable": 0x22, "clear_fault": 0x26,
    "torque": 0x23, "velocity": 0x24, "position": 0x25,
    "calib": 0x27, "anticog": 0x28, "set_anticog": 0x2A, "set_ff": 0x2B, "anticog_sweep": 0x2E,
           "dump_cog": 0x2C, "home": 0x2D, "save": 0x3F, "reset": 0x3B,
    "getcfg": 0x3E,
    "curkp": 0x40, "curki": 0x41, "velkp": 0x42, "velki": 0x43, "poskp": 0x44,
    "curbw": 0x58, "motor_profile": 0x5F,
    "uv": 0x56, "ov": 0x57,
}

TAIL = b"\x00\x00\x80\x7f"
CFG_MAGIC = b"$FOCFG,"
NCH = 22

# 运行模式
MODES = ["电流", "速度", "位置", "轨迹"]


def pack_cmd(cid, val=None):
    """打包一条 5 字节命令帧."""
    b = bytearray(5)
    b[0] = cid & 0xFF
    if val is not None:
        b[1:5] = struct.pack("<f", float(val))
    return bytes(b)


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
