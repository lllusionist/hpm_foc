# -*- coding: utf-8 -*-
"""波形录制: 收集全分辨率通道数据 + 命令事件, 导出为自描述 CSV.

- 单线程使用 (主线程: _tick 与 send 都在 GUI 线程, 无需加锁)
- 每个通道一个 array('f'), 时间 = 相对录制起点的秒数
- 命令事件与数据写在同一文件: # 注释行 (解析器可跳过)
"""
import os
import time
from array import array

from .channels import CHANNELS, NCH


class Recorder:
    def __init__(self, fs=5000.0):
        self.fs = float(fs)
        self.active = False
        self.bufs = [array('f') for _ in range(NCH)]
        # 事件: (t_sec, 命令名, 渲染文本)
        self.events = []
        self.count = 0
        self.start_wall = None

    # ---------- 控制 ----------
    def start(self, fs=None):
        if fs:
            self.fs = float(fs)
        for b in self.bufs:
            del b[:]
        self.events = []
        self.count = 0
        self.active = True
        self.start_wall = time.time()

    def stop(self):
        self.active = False

    def is_active(self):
        return self.active

    def duration(self):
        return self.count / self.fs if self.fs > 0 else 0.0

    def _now(self):
        return self.count / self.fs if self.fs > 0 else 0.0

    # ---------- 写入 ----------
    def append(self, vals):
        if not self.active:
            return
        n = min(len(vals), NCH)
        for c in range(n):
            try:
                self.bufs[c].append(float(vals[c]))
            except (TypeError, ValueError):
                self.bufs[c].append(0.0)
        for c in range(n, NCH):
            self.bufs[c].append(0.0)
        self.count += 1

    def add_event(self, name, value=None):
        """记录一条命令事件; 连续相同值自动合并 (滑条连续发送)."""
        if not self.active or not name:
            return
        text = name if value is None else "%s=%s" % (name, value)
        if self.events and self.events[-1][2] == text:
            return
        self.events.append((self._now(), name, text))


def default_dir():
    """优先桌面 (兼容 OneDrive 重定向), 回退到用户主目录."""
    candidates = []
    try:
        from PySide6.QtCore import QStandardPaths
        d = QStandardPaths.writableLocation(QStandardPaths.DesktopLocation)
        if d:
            candidates.append(d)
    except Exception:
        pass
    home = os.path.expanduser("~")
    candidates.append(os.path.join(home, "Desktop"))
    candidates.append(home)
    for d in candidates:
        if d and os.path.isdir(d):
            return d
    return os.getcwd()


def make_filename(prefix="foc_wave"):
    return "%s_%s.csv" % (prefix, time.strftime("%Y%m%d_%H%M%S"))


def write_csv(rec, path, meta=None):
    """把录制内容写成 CSV. meta 为有序 dict(标题->值), 作为 # 注释头."""
    n = rec.count
    fs = rec.fs if rec.fs > 0 else 1.0
    legend = " ".join(
        "%d=%s(%s%s)" % (i,
                         CHANNELS[i][1],
                         CHANNELS[i][0],
                         ("/" + CHANNELS[i][2]) if CHANNELS[i][2] else "")
        for i in range(NCH)
    )
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        f.write("# FOC Scope 波形记录\n")
        if meta:
            for k, v in meta.items():
                f.write("# %s: %s\n" % (k, v))
        f.write("# 通道: %s\n" % legend)
        if rec.events:
            f.write("# 事件(t 相对录制起点):\n")
            for t, _name, text in rec.events:
                f.write("#   t=+%.3f %s\n" % (t, text))
        # 表头
        f.write("t_s," + ",".join(CHANNELS[i][0] for i in range(NCH)) + "\n")
        # 数据 (手动拼接, 比 csv.writer 快)
        bufs = rec.bufs
        for k in range(n):
            parts = ["%.6g" % (k / fs)]
            for c in range(NCH):
                parts.append("%.6g" % bufs[c][k])
            f.write(",".join(parts))
            f.write("\n")
    return n
