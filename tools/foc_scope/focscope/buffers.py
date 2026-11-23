# -*- coding: utf-8 -*-
"""波形数据存储与抽点.

- 每个通道一个 array('f'), 绝对样本号 = 数组下标 + base
- 超出 max_span 的旧数据被裁剪
- 样本数远大于像素列数时, 用每列 min/max 包络抽点, 避免混叠
"""
import math
from array import array

from .channels import NCH


class DataStore:
    def __init__(self, fs=1000.0, max_span=10.0, cache_span=None):
        self.fs = float(fs)
        self.max_span = float(max_span)
        # 缓存时长: 实际保留多少秒数据 (默认等于最大显示时长)
        self.cache_span = float(cache_span) if cache_span else float(max_span)
        self.bufs = [array('f') for _ in range(NCH)]
        self.base = 0
        self.total = 0
        self.nch = 0
        # 运行统计 (自上次 reset 起): min / max / sum / 样本数
        self.reset_stats()

    # ---------- 写入 ----------
    def append(self, vals):
        n = min(len(vals), NCH)
        if n > self.nch:
            # 帧新增通道时，前面的样本仍占用同一条时间轴。
            missing = array('f', [math.nan]) * self.count()
            for c in range(self.nch, n):
                self.bufs[c].extend(missing)
            self.nch = n
        for c in range(self.nch):
            val = vals[c] if c < n else math.nan
            if not math.isfinite(val):
                self.bufs[c].append(math.nan)
                continue
            self.bufs[c].append(val)
            if val < self.st_min[c]:
                self.st_min[c] = val
            if val > self.st_max[c]:
                self.st_max[c] = val
            self.st_sum[c] += val
            self.st_n[c] += 1
        self.total += 1
        self.trim()

    def reset_stats(self):
        self.st_min = [float("inf")] * NCH
        self.st_max = [float("-inf")] * NCH
        self.st_sum = [0.0] * NCH
        self.st_n = [0] * NCH

    def stats(self, c):
        """返回 (min, max, mean) 原始值; 无样本返回 None."""
        if not self.st_n[c]:
            return None
        return (self.st_min[c], self.st_max[c], self.st_sum[c] / self.st_n[c])

    def clear(self):
        for b in self.bufs:
            del b[:]
        self.base = 0
        self.total = 0
        self.nch = 0
        self.reset_stats()

    def trim(self):
        if not self.bufs[0]:
            return
        keep = max(100, int(self.cache_span * self.fs))
        chunk = max(64, keep // 8)
        if len(self.bufs[0]) > keep + chunk:
            n = len(self.bufs[0]) - keep
            for b in self.bufs:
                del b[:n]
            self.base += n

    # ---------- 查询 ----------
    def count(self):
        return len(self.bufs[0])

    def t_left(self):
        return self.base / self.fs

    def t_right(self):
        return self.total / self.fs

    def latest(self, c):
        b = self.bufs[c]
        return b[-1] if b and math.isfinite(b[-1]) else None

    def latest_values(self):
        return [self.latest(c) for c in range(NCH)]

    def sample_at(self, c, k):
        j = k - self.base
        b = self.bufs[c]
        if 0 <= j < len(b) and math.isfinite(b[j]):
            return b[j]
        return None

    def k_range(self, tL, tR):
        k0 = max(self.base, int(tL * self.fs))
        k1 = min(int(self.total), int(tR * self.fs) + 1)
        if k1 <= k0 and self.total > 0:
            k0 = max(self.base, min(k0, self.total - 1))
            k1 = k0 + 1
        return k0, k1

    # ---------- 抽点 ----------
    def raw(self, c, k0, k1):
        """返回 (k0, [v0, v1, ...]) 绝对起始号与原始值列表."""
        b = self.base
        bbuf = self.bufs[c]
        s = max(0, k0 - b)
        e = min(len(bbuf), k1 - b)
        if e <= s:
            return k0, []
        return b + s, list(bbuf[s:e])

    def envelope(self, c, k0, k1, ncol):
        """每像素列取 min/max，无有效值的列用 NaN 保留波形断点。"""
        if ncol < 1 or k1 <= k0:
            return []
        b = self.base
        bbuf = self.bufs[c]
        n = k1 - k0
        col_w = n / float(ncol)
        inner = max(1, int(n / (4.0 * ncol)))
        out = []
        for col in range(ncol):
            s = k0 + int(col * col_w)
            e = k0 + int((col + 1) * col_w)
            if e <= s:
                e = s + 1
            if e > k1:
                e = k1
            si = s - b
            ei = e - b
            if si < 0:
                si = 0
            if ei > len(bbuf):
                ei = len(bbuf)
            if ei <= si:
                continue
            mn, mx = math.inf, -math.inf
            imn = imx = b + si
            j = si
            while j < ei:
                vv = bbuf[j]
                if vv < mn:
                    mn = vv
                    imn = b + j
                if vv > mx:
                    mx = vv
                    imx = b + j
                j += inner
            kl = ei - 1
            vv = bbuf[kl]
            if vv < mn:
                mn = vv
                imn = b + kl
            if vv > mx:
                mx = vv
                imx = b + kl
            if mn == math.inf:
                mn = mx = math.nan
            out.append((imn, mn, imx, mx))
        return out
