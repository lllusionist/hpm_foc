# -*- coding: utf-8 -*-
"""离线分析工具: FFT / CSV 记录解析 / Clarke-Park 变换 (纯函数, 便于单测)."""
import math
import re

try:
    import numpy as np
    HAVE_NUMPY = True
except Exception:
    np = None
    HAVE_NUMPY = False

SQRT3 = math.sqrt(3.0)


def fft_magnitude(x, fs, win=True):
    """返回 (freq, amp): 单边幅值谱. amp 为对应正弦的幅值(已做窗归一)."""
    if not HAVE_NUMPY:
        return None, None
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 8 or fs <= 0:
        return None, None
    x = x - x.mean()
    if win:
        w = np.hanning(n)
        x = x * w
        corr = 2.0 / w.sum()
    else:
        corr = 2.0 / n
    spec = np.abs(np.fft.rfft(x)) * corr
    freq = np.fft.rfftfreq(n, 1.0 / fs)
    return freq, spec


def to_db(amp, floor=1e-6):
    if not HAVE_NUMPY:
        return None
    return 20.0 * np.log10(np.maximum(amp, floor))


def clarke(ia, ib, ic):
    """幅值不变 Clarke: (ialpha, ibeta)."""
    ia = np.asarray(ia, float)
    ib = np.asarray(ib, float)
    ic = np.asarray(ic, float)
    ialpha = ia
    ibeta = (ia + 2.0 * ib) / SQRT3
    return ialpha, ibeta


def park(ialpha, ibeta, theta):
    """Park: 返回 (id, iq)."""
    ialpha = np.asarray(ialpha, float)
    ibeta = np.asarray(ibeta, float)
    theta = np.asarray(theta, float)
    c = np.cos(theta)
    s = np.sin(theta)
    return ialpha * c + ibeta * s, -ialpha * s + ibeta * c


def _is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def load_csv(path):
    """解析上位机记录 CSV: 返回 (fs 或 None, 表头名列表, numpy 数据数组[N, 列])."""
    fs = None
    names = None
    rows = []
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as f:
        for line in f:
            if line.startswith("#"):
                m = re.search(r"采样率[:：]?\s*([\d.]+)\s*Hz", line)
                if m:
                    try:
                        fs = float(m.group(1))
                    except ValueError:
                        pass
                continue
            s = line.strip()
            if not s:
                continue
            parts = s.split(",")
            if names is None:
                if not _is_float(parts[0]):      # 表头行 (t_s,...)
                    names = parts
                    continue
                names = ["t_s"] + ["c%d" % i for i in range(1, len(parts))]
            rows.append(parts)
    if not HAVE_NUMPY:
        raise RuntimeError("需要 numpy 才能回放")
    arr = np.array(rows, dtype=float)
    return fs, names, arr
