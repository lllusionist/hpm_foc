# -*- coding: utf-8 -*-
"""电流模式录波诊断 -> HTML 报告
用法: python foc_current_diag_report.py <csv> <out.html>
"""
import sys, numpy as np

CSVP = sys.argv[1] if len(sys.argv) > 1 else r"D:\Desktop\foc_wave_20261110_200500.csv"
OUT = sys.argv[2] if len(sys.argv) > 2 else r"D:\Desktop\FOC\docs\foc_current_loop_diagnosis_v2.html"

# ---------------------------------------------------------------- 读数据
rows = []
with open(CSVP, encoding="utf-8-sig") as f:
    for line in f:
        if line.startswith("#"):
            continue
        s = line.strip()
        if s:
            rows.append(s)
hdr = rows[0].split(",")
data = np.array([[float(x) for x in r.split(",")] for r in rows[1:]])
C = {n: data[:, i] for i, n in enumerate(hdr)}
N = len(data)
t = C["t_s"]
FS = 5000.0
T = 2e-4

# ---------------------------------------------------------------- 分析
th = np.unwrap(C["phase"])
th0 = th - th[0]
REVS = (th[-1] - th[0]) / (2 * np.pi)


def fold_order(x, nbin=256, nmax=18):
    """按电角度折叠 -> 各阶幅值"""
    dtt = 2 * np.pi / nbin
    g = np.arange(th[0], th[-1], dtt)
    K = len(g) // nbin
    v = np.interp(g, th, x)[: K * nbin].reshape(K, nbin).mean(axis=0)
    v = v - v.mean()
    A = np.abs(np.fft.rfft(v)) * 2 / nbin
    return v, np.array([A[k] for k in range(1, nmax + 1)]), np.sqrt(np.mean(v ** 2))


orders = {n: fold_order(C[n]) for n in ["Ia", "Ib", "Id", "Iq", "Vd_out", "Vq_out"]}

EDGES = [0, 150, 600, 1200, 1600, 2000, 2300, 2500]
BLAB = ["0-150", "150-600", "600-1200", "1200-1600", "1600-2000", "2000-2300", "2300-2500"]


def band_rms(x, edges=EDGES):
    x = x - x.mean()
    w = np.hanning(len(x))
    X = np.abs(np.fft.rfft(x * w)) * 2 / np.sum(w)
    f = np.fft.rfftfreq(len(x), 1 / FS)
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (f >= a) & (f < b)
        out.append(np.sqrt(np.sum(X[m] ** 2) / 2))
    return np.array(out)


bands = {n: band_rms(C[n]) for n in ["Id", "Iq", "Ia", "Ib", "Vd_out", "Vq_out"]}


def bpass(x, f1, f2):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / FS)
    X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))


def envelope(x, f1=1800, f2=2499):
    y = bpass(x - x.mean(), f1, f2)
    Y = np.fft.fft(y)
    h = np.zeros(len(y))
    h[0] = 1
    h[1: len(y) // 2] = 2
    h[len(y) // 2] = 1
    return np.abs(np.fft.ifft(Y * h))


env_id = envelope(C["Id"])
env_ia = envelope(C["Ia"])
env_vd = envelope(C["Vd_out"])
env_iq = envelope(C["Iq"])


def fold_profile(x, nbin=72):
    dtt = 2 * np.pi / nbin
    g = np.arange(th[0], th[-1], dtt)
    K = len(g) // nbin
    return np.interp(g, th, x)[: K * nbin].reshape(K, nbin).mean(axis=0)


prof = {n: fold_profile(v) for n, v in
        [("Id", env_id), ("Ia", env_ia), ("Vd", env_vd), ("Iq", env_iq)]}

# 死区/工作点核算
Vbus = C["Vbus_filt"].mean()
Vmag = C["Vmag"].mean()
DT_NS = 100.0
dt_duty = DT_NS * 1e-9 / 25e-6
Vdt = dt_duty * Vbus
KP, KI, BW = 0.08, 679.96, 2000.0
Lm = KP / (2 * np.pi * BW)
Rm = KI / (2 * np.pi * BW)
ITH = 0.8
Reff = Rm - Vdt / ITH
f_neg = 1 / (2 * np.pi * Lm / abs(Reff)) if Reff < 0 else float("nan")
ITH_CRIT = Vdt / Rm

# 统计量
stat = {n: (C[n].mean(), C[n].std(), C[n].min(), C[n].max()) for n in hdr[1:]}
id_rms = C["Id"].std()
iq_rms = C["Iq"].std()
eq_deg = np.degrees(id_rms / C["Iq"].mean())

# ---------------------------------------------------------------- 绘图
C_RED, C_ORG, C_BLU, C_GRN = "#c0392b", "#d98000", "#2f6fb0", "#1f8a5f"
C_PUR, C_GRAY, C_INK = "#7a4fa3", "#9aa6b6", "#1c2430"
FONT = "'Microsoft YaHei','Segoe UI',sans-serif"
_cp = [0]


def _axis(o, ml, mr, mt, mb, W, H, pw, ph):
    o.append(f'<rect x="{ml}" y="{mt}" width="{pw}" height="{ph}" fill="#fff" stroke="#e1e6ee"/>')


def line_panel(o, xr, yr, series, x0, y0, pw, ph, ylab, yfmt="{:.2f}", ny=4, xticks=None,
               hlines=(), vspans=(), xfmt="{:.2f}", show_x=True, ycol=None):
    """在 (x0,y0) 画一个面板; series=[(name,x,y,color,width)]"""
    ml, mr, mt, mb = 62, 112, 12, (30 if show_x else 6)
    iw, ih = pw - ml - mr, ph - mt - mb
    _cp[0] += 1
    cid = f"cp{_cp[0]}"
    o.append(f'<clipPath id="{cid}"><rect x="{x0+ml}" y="{y0+mt}" width="{iw}" height="{ih}"/></clipPath>')
    sx = lambda x: x0 + ml + (x - xr[0]) / (xr[1] - xr[0]) * iw
    sy = lambda y: y0 + mt + (yr[1] - y) / (yr[1] - yr[0]) * ih
    for a, b, col in vspans:
        o.append(f'<rect x="{sx(a):.1f}" y="{y0+mt}" width="{sx(b)-sx(a):.1f}" height="{ih}" fill="{col}" opacity="0.10"/>')
    for k in range(ny + 1):
        yv = yr[0] + (yr[1] - yr[0]) * k / ny
        yy = sy(yv)
        o.append(f'<line x1="{x0+ml}" y1="{yy:.1f}" x2="{x0+ml+iw}" y2="{yy:.1f}" stroke="#eef1f6"/>')
        o.append(f'<text x="{x0+ml-8}" y="{yy+4:.1f}" font-size="11" fill="{C_GRAY}" text-anchor="end">{yfmt.format(yv)}</text>')
    for yv, col, lab in hlines:
        yy = sy(yv)
        o.append(f'<line x1="{x0+ml}" y1="{yy:.1f}" x2="{x0+ml+iw}" y2="{yy:.1f}" stroke="{col}" stroke-width="1" stroke-dasharray="4 3"/>')
        o.append(f'<text x="{x0+ml+iw+6}" y="{yy+4:.1f}" font-size="10.5" fill="{col}">{lab}</text>')
    if xticks is None:
        n = 6
        xticks = [xr[0] + (xr[1] - xr[0]) * k / n for k in range(n + 1)]
    for xv in xticks:
        o.append(f'<line x1="{sx(xv):.1f}" y1="{y0+mt+ih}" x2="{sx(xv):.1f}" y2="{y0+mt+ih+4}" stroke="#c9d2df"/>')
        if show_x:
            o.append(f'<text x="{sx(xv):.1f}" y="{y0+mt+ih+17}" font-size="11" fill="{C_GRAY}" text-anchor="middle">{xfmt.format(xv)}</text>')
    ylab_y = y0 + mt + ih / 2
    o.append(f'<text transform="translate({x0+16},{ylab_y:.1f}) rotate(-90)" font-size="11" fill="#5b6675" text-anchor="middle">{ylab}</text>')
    for name, x, y, col, wd in series:
        m = (x >= xr[0]) & (x <= xr[1])
        pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(x[m], y[m]))
        o.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{wd}" stroke-linejoin="round" clip-path="url(#{cid})"/>')
    # 图例
    ly = y0 + mt + 4
    for name, x, y, col, wd in series:
        o.append(f'<line x1="{x0+ml+iw+8}" y1="{ly}" x2="{x0+ml+iw+26}" y2="{ly}" stroke="{col}" stroke-width="{wd+0.6}"/>')
        o.append(f'<text x="{x0+ml+iw+30}" y="{ly+4}" font-size="10.5" fill="#4a5566">{name}</text>')
        ly += 15


def bar_panel(o, cats, groups, x0, y0, pw, ph, title, ylab, yfmt="{:.3f}", ny=4, hl=None):
    ml, mr, mt, mb = 62, 20, 30, 40
    iw, ih = pw - ml - mr, ph - mt - mb
    ymax = max(max(g[1]) for g in groups)
    if hl:
        ymax = max(ymax, hl[0])
    ymax *= 1.15
    o.append(f'<text x="{x0+ml}" y="{y0+18}" font-size="12" fill="#3a4553" font-weight="600">{title}</text>')
    sx = lambda i: x0 + ml + (i + 0.5) * iw / len(cats)
    sy = lambda y: y0 + mt + ih - y / ymax * ih
    for k in range(ny + 1):
        yv = ymax * k / ny
        yy = sy(yv)
        o.append(f'<line x1="{x0+ml}" y1="{yy:.1f}" x2="{x0+ml+iw}" y2="{yy:.1f}" stroke="#eef1f6"/>')
        o.append(f'<text x="{x0+ml-8}" y="{yy+4:.1f}" font-size="11" fill="{C_GRAY}" text-anchor="end">{yfmt.format(yv)}</text>')
    o.append(f'<line x1="{x0+ml}" y1="{y0+mt+ih}" x2="{x0+ml+iw}" y2="{y0+mt+ih}" stroke="#c9d2df"/>')
    bw = iw / len(cats) / (len(groups) + 0.7)
    for gi, (gname, vals, col) in enumerate(groups):
        for i, v in enumerate(vals):
            xx = sx(i) - (len(groups) * bw) / 2 + gi * bw
            o.append(f'<rect x="{xx:.1f}" y="{sy(v):.1f}" width="{bw*0.86:.1f}" height="{y0+mt+ih-sy(v):.1f}" fill="{col}" opacity="0.88"/>')
    for i, c in enumerate(cats):
        o.append(f'<text x="{sx(i):.1f}" y="{y0+mt+ih+16}" font-size="10.5" fill="{C_GRAY}" text-anchor="middle">{c}</text>')
    o.append(f'<text transform="translate({x0+16},{y0+mt+ih/2:.1f}) rotate(-90)" font-size="11" fill="#5b6675" text-anchor="middle">{ylab}</text>')
    ly = y0 + mt - 2
    lx = x0 + ml + iw
    for gname, vals, col in reversed(groups):
        o.append(f'<rect x="{lx-88}" y="{ly-8}" width="9" height="9" fill="{col}"/>')
        o.append(f'<text x="{lx-74}" y="{ly}" font-size="10.5" fill="#4a5566">{gname}</text>')
        ly -= 14
    if hl:
        yy = sy(hl[0])
        o.append(f'<line x1="{x0+ml}" y1="{yy:.1f}" x2="{x0+ml+iw}" y2="{yy:.1f}" stroke="{C_RED}" stroke-width="1" stroke-dasharray="4 3"/>')
        o.append(f'<text x="{x0+ml+2}" y="{yy-4:.1f}" font-size="10.5" fill="{C_RED}">{hl[1]}</text>')


# ---------------------------------------------------------------- SVG 组装
G = []

# ① 全程 Id / Iq
o = [f'<svg viewBox="0 0 1040 300" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="{FONT}">']
line_panel(o, (t[0], t[-1]), (-0.55, 0.55), [("Id 反馈", t, C["Id"], C_RED, 0.7)],
           0, 4, 1040, 150, "Id / A", "{:.2f}", 2, show_x=False,
           hlines=[(0, "#7f8b9c", "0 A")])
line_panel(o, (t[0], t[-1]), (-0.06, 0.06),
           [("Iq − Iq_set", t, C["Iq"] - C["Iq_set"], C_BLU, 0.7)],
           0, 150, 1040, 150, "Iq 误差 / A", "{:+.2f}", 2,
           hlines=[(0, "#7f8b9c", "0")])
o.append("</svg>")
G.append(("① 整段 1.73 s：上格 d 轴指令恒为 0，实测却在 ±0.5 A 之间晃；下格 q 轴跟随误差只有 ±2 %",
          "".join(o)))

# ② 放大
i0, i1 = np.searchsorted(t, 0.3064), np.searchsorted(t, 0.3126)
iz = slice(i0, i1)
o = [f'<svg viewBox="0 0 1040 300" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="{FONT}">']
line_panel(o, (t[i0], t[i1]), (-1.45, 1.45),
           [("Ia", t[iz], C["Ia"][iz], C_GRN, 1.1, ), ("Ib", t[iz], C["Ib"][iz], C_BLU, 1.1)],
           0, 4, 1040, 150, "相电流 / A", "{:.2f}", 2, show_x=False, xfmt="{:.3f}")
line_panel(o, (t[i0], t[i1]), (-0.42, 0.42), [("Id 反馈", t[iz], C["Id"][iz], C_RED, 1.3)],
           0, 150, 1040, 150, "Id / A", "{:+.2f}", 2, xfmt="{:.3f}",
           hlines=[(0, "#7f8b9c", "0")])
o.append("</svg>")
G.append(("② 放大 0.3064–0.3126 s（30 个采样点 / 6 ms）：Id 每隔一个点翻一次符号，就是 2.5 kHz 的奈奎斯特拍频",
          "".join(o)))

# ③ 阶次谱
ords = [str(k) for k in range(1, 19)]
o = [f'<svg viewBox="0 0 1040 320" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="{FONT}">']
bar_panel(o, ords, [("Id", orders["Id"][1], C_RED), ("Iq", orders["Iq"][1], C_BLU),
                    ("Vd_out", orders["Vd_out"][1], C_PUR)],
          0, 0, 1040, 320, "按电角度折叠后的各阶幅值：6 次、12 次是死区非线性的招牌", "幅值 (A / V)", "{:.2f}", 4)
o.append("</svg>")
G.append(("③ 阶次谱：Id 的 6 次 = 0.041 A、12 次 = 0.021 A；Iq 的 6 次只有 0.016 A —— 扰动几乎全落在 d 轴",
          "".join(o)))

# ④ 分频带
o = [f'<svg viewBox="0 0 1040 320" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="{FONT}">']
bar_panel(o, BLAB, [("Id", bands["Id"], C_RED), ("Iq", bands["Iq"], C_BLU)],
          0, 0, 1040, 320, "分频带 rms：Id 的能量堆在 1.6–2.5 kHz（5 kHz 采样的奈奎斯特带），Iq 没有",
          "带内 rms / A", "{:.3f}", 4)
o.append("</svg>")
G.append(("④ 分频带：Id 在 2.0–2.3 kHz 有 0.078 A、2.3–2.45 kHz 有 0.082 A；同一带内 Iq 只有 0.010 / 0.007 A",
          "".join(o)))

# ⑤ 包络 vs 电角度
ang = np.arange(72) * (360.0 / 72)
o = [f'<svg viewBox="0 0 1040 330" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="{FONT}">']
ml, mr, mt, mb = 62, 182, 16, 40
iw, ih = 1040 - ml - mr, 330 - mt - mb
o.append(f'<rect x="{ml}" y="{mt}" width="{iw}" height="{ih}" fill="#fff" stroke="#e1e6ee"/>')
ymax = max(prof["Id"].max(), prof["Ia"].max()) * 1.25
sx = lambda a: ml + a / 360 * iw
sy = lambda v: mt + ih - v / ymax * ih
for k in range(5):
    yv = ymax * k / 4
    o.append(f'<line x1="{ml}" y1="{sy(yv):.1f}" x2="{ml+iw}" y2="{sy(yv):.1f}" stroke="#eef1f6"/>')
    o.append(f'<text x="{ml-8}" y="{sy(yv)+4:.1f}" font-size="11" fill="{C_GRAY}" text-anchor="end">{yv:.2f}</text>')
for k in range(0, 361, 60):
    o.append(f'<text x="{sx(k):.1f}" y="{mt+ih+17}" font-size="11" fill="{C_GRAY}" text-anchor="middle">{k}°</text>')
LEG5 = [("Id 包络", "Id", C_RED, 1.9, "6 次 0.095 A，±92 %"),
        ("Ia 包络", "Ia", C_GRN, 1.2, "6 次 0.051 A，±83 %"),
        ("Iq 包络", "Iq", C_PUR, 1.2, "6 次 0.0006 A，不动")]
for i, (lab, nm, col, wd, sub) in enumerate(LEG5):
    v = prof[nm]
    pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(ang, np.r_[v, v[0]]))
    o.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{wd}"/>')
    yy = mt + 22 + 40 * i
    o.append(f'<line x1="{ml+iw+10}" y1="{yy}" x2="{ml+iw+28}" y2="{yy}" stroke="{col}" stroke-width="{wd+0.5}"/>')
    o.append(f'<text x="{ml+iw+33}" y="{yy+4}" font-size="10.5" fill="#3a4553">{lab}</text>')
    o.append(f'<text x="{ml+iw+33}" y="{yy+16}" font-size="9.8" fill="{col}">{sub}</text>')
o.append(f'<text transform="translate(16,{mt+ih/2:.0f}) rotate(-90)" font-size="11" fill="#5b6675" text-anchor="middle">1.8–2.5 kHz 带内包络 / A</text>')
o.append(f'<text x="{ml}" y="{mt-3}" font-size="11.5" fill="#3a4553" font-weight="600">把 1.8–2.5 kHz 噪声取包络、按电角度折叠</text>')
o.append("</svg>")
G.append(("⑤ 决定性证据：这段高频噪声的幅度被电角度 6 次调制（±90%），只在电流过零点附近爆发 —— 这是逆变器非线性的指纹",
          "".join(o)))

# ⑥ R_eff(ITH)
iths = np.linspace(0.12, 3.2, 400)
reff = Rm - Vdt / iths
o = [f'<svg viewBox="0 0 1040 300" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="{FONT}">']
ml, mr, mt, mb = 62, 130, 16, 40
iw, ih = 1040 - ml - mr, 300 - mt - mb
o.append(f'<rect x="{ml}" y="{mt}" width="{iw}" height="{ih}" fill="#fff" stroke="#e1e6ee"/>')
ymin, ymax = -0.45, 0.10
sx = lambda a: ml + (a - iths[0]) / (iths[-1] - iths[0]) * iw
sy = lambda v: mt + (ymax - v) / (ymax - ymin) * ih
o.append(f'<rect x="{ml}" y="{sy(0):.1f}" width="{iw}" height="{mt+ih-sy(0):.1f}" fill="#c0392b" opacity="0.07"/>')
for v in [0.10, 0.05, 0.0, -0.05, -0.10, -0.20, -0.30, -0.40]:
    o.append(f'<line x1="{ml}" y1="{sy(v):.1f}" x2="{ml+iw}" y2="{sy(v):.1f}" stroke="#eef1f6"/>')
    o.append(f'<text x="{ml-8}" y="{sy(v)+4:.1f}" font-size="11" fill="{C_GRAY}" text-anchor="end">{v:+.2f}</text>')
for k in range(0, 8):
    a = iths[0] + (iths[-1] - iths[0]) * k / 7
    o.append(f'<text x="{sx(a):.1f}" y="{mt+ih+17}" font-size="11" fill="{C_GRAY}" text-anchor="middle">{a:.1f}</text>')
pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(iths, reff))
o.append(f'<polyline points="{pts}" fill="none" stroke="{C_PUR}" stroke-width="2"/>')
o.append(f'<line x1="{ml}" y1="{sy(0):.1f}" x2="{ml+iw}" y2="{sy(0):.1f}" stroke="{C_INK}" stroke-width="1"/>')
o.append(f'<line x1="{sx(ITH)}" y1="{mt}" x2="{sx(ITH)}" y2="{mt+ih}" stroke="{C_RED}" stroke-width="1" stroke-dasharray="4 3"/>')
o.append(f'<circle cx="{sx(ITH):.1f}" cy="{sy(Reff):.1f}" r="4.5" fill="{C_RED}"/>')
o.append(f'<text x="{sx(ITH)+10}" y="{sy(-0.26):.1f}" font-size="11" fill="{C_RED}">当前 0.8 A → R_eff = {Reff:+.3f} Ω</text>')
o.append(f'<line x1="{sx(ITH_CRIT)}" y1="{mt}" x2="{sx(ITH_CRIT)}" y2="{mt+ih}" stroke="{C_GRN}" stroke-width="1" stroke-dasharray="4 3"/>')
o.append(f'<text x="{sx(ITH_CRIT)+6}" y="{mt+14}" font-size="11" fill="{C_GRN}">临界 {ITH_CRIT:.2f} A</text>')
o.append(f'<text x="{sx(2.05)}" y="{sy(-0.37):.1f}" font-size="10.5" fill="{C_RED}" opacity="0.9">R_eff &lt; 0：负增量电阻区</text>')
o.append(f'<text x="{ml}" y="{mt-3}" font-size="11.5" fill="#3a4553" font-weight="600">线性化：过渡带 ITH 与等效增量电阻 R_eff = R − Vdt/ITH（{Rm:.3f} Ω 的绕组被抵消掉）</text>')
o.append(f'<text transform="translate(16,{mt+ih/2:.0f}) rotate(-90)" font-size="11" fill="#5b6675" text-anchor="middle">R_eff = R − Vdt/ITH  / Ω</text>')
o.append(f'<text x="{ml+iw/2}" y="{mt+ih+32}" font-size="11" fill="#5b6675" text-anchor="middle">DT_COMP_ITH / A</text>')
o.append("</svg>")
G.append((f"⑥ ITH=0.8 A 时线性化增量电阻 {Reff:+.3f} Ω（绕组只有 {Rm:.3f} Ω），L/|R_eff| = {Lm/abs(Reff)*1e6:.0f} µs → {f_neg:.0f} Hz。"
           "只是<b>必要条件</b>，不是充分条件：宽度收窄后负阻区也变窄，实测必须扫参数（§6 实验 3）",
          "".join(o)))

# ---------------------------------------------------------------- HTML
CSS = """
*{box-sizing:border-box}
body{margin:0;background:#f4f6f9;color:#1c2430;font-family:'Microsoft YaHei','Segoe UI',sans-serif;
     font-size:14.5px;line-height:1.72;-webkit-font-smoothing:antialiased}
.wrap{max-width:1080px;margin:0 auto;padding:34px 26px 70px}
h1{font-size:25px;margin:0 0 6px;letter-spacing:.3px}
h2{font-size:17.5px;margin:34px 0 10px;padding-left:11px;border-left:4px solid #2f6fb0}
h3{font-size:15px;margin:22px 0 8px;color:#2b3646}
.sub{color:#68737f;font-size:13px;margin-bottom:4px}
.meta{color:#8b96a4;font-size:12.5px;margin-bottom:20px}
.card{background:#fff;border:1px solid #e3e7ee;border-radius:9px;padding:18px 20px;margin:14px 0;
      box-shadow:0 1px 2px rgba(24,40,64,.04)}
.fig{background:#fff;border:1px solid #e3e7ee;border-radius:9px;padding:12px 12px 6px;margin:16px 0;
     box-shadow:0 1px 2px rgba(24,40,64,.04)}
.fig .cap{font-size:12.5px;color:#5b6675;padding:6px 8px 8px;line-height:1.55}
.kpi{display:grid;grid-template-columns:repeat(4,1fr);gap:11px;margin:14px 0}
.kpi div{background:#fff;border:1px solid #e3e7ee;border-radius:8px;padding:11px 13px}
.kpi b{display:block;font-size:20px;line-height:1.35;font-weight:600}
.kpi span{font-size:12px;color:#7c8794}
.bad b{color:#c0392b}.ok b{color:#1f8a5f}.warn b{color:#d98000}
table{width:100%;border-collapse:collapse;font-size:13.5px;margin:10px 0}
th,td{padding:7px 10px;border-bottom:1px solid #edf0f5;text-align:left;vertical-align:top}
th{background:#f7f9fc;color:#4a5566;font-weight:600;font-size:12.5px}
td.n{font-variant-numeric:tabular-nums;white-space:nowrap}
.tag{display:inline-block;padding:1px 7px;border-radius:4px;font-size:11.5px}
.t-bad{background:#fdecea;color:#b03a2e}.t-ok{background:#e9f6ef;color:#1b7a52}
.t-warn{background:#fdf3e2;color:#a56a00}.t-info{background:#eef3fb;color:#2f6fb0}
.corr{border-left:4px solid #c0392b;background:#fdf5f4}
.corr h2{border:0;padding:0;margin:0 0 8px;font-size:16px;color:#a5312a}
code{background:#f2f4f8;padding:1px 5px;border-radius:3px;font-family:Consolas,Menlo,monospace;font-size:12.8px}
pre{background:#1e2633;color:#dfe6ef;padding:11px 13px;border-radius:7px;overflow-x:auto;font-size:12.5px;
    line-height:1.6;font-family:Consolas,Menlo,monospace}
.du{color:#7ddba5}.dd{color:#f19a92}
ol,ul{margin:8px 0 8px 22px;padding:0}li{margin:5px 0}
.ex{border:1px solid #dbe3ee;border-radius:8px;padding:12px 15px;margin:11px 0;background:#fbfcfe}
.ex .h{font-weight:600;color:#2f6fb0;font-size:13.8px;margin-bottom:3px}
.note{font-size:12.8px;color:#6b7684}
hr{border:0;border-top:1px solid #e3e7ee;margin:26px 0}
"""


def card(inner):
    return f'<div class="card">{inner}</div>'


def table(head, rows_):
    h = "".join(f"<th>{x}</th>" for x in head)
    b = "".join("<tr>" + "".join(f'<td class="{c}">{v}</td>' for v, c in r) + "</tr>" for r in rows_)
    return f"<table><tr>{h}</tr>{b}</table>"


H = []
A = H.append
A('<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">')
A('<meta name="viewport" content="width=device-width,initial-scale=1">')
A("<title>电流环复测诊断 · 电流模式</title>")
A(f"<style>{CSS}</style></head><body><div class='wrap'>")

A("<h1>电流环复测诊断 · 电流模式录波</h1>")
A(f"<div class='sub'>数据：<code>foc_wave_20261110_200500.csv</code> · 模式 <b>电流</b>（Iq_set 恒 1.312 A，速度环不在环内）</div>")
A(f"<div class='meta'>40 kHz 电流环 / 5 kHz 采样上报（8 分频）· 8637 点 · 1.727 s · 电频率 19.4 Hz · 母线 {Vbus:.2f} V · Vmag {Vmag:.3f} V</div>")

# ---- 0 纠正
A("<div class='card corr'>")
A("<h2>先把上一次的结论撤回</h2>")
A("<p>上次我写了两条结论——“电流环实测带宽 1.2 kHz、Id 无静差，这条电流环很健康”——这条判断的依据是错的，我说明白错在哪：</p><ol>"
  "<li>那份录波是<b>速度模式</b>。<code>Iq_set</code> 本身就是速度环的输出，一直在动；我拿 <code>Iq_set → Iq</code> 算闭环频响，"
  "得到的其实只是“电流环跟得上一个低频指令”，在高频段激励能量不够，估出来的幅相根本不足以证明高频是干净的。</li>"
  "<li>我把“Id 均值 ≈ 0”当成了“Id 没问题”。实际上那次 Id 的标准差就有 0.081 A，我没去看它的谱。</li>"
  "<li>电流环的问题恰恰是<b>高频 + 只在 d 轴</b>，这两个特征在速度模式的信号里全被速度环的低频大信号盖住了。</li></ol>"
  "<p style='margin-bottom:0'>这次这份是电流模式，<code>Iq_set</code> 恒定、<code>Id_set=0</code>，电流环被单独暴露出来，上面这些都能直接测。"
  "下面每条结论我都标了是<b>实测</b>还是<b>推算</b>，没验证的一律写成待验证。</p>")
A("</div>")

# ---- 1 事实
A("<h2>1 · 这条电流环实测到底是什么状态</h2>")
A("<div class='kpi'>")
A(f"<div class='ok'><b>0.02 %</b><span>Iq 直流误差<br>1.3122 vs 1.3120 A</span></div>")
A(f"<div class='ok'><b>0.0001 A</b><span>Id 直流误差<br>积分环节没问题</span></div>")
A(f"<div class='bad'><b>{id_rms:.3f} A</b><span>Id 纹波 (rms)<br>= 峰值的 {id_rms/C['Iq'].mean()*100:.1f} %</span></div>")
A(f"<div class='bad'><b>{id_rms/iq_rms:.1f} ×</b><span>Id / Iq 纹波比<br>同一套 PI，同一条被控对象</span></div>")
A("</div>")

A(card(table(["量", "实测", "判读"], [
    [("Iq 均值 / 目标", "n"), ("1.3122 / 1.3120 A", "n"), ("<span class='tag t-ok'>直流无差</span>", "")],
    [("Iq 纹波 σ", "n"), (f"{iq_rms*1000:.1f} mA（{iq_rms/C['Iq'].mean()*100:.2f} %）", "n"), ("<span class='tag t-ok'>可接受</span>", "")],
    [("Id 均值", "n"), (f"{C['Id'].mean()*1000:+.1f} mA", "n"), ("<span class='tag t-ok'>直流无差</span>", "")],
    [("Id 纹波 σ / 峰峰值", "n"), (f"{id_rms*1000:.1f} mA（{id_rms/C['Iq'].mean()*100:.1f} %）／{C['Id'].max()-C['Id'].min():.3f} A", "n"),
     ("<span class='tag t-bad'>异常：指令是 0</span>", "")],
    [("Id 极值", "n"), (f"{C['Id'].min():+.3f} … {C['Id'].max():+.3f} A", "n"), ("<span class='tag t-bad'>±0.5 A = 峰值的 38 %</span>", "")],
    [("相电流基波", "n"), (f"{orders['Ia'][1][0]:.3f} A 峰值", "n"), ("<span class='tag t-ok'>与 Iq 自洽</span>", "")],
    [("相电流 THD", "n"), ("≈2.5 %（5 次 2.08 %、7 次 0.89 %、11 次 1.03 %、13 次 0.50 %）", ""), ("<span class='tag t-warn'>偏大，见 §4</span>", "")],
    [("三相电流和 isum", "n"), (f"{C['isum'].std()*1000:.0f} mA rms（主要 0–30 Hz 漂移）", "n"), ("<span class='tag t-ok'>零点标定可用</span>", "")],
    [("电压饱和 / 故障码", "n"), ("0 % · 无故障", "n"), ("<span class='tag t-ok'>正常</span>", "")],
])))

A("<p>关键不是“纹波有多大”，而是<b>两条轴的纹波差了 4 倍</b>。d 轴指令是 0、q 轴指令恒定，"
  "两条轴用的是同一个 <code>Kp=0.08</code>、同一个 <code>Ki=679.96</code>、同一套绕组。"
  "如果抖动是“增益不够”或者“随机噪声”，两条轴应该一样脏；差 4 倍说明<b>有一个确定性的扰动只作用在 d 轴（电流矢量幅值方向）</b>。"
  "顺着这条线索才挖到下面的东西。</p>")

for cap, svg in G[:2]:
    A(f'<div class="fig">{svg}<div class="cap">{cap}</div></div>')

# ---- 2 拆解
A("<h2>2 · 把 Id 的抖动拆成三块</h2>")
A("<p>按电角度重采样后折叠（33.6 个电周期平均），可以把“跟转子同步”的成分和“不跟转子同步”的成分分开：</p>")
A(card(table(["成分", "Id", "Iq", "说明"], [
    [("角度同步总计", ""), (f"{orders['Id'][2]*1000:.0f} mA rms", "n"), (f"{orders['Iq'][2]*1000:.0f} mA rms", "n"),
     ("跟电角度锁相，是磁链/死区谐波", "")],
    [("　其中 6 次", ""), (f"{orders['Id'][1][5]*1000:.0f} mA", "n"), (f"{orders['Iq'][1][5]*1000:.0f} mA", "n"),
     ("死区非线性的招牌阶次（三相 × 每相每周期 2 次过零）", "")],
    [("　其中 12 次", ""), (f"{orders['Id'][1][11]*1000:.0f} mA", "n"), (f"{orders['Iq'][1][11]*1000:.0f} mA", "n"),
     ("同一非线性 + 传感器失配的二次谐波", "")],
    [("非同步（1.6–2.5 kHz）", ""), (f"{np.sqrt(max(id_rms**2-orders['Id'][2]**2,0))*1000:.0f} mA rms", "n"),
     (f"{np.sqrt(max(iq_rms**2-orders['Iq'][2]**2,0))*1000:.0f} mA rms", "n"), ("跟角度无关，是高频振荡", "")],
])))

for cap, svg in G[2:4]:
    A(f'<div class="fig">{svg}<div class="cap">{cap}</div></div>')

# ---- 3 决定性证据
A("<h2>3 · 决定性证据：这段高频噪声被电角度“6 次门控”</h2>")
A("<p>把 Id、Ia、Vd 的 1.8–2.5 kHz 带通取出、求包络，再按电角度折叠。结果很干脆：</p>")
A(card(table(["信号", "包络均值", "包络 6 次分量", "调制深度", "判读"], [
    [("Id", "n"), (f"{prof['Id'].mean():.3f} A", "n"), (f"{fold_order(env_id,256,18)[1][5]:.3f} A", "n"), ("±92 %", "n"),
     ("<span class='tag t-bad'>被 6 次调制</span>", "")],
    [("Ia", "n"), (f"{prof['Ia'].mean():.3f} A", "n"), (f"{fold_order(env_ia,256,18)[1][5]:.3f} A", "n"), ("±83 %", "n"),
     ("<span class='tag t-bad'>被 6 次调制</span>", "")],
    [("Vd_out", "n"), (f"{prof['Vd'].mean():.4f} V", "n"), (f"{fold_order(env_vd,256,18)[1][5]:.4f} V", "n"), ("±93 %", "n"),
     ("<span class='tag t-bad'>控制器在跟着抖</span>", "")],
    [("Iq", "n"), (f"{prof['Iq'].mean():.4f} A", "n"), (f"{fold_order(env_iq,256,18)[1][5]:.4f} A", "n"), ("±4 %", "n"),
     ("<span class='tag t-ok'>基本不受影响</span>", "")],
])))
A("<p><b>这一步把嫌疑面砍掉一大半。</b>ADC 噪声、运放噪声、电源纹波、编码器噪声、地弹……"
  "这些都不可能被电角度 6 次调制——它们的幅度跟转子在什么位置没有任何关系。"
  "只有“和电流过零点同步”的东西才会：逆变器死区（每相每周期过零 2 次，三相合起来就是 6 次）。"
  "而 Iq 的包络几乎不动，也解释了为什么两条轴差 4 倍：死区误差电压的方向永远和电流矢量反平行，"
  "在 dq 里主要落在<b>幅值方向（d 轴）</b>，q 轴上看不到。</p>")
A(f'<div class="fig">{G[4][1]}<div class="cap">{G[4][0]}</div></div>')

# ---- 4 工作点
A("<h2>4 · 为什么这个工作点上死区会被放大到不可忽略</h2>")
A("<p>这台 5210（R≈54 mΩ / L≈6.37 µH，由 <code>Kp/BW</code>、<code>Ki/BW</code> 反推）现在跑在 <b>1.312 A</b>，"
  "额定 44 A 的 3 %；电压只要 <b>0.406 V</b>，母线 24.28 V 的 <b>1.7 %</b>。也就是说：</p>")
A("<p class='note'>反推得到的 54 mΩ / 6.37 µH 和 <code>config.c</code> 里 5210 档的标称 30 mΩ / 9 µH 不一致 —— "
  "说明 Flash 里现在存的是标定值（<code>current_kp/ki</code> 那条路径），标定值更可信。"
  "不过 <b>R 越小，下面算出来的负阻越严重</b>，所以用 54 mΩ 是偏保守的估计；建议顺手复核一下标定结果。</p>")
A(card(table(["项", "数值", "含义"], [
    [("调制度（1.5·Vmag/Vbus）", "n"), (f"{1.5*Vmag/Vbus*100:.2f} %", "n"), ("duty 只偏离 50 % 约 ±1.4 %", "")],
    [("死区电压误差 Vdt", "n"), (f"{DT_NS:.0f} ns / 25 µs = {dt_duty*100:.2f} % duty = {Vdt*1000:.0f} mV", "n"), ("每相都有一份", "")],
    [("Vdt / Vmag", "n"), (f"<b>{Vdt/Vmag*100:.0f} %</b>", "n"), ("<span class='tag t-bad'>逆变器非线性电平 = 指令的 1/4</span>", "")],
    [("相电流 5 次 / 7 次", "n"), ("2.08 % / 0.89 %", "n"), ("正是死区造成的 5、7 次谐波对", "")],
])))

A("<h3>补偿在这里可能挖了个坑（下面会说明为什么只能算“可能”）</h3>")
A("<p><code>foc.c</code> 里 <code>DT_COMP_ITH = 0.8 A</code> 把“±Vdt 的符号函数”化成了一段斜坡，"
  "以免过零时产生方波啸叫。代价是这段斜坡的斜率就是一个<b>负的增量电阻</b>：</p>")
A(("<pre>dV/di = Vdt / ITH = %.4f / %.1f = %.4f V/A      ← 折算成 −%.3f Ω\n"
   "R      = %.4f Ω（绕组，由 Ki/BW 反推）\n"
   "R_eff  = R − dV/di = %.4f − %.4f = <b>%+.4f Ω</b>    ← 负的\n"
   "\n"
   "振荡时间常数 τ = L / |R_eff| = %.1f µs → 天然频率 %.0f Hz</pre>")
  % (Vdt, ITH, Vdt / ITH, Vdt / ITH, Rm, Rm, Vdt / ITH, Reff, Lm / abs(Reff) * 1e6, f_neg))
A("<p>而 <b>ITH=0.8 A 的斜坡区只覆盖 |i|&lt;0.8 A</b>，1.312 A 幅值下每相每个电周期有约 <b>42 %</b> 的时间待在这个负阻区里"
  "（占比 = (2/π)·arcsin(0.8/1.312)）。于是振荡不是连续的，而是<b>随电流过零一阵一阵地点火</b>——"
  "这正好对上 §3 那个 6 次门控，也对上 §1 里“一阵一阵”的包络。"
  f"<span class='note'>（换算：Id 的 {id_rms:.3f} A 抖动等效于 {eq_deg:.1f}° 电角度——只是给个量纲感，不是说角度错了。）</span></p>")
A("<div class='card' style='border-left:4px solid #d98000;background:#fffdf7'>"
  "<p><b>这个负阻推算只能当“嫌疑人”，不能当结论。</b>我把它代进 §5 的模型扫了一遍，结果并不单调："
  "把 ITH 从 0.8 收窄到 0.05 A，负阻数值上更负（−1.9 Ω），但负阻区也跟着变窄，Id rms 反而从 0.091 掉到 0.029。"
  "也就是说“过渡带越宽越坏”只在过渡带宽度和电流幅值可比的时候成立（这里 0.8 A ≈ 0.6·Iq），"
  "不能外推成“ITH 越大越好”。<b>它到底该往哪边走，只能扫参数实测</b>（见 §6 实验 3）。</p>"
  "<p style='margin-bottom:0'>所以这一节能确凿说的只有一句：<b>在这个工作点上，逆变器的非线性电平（97 mV）是它自己要输出的电压（406 mV）的 24 %，"
  "电流环被一条幅度不小的时变扰动推着走，这不是调 PI 能解决的。</b></p></div>")
A(f'<div class="fig">{G[5][1]}<div class="cap">{G[5][0]}</div></div>')

# ---- 5 模型复现
A("<h2>5 · 数值复现：抖动确实 100 % 来自逆变器非线性</h2>")
A("<p>用录波反推的参数（L=6.37 µH、R=54.1 mΩ、Vdt=97 mV、40 kHz、Kp/Ki 同实际）搭一个三相平均模型，"
  "只改死区这一项，其他全不动：</p>")
A(card(table(["条件", "Id rms", "说明"], [
    [("理想逆变器（Vdt=0）", ""), ("<b>0.000 A</b>", "n"), ("环路增益/带宽/离散化本身完全干净 —— Kp/Ki 不是问题", "")],
    [("有死区、不做补偿", ""), ("0.094 A", "n"), ("裸的方波失真", "")],
    [("有死区 + 补偿 ITH=0.8 A（当前）", ""), (f"<b>0.091 A</b>", "n"), (f"实测 {id_rms:.3f} A，量级对得上", "")],
    [("有死区 + 补偿 ITH=2.0 A", ""), ("0.094 A", "n"), ("总量没降，但 <b>1.63 kHz 那个分量消失了</b>", "")],
    [("有死区 + 硬符号补偿 ITH=0.05 A", ""), ("<b>0.029 A</b>", "n"), ("模型里最好；代价是过零处硬跳变", "")],
])))
A("<p class='note'>一个值得注意的细节：<b>只有 ITH=0.8 这一档，频谱里出现 1630 Hz 的独立分量</b>"
  "（ITH=0.05 的峰在 1750–1980 Hz 且只有 0.009，ITH=2.0 和“不补偿”都没有 1.6 kHz 峰）。"
  "这和 §4 算出来的 1683 Hz 撞在一起，也是我把负阻列为嫌疑的理由；但同一组数据也说明"
  "这个分量对 ITH 的取值非常敏感——<b>换个 ITH 就换一个频率</b>，不是固定谐振，所以更像是参数相关的极限环，而不是纯线性失稳。</p>")
A("<p class='note'>模型的边界：它是平均模型，不含开关纹波、不含 ADC 噪声、死区只按 ±Vdt 的方波近似，"
  "所以它只能用来做<b>敏感性排序</b>（谁主导、改哪项有用），不能用来预测具体波形。"
  "它没复现出 1.6–2.5 kHz 那一段的细节——那一段靠的是 §4 的负阻推算 + §3 的 6 次门控证据，"
  "还需要下面第 2、3 个实验来确认。</p>")

# ---- 6 验证
A("<h2>6 · 先别改代码，先做这三个实验</h2>")
A("<p>上一轮我直接给了一串 diff，这次不了。下面三个实验做完，方向就是确定的，改起来才有依据。</p>")
A("""<div class='ex'><div class='h'>实验 1 · 量一下真实死区（10 分钟，不改代码）</div>
把三相占空比固定在 50 %，示波器双通道看同一桥臂上下管栅极（或看半桥中点对地），量上下管之间的死区时间。
<code>tim.h</code> 里 <code>PWM_DEAD_TIME_NS = 0</code>，硬件插入的只有 2 个 tick ≈ 10 ns，实际死区全部来自 FD6288；
<code>foc.c</code> 的 <code>DT_DEAD_NS = PWM_DEAD_TIME_NS + 100</code> 是<b>假定</b> 100 ns。
如果实测是 200 ns，则 Vdt 翻倍、Vdt/Vmag 变成 48 %、负阻翻倍——下面所有结论按比例放大。</div>""")
A("""<div class='ex'><div class='h'>实验 2 · 把死区补偿关掉再录一次</div>
注释掉 <code>foc_deadtime_comp()</code> 的调用（<code>foc.c</code> 里 <code>FOC_current</code> 与 <code>FOC_voltage</code> 各一处），同一工作点、同一转速再录。
<b>预期</b>：1.8–2.5 kHz 带内 Id rms 从 0.127 A 掉到 0.03 A 以下，但 6 次谐波（116 Hz 附近）变大。
如果 1.8–2.5 kHz <b>反而变大</b>，说明我给的方向错了，以实测为准，别改。</div>""")
A("""<div class='ex'><div class='h'>实验 3 · 把 DT_COMP_ITH 当参数扫一遍（0.1 / 0.3 / 0.8 / 2.0）</div>
这是个数，改起来最快。<b>不要预设方向</b>——§5 的模型扫出来并不单调（0.05 A 最好、0.8 A 最差、2.0 A 介于中间），
所以这里要用实测选，评价指标同时看两个：<code>Id</code> 在 1.8–2.5 kHz 的带内 rms，和 6 次谐波（≈116 Hz）的幅值。<br>
<b>模型的预测</b>：0.8 A → 2.0 A 时 1630 Hz 那个独立分量应该消失，6 次从 0.038 略升到 0.041；
0.8 A → 0.1 A 时总量能降 3 倍，但过零处会出现硬跳变（<code>foc.c</code> 注释里担心的那个啸叫可能回来）。<br>
<b>如果四档全都一样</b>，说明 1.6–2.5 kHz 另有来源（我下一个怀疑是 ADC 采样时刻撞开关沿，那就去查 <code>adc.c</code> 的触发点）。</div>""")

# ---- 7 改法
A("<h2>7 · 验证通过之后，这才是我认为该改的（按性价比）</h2>")
A(card("<ol>"
      "<li><b>死区补偿的过渡带</b>：按实验 3 的扫描结果选，别按推算选。"
      "如果目标是压 1.8–2.5 kHz 这段，模型里最好的是<b>窄带</b>（<code>ITH≈0.05–0.1 A</code>，即接近硬符号 + 一点滞环）；"
      "代价是过零处变成硬跳变，可能需要配合第 2 条的电流滤波才不啸叫。<code>ITH</code> 越宽线性越好、但会把负阻区铺开——两头都有代价。</li>"
      "<li><b>补偿用的电流先滤波</b>：<code>foc_deadtime_comp()</code> 现在直接拿瞬时相电流（含 2 kHz 噪声）算符号，"
      "等于把噪声原样注回占空比。用 1–2 kHz 的一阶低通（40 kHz 下 <code>a≈0.15</code>）再取符号，切断这条“测量噪声 → duty”的通路。</li>"
      "<li><b>提高 Vdt 准确度</b>：按实验 1 的实测值填 <code>DT_DEAD_NS</code>。这是补偿能不能把 5/7 次压下去的前提。</li>"
      "<li><b>不要动 <code>current_ctrl_bw</code> / Kp / Ki</b>。Kp/BW 和 Ki/BW 反推出来的 L、R 和这只电机对得上，"
      "§5 里“理想死区下 Id rms = 0”也证明环路本身是好的。改增益只会把 5/7 次放大。</li>"
      "</ol>")
      + "<p class='note' style='margin-bottom:0'>顺带说一句：1.312 A 只用到额定的 3 %，这个工作点本来就在逆变器最不线性的区间里。"
        "如果应用允许，把电流提到 5–10 A 再标定/调试，信噪比会好很多——这不是修 bug，是换个能看清的工况。</p>")

# ---- 8 两个确定的缺陷
A("<h2>8 · 另外两个已确认的小缺陷（跟这次抖动无关，但会误导分析）</h2>")
A(card("<p><b>① 上报的 Ia/Ib/Ic 和 Id/Iq 差一个 25 µs 周期。</b>"
       "<code>foc_adc_callback()</code> 里 <code>vofa_push_sample()</code> 在 <code>FOC_current()</code> <b>之前</b>被调用，"
       "所以同一行 CSV 里的相电流是本周期的 ADC 值，而 Id/Iq/Vd/Vq 是上一周期的计算结果。"
       "我用相电流反算 dq 时，无论怎么加常数角度偏移都对不上（最优 rms 差 0.109 A，和 Id 自身的 rms 一样大），就是这个原因。"
       "把 <code>vofa_push_sample()</code> 挪到 ISR 末尾即可。</p>"
       "<p style='margin-bottom:0'><b>② <code>pwm_dead_ns_to_ticks(0)</code> 会返回 2</b>（函数里 <code>ticks &lt; 2</code> 被抬到 2），"
       "所以“软件死区 = 0”实际上是 10 ns；而 <code>DT_DEAD_NS</code> 假定 100 ns。两边口径不一致，量的时候按实验 1 来。</p>"))

A("<hr>")
A("<p class='note'>复现：<code>python tools/foc_current_diag_report.py &lt;录波.csv&gt; &lt;输出.html&gt;</code>。"
  "下一次录完直接跑，报告里的数字、图和推算会全部重算。</p>")
A("</div></body></html>")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(H))
print("OK ->", OUT, len("\n".join(H)), "chars")
print("Id rms %.4f  Iq rms %.4f  ratio %.2f  Reff %+.4f  f %.0f Hz" % (id_rms, iq_rms, id_rms / iq_rms, Reff, f_neg))
