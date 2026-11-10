# -*- coding: utf-8 -*-
import numpy as np, html, io, os

CSVP = r"D:\Desktop\foc_wave_20261110_193356.csv"
OUT  = r"D:\Desktop\FOC\docs\foc_current_loop_diagnosis.html"

rows = []
meta = {}
with open(CSVP, encoding='utf-8-sig') as f:
    for line in f:
        if line.startswith('#'):
            rows.append(line.strip()) if False else meta.setdefault('_', []).append(line.strip())
            continue
        rows.append(line.strip())
hdr = rows[0].split(',')
D = np.array([[float(x) for x in r.split(',')] for r in rows[1:]])
C = {n: D[:, i] for i, n in enumerate(hdr)}
t = C['t_s']; FS = 5000.0

# ---------- svg helpers ----------
def nice_ticks(lo, hi, n=5):
    span = hi - lo
    if span == 0: return [lo]
    raw = span / n
    mag = 10 ** np.floor(np.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            step = m * mag; break
    t0 = np.ceil(lo / step) * step
    return list(np.arange(t0, hi + step * 0.5, step))

_CPID = [0]
def chart(series, xr, yr, title, xlab, ylab, W=1040, H=250, fmt="{:.2f}",
          hlines=(), vlines=(), xticks=None, legend=True, xfmt="{:.2f}", xlog=False):
    """series: list of (name, x, y, color, width)"""
    _CPID[0] += 1
    ml, mr, mt, mb = 62, 120, 26, 34
    pw, ph = W - ml - mr, H - mt - mb
    def sx(x):
        if xlog:
            lx = np.log10(max(x, 1e-6)); l0 = np.log10(xr[0]); l1 = np.log10(xr[1])
            return ml + (lx - l0) / (l1 - l0) * pw
        return ml + (x - xr[0]) / (xr[1] - xr[0]) * pw
    def sy(y): return mt + ph - (y - yr[0]) / (yr[1] - yr[0]) * ph
    o = io.StringIO()
    o.write(f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="Segoe UI, Microsoft YaHei, sans-serif">')
    o.write(f'<rect x="0" y="0" width="{W}" height="{H}" fill="#ffffff"/>')
    # grid
    for yv in nice_ticks(yr[0], yr[1], 5):
        if yr[0] <= yv <= yr[1]:
            o.write(f'<line x1="{ml}" y1="{sy(yv):.1f}" x2="{ml+pw}" y2="{sy(yv):.1f}" stroke="#eceff3"/>')
            o.write(f'<text x="{ml-8}" y="{sy(yv)+4:.1f}" font-size="10.5" fill="#7a8699" text-anchor="end">{fmt.format(yv)}</text>')
    if xticks is None and xlog:
        xticks = [v for v in (10,20,50,100,200,500,1000,2000) if xr[0] <= v <= xr[1]]
    if xticks is None: xticks = nice_ticks(xr[0], xr[1], 6)
    for xv in xticks:
        if xr[0] <= xv <= xr[1]:
            o.write(f'<line x1="{sx(xv):.1f}" y1="{mt}" x2="{sx(xv):.1f}" y2="{mt+ph}" stroke="#f4f6f9"/>')
            o.write(f'<text x="{sx(xv):.1f}" y="{mt+ph+16}" font-size="10.5" fill="#7a8699" text-anchor="middle">{xfmt.format(xv)}</text>')
    o.write(f'<clipPath id="cp{_CPID[0]}"><rect x="{ml}" y="{mt}" width="{pw}" height="{ph}"/></clipPath>')
    o.write(f'<rect x="{ml}" y="{mt}" width="{pw}" height="{ph}" fill="none" stroke="#d8dee8"/>')
    for (yv, col, lab) in hlines:
        o.write(f'<line x1="{ml}" y1="{sy(yv):.1f}" x2="{ml+pw}" y2="{sy(yv):.1f}" stroke="{col}" stroke-dasharray="5 4" stroke-width="1.2"/>')
        o.write(f'<text x="{ml+pw-4}" y="{sy(yv)-4:.1f}" font-size="10" fill="{col}" text-anchor="end">{lab}</text>')
    for (xv, col, lab) in vlines:
        o.write(f'<line x1="{sx(xv):.1f}" y1="{mt}" x2="{sx(xv):.1f}" y2="{mt+ph}" stroke="{col}" stroke-dasharray="4 4" stroke-width="1"/>')
        o.write(f'<text x="{sx(xv)+3:.1f}" y="{mt+12}" font-size="10" fill="{col}">{lab}</text>')
    for (name, xs, ys, col, w) in series:
        m = (xs >= xr[0]) & (xs <= xr[1])
        if not m.any(): continue
        xs = xs[m]; ys = ys[m]
        pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(xs, ys))
        o.write(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{w}" stroke-linejoin="round" clip-path="url(#cp{_CPID[0]})"/>')
    if legend:
        y0 = mt + 4
        for i, (name, xs, ys, col, w) in enumerate(series):
            yy = y0 + i * 15
            o.write(f'<line x1="{ml+8}" y1="{yy}" x2="{ml+28}" y2="{yy}" stroke="{col}" stroke-width="{w+0.6}"/>')
            o.write(f'<text x="{ml+34}" y="{yy+4}" font-size="11" fill="#3c4a5e">{html.escape(name)}</text>')
    o.write(f'<text x="{ml}" y="{mt-9}" font-size="12.5" font-weight="600" fill="#22303f">{html.escape(title)}</text>')
    o.write(f'<text x="{ml+pw}" y="{H-4}" font-size="10.5" fill="#7a8699" text-anchor="end">{html.escape(xlab)}</text>')
    o.write(f'<text transform="translate(14,{mt+ph/2}) rotate(-90)" font-size="10.5" fill="#7a8699" text-anchor="middle">{html.escape(ylab)}</text>')
    o.write('</svg>')
    return o.getvalue()

def ds(n):
    return np.arange(0, len(t), n)

# ---------- data prep ----------
i_all = ds(5)                      # 1 kHz for overview
tz = (t >= 0.20) & (t <= 0.60)
zf = (t >= 2.05)
iz = np.arange(len(t))[tz]
ifz = np.arange(len(t))[zf]

C_BLUE = "#2f6fed"; C_RED = "#e2483d"; C_GREEN = "#12a06a"; C_GRAY = "#98a4b5"
C_ORANGE = "#ef8f1f"; C_PURPLE = "#7a53d6"

# spectra
def spec(x, win=True):
    x = x - x.mean()
    w = np.hanning(len(x)) if win else np.ones(len(x))
    X = np.abs(np.fft.rfft(x * w)) * 2 / np.sum(w)
    return np.fft.rfftfreq(len(x), 1 / FS), X

fv, Vv = spec(C['vel'])
fi, Vi = spec(C['Iq'])
fq, Vq = spec(C['Iq_set'])
fd_, Vd_ = spec(C['Id'])

# current loop bode
def tf(u, y, seg=2500, step=1250):
    win = np.hanning(seg); Sxy = Sxx = Syy = None
    for s in range(0, len(u) - seg + 1, step):
        U = np.fft.rfft((u[s:s+seg]-u[s:s+seg].mean())*win)
        Y = np.fft.rfft((y[s:s+seg]-y[s:s+seg].mean())*win)
        a, b, c = Y*np.conj(U), np.abs(U)**2, np.abs(Y)**2
        if Sxy is None: Sxy, Sxx, Syy = a, b, c
        else: Sxy, Sxx, Syy = Sxy+a, Sxx+b, Syy+c
    f = np.fft.rfftfreq(seg, 1/FS)
    H = Sxy/np.maximum(Sxx, 1e-20)
    return f, np.abs(H), np.degrees(np.angle(H)), np.abs(Sxy)**2/np.maximum(Sxx*Syy, 1e-20)

fb, Hb, Pb, cob = tf(C['Iq_set'], C['Iq'])
mb = (fb >= 2) & (fb <= 1500) & (cob > 0.6)

# numbers for the report
w = (t >= 0.2) & (t <= 1.8)
w2 = (t >= 2.05)
vel_all = C['vel']; iqs = C['Iq_set']; iq = C['Iq']
mfit = (iqs < 7.99) & (iqs > -1.9)
A = np.vstack([np.ones(mfit.sum()), -vel_all[mfit]]).T
coef, *_ = np.linalg.lstsq(A, iqs[mfit], rcond=None)
resid = (iqs[mfit] - (coef[0] - coef[1]*vel_all[mfit])).std()

# ================= charts =================
ch1 = chart([("速度反馈 vel", t[i_all], C['vel'][i_all], C_BLUE, 1.1),
             ("速度目标 1.876 rps", t[i_all], C['vel_setpoint'][i_all], C_GRAY, 1.6)],
            (0, 2.53), (0.4, 2.4), "① 速度反馈 vs 目标（全段 2.53 s）", "时间 / s", "转速 / rps",
            hlines=[(1.876, C_GRAY, "目标 1.876")], xticks=[0, 0.5, 1.0, 1.5, 2.0, 2.5])

ch2 = chart([("Iq_set 目标", t[i_all], C['Iq_set'][i_all], C_ORANGE, 2.2),
             ("Iq 反馈", t[i_all], C['Iq'][i_all], C_RED, 0.8)],
            (0, 2.53), (-4, 9), "② q 轴电流 vs 目标（注意 8 A 限幅线）", "时间 / s", "电流 / A",
            hlines=[(8.0, C_RED, "current_limit = 8 A")], xticks=[0, 0.5, 1.0, 1.5, 2.0, 2.5])

ch3 = chart([("Iq_set 目标", t[iz], C['Iq_set'][iz], C_ORANGE, 2.6),
             ("Iq 反馈", t[iz], C['Iq'][iz], C_RED, 1.0)],
            (0.20, 0.60), (4.6, 8.6), "③ 放大 0.20–0.60 s：Iq 抖动的细节", "时间 / s", "电流 / A", xfmt="{:.2f}")

ch4 = chart([("速度反馈", t[ifz], C['vel'][ifz], C_BLUE, 1.6),
             ("Iq / 7.68（折算到 rps）", t[ifz], C['Iq'][ifz]/7.68, C_RED, 0.9)],
            (2.05, 2.53), (0.2, 2.6), "④ 2.05–2.53 s：负载脱开后的大幅 2.2 Hz 振荡", "时间 / s", "rps / 折算",
            hlines=[(1.876, C_GRAY, "目标")])

def bands_from(f, X, grid):
    """把谱按 1/6 倍频程分带平均"""
    fx, yx, ph = [], [], []
    for fc in grid:
        m = (f >= fc * 2 ** (-1 / 12)) & (f < fc * 2 ** (1 / 12))
        if not m.any(): continue
        fx.append(fc); yx.append(X[m].mean())
    return np.array(fx), np.array(yx)

def db(a):
    return 20 * np.log10(np.maximum(np.abs(a), 1e-6))

GRID = 10 ** np.arange(np.log10(8), np.log10(2200), 1 / 12)

# ⑤ 频谱（dB，对数频率）
f5, v5 = bands_from(fi, Vi, GRID)
f5b, v5b = bands_from(fq, Vq, GRID)
f5c, v5c = bands_from(fv, Vv * 7.68, GRID)
ch5 = chart([("Iq 反馈", f5, db(v5), C_RED, 1.6),
             ("Iq_set 目标", f5b, db(v5b), C_ORANGE, 1.2),
             ("速度反馈 × 7.68", f5c, db(v5c), C_BLUE, 1.0)],
            (8, 2200), (-70, 12), "⑤ 频谱（dB）：Iq 与 Iq_set / 速度×7.68 几乎完全重合", "频率 / Hz", "dB re 1 A",
            xlog=True, xfmt="{:.0f}", vlines=[(27, C_PURPLE, "27"), (33.6, C_GREEN, "2×f_e"), (700, "#b9c4d4", "700")])

ch6 = chart([("Id 反馈", t[i_all], C['Id'][i_all], C_GREEN, 0.9)],
            (0, 2.53), (-0.5, 0.5), "⑥ d 轴电流：全程贴在 0 附近，无静差、无低频摆动", "时间 / s", "Id / A")

fg, Hg = bands_from(fb, Hb, GRID)
_, Pg = bands_from(fb, Pb, GRID)
ch7 = chart([("实测 |T|", fg, db(Hg), C_BLUE, 1.8)],
            (8, 2200), (-30, 8), "⑦ 电流环实测闭环幅频（Iq_set → Iq）：1.2 kHz 仍≈0 dB，1.4 kHz 才 −4.6 dB", "频率 / Hz", "dB",
            xlog=True, xfmt="{:.0f}", hlines=[(0, C_GRAY, "0 dB"), (-3, C_GRAY, "−3 dB")])

ch8 = chart([("实测相位", fg, Pg, C_PURPLE, 1.6)],
            (8, 2200), (-110, 10), "⑧ 电流环实测闭环相频：1 kHz 约 −50°，2 kHz 滚到 −110°（延迟主导，无异常跌落）", "频率 / Hz", "deg",
            xlog=True, xfmt="{:.0f}", hlines=[(-45, C_GRAY, "−45°")])

# ================= html =================
CSS = """
:root{--ink:#1d2733;--ink2:#4a5a6e;--ink3:#7a8699;--line:#e3e8ef;--bg:#f7f9fc;--accent:#2f6fed;}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font-family:"Segoe UI","Microsoft YaHei",-apple-system,sans-serif;font-size:15px;line-height:1.7}
.wrap{max-width:1120px;margin:0 auto;padding:28px 20px 80px}
h1{font-size:26px;margin:0 0 6px;letter-spacing:-.2px}
.sub{color:var(--ink3);font-size:13.5px;margin-bottom:22px}
.card{background:#fff;border:1px solid var(--line);border-radius:12px;padding:20px 22px;margin:16px 0;box-shadow:0 1px 2px rgba(20,40,80,.04)}
.card h2{font-size:17px;margin:0 0 12px;display:flex;align-items:center;gap:8px}
.card h3{font-size:15px;margin:18px 0 8px;color:var(--ink2)}
.verdict{border-left:4px solid var(--accent);background:#f4f8ff}
.verdict p{margin:8px 0}
.big{font-size:19px;font-weight:600;color:#1a2b45}
.kpi{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin:6px 0 2px}
.kpi div{background:#fff;border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.kpi .v{font-size:21px;font-weight:650;letter-spacing:-.3px}
.kpi .k{font-size:12px;color:var(--ink3);margin-top:2px}
.bad{color:#d13b2e}.warn{color:#c07a12}.ok{color:#12855c}
table{width:100%;border-collapse:collapse;font-size:13.5px;margin:10px 0}
th,td{padding:8px 10px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{background:#f4f7fb;font-weight:600;color:var(--ink2);font-size:12.5px}
code{background:#f1f4f9;padding:1px 5px;border-radius:4px;font-family:Consolas,Monaco,monospace;font-size:12.8px;color:#2a4a7a}
pre{background:#0f1a2b;color:#d7e3f4;padding:14px 16px;border-radius:10px;overflow:auto;font-size:12.8px;line-height:1.6;font-family:Consolas,Monaco,monospace}
pre .c{color:#7d8fa8}.pre-d{color:#ff8f7a}.pre-a{color:#7ae0a8}
ul,ol{padding-left:22px}li{margin:5px 0}
.tag{display:inline-block;font-size:11.5px;padding:1px 8px;border-radius:20px;background:#eaf1ff;color:#2f6fed;margin-left:6px}
.tag.hot{background:#ffefec;color:#d13b2e}
.tag.mid{background:#fff6e6;color:#b3780c}
.tag.ok{background:#eaf7f1;color:#12855c}
.fig{margin:6px 0 18px;background:#fff;border:1px solid var(--line);border-radius:10px;padding:8px 6px}
.note{font-size:13px;color:var(--ink3);border-top:1px dashed var(--line);padding-top:10px;margin-top:14px}
.arrow{display:block;text-align:center;color:#b9c4d4;margin:2px 0}
"""

def kpi(v, k, cls=""):
    return f'<div><div class="v {cls}">{v}</div><div class="k">{k}</div></div>'

H = []
H.append(f"""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>FOC 电流环波形诊断</title><style>{CSS}</style></head><body><div class="wrap">
<h1>FOC 电流环波形诊断</h1>
<div class="sub">数据：{html.escape(os.path.basename(CSVP))} ｜ 5 kHz 采样 ／ 12 644 点 ／ 2.53 s ｜ 速度模式 ｜ 故障码 0
｜ 参数：电流 Kp=0.08 Ki=679.96 BW=2000 Hz,速度 Kp=7.68 Ki=241.2</div>""")

H.append("""<div class="card verdict">
<h2>结论：电流环没问题，抖动来自速度环</h2>
<p class="big">Iq 的波形 ≈ 速度反馈 × 7.745 —— 电流只是在忠实地复现速度环的输出。</p>
<p>对 <code>Iq_set</code> 和 <code>vel</code> 做最小二乘拟合：</p>
<p><code>Iq_set = 15.00 − 7.745 × vel</code>,相关系数 <b>r = −0.9975</b>;扣掉这一项后残差只有 <b>0.15 A</b>(总标准差 2.18 A)。
也就是说,电流参考里 <b>99.5%</b> 的波动都是速度反馈乘上速度环 P 增益 7.68 放大出来的。</p>
<p>而电流环对参考的跟踪误差只有 <b>0.234 A rms</b>,Id 标准差 <b>0.081 A</b>,闭环增益到 <b>1.2 kHz</b> 仍有 0 dB、1.4 kHz 才 −4.6 dB,1 kHz 处相位 −50° —— 这是一条很健康的电流环。</p>
</div>""")

H.append('<div class="card"><h2>关键数字</h2><div class="kpi">'
         + kpi("−36 %", "稳态速度误差（1.199 / 1.876 rps）", "bad")
         + kpi("7.745", "Iq_set ↔ vel 拟合斜率（P 增益 7.68）", "warn")
         + kpi("0.9975", "两者相关系数", "bad")
         + kpi("3.5 %", "Iq_set 顶在 8 A 限幅的时间占比", "bad")
         + kpi("0.081 A", "Id 标准差（d 轴很干净）", "ok")
         + kpi("0.234 A", "电流环跟踪误差 rms", "ok")
         + '</div></div>')

H.append('<div class="card"><h2>一、先看电流波形"怪"在哪</h2>')
H.append(f'<div class="fig">{ch2}</div>')
H.append("""<p>Iq 不是一条直流,而是一条在 6～8 A 之间大幅摆动、并且带着高频毛刺的曲线。放大到 0.2–0.6 s:</p>""")
H.append(f'<div class="fig">{ch3}</div>')
H.append("""<p>可以看到两层东西:</p>
<ul>
<li><b>低频大摆动</b>(约 0.6 Hz / 2.2 Hz):幅值 ±1 A 以上,就是"忽大忽小"的那部分;</li>
<li><b>高频毛刺</b>(300–700 Hz 带内 rms 0.31 A、700–1500 Hz 带内 rms 0.11 A):就是听起来"沙沙"的抖动噪音。</li>
</ul>
<p>这两层都不在电流环里 —— 它们原封不动地出现在 <code>Iq_set</code> 上(见下图频谱,Iq 与 Iq_set 的谱几乎重合)。</p>""")
H.append(f'<div class="fig">{ch5}</div>')
H.append("""<p>把速度谱乘上 7.68 叠上去,三条曲线在 700 Hz 以下几乎重合,说明<b>电流的每一份抖动都能在速度反馈里找到同一个来源</b>。
1 kHz 以上 Iq 略高于 Iq_set,那才是电流环自身的一点噪声底(约 0.1 A)。</p>
</div>""")

H.append('<div class="card"><h2>二、电流环本身是好的（可以排除）</h2>')
H.append(f'<div class="fig">{ch6}</div>')
H.append("""<p>d 轴电流全程贴在 0 附近(σ=0.081 A,峰值 0.48 A),没有漂、没有低频摆动。用实测数据做 <code>Iq_set → Iq</code> 的闭环频响:</p>""")
H.append(f'<div class="fig">{ch7}</div>')
H.append(f'<div class="fig">{ch8}</div>')
H.append("""<table><tr><th>检查项</th><th>实测</th><th>判断</th></tr>
<tr><td>电流环跟踪误差</td><td>0.234 A rms / 3.1 A p2p(含限幅瞬间)</td><td class="ok">正常</td></tr>
<tr><td>闭环 −3 dB 带宽</td><td>1.2 kHz 处仍 0 dB,1.4 kHz 为 −4.6 dB;估计 −3 dB 在 1.5~1.8 kHz(设计 2 kHz,受 40 kHz 采样与数字延迟限制)</td><td class="ok">正常</td></tr>
<tr><td>相位</td><td>1 kHz −50°,1.4 kHz −76°(延迟主导,单调下降无异常跌落)</td><td class="ok">正常</td></tr>
<tr><td>Id 调节</td><td>σ=0.081 A,无静差</td><td class="ok">正常</td></tr>
<tr><td>6 次谐波(死区补偿残差)</td><td>Id 0.069 A / Vd 0.079 V</td><td class="warn">偏大但可接受</td></tr>
<tr><td>三相电流和 isum</td><td>σ=0.060 A(Ic 由 Ia+Ib 重构后的残差)</td><td class="ok">正常</td></tr>
<tr><td>电压饱和标志</td><td>全程 0</td><td class="ok">未饱和</td></tr>
</table>
<p class="note">说明:因为 VOFA 采样与 PWM 同步(40 kHz 8 分频),日志里看到的不是开关纹波混叠,而是真实的低频演化;所以上面这些数字可以直接当结论用。
唯一值得注意的是 <code>Vmag</code> 平均只有 0.53 V(母线 24 V,调制度 2.2%),死区 100 ns 折算的电压误差约 96 mV,
占指令电压的 18% —— 这就是 6 次谐波的来源,也是 Id 上那点残余纹波的原因。</p>
</div>""")

H.append('<div class="card"><h2>三、真正的问题：速度环积分器被"焊死"了</h2>')
H.append(f'<div class="fig">{ch1}</div>')
H.append("""<p>目标 1.876 rps,实际只有 1.199 rps(−36 %),而且 2.0 s 之后负载一松,速度立刻跳到 1.84 rps,同时电流从 7 A 掉到 0.7 A。
两者都说明:电流环有能力,速度环没把工作点搬过去。</p>
<p>看 <code>foc_ctrl.c</code> 的两个细节:</p>
<pre><span class="c">/* 1. 积分器限幅 ±0.5 A */</span>
float vint_lim = (foc_get_stage() == ANTICOGGING) ? 4.0f : <span class="pre-d">0.5f</span>;
Controller.vel_integrator = CLAMP(Controller.vel_integrator, -vint_lim, vint_lim);

<span class="c">/* 2. 只要这一拍限幅,积分器每拍 decay 1% (5 kHz 下 ≈ 每 20 ms 衰减 63%) */</span>
else if (limited)
        Controller.vel_integrator <span class="pre-d">*= 0.99f</span>;</pre>
<p>反推一下就明白:P 项 = 7.68 × (1.876−1.199) = <b>5.2 A</b>,而积分器最大只能给 <b>0.5 A</b>。
两个工况下反算出的积分器都是 0.48～0.53 A,<b>也就是长期顶在限幅上</b>。
一个积分器被钉在限幅上、还会在每次限幅后被 0.99 衰减,等于没有积分 —— 速度环实际就是个纯 P 环,必然留静差,
而且负载一重就只能靠 P 项硬顶,顶到 8 A 限幅(样本的 3.5 %)就变成削顶的非线性极限环。</p>
</div>""")

H.append('<div class="card"><h2>四、第二层问题：速度反馈自带的噪声被放大 7.7 倍</h2>')
H.append(f'<div class="fig">{ch4}</div>')
H.append("""<p>2.05 s 之后负载脱开、电流只剩 0.7 A,但速度依然在 1.6～2.2 rps 之间以 2.2 Hz 摆动,电流跟着在 +2.7 / −2.5 A 之间翻 —— 
这是一个<b>速度环极限环</b>,不是电流环在振。</p>
<p>再看 300–700 Hz 那 0.31 A 的毛刺。按实测的机械惯性反算(<code>dq/dt</code> / <code>ΔIq</code> ≈ 89 rad/s²/A),
要让转子真的以 0.03 rps、500 Hz 抖动,需要 <b>7.6 A</b> 的高频电流;实际只有 0.3 A。
所以这部分<b>不可能是真实运动,只能是测速噪声</b>,而它经 <code>vel_kp=7.68</code> 原样出现在电流参考上。</p>
<p>噪声源在编码器 PLL 的时基:<code>encoder.c</code> 里 PLL 用写死的 <code>period = 1/5000 s</code>,而实际取帧时刻由 FreeRTOS tick(10 kHz)驱动,
会被 40 kHz ADC 中断和其它任务挤动。用同样的增益做了数值仿真:</p>
<table><tr><th>工况</th><th>速度估计 σ</th><th>对应电流噪声(×7.68)</th></tr>
<tr><td>理想(严格 200 µs、整数计数)</td><td>0.00002 rps</td><td>≈0</td></tr>
<tr><td>采样间隔抖动 ±100 µs</td><td><b>0.117 rps</b></td><td>0.90 A</td></tr>
<tr><td>2 % 帧丢失</td><td>0.060 rps</td><td>0.46 A</td></tr>
<tr><td><b>本机实测</b>(0.12–0.48 s 窗口)</td><td><b>0.086 rps</b></td><td>0.66 A</td></tr>
</table>
<p>实测值正好落在"时基抖动/丢帧"这一档,而理想情况几乎为零 —— 所以抖动是<b>时基</b>问题,不是编码器分辨率问题(21 位、0.0003 rps/count,绰绰有余)。</p>
</div>""")

H.append("""<div class="card"><h2>五、因果链</h2>
<svg viewBox="0 0 1040 208" width="100%" font-family="Segoe UI, Microsoft YaHei, sans-serif">
<defs><marker id="a" markerWidth="9" markerHeight="9" refX="7" refY="3" orient="auto"><path d="M0,0 L0,6 L7,3 z" fill="#9aa8bd"/></marker></defs>
<g font-size="12.5">
<rect x="20" y="24" width="230" height="58" rx="9" fill="#fff3f0" stroke="#e8a295"/>
<text x="135" y="47" text-anchor="middle" fill="#a8341f" font-weight="600">编码器 PLL 假设固定 200 µs</text>
<text x="135" y="66" text-anchor="middle" fill="#a8341f">实际取帧时刻被 tick/中断挤动</text>

<rect x="20" y="122" width="230" height="58" rx="9" fill="#fff3f0" stroke="#e8a295"/>
<text x="135" y="145" text-anchor="middle" fill="#a8341f" font-weight="600">速度反馈 σ≈0.09 rps</text>
<text x="135" y="164" text-anchor="middle" fill="#a8341f">300–700 Hz 噪声底</text>

<rect x="300" y="122" width="220" height="58" rx="9" fill="#fff8ea" stroke="#e5c07b"/>
<text x="410" y="145" text-anchor="middle" fill="#9a6b0d" font-weight="600">× vel_kp = 7.68</text>
<text x="410" y="164" text-anchor="middle" fill="#9a6b0d">P 环就是一台噪声放大器</text>

<rect x="570" y="122" width="200" height="58" rx="9" fill="#eef4ff" stroke="#9dbaf3"/>
<text x="670" y="145" text-anchor="middle" fill="#2f4f8f" font-weight="600">Iq_set 抖动 ±0.3 A</text>
<text x="670" y="164" text-anchor="middle" fill="#2f4f8f">+ 低频极限环 ±1 A</text>

<rect x="820" y="122" width="200" height="58" rx="9" fill="#eaf7f1" stroke="#8ecfb2"/>
<text x="920" y="145" text-anchor="middle" fill="#12704f" font-weight="600">电流环 1:1 复现</text>
<text x="920" y="164" text-anchor="middle" fill="#12704f">波形"怪"、有噪声</text>

<rect x="300" y="24" width="220" height="58" rx="9" fill="#fff3f0" stroke="#e8a295"/>
<text x="410" y="47" text-anchor="middle" fill="#a8341f" font-weight="600">积分器钉在 ±0.5 A</text>
<text x="410" y="66" text-anchor="middle" fill="#a8341f">限幅时还被 ×0.99</text>

<rect x="570" y="24" width="200" height="58" rx="9" fill="#fff8ea" stroke="#e5c07b"/>
<text x="670" y="47" text-anchor="middle" fill="#9a6b0d" font-weight="600">纯 P 环 → 静差 36 %</text>
<text x="670" y="66" text-anchor="middle" fill="#9a6b0d">负载一重就顶 8 A 限幅</text>

<rect x="820" y="24" width="200" height="58" rx="9" fill="#fff8ea" stroke="#e5c07b"/>
<text x="920" y="47" text-anchor="middle" fill="#9a6b0d" font-weight="600">削顶非线性极限环</text>
<text x="920" y="66" text-anchor="middle" fill="#9a6b0d">0.6 / 2.2 Hz 大摆动</text>

<path d="M135 82 L135 118" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M250 151 L296 151" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M520 151 L566 151" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M770 151 L816 151" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M410 82 L410 118" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M520 53 L566 53" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M770 53 L816 53" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
<path d="M920 82 L920 118" stroke="#9aa8bd" stroke-width="1.6" marker-end="url(#a)"/>
</g></svg>
</div>""")

H.append("""<div class="card"><h2>六、建议改法（按收益排序）</h2>

<h3>① 让速度积分器真正工作 <span class="tag hot">最高优先级</span></h3>
<p><code>foc_ctrl.c</code> 的 <code>CONTROLLER_loop()</code>:把 ±0.5 A 放开到跟电流限幅同量级,并把"每拍 ×0.99"换成标准反算抗饱和。</p>
<pre><span class="pre-d">- float vint_lim = (foc_get_stage() == ANTICOGGING) ? 4.0f : 0.5f;</span>
<span class="pre-a">+ float vint_lim = (foc_get_stage() == ANTICOGGING) ? 4.0f</span>
<span class="pre-a">+                                             : Config.current_limit;</span>
...
<span class="pre-d">- else if (limited)</span>
<span class="pre-d">-     Controller.vel_integrator *= 0.99f;</span>
<span class="pre-d">- else</span>
<span class="pre-d">-     Controller.vel_integrator += (vel_ki * CTRL_PERIOD) * v_err;</span>
<span class="pre-a">+ else if (!limited)</span>
<span class="pre-a">+     Controller.vel_integrator += (vel_ki * CTRL_PERIOD) * v_err;</span>
<span class="pre-a">+ /* 限幅时冻结积分即可, 由下面的 CLAMP 收边 */</span>
...
<span class="pre-d">- Controller.vel_integrator = CLAMP(Controller.vel_integrator, -vint_lim, vint_lim);</span>
<span class="pre-a">+ /* 让 P+I 不超限幅, 而不是把积分器单独掐死在 0.5 A */</span>
<span class="pre-a">+ float i_room = Config.current_limit - fabsf(iq_set - Controller.vel_integrator);</span>
<span class="pre-a">+ Controller.vel_integrator = CLAMP(Controller.vel_integrator,</span>
<span class="pre-a">+                                   -(Config.current_limit - fabsf(vel_kp * v_err)),</span>
<span class="pre-a">+                                    (Config.current_limit - fabsf(vel_kp * v_err)));</span></pre>
<p>改完静差应该在一个来回内被积掉(1.876 rps 时负载只要 0.7 A,现在却要 5.2 A 的 P 项去顶)。</p>

<h3>② 修编码器时基 <span class="tag hot">最高优先级</span></h3>
<p><code>encoder.c</code> 的 <code>ENCODER_loop()</code>:不要再假设"每帧正好 200 µs",用核周期计数器量真实间隔:</p>
<pre><span class="pre-a">+ #include "hpm_csr_drv.h"</span>
<span class="pre-a">+ static uint32_t last_cyc;</span>
...
<span class="pre-a">+ uint32_t now = hpm_csr_get_core_cycle();</span>
<span class="pre-a">+ float period = (float)(now - last_cyc) /</span>
<span class="pre-a">+                (float)clock_get_frequency(clock_cpu0);</span>
<span class="pre-a">+ last_cyc = now;</span>
<span class="pre-a">+ if (period < 50e-6f || period > 1e-3f) period = 1.0f / (float)ENCODER_UPDATE_FREQ;</span>
<span class="pre-d">- float period = 1.0f / (float)ENCODER_UPDATE_FREQ;</span></pre>
<p>PLL 下面两处 <code>period</code> 已经在用它,不用改。仿真里这一条能把速度估计噪声从 0.117 rps 压到接近 0。
改完再看 <code>vel</code> 的 300–700 Hz 分量是否掉到 0.01 rps 以下。</p>

<h3>③ 重新整定速度环 <span class="tag mid">改完①②再做</span></h3>
<p>现在 <code>vel_kp=7.68 A/rps</code> 相当于"1 rps 误差给 7.68 A",配上 8 A 限幅就是半个 bang-bang。
建议先降到 <b>2～3 A/rps</b>,<code>vel_ki</code> 先保持 241,用现有的 <code>FOC_step_start()</code> 阶跃工具看速度响应,
或直接阶跃 <code>vel_setpoint</code> 看超调。若 2.2 Hz 那个摆动还在,说明是机械侧的共振(联轴器/弹性安装),
再在速度反馈上加一个 2 Hz 附近的可调陷波,或把 <code>ENCODER_PLL_BW_HZ</code> 从 300 Hz 降到 120～150 Hz 换取平滑(代价是相位滞后)。</p>

<h3>④ 顺手核一下电流限幅 <span class="tag ok">低优先</span></h3>
<p><code>current_limit = 8 A</code> 被顶到的时间占 3.5 %,而 5 mΩ×20 倍的采样链量程到 ±16.4 A。
0-2 s 那段"7 A 才 1.0 rps、松一下 0.7 A 就 1.84 rps"很像机械侧卡滞/静摩擦;
如果是负载本身需要,8 A 的上限需要重新评估。</p>

<h3>⑤ 不用动的部分</h3>
<ul>
<li>电流环 Kp/Ki(0.08 / 679.96)、2 kHz 带宽:实测到 1.2 kHz 仍 0 dB、Id 无静差,不必改;</li>
<li>死区补偿:6 次谐波残差 0.08 V(指令电压的 15 %),在这个 24 V 母线下已经算不错,加 ITH 过渡带只会更糊;</li>
<li>ADC 采样点:40 kHz 同步采样,2 % 占空比下低边窗口很宽,采样时刻没问题。</li>
</ul>
</div>""")

H.append("""<div class="card"><h2>七、一个 5 分钟就能验证的实验</h2>
<p>把模式切到<b>电流模式(CONTROL_MODE_CURRENT_RAMP)</b>,给一个恒定 <code>Iq_set = 6 A</code>,再录一次同样的波形。预期:</p>
<ul>
<li>如果 <code>Iq</code> 变成一条 ±0.25 A 以内的平直带 → 彻底确认"电流环没问题,问题全在速度环",按 ①②③ 改;</li>
<li>如果 <code>Iq</code> 依旧大幅抖动 → 那才需要回头查电流环,重点看 <code>FOC.Ia/Ib</code> 的快照是否被 DMA 时序打断,
以及 <code>isum</code> 是否漂(现在 σ=0.06 A,基本正常)。</li>
</ul>
<p class="note">另外可以顺手观察 <code>phase_vel</code> 在恒流模式下的标准差:真实机械速度在恒流下应该非常平稳(σ &lt; 0.01 rps),
如果还在 0.08 rps 上下,那就是 ② 的时基问题被直接坐实。</p>
</div>""")

H.append('<div class="card"><h2>附:数据分段统计</h2><table>'
         '<tr><th>段</th><th>vel 均值</th><th>vel σ</th><th>Iq 均值</th><th>Iq σ</th><th>Id σ</th><th>顶 8 A 占比</th></tr>'
         f'<tr><td>0.2–1.8 s（带载）</td><td>1.078</td><td>0.107</td><td>6.65 A</td><td>0.80 A</td><td>0.081 A</td><td>约 5 %</td></tr>'
         f'<tr><td>2.05–2.53 s（脱开）</td><td>1.696</td><td>0.242</td><td>1.87 A</td><td>1.89 A</td><td>0.084 A</td><td>0 %</td></tr>'
         f'<tr><td>全程</td><td>1.199</td><td>0.282</td><td>5.71 A</td><td>2.18 A</td><td>0.082 A</td><td>3.5 %</td></tr>'
         '</table><p class="note">Id 的 σ 在两段里几乎不变(0.081 / 0.084 A),再次说明电流环不受外环工况影响。</p></div>')

H.append('</div></body></html>')

with open(OUT, 'w', encoding='utf-8') as f:
    f.write("\n".join(H))
print("written", OUT, os.path.getsize(OUT))

with open(r"D:/Desktop/FOC/tools/_dbg.html", "w", encoding="utf-8") as f:
    f.write(f"<!DOCTYPE html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body><div class='wrap'>")
    f.write(f"<div class='fig'>{ch5}</div><div class='fig'>{ch7}</div><div class='fig'>{ch8}</div>")
    f.write(f"<div class='fig'>{ch4}</div></div></body></html>")

with open(r"D:/Desktop/FOC/tools/_dbg2.html", "w", encoding="utf-8") as f:
    f.write(f"<!DOCTYPE html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body><div class='wrap'>")
    f.write("".join(H[-5:-1]))
    f.write("</div></body></html>")
