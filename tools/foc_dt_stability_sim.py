# -*- coding: utf-8 -*-
"""死区非线性敏感的快速核查模型（三相平均模型，40 kHz）

目的只有一个：判断电流环的 Id 抖动到底来自"环路增益/带宽"还是"逆变器死区非线性"。
用法: python foc_dt_stability_sim.py
输出: 四种死区条件下的 Id rms，用来给改动优先级排序。

模型边界：不含开关纹波、不含 ADC 噪声、死区按 ±Vdt 方波近似。只能做敏感性对比，
不能预测具体波形，也不能替代实测。
"""
import numpy as np

# ---- 由录波反推的参数（Kp/BW -> L, Ki/BW -> R）----
KP, KI, BW = 0.08, 679.96, 2000.0
L = KP / (2 * np.pi * BW)          # 6.37 uH
R = KI / (2 * np.pi * BW)          # 54.1 mOhm
VBUS = 24.28
VDT = (100e-9 / 25e-6) * VBUS      # 100ns 死区 -> 0.097 V
WE = 122.0                         # 电角速度 rad/s
IQ_REF = 1.312
LAM = (0.404 - R * IQ_REF) / WE    # 由稳态 Vq 反推磁链
T = 1.0 / 40000.0
FLOG = 5000.0                      # 与上位机同速率抽样


def run(ith, vdt=VDT, comp=True, ncyc=6000, noise=0.0, seed=3):
    ia, ib = -1.0, 0.6
    ind = inq = 0.0
    th = 0.0
    out = []
    rng = np.random.default_rng(seed)
    for k in range(ncyc):
        ma = ia + noise * rng.standard_normal()
        mb = ib + noise * rng.standard_normal()
        mc = -(ia + ib) + noise * rng.standard_normal()
        al = (2 * ma - mb - mc) / 3
        be = (mb - mc) / np.sqrt(3)
        idm = al * np.cos(th) + be * np.sin(th)
        iqm = -al * np.sin(th) + be * np.cos(th)
        ed, eq = -idm, IQ_REF - iqm
        vd = ed * KP + ind - WE * L * iqm
        vq = eq * KP + inq + WE * (L * idm + LAM)
        if 1.5 * np.hypot(vd, vq) / VBUS <= 0.9 * 0.866:
            ind += ed * KI * T
            inq += eq * KI * T
        va_ = vd * np.cos(th) - vq * np.sin(th)
        vb_ = vd * np.sin(th) + vq * np.cos(th)
        ua, ub, uc = va_, -0.5 * va_ + 0.8660254 * vb_, -0.5 * va_ - 0.8660254 * vb_
        if comp:
            ua += vdt * np.clip(ma / ith, -1, 1)
            ub += vdt * np.clip(mb / ith, -1, 1)
            uc += vdt * np.clip(mc / ith, -1, 1)
        ua -= vdt * np.sign(ma)
        ub -= vdt * np.sign(mb)
        uc -= vdt * np.sign(mc)
        ea = -LAM * WE * np.sin(th)
        eb = -LAM * WE * np.sin(th - 2.0944)
        ec = -LAM * WE * np.sin(th + 2.0944)
        vn = (ua + ub + uc) / 3
        ia += (ua - vn - R * ia - ea) / L * T
        ib += (ub - vn - R * ib - eb) / L * T
        th = (th + WE * T) % (2 * np.pi)
        if k >= 2000 and k % int(40000 / FLOG) == 0:
            out.append((idm, iqm))
    return np.array(out)


def report(a, lab):
    idq = a[:, 0] - a[:, 0].mean()
    w = np.hanning(len(idq))
    X = np.abs(np.fft.rfft(idq * w)) * 2 / np.sum(w)
    f = np.fft.rfftfreq(len(idq), 1 / FLOG)
    top = np.argsort(X)[::-1][:3]
    print("%-26s Id rms = %.4f A   Iq std = %.4f A   主峰 %s"
          % (lab, np.sqrt(np.mean(idq ** 2)), np.std(a[:, 1]),
             ", ".join("%.0fHz:%.3f" % (f[i], X[i]) for i in top)))


if __name__ == "__main__":
    print("L=%.3f uH  R=%.4f ohm  Vdt=%.4f V  临界 ITH = Vdt/R = %.2f A"
          % (L * 1e6, R, VDT, VDT / R))
    report(run(0.8, vdt=0.0), "理想逆变器 (Vdt=0)")
    report(run(0.8, comp=False), "有死区 / 不补偿")
    report(run(0.8), "有死区 + 补偿 ITH=0.8 (当前)")
    report(run(0.05), "有死区 + 硬符号 ITH=0.05")
    report(run(2.0), "有死区 + 补偿 ITH=2.0")
