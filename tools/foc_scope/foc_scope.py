# -*- coding: utf-8 -*-
"""
FOC Scope - 本地图表上位机 (tkinter, MATLAB 风格)

- 波形: VOFA Justfloat (N 个 float32 小端 + 帧尾 00 00 80 7F)
- 命令: 固件 5 字节帧 (id + 4 字节小端 float)
- 交互: 滚轮缩放时间轴 / Ctrl或Shift+滚轮(左刻度) 缩放幅值 / 拖动平移 / 双击自动量程
- 捕捉: 鼠标悬停在波形上显示各通道在该时刻的读数
- 记忆: 通道名/显示/增益/偏置、滑条范围与实时、最大显示时长 存 scope_cfg.json

依赖: Python3 + tkinter(标准库) + pyserial
"""
import os
import json
import math
import struct
import threading
import queue
import time
from array import array
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

# ---------------------------- 通道默认值 ----------------------------
N_DEFAULT = 21
DEFAULT_NAMES = (["data%d" % (i + 1) for i in range(9)]
                 + ["Vbus", "Vmag", "sat", "Vd", "Vq", "ang", "ang_vel",
                    "isum", "Va", "Vb", "Vc", "fault"])
MATLAB_COLORS = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30",
                 "#4DBEEE", "#A2142F", "#000000", "#7F7F7F",
                 "#9467BD", "#17BECF", "#8C564B"]

MAX_CH = 32
CAP = 1 << 17            # 每通道 131072 样本
TRIM = CAP // 4
GRID_MAX = 24            # 网格/刻度对象池大小

TAIL = b"\x00\x00\x80\x7f"
PALETTE = MATLAB_COLORS + ["#000000"]

CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scope_cfg.json")

# MATLAB 浅色主题
UI_BG   = "#f0f0f0"
FIELD   = "#ffffff"
PLOTBG  = "#ffffff"
GRID    = "#e6e6e6"
AXIS    = "#262626"
FG      = "#1a1a1a"
DIM     = "#5a5a5a"

CMD = {
    "opmode": 0x20, "enable": 0x21, "disable": 0x22,
    "torque": 0x23, "velocity": 0x24, "position": 0x25,
    "calib": 0x27, "home": 0x2D, "save": 0x3F, "reset": 0x3B,
    "curkp": 0x40, "curki": 0x41, "velkp": 0x42, "velki": 0x43, "poskp": 0x44,
    "curbw": 0x58,
    "uv": 0x56, "ov": 0x57,
}

# 滑条默认: (标签, 最小, 最大, 步长)
SLIDER_DEF = [
    ("torque",   "力矩 A",   0.0,    8.0,    0.05),
    ("velocity", "速度 rps", -30.0,  30.0,   0.1),
    ("position", "位置 圈",  0.0,    6.2832, 0.05),
]

# 保护阈值滑条 (发出去就生效, 点"保存(3F)"落到 Flash)
PROT_DEF = [
    ("uv", "欠压 V", 0.0, 30.0, 0.1),
    ("ov", "过压 V", 0.0, 30.0, 0.1),
]


class ScopeApp:
    def __init__(self, root):
        self.root = root
        root.title("FOC Scope")
        root.configure(bg=UI_BG)
        self.cfg = self.load_cfg()
        root.geometry(self.cfg.get("window", "1400x800"))

        # 通道属性 (可记忆)
        chs = self.cfg.get("channels", [])
        self.ch_names = [chs[i].get("name", DEFAULT_NAMES[i]) if i < len(chs) else DEFAULT_NAMES[i]
                         for i in range(N_DEFAULT)]
        self.ch_colors = list(MATLAB_COLORS)
        self.vis = [(chs[i].get("visible", True) if i < len(chs) else True) for i in range(N_DEFAULT)]
        self.gain = [(chs[i].get("gain", 1.0) if i < len(chs) else (1000.0 if i == 8 else 1.0))
                     for i in range(N_DEFAULT)]
        self.off = [(chs[i].get("offset", 0.0) if i < len(chs) else 0.0) for i in range(N_DEFAULT)]
        # 扩充到 MAX_CH
        while len(self.ch_names) < MAX_CH:
            self.ch_names.append("data%d" % (len(self.ch_names) + 1))
        self.vis += [True] * (MAX_CH - len(self.vis))
        self.gain += [1.0] * (MAX_CH - len(self.gain))
        self.off += [0.0] * (MAX_CH - len(self.off))

        # 数据
        self.bufs = [array('f') for _ in range(MAX_CH)]
        self.base = 0
        self.total = 0
        self.nch = N_DEFAULT
        self.q = queue.Queue(maxsize=20000)
        self.ser = None
        self.ser_thread = None
        self.running = False
        self.tx_lock = threading.Lock()
        self.rx_bytes = 0

        # 视图
        self.follow = True
        self.paused = False         # 暂停波形显示
        self.t_span = 0.5
        self.t_right = 0.0
        self.y_min, self.y_max = -0.2, 0.2
        self.cursor = None
        self.drag = None
        self.max_span = float(self.cfg.get("max_span", 10.0))
        self.dirty = True
        self.last_nvis = 0      # 当前窗口内的采样点数 (状态栏显示)

        # 速率
        self.fs = 1000.0
        self.frames = 0
        self.stat_t = time.time()
        self.stat_frames = 0
        self.stat_bytes = 0

        # 演示
        self.demo = tk.BooleanVar(value=False)
        self.demo_t = 0.0

        self.sliders = {}
        self.vis_vars = []
        self.val_labels = []
        self.name_labels = []
        self._save_job = None
        self._tick_job = None

        self.build_ui()
        self.bind_events()
        self.apply_slider_cfg()
        self.tick()

    # ==================== 配置记忆 ====================
    def load_cfg(self):
        try:
            with open(CFG_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def save_cfg(self):
        st = {}
        for kind, d in self.sliders.items():
            st[kind] = {"min": d['mn'], "max": d['mx'], "step": d['step'],
                        "value": d['scale'].get(), "live": bool(d['live'].get())}
        cfg = {
            "window": self.root.geometry(),
            "max_span": self.max_span,
            "channels": [{"name": self.ch_names[i], "visible": bool(self.vis[i]),
                          "gain": self.gain[i], "offset": self.off[i]}
                         for i in range(N_DEFAULT)],
            "sliders": st,
        }
        try:
            with open(CFG_PATH, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=1)
        except Exception:
            pass

    def save_cfg_later(self):
        if self._save_job:
            self.root.after_cancel(self._save_job)
        self._save_job = self.root.after(800, self._do_save)

    def _do_save(self):
        self._save_job = None
        self.save_cfg()

    def apply_slider_cfg(self):
        for kind, d in self.sliders.items():
            s = (self.cfg.get("sliders") or {}).get(kind)
            if not s:
                continue
            try:
                mn, mx, st = float(s["min"]), float(s["max"]), float(s["step"])
                if mx > mn and st > 0:
                    d['mn'], d['mx'], d['step'] = mn, mx, st
                    d['scale'].config(from_=mn, to=mx, resolution=st)
                d['scale'].set(float(s.get("value", mn)))
                d['live'].set(bool(s.get("live", True)))
                d['lbl'].config(text=self.num(d['scale'].get()))
            except Exception:
                pass

    # ==================== UI ====================
    def name_of(self, c):
        return self.ch_names[c] if c < len(self.ch_names) else "data%d" % (c + 1)

    def color_of(self, c):
        return self.ch_colors[c] if c < len(self.ch_colors) else PALETTE[c % len(PALETTE)]

    def build_ui(self):
        root = self.root
        top = tk.Frame(root, bg=UI_BG); top.pack(side=tk.TOP, fill=tk.X)
        left = tk.Frame(root, bg=UI_BG, width=280); left.pack(side=tk.LEFT, fill=tk.Y)
        left.pack_propagate(False)
        right = tk.Frame(root, bg=UI_BG, width=272); right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        def tbtn(parent, text, cmd):
            return tk.Button(parent, text=text, command=cmd, relief=tk.FLAT,
                             bg="#dcdcdc", activebackground="#c8c8c8", padx=9, pady=3)

        # 顶栏
        tbtn(top, "连接", self.connect).pack(side=tk.LEFT, padx=(6, 2), pady=6)
        self.btn_disconnect = tbtn(top, "断开", self.disconnect)
        self.btn_disconnect.config(state=tk.DISABLED)
        self.btn_disconnect.pack(side=tk.LEFT, padx=2, pady=6)
        self.btn_pause = tbtn(top, "暂停显示", self.toggle_pause)
        self.btn_pause.pack(side=tk.LEFT, padx=2, pady=6)
        self.btn_follow = tbtn(top, "跟随", self.toggle_follow)
        self.btn_follow.pack(side=tk.LEFT, padx=2, pady=6)
        tbtn(top, "清空", self.clear).pack(side=tk.LEFT, padx=2, pady=6)
        tbtn(top, "自动量程", self.autoscale).pack(side=tk.LEFT, padx=2, pady=6)
        tk.Checkbutton(top, text="演示数据", variable=self.demo, bg=UI_BG, fg=FG,
                       activebackground=UI_BG, command=self.on_demo).pack(side=tk.LEFT, padx=10)
        self.lbl_status = tk.Label(top, text="未连接", bg=UI_BG, fg=DIM)
        self.lbl_status.pack(side=tk.RIGHT, padx=10)

        def sec(parent, title):
            tk.Label(parent, text=title, bg=UI_BG, fg=DIM, anchor="w").pack(
                fill=tk.X, padx=8, pady=(9, 2))

        # 端口
        sec(left, "端口")
        f = tk.Frame(left, bg=UI_BG); f.pack(fill=tk.X, padx=8)
        self.cmb_port = ttk.Combobox(f, values=self.port_list(), width=14)
        self.cmb_port.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(f, text="刷新", relief=tk.FLAT, bg="#dcdcdc",
                  command=lambda: self.cmb_port.config(values=self.port_list())).pack(side=tk.LEFT, padx=3)

        # 运行
        sec(left, "运行")
        f = tk.Frame(left, bg=UI_BG); f.pack(fill=tk.X, padx=8)
        tk.Label(f, text="模式", bg=UI_BG, fg=FG, width=6, anchor="w").pack(side=tk.LEFT)
        self.cmb_mode = ttk.Combobox(f, values=["0 电流", "1 速度", "2 位置", "3 轨迹"],
                                     width=8, state="readonly")
        self.cmb_mode.current(0)
        self.cmb_mode.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(f, text="发", relief=tk.FLAT, bg="#dcdcdc", padx=6,
                  command=lambda: self.send(CMD["opmode"], float(self.cmb_mode.current()))).pack(side=tk.LEFT, padx=3)

        f = tk.Frame(left, bg=UI_BG); f.pack(fill=tk.X, padx=8, pady=4)
        for text, key, col in (("使能", "enable", "#cfe3ff"), ("失能", "disable", "#ffd6d6"),
                               ("校准", "calib", "#dcdcdc"), ("回零", "home", "#dcdcdc")):
            tk.Button(f, text=text, bg=col, relief=tk.FLAT, width=6, activebackground="#bfbfbf",
                      command=lambda k=key: self.send(CMD[k], 0.0)).pack(side=tk.LEFT, padx=2)

        # 目标值 (滑条, 默认实时发送)
        sec(left, "目标值 (滑条)")
        tk.Label(left, text="双击参数名可配置 最小值/最大值/步长", bg=UI_BG, fg=DIM,
                 anchor="w").pack(fill=tk.X, padx=8)
        for kind, label, mn, mx, step in SLIDER_DEF:
            self.add_slider(left, kind, label, mn, mx, step)

        # 保护阈值 (滑条, 改完点 保存(3F))
        sec(left, "保护阈值 (V)")
        for kind, label, mn, mx, step in PROT_DEF:
            self.add_slider(left, kind, label, mn, mx, step)

        # 增益
        sec(left, "增益")
        for label, key, default in (("带宽Hz", "curbw", "1000"),
                                    ("电流Kp", "curkp", ""), ("电流Ki", "curki", ""),
                                    ("速度Kp", "velkp", "0.25"), ("速度Ki", "velki", "0.01"),
                                    ("位置Kp", "poskp", "20")):
            f = tk.Frame(left, bg=UI_BG); f.pack(fill=tk.X, padx=8, pady=1)
            tk.Label(f, text=label, bg=UI_BG, fg=FG, width=7, anchor="w").pack(side=tk.LEFT)
            e = tk.Entry(f, width=8, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
            e.insert(0, default); e.pack(side=tk.LEFT, fill=tk.X, expand=True)
            tk.Button(f, text="发", relief=tk.FLAT, bg="#dcdcdc", padx=6,
                      command=lambda k=key, ent=e: self.send_entry(k, ent)).pack(side=tk.LEFT, padx=3)

        # 存储 / 原始帧
        sec(left, "存储 / 原始帧")
        f = tk.Frame(left, bg=UI_BG); f.pack(fill=tk.X, padx=8)
        tk.Button(f, text="保存(3F)", relief=tk.FLAT, bg="#dcdcdc",
                  command=lambda: self.send(CMD["save"], 0.0)).pack(side=tk.LEFT, padx=2)
        tk.Button(f, text="恢复默认(3B)", relief=tk.FLAT, bg="#ffd6d6",
                  command=self.reset_cfg).pack(side=tk.LEFT, padx=2)
        f = tk.Frame(left, bg=UI_BG); f.pack(fill=tk.X, padx=8, pady=3)
        self.ent_raw = tk.Entry(f, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
        self.ent_raw.insert(0, "20 01 00 00 00")
        self.ent_raw.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(f, text="发", relief=tk.FLAT, bg="#dcdcdc", padx=6,
                  command=self.send_raw).pack(side=tk.LEFT, padx=3)

        tk.Label(left, text="滚轮=时间缩放(跟随时不暂停)\nCtrl/Shift+滚轮 或 左刻度上滚=幅值\n拖动=平移  双击绘图区=自动量程\n鼠标悬停=捕捉读数\n双击通道名=改名",
                 bg=UI_BG, fg=DIM, justify=tk.LEFT, anchor="w").pack(fill=tk.X, padx=8, pady=10)

        # 右侧: 波形显示选项
        sec(right, "波形显示")
        f = tk.Frame(right, bg=UI_BG); f.pack(fill=tk.X, padx=8)
        tk.Button(f, text="全部显示", relief=tk.FLAT, bg="#dcdcdc",
                  command=lambda: self.set_all_vis(True)).pack(side=tk.LEFT, padx=2)
        tk.Button(f, text="全部隐藏", relief=tk.FLAT, bg="#dcdcdc",
                  command=lambda: self.set_all_vis(False)).pack(side=tk.LEFT, padx=2)
        f = tk.Frame(right, bg=UI_BG); f.pack(fill=tk.X, padx=8, pady=(6, 0))
        tk.Label(f, text="最大显示时长(s)", bg=UI_BG, fg=FG).pack(side=tk.LEFT)
        self.ent_maxspan = tk.Entry(f, width=6, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
        self.ent_maxspan.insert(0, self.num(self.max_span))
        self.ent_maxspan.pack(side=tk.LEFT, padx=4)
        self.ent_maxspan.bind("<Return>", lambda e: self.apply_max_span())
        self.ent_maxspan.bind("<FocusOut>", lambda e: self.apply_max_span())

        tk.Label(right, text="  通道(双击改名)  gain  offset     值", bg=UI_BG, fg=DIM,
                 anchor="w").pack(fill=tk.X, padx=8, pady=(6, 0))
        for i in range(N_DEFAULT):
            f = tk.Frame(right, bg=UI_BG); f.pack(fill=tk.X, padx=8, pady=1)
            v = tk.BooleanVar(value=self.vis[i])
            self.vis_vars.append(v)
            tk.Checkbutton(f, variable=v, bg=UI_BG, activebackground=UI_BG,
                           command=lambda i=i: self.on_vis(i)).pack(side=tk.LEFT)
            tk.Frame(f, bg=self.color_of(i), width=10, height=10).pack(side=tk.LEFT, padx=2)
            nm = tk.Label(f, text=self.name_of(i), bg=UI_BG, fg=FG, width=6,
                          anchor="w", cursor="hand2")
            nm.pack(side=tk.LEFT)
            nm.bind("<Double-Button-1>", lambda e, i=i: self.rename_channel(i))
            self.name_labels.append(nm)
            ge = tk.Entry(f, width=5, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
            ge.insert(0, self.num(self.gain[i])); ge.pack(side=tk.LEFT, padx=1)
            ge.bind("<FocusOut>", lambda e, i=i, ent=ge: self.on_gain(i, ent))
            oe = tk.Entry(f, width=5, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
            oe.insert(0, self.num(self.off[i])); oe.pack(side=tk.LEFT, padx=1)
            oe.bind("<FocusOut>", lambda e, i=i, ent=oe: self.on_off(i, ent))
            vl = tk.Label(f, text="-", bg=UI_BG, fg=self.color_of(i), width=9, anchor="e")
            vl.pack(side=tk.LEFT)
            self.val_labels.append(vl)

        # 画布 (按 z 序创建: 网格 -> 边框 -> 波形 -> 值 -> 光标 -> 捕捉)
        self.canvas = tk.Canvas(root, bg=PLOTBG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        cv = self.canvas
        self.grid_v = [cv.create_line(0, 0, 0, 0, fill=GRID, state=tk.HIDDEN) for _ in range(GRID_MAX)]
        self.grid_h = [cv.create_line(0, 0, 0, 0, fill=GRID, state=tk.HIDDEN) for _ in range(GRID_MAX)]
        self.grid_tx = [cv.create_text(0, 0, fill=DIM, font=("Consolas", 8), state=tk.HIDDEN)
                        for _ in range(GRID_MAX)]
        self.grid_ty = [cv.create_text(0, 0, anchor="e", fill=DIM, font=("Consolas", 8),
                                       state=tk.HIDDEN) for _ in range(GRID_MAX)]
        self.axis_box = cv.create_rectangle(0, 0, 0, 0, outline=AXIS)
        self.lines = [cv.create_line(0, 0, 0, 0, fill=self.color_of(i), width=1, state=tk.HIDDEN)
                      for i in range(MAX_CH)]
        self.val_texts = [cv.create_text(0, 0, anchor="e", fill=self.color_of(i),
                                         font=("Consolas", 9), text="") for i in range(MAX_CH)]
        self.cur_line = cv.create_line(0, 0, 0, 0, fill="#909090", dash=(4, 4), state=tk.HIDDEN)
        self.marks = [cv.create_oval(0, 0, 0, 0, fill=self.color_of(i), outline="", state=tk.HIDDEN)
                      for i in range(MAX_CH)]
        self.info_box = cv.create_rectangle(0, 0, 0, 0, fill="#ffffe0", outline="#a0a000",
                                            state=tk.HIDDEN)
        self.info_txt = cv.create_text(0, 0, anchor="nw", justify=tk.LEFT, fill=FG,
                                       font=("Consolas", 9), text="", state=tk.HIDDEN)

    # ---------- 滑条 ----------
    def add_slider(self, parent, kind, label, mn, mx, step):
        f = tk.Frame(parent, bg=UI_BG); f.pack(fill=tk.X, padx=8, pady=(2, 0))
        name = tk.Label(f, text=label, bg=UI_BG, fg=FG, width=7, anchor="w", cursor="hand2")
        name.pack(side=tk.LEFT)
        name.bind("<Double-Button-1>", lambda e, k=kind: self.open_range_dialog(k))
        lbl = tk.Label(f, text="0", bg=UI_BG, fg="#00337f", anchor="w")
        lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)
        live = tk.BooleanVar(value=True)          # 默认实时发送
        tk.Checkbutton(f, text="实时", variable=live, bg=UI_BG, fg=FG,
                       activebackground=UI_BG,
                       command=self.save_cfg_later).pack(side=tk.LEFT, padx=2)
        tk.Button(f, text="发", relief=tk.FLAT, bg="#dcdcdc", padx=8,
                  command=lambda k=kind: self.send(CMD[k], self.sliders[k]['scale'].get())
                  ).pack(side=tk.LEFT)

        sc = tk.Scale(parent, from_=mn, to=mx, resolution=step, orient=tk.HORIZONTAL,
                      length=250, bg=UI_BG, fg=FG, troughcolor="#d8d8d8",
                      activebackground="#bfbfbf", highlightthickness=0, bd=0,
                      showvalue=0, command=lambda val, k=kind: self.on_slider(k, val))
        sc.pack(fill=tk.X, padx=8)
        sc.bind("<ButtonRelease-1>", lambda ev, k=kind: self.send(CMD[k], sc.get()))

        self.sliders[kind] = dict(scale=sc, mn=float(mn), mx=float(mx), step=float(step),
                                  live=live, lbl=lbl, label=label, last_tx=0.0)

    def open_range_dialog(self, kind):
        d = self.sliders[kind]
        win = tk.Toplevel(self.root)
        win.title("配置 " + d['label'])
        win.configure(bg=UI_BG)
        win.resizable(False, False)
        win.transient(self.root)
        win.grab_set()
        entries = {}
        for i, (text, val) in enumerate((("最小值", d['mn']), ("最大值", d['mx']), ("步长", d['step']))):
            tk.Label(win, text=text, bg=UI_BG, fg=FG, anchor="w", width=6).grid(
                row=i, column=0, padx=10, pady=4, sticky="w")
            e = tk.Entry(win, width=12, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
            e.insert(0, self.num(val)); e.grid(row=i, column=1, padx=10, pady=4)
            entries[text] = e

        def ok():
            try:
                mn = float(entries["最小值"].get()); mx = float(entries["最大值"].get())
                st = float(entries["步长"].get())
                if mx <= mn or st <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showwarning("提示", "范围/步长非法 (需 最大>最小 且 步长>0)", parent=win)
                return
            d['mn'], d['mx'], d['step'] = mn, mx, st
            d['scale'].config(from_=mn, to=mx, resolution=st)
            self.save_cfg_later()
            win.destroy()

        f = tk.Frame(win, bg=UI_BG); f.grid(row=3, column=0, columnspan=2, pady=8)
        tk.Button(f, text="确定", relief=tk.FLAT, bg="#dcdcdc", padx=14, command=ok).pack(side=tk.LEFT, padx=6)
        tk.Button(f, text="取消", relief=tk.FLAT, bg="#dcdcdc", padx=14, command=win.destroy).pack(side=tk.LEFT, padx=6)
        entries["最小值"].focus_set()

    def rename_channel(self, i):
        win = tk.Toplevel(self.root)
        win.title("重命名通道")
        win.configure(bg=UI_BG); win.resizable(False, False)
        win.transient(self.root); win.grab_set()
        tk.Label(win, text="名称", bg=UI_BG, fg=FG).grid(row=0, column=0, padx=10, pady=8)
        e = tk.Entry(win, width=16, bg=FIELD, fg=FG, relief=tk.SOLID, bd=1)
        e.insert(0, self.name_of(i)); e.grid(row=0, column=1, padx=10, pady=8)

        def ok():
            nm = e.get().strip()
            if nm:
                self.ch_names[i] = nm
                self.name_labels[i].config(text=nm)
                self.save_cfg_later()
                self.mark_dirty()
            win.destroy()

        f = tk.Frame(win, bg=UI_BG); f.grid(row=1, column=0, columnspan=2, pady=(0, 8))
        tk.Button(f, text="确定", relief=tk.FLAT, bg="#dcdcdc", padx=14, command=ok).pack(side=tk.LEFT, padx=6)
        tk.Button(f, text="取消", relief=tk.FLAT, bg="#dcdcdc", padx=14, command=win.destroy).pack(side=tk.LEFT, padx=6)
        e.focus_set(); e.select_range(0, tk.END)

    def on_slider(self, kind, val):
        d = self.sliders[kind]
        d['lbl'].config(text=self.num(float(val)))
        if d['live'].get():
            now = time.time()
            if now - d['last_tx'] > 0.05:
                d['last_tx'] = now
                self.send(CMD[kind], float(val))

    def num(self, v):
        return ("%.4f" % v).rstrip("0").rstrip(".") if abs(v) < 100 else "%.0f" % v

    def set_all_vis(self, on):
        for i, v in enumerate(self.vis_vars):
            v.set(on)
            self.vis[i] = on
        self.mark_dirty()
        self.save_cfg_later()

    def apply_max_span(self):
        try:
            v = float(self.ent_maxspan.get())
            if v <= 1e-4:
                raise ValueError
        except ValueError:
            return
        self.max_span = v
        self.t_span = min(self.t_span, v)
        self._trim()
        self.mark_dirty()
        self.save_cfg_later()

    # ==================== 事件 ====================
    def mark_dirty(self):
        self.dirty = True

    def bind_events(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        c = self.canvas
        c.bind("<Configure>", lambda e: self.mark_dirty())
        c.bind("<MouseWheel>", self.on_wheel)
        c.bind("<ButtonPress-1>", self.on_press)
        c.bind("<B1-Motion>", self.on_motion)
        c.bind("<ButtonRelease-1>", lambda e: setattr(self, "drag", None))
        c.bind("<Double-Button-1>", self.on_dbl)
        c.bind("<Motion>", self.on_move)
        c.bind("<Leave>", self.on_leave)

    def on_close(self):
        self.save_cfg()
        if self._tick_job:
            try: self.root.after_cancel(self._tick_job)
            except Exception: pass
            self._tick_job = None
        self.root.destroy()

    def on_vis(self, i):
        self.vis[i] = self.vis_vars[i].get()
        self.mark_dirty()
        self.save_cfg_later()

    def on_gain(self, i, ent):
        try: self.gain[i] = float(ent.get())
        except ValueError: pass
        self.mark_dirty(); self.save_cfg_later()

    def on_off(self, i, ent):
        try: self.off[i] = float(ent.get())
        except ValueError: pass
        self.mark_dirty(); self.save_cfg_later()

    def on_demo(self):
        if self.demo.get():
            self.disconnect()
            self.paused = False

    def toggle_pause(self):
        self.paused = not self.paused
        if not self.paused:
            self.follow = True
        self.btn_pause.config(text="继续显示" if self.paused else "暂停显示",
                              bg="#f6d66b" if self.paused else "#dcdcdc")
        self.mark_dirty()

    def toggle_follow(self):
        self.follow = not self.follow
        self.mark_dirty()

    def clear(self):
        for b in self.bufs:
            del b[:]
        self.base = 0
        self.total = 0
        self.frames = 0
        self.mark_dirty()

    # ==================== 串口 ====================
    def port_list(self):
        return [p.device for p in list_ports.comports()] if HAVE_SERIAL else []

    def connect(self):
        if not HAVE_SERIAL:
            messagebox.showerror("错误", "未安装 pyserial: pip install pyserial")
            return
        dev = self.cmb_port.get().strip()
        if not dev:
            messagebox.showwarning("提示", "先选择串口 (可点刷新)")
            return
        try:
            self.ser = serial.Serial(dev, 115200, timeout=0.05)
        except Exception as e:
            messagebox.showerror("打开失败", str(e))
            return
        self.demo.set(False)
        self.paused = False
        self.btn_pause.config(text="暂停显示", bg="#dcdcdc")
        self.running = True
        self.ser_thread = threading.Thread(target=self.reader, daemon=True)
        self.ser_thread.start()
        self.btn_disconnect.config(state=tk.NORMAL)

    def disconnect(self):
        self.running = False
        if self.ser:
            try: self.ser.close()
            except Exception: pass
            self.ser = None
        self.btn_disconnect.config(state=tk.DISABLED)

    def reader(self):
        buf = bytearray()
        while self.running and self.ser:
            try:
                data = self.ser.read(4096)
            except Exception:
                break
            if not data:
                continue
            self.rx_bytes += len(data)
            buf += data
            while True:
                i = buf.find(TAIL)
                if i < 0:
                    break
                payload = bytes(buf[:i])
                del buf[:i + 4]
                if i > 0 and i % 4 == 0:
                    try:
                        self.q.put_nowait(struct.unpack("<%df" % (i // 4), payload))
                    except Exception:
                        pass
            if len(buf) > 8192:
                del buf[:-4]

    def send(self, cid, val=None):
        if not self.ser:
            self.flash("未连接")
            return
        b = bytearray(5)
        b[0] = cid
        if val is not None:
            b[1:5] = struct.pack("<f", float(val))
        try:
            with self.tx_lock:
                self.ser.write(bytes(b))
        except Exception as e:
            self.flash("发送失败: %s" % e)

    def send_entry(self, key, ent):
        try:
            self.send(CMD[key], float(ent.get() or 0))
        except ValueError:
            self.flash("数值非法")

    def send_raw(self):
        parts = self.ent_raw.get().replace(",", " ").split()
        if len(parts) != 5:
            self.flash("需要 5 个字节"); return
        try:
            b = bytes(int(p, 16) for p in parts)
        except ValueError:
            self.flash("非法字节"); return
        if not self.ser:
            self.flash("未连接"); return
        with self.tx_lock:
            self.ser.write(b)

    def reset_cfg(self):
        if messagebox.askyesno("确认", "恢复默认会清掉校准参数(需重新校准), 继续?"):
            self.send(CMD["reset"], 0.0)

    def flash(self, msg):
        self.lbl_status.config(text=msg)

    # ==================== 数据 ====================
    def ingest(self):
        if self.paused:
            return
        if self.demo.get() and not self.ser:
            dt = 0.033
            for _ in range(max(1, int(self.fs * dt))):
                t = self.demo_t
                self.demo_t += 1.0 / self.fs
                ia = 0.3 * math.sin(2 * math.pi * 20 * t)
                ib = 0.3 * math.sin(2 * math.pi * 20 * t - 2.094)
                ic = 0.3 * math.sin(2 * math.pi * 20 * t + 2.094)
                idd = 0.02 * math.sin(2 * math.pi * 3 * t)
                iq = 0.3 * math.cos(2 * math.pi * 3 * t)
                vel = 2.0 + 0.05 * math.sin(2 * math.pi * 1 * t)
                self.push((ia, ib, ic, idd, iq, 0.3, 2.0, vel, 1.5 * 7 * 0.00154 * iq))
            return

        cnt = 0
        while cnt < 20000:
            try:
                vals = self.q.get_nowait()
            except queue.Empty:
                break
            self.push(vals)
            cnt += 1

    def push(self, vals):
        n = min(len(vals), MAX_CH)
        if n > self.nch:
            self.nch = n
        for c in range(self.nch):
            self.bufs[c].append(vals[c] if c < n else 0.0)
        self.total += 1
        self._trim()
        self.frames += 1
        self.dirty = True

    def _trim(self):
        keep = max(100, int(self.max_span * self.fs))
        chunk = max(64, keep // 8)
        if len(self.bufs[0]) > keep + chunk:
            n = len(self.bufs[0]) - keep
            for c in range(MAX_CH):
                del self.bufs[c][:n]
            self.base += n
        if len(self.bufs[0]) > CAP:
            for c in range(MAX_CH):
                del self.bufs[c][:TRIM]
            self.base += TRIM

    # ==================== 绘制 ====================
    def plot_rect(self):
        return (68, 12, self.canvas.winfo_width() - 12, self.canvas.winfo_height() - 28)

    def nice_step(self, span, px):
        rough = span / max(2.0, px / 70.0)
        p = 10.0 ** math.floor(math.log10(rough))
        m = rough / p
        m = 1.0 if m < 1.5 else 2.0 if m < 3 else 5.0 if m < 7 else 10.0
        return m * p

    def draw(self):
        cv = self.canvas
        x0, y0, x1, y1 = self.plot_rect()
        pw, ph = x1 - x0, y1 - y0
        if pw < 20 or ph < 20:
            return

        if self.t_span > self.max_span:
            self.t_span = self.max_span
        if self.follow and not self.paused:
            self.t_right = self.total / max(1.0, self.fs)
        tL = self.t_right - self.t_span
        ymin, ymax = self.y_min, self.y_max
        yspan = max(1e-12, ymax - ymin)
        ysc = ph / yspan

        # ---- 网格 (复用对象) ----
        tstep = self.nice_step(self.t_span, pw)
        t = math.ceil(tL / tstep) * tstep
        i = 0
        while t <= self.t_right + 1e-12 and i < GRID_MAX:
            gx = x0 + (t - tL) / self.t_span * pw
            cv.coords(self.grid_v[i], gx, y0, gx, y1)
            cv.itemconfig(self.grid_v[i], state=tk.NORMAL)
            cv.coords(self.grid_tx[i], gx, y1 + 12)
            cv.itemconfig(self.grid_tx[i], state=tk.NORMAL, text=self.fmt(t, tstep))
            t += tstep; i += 1
        for j in range(i, GRID_MAX):
            cv.itemconfig(self.grid_v[j], state=tk.HIDDEN)
            cv.itemconfig(self.grid_tx[j], state=tk.HIDDEN)

        ystep = self.nice_step(yspan, ph)
        v = math.ceil(ymin / ystep) * ystep
        i = 0
        while v <= ymax + 1e-12 and i < GRID_MAX:
            gy = y1 - (v - ymin) * ysc
            cv.coords(self.grid_h[i], x0, gy, x1, gy)
            cv.itemconfig(self.grid_h[i], state=tk.NORMAL)
            cv.coords(self.grid_ty[i], x0 - 7, gy)
            cv.itemconfig(self.grid_ty[i], state=tk.NORMAL, text=self.fmt(v, ystep))
            v += ystep; i += 1
        for j in range(i, GRID_MAX):
            cv.itemconfig(self.grid_h[j], state=tk.HIDDEN)
            cv.itemconfig(self.grid_ty[j], state=tk.HIDDEN)
        cv.coords(self.axis_box, x0, y0, x1, y1)

        # ---- 可见样本范围 (至少 2 点) ----
        k0 = max(self.base, int(math.floor(tL * self.fs)))
        k1 = min(int(math.ceil(self.t_right * self.fs)), self.total - 1)
        if k0 > k1:
            k0 = min(k0, max(0, self.total - 1)); k1 = k0
        if k1 <= k0 and self.total >= 2:
            k0 = max(0, min(k0, self.total - 2)); k1 = k0 + 1
        nvis = k1 - k0 + 1
        self.last_nvis = nvis

        if nvis >= 2:
            i0 = k0 - self.base
            i1 = k1 - self.base + 1
            nmax = max(2, i1 - i0)
            # 样本比像素多很多时用 min/max 包络抽点 (每个像素列取最小/最大), 避免抽点混叠
            envelope = nvis > 2 * pw
            xsc = pw / float(nmax - 1)
            if envelope:
                ncol = max(1, int(pw))
                col_w = nvis / float(ncol)
                inner = max(1, int(nvis / (4.0 * pw)))   # 每列最多扫 4 个样本
            else:
                dx = xsc

            for c in range(self.nch):
                g, o = self.gain[c], self.off[c]
                buf = self.bufs[c]
                if not self.vis[c]:
                    cv.itemconfig(self.lines[c], state=tk.HIDDEN)
                    cv.itemconfig(self.val_texts[c], state=tk.HIDDEN)
                    continue
                if not envelope:
                    n = nmax
                    pts = [0.0] * (2 * n)
                    px = x0
                    j = 0
                    for kk in range(i0, i1):
                        pts[j] = px
                        pts[j + 1] = y1 - (buf[kk] * g + o - ymin) * ysc
                        j += 2
                        px += dx
                    cv.coords(self.lines[c], *pts)
                    cv.itemconfig(self.lines[c], state=tk.NORMAL)
                else:
                    pts = []
                    for col in range(ncol):
                        s0 = i0 + int(col * col_w)
                        e0 = i0 + int((col + 1) * col_w)
                        if e0 <= s0:
                            e0 = s0 + 1
                        if e0 > i1:
                            e0 = i1
                        v0 = buf[s0] * g + o
                        mn = mx = v0
                        imn = imx = s0
                        kk = s0
                        while kk < e0:
                            vv = buf[kk] * g + o
                            if vv < mn: mn = vv; imn = kk
                            if vv > mx: mx = vv; imx = kk
                            kk += inner
                        kl = e0 - 1
                        vv = buf[kl] * g + o
                        if vv < mn: mn = vv; imn = kl
                        if vv > mx: mx = vv; imx = kl
                        if imn <= imx:
                            pts += [x0 + (imn - i0) * xsc, y1 - (mn - ymin) * ysc,
                                    x0 + (imx - i0) * xsc, y1 - (mx - ymin) * ysc]
                        else:
                            pts += [x0 + (imx - i0) * xsc, y1 - (mx - ymin) * ysc,
                                    x0 + (imn - i0) * xsc, y1 - (mn - ymin) * ysc]
                    if len(pts) >= 4:
                        cv.coords(self.lines[c], *pts)
                        cv.itemconfig(self.lines[c], state=tk.NORMAL)
                    else:
                        cv.itemconfig(self.lines[c], state=tk.HIDDEN)

                last = buf[-1] if buf else None
                if last is None:
                    cv.itemconfig(self.val_texts[c], state=tk.HIDDEN)
                else:
                    cv.itemconfig(self.val_texts[c], state=tk.NORMAL,
                                  text="%s %s" % (self.name_of(c), self.fmt(last * g + o, 0.001)),
                                  fill=self.color_of(c))
                    cv.coords(self.val_texts[c], x1 - 6, y0 + 10 + 12 * c)
                if c < len(self.val_labels):
                    self.val_labels[c].config(
                        text="-" if last is None else self.fmt(last * g + o, 0.001))
        else:
            for c in range(MAX_CH):
                cv.itemconfig(self.lines[c], state=tk.HIDDEN)
                cv.itemconfig(self.val_texts[c], state=tk.HIDDEN)

        # ---- 光标 + 捕捉读数 ----
        self.draw_cursor(tL, pw, x0, y0, x1, y1)

    def draw_cursor(self, tL, pw, x0, y0, x1, y1):
        cv = self.canvas
        if not self.cursor:
            cv.itemconfig(self.cur_line, state=tk.HIDDEN)
            cv.itemconfig(self.info_box, state=tk.HIDDEN)
            cv.itemconfig(self.info_txt, state=tk.HIDDEN)
            for m in self.marks:
                cv.itemconfig(m, state=tk.HIDDEN)
            return

        cx = self.cursor[0]
        inside = (x0 <= cx <= x1) and (self.total > 0)
        if not inside:
            cv.itemconfig(self.cur_line, state=tk.HIDDEN)
            cv.itemconfig(self.info_box, state=tk.HIDDEN)
            cv.itemconfig(self.info_txt, state=tk.HIDDEN)
            for m in self.marks:
                cv.itemconfig(m, state=tk.HIDDEN)
            return

        tcur = tL + (cx - x0) / pw * self.t_span
        k = int(round(tcur * self.fs))
        k = max(self.base, min(k, self.total - 1))
        xk = x0 + (k / self.fs - tL) / self.t_span * pw

        cv.coords(self.cur_line, xk, y0, xk, y1)
        cv.itemconfig(self.cur_line, state=tk.NORMAL)

        lines = ["t = %.5f s" % (k / self.fs)]
        ymin, ymax = self.y_min, self.y_max
        yspan = max(1e-12, ymax - ymin)
        ysc = (y1 - y0) / yspan
        for c in range(self.nch):
            if not self.vis[c]:
                cv.itemconfig(self.marks[c], state=tk.HIDDEN)
                continue
            s = self.bufs[c][k - self.base] if 0 <= k - self.base < len(self.bufs[c]) else None
            if s is None:
                cv.itemconfig(self.marks[c], state=tk.HIDDEN)
                continue
            vv = s * self.gain[c] + self.off[c]
            yk = y1 - (vv - ymin) * ysc
            cv.coords(self.marks[c], xk - 2.5, yk - 2.5, xk + 2.5, yk + 2.5)
            cv.itemconfig(self.marks[c], state=tk.NORMAL)
            lines.append("%-8s %s" % (self.name_of(c), self.fmt(vv, 0.0001)))
        for c in range(self.nch, MAX_CH):
            cv.itemconfig(self.marks[c], state=tk.HIDDEN)

        cv.coords(self.info_txt, x0 + 8, y0 + 6)
        cv.itemconfig(self.info_txt, state=tk.NORMAL, text="\n".join(lines))
        bb = cv.bbox(self.info_txt)
        if bb:
            cv.coords(self.info_box, bb[0] - 5, bb[1] - 4, bb[2] + 5, bb[3] + 4)
            cv.itemconfig(self.info_box, state=tk.NORMAL)
            cv.tag_raise(self.info_txt)   # 文字压在框上面

    def fmt(self, v, step):
        if v == 0:
            return "0"
        a = abs(v)
        if a >= 10000 or a < 0.001:
            return "%.3g" % v
        if step >= 1:
            return "%.0f" % v
        if step >= 0.1:
            return "%.1f" % v
        if step >= 0.01:
            return "%.2f" % v
        return "%.4f" % v

    # ==================== 交互 ====================
    def on_wheel(self, e):
        x0, y0, x1, y1 = self.plot_rect()
        amp = (e.x < x0) or (e.state & 0x0004) != 0 or (e.state & 0x0001) != 0
        factor = 1.1 ** (-e.delta / 120.0)
        if amp:
            my = min(max(e.y, y0), y1)
            frac = (y1 - my) / max(1, y1 - y0)
            vcur = self.y_min + frac * (self.y_max - self.y_min)
            span = min(max((self.y_max - self.y_min) * factor, 1e-9), 1e9)
            self.y_min = vcur - frac * span
            self.y_max = self.y_min + span
        elif self.follow:
            self.t_span = min(max(self.t_span * factor, 1e-7), self.max_span)
        else:
            mx = min(max(e.x, x0), x1)
            frac = (mx - x0) / max(1, x1 - x0)
            tcur = (self.t_right - self.t_span) + frac * self.t_span
            span = min(max(self.t_span * factor, 1e-7), self.max_span)
            self.t_right = tcur + (1 - frac) * span
            self.t_span = span
        self.mark_dirty()

    def on_press(self, e):
        self.drag = (e.x, e.y, self.t_right, self.y_min, self.y_max)

    def on_motion(self, e):
        self.cursor = (e.x, e.y)
        self.mark_dirty()
        if not self.drag:
            return
        x0, y0, x1, y1 = self.plot_rect()
        pw, ph = max(1, x1 - x0), max(1, y1 - y0)
        self.t_right = self.drag[2] - (e.x - self.drag[0]) / pw * self.t_span
        self.follow = False
        dyv = (e.y - self.drag[1]) / ph * (self.drag[4] - self.drag[3])
        self.y_min = self.drag[3] + dyv
        self.y_max = self.drag[4] + dyv

    def on_move(self, e):
        self.cursor = (e.x, e.y)
        self.mark_dirty()

    def on_leave(self, e=None):
        self.cursor = None
        self.mark_dirty()

    def on_dbl(self, e):
        self.follow = True
        self.autoscale()

    def autoscale(self):
        tL = self.t_right - self.t_span
        k0 = max(self.base, int(tL * self.fs))
        k1 = min(self.total - 1, int(self.t_right * self.fs))
        if k1 <= k0:
            return
        stride = max(1, (k1 - k0) // 2000)
        lo, hi = float("inf"), float("-inf")
        for c in range(self.nch):
            if not self.vis[c]:
                continue
            g, o = self.gain[c], self.off[c]
            k = k0
            while k <= k1:
                s = self.bufs[c][k - self.base] if 0 <= k - self.base < len(self.bufs[c]) else None
                if s is not None:
                    vv = s * g + o
                    if vv < lo: lo = vv
                    if vv > hi: hi = vv
                k += stride
        if lo == float("inf"):
            return
        if hi - lo < 1e-9:
            hi += 0.5; lo -= 0.5
        pad = (hi - lo) * 0.1
        self.y_min, self.y_max = lo - pad, hi + pad
        self.follow = True
        self.mark_dirty()

    # ==================== 主循环 ====================
    def tick(self):
        self.ingest()
        if self.dirty:
            self.draw()
            self.dirty = False
        now = time.time()
        if now - self.stat_t >= 0.5:
            fps = (self.frames - self.stat_frames) / (now - self.stat_t)
            kbs = (self.rx_bytes - self.stat_bytes) / (now - self.stat_t) / 1024.0
            if self.paused:
                state = "已暂停显示"
            elif self.follow:
                state = "跟随"
            else:
                state = "平移/缩放"
            self.lbl_status.config(text="%s | %.0f 帧/s | %.0f Hz | 窗内 %d 点 | %.0f kB/s" %
                                        (state, fps, self.fs, self.last_nvis, kbs))
            self.stat_t, self.stat_frames, self.stat_bytes = now, self.frames, self.rx_bytes
        self._tick_job = self.root.after(33, self.tick)


def main():
    root = tk.Tk()
    ScopeApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
