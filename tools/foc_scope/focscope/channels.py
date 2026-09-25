# -*- coding: utf-8 -*-
"""通道语义定义.

启动时自动从固件 vofa.c 的 vofa_snapshot() 解析通道 (短名/显示名/单位),
固件改了波形通道, 上位机重启即同步; 解析失败则回退到内置默认表.
"""

from . import theme
from .vofa_source import load_channels

# 回退默认表 (解析不到 vofa.c 时使用): (短名, 中文显示名, 单位)
_FALLBACK = [
    ("Ia",          "A相电流",     "A"),
    ("Ib",          "B相电流",     "A"),
    ("Ic",          "C相电流",     "A"),
    ("Id",          "D轴电流",     "A"),
    ("Iq",          "Q轴电流",     "A"),
    ("Iq_set",      "Q轴电流指令", "A"),
    ("vel_set",     "转速指令",    "rps"),
    ("vel",         "电机转速",    "rps"),
    ("torque",      "电机转矩",    "N·m"),
    ("Vbus",        "母线电压",    "V"),
    ("Vmag",        "电压幅值",    "V"),
    ("sat",         "饱和标志",    ""),
    ("Vd",          "D轴电压",     "V"),
    ("Vq",          "Q轴电压",     "V"),
    ("phase",       "电角度",      "rad"),
    ("phase_vel",   "电角速度",    "rad/s"),
    ("isum",        "三相电流和",  "A"),
    ("Va",          "A相电压",     "V"),
    ("Vb",          "B相电压",     "V"),
    ("Vc",          "C相电压",     "V"),
    ("fault",       "故障码",      ""),
    ("Ibus",        "母线电流",    "A"),
]

_parsed = load_channels()
_source = _parsed or _FALLBACK
# 自动绑定固件通道: (短名, 显示名, 单位, 颜色)
CHANNELS = [(s, d, u, theme.CURVE[i % len(theme.CURVE)])
            for i, (s, d, u) in enumerate(_source)]
NCH = len(CHANNELS)
FROM_VOFA = _parsed is not None

# 故障码 (固件 foc.h FOC_FAULT_*)
FAULT_TEXT = {0: "无", 1: "相过流", 2: "母线过压", 3: "母线欠压", 4: "ADC故障"}


def index_of(name):
    """按短名查找通道下标; 先精确匹配, 再前缀匹配. 找不到返回 None."""
    for i, c in enumerate(CHANNELS):
        if c[0] == name:
            return i
    for i, c in enumerate(CHANNELS):
        if c[0].startswith(name):
            return i
    return None


def indices_of(*names):
    out = []
    for n in names:
        i = index_of(n)
        if i is not None:
            out.append(i)
    return out

# 驱动器状态灯: (键, 显示名, 类型)
#   类型 enabled=本地使能状态; fault=等于某故障码; ok=fault==0
STATUS_ITEMS = [
    ("enabled", "电机使能", "enabled"),
    ("ok",      "运行正常", "ok"),
    ("f_oc",    "相过流",   1),
    ("f_ov",    "母线过压", 2),
    ("f_uv",    "母线欠压", 3),
    ("f_adc",   "ADC故障",  4),
]


class ChannelModel:
    """通道的名称/颜色/增益/偏置/是否显示在"选择性波形图"."""

    # 顶部固定显示三相电流的通道 (按名字自动定位)
    TOP_CHANNELS = tuple(indices_of("Ia", "Ib", "Ic")) or (0, 1, 2)

    def __init__(self):
        self.short = [c[0] for c in CHANNELS]
        self.disp = [c[1] for c in CHANNELS]
        self.unit = [c[2] for c in CHANNELS]
        self.color = [c[3] for c in CHANNELS]
        self.gain = [1.0] * NCH
        self.off = [0.0] * NCH
        self.shown = [False] * NCH
        # 默认: 选择性波形图显示 转速反馈/指令
        for i in indices_of("vel", "vel_set"):
            self.shown[i] = True
        if not any(self.shown):
            for i in (6, 7):
                if i < NCH:
                    self.shown[i] = True

    def name(self, i):
        u = self.unit[i]
        return "%s%s" % (self.disp[i], ("/" + u) if u else "")

    def short_name(self, i):
        return self.short[i]

    def value(self, i, raw):
        return raw * self.gain[i] + self.off[i]

    def set_shown(self, i, on):
        self.shown[i] = bool(on)

    def shown_channels(self):
        return [i for i in range(NCH) if self.shown[i]]

    # ---------- 配置读写 ----------
    def to_cfg(self):
        return {
            "name": list(self.disp),
            "short": list(self.short),
            "gain": list(self.gain),
            "off": list(self.off),
            "shown": list(self.shown),
        }

    def load_cfg(self, cfg):
        if not cfg:
            return
        for key, attr in (("name", "disp"), ("short", "short"), ("gain", "gain"),
                          ("off", "off")):
            vals = cfg.get(key)
            if isinstance(vals, list):
                cur = getattr(self, attr)
                for i in range(min(len(vals), NCH)):
                    try:
                        cur[i] = vals[i]
                    except Exception:
                        pass
        shown = cfg.get("shown")
        if isinstance(shown, list):
            for i in range(min(len(shown), NCH)):
                self.shown[i] = bool(shown[i])
        else:
            # 兼容旧配置: 取原图2 可见性
            vis = cfg.get("vis")
            if isinstance(vis, list) and len(vis) == 2:
                for i in range(min(len(vis[1]), NCH)):
                    self.shown[i] = bool(vis[1][i])
