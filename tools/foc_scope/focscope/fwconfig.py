# -*- coding: utf-8 -*-
"""从固件源码自动读取配置参数.

读取 foc/core/outer 下的:
  src/config.c  : config_init() 默认值 + config_debug_overrides() 覆盖
  inc/foc.h     : FOC_FAULT_* 故障码
  inc/vofa.h    : VOFA_SAMPLE_DECIM / 采样率注释
  src/foc_ctrl.c: CTRL_PERIOD 控制周期

得到: 参数默认值/限幅、故障码文本、采样率. 解析失败一律回退到调用方给的默认值.
"""
import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))

_ROOTS = [
    os.path.join(_HERE, "..", "..", "..", "foc", "core", "outer"),
    os.path.join(_HERE, "..", "foc", "core", "outer"),
]


def _find_root():
    env = os.environ.get("FOC_OUTER_DIR")
    if env and os.path.isdir(env):
        return os.path.abspath(env)
    for r in _ROOTS:
        p = os.path.abspath(r)
        if os.path.isdir(p):
            return p
    return None


def _read(path):
    if not path or not os.path.isfile(path):
        return None
    for enc in ("utf-8", "gbk", "latin-1"):
        try:
            with open(path, "r", encoding=enc) as f:
                return f.read()
        except UnicodeDecodeError:
            continue
        except OSError:
            return None
    return None


_NUM = re.compile(r"^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?[fF]?$")


def _to_num(expr):
    s = expr.strip().rstrip(";").strip()
    if s in ("false", "FALSE"):
        return 0
    if s in ("true", "TRUE"):
        return 1
    if _NUM.match(s):
        s = s.rstrip("fF")
        try:
            f = float(s)
            return int(f) if f == int(f) else f
        except ValueError:
            return None
    return None


def _assignments(body):
    out = {}
    if not body:
        return out
    for m in re.finditer(r"Config\.(\w+)\s*=\s*([^;]+);", body):
        v = _to_num(m.group(2))
        if v is not None:
            out[m.group(1)] = v
    return out


def _func_body(text, name):
    m = re.search(r"\b%s\s*\([^)]*\)\s*\{(.*?)\n\}" % re.escape(name), text, re.S)
    return m.group(1) if m else None


def _parse_faults(text):
    """解析 FOC_FAULT_* 故障码.

    兼容两种固件写法:
      - 旧: 单值枚举   #define FOC_FAULT_OVER_VOLTAGE 2
      - 新: 位掩码     #define FOC_FAULT_OVER_VOLTAGE (1u << 1)
    返回 (by_code, by_name):
      - by_name[name] = 掩码值 (位掩码语义, 可直接用 & 判断)
      - by_code[bit]  = 文本  (旧枚举时 key 是枚举值; 新位掩码时 key 是 bit 序号)
    """
    by_code, by_name = {}, {}
    fallback = {"ADC": "ADC故障"}
    if not text:
        return by_code, by_name
    for line in text.splitlines():
        # 值: 十进制整数 或 (1u << N) 位掩码表达式
        m = re.fullmatch(
            r"\s*#define\s+FOC_FAULT_(\w+)\s+"
            r"(?:(\d+)|\(1u?\s*<<\s*(\d+)\))\s*"
            r"(?://\s*(.*?)|/\*\s*(.*?)\s*\*/)?\s*", line
        )
        if not m:
            continue
        name = m.group(1)
        cmt = (m.group(4) or m.group(5) or "").strip()
        if m.group(2) is not None:
            # 旧式单值枚举
            code = int(m.group(2))
            by_name[name] = code
            if code == 0:
                by_code[code] = "无"
            else:
                by_code[code] = cmt or fallback.get(name, name)
        else:
            # 新式位掩码 (1u << N): 值 = 1<<N, key 用 bit 序号
            bit = int(m.group(3))
            code = 1 << bit
            by_name[name] = code
            by_code[bit] = cmt or fallback.get(name, name)
    return by_code, by_name


def _parse_fs(vofa_text, ctrl_text):
    fs = None
    if vofa_text:
        m = re.search(r"(\d+(?:\.\d+)?)\s*kHz\s*/", vofa_text)
        d = re.search(r"#define\s+VOFA_SAMPLE_DECIM\s+(\d+)", vofa_text)
        if m and d:
            try:
                fs = float(m.group(1)) * 1000.0 / float(d.group(1))
            except (ValueError, ZeroDivisionError):
                fs = None
        if fs is None:
            m = re.search(r"=\s*(\d+(?:\.\d+)?)\s*kHz", vofa_text)
            if m:
                fs = float(m.group(1)) * 1000.0
    if fs is None and ctrl_text:
        m = re.search(r"#define\s+CTRL_PERIOD\s*\(\s*1\.0f?\s*/\s*(\d+(?:\.\d+)?)", ctrl_text)
        if m:
            fs = float(m.group(1))
    return fs


class FwConfig:
    def __init__(self):
        self.root = _find_root()
        self.params = {}
        self.faults = {}
        self.fault_by_name = {}
        self.fs = None
        self.sources = []
        self.ok = False
        self._load()

    def _load(self):
        if not self.root:
            return
        cfg_c = os.path.join(self.root, "src", "config.c")
        foc_h = os.path.join(self.root, "inc", "foc.h")
        vofa_h = os.path.join(self.root, "inc", "vofa.h")
        ctrl_c = os.path.join(self.root, "src", "foc_ctrl.c")

        cfg_text = _read(cfg_c)
        if cfg_text:
            self.sources.append(cfg_c)
            self.params.update(_assignments(_func_body(cfg_text, "config_init")))
            self.params.update(_assignments(_func_body(cfg_text, "config_debug_overrides")))
            # 注意: 调试覆盖应优先生效, 上面顺序保证覆盖在后

        foc_text = _read(foc_h)
        if foc_text:
            self.sources.append(foc_h)
            self.faults, self.fault_by_name = _parse_faults(foc_text)

        vofa_text = _read(vofa_h)
        ctrl_text = _read(ctrl_c)
        if vofa_text:
            self.sources.append(vofa_h)
        if ctrl_text:
            self.sources.append(ctrl_c)
        self.fs = _parse_fs(vofa_text, ctrl_text)

        self.ok = bool(self.params)

    # ---------- 取值 ----------
    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_float(self, name, default):
        try:
            return float(self.params.get(name, default))
        except (TypeError, ValueError):
            return float(default)

    def fault_code(self, suffix, default=None):
        """按 FOC_FAULT_<suffix> 查故障码."""
        v = self.fault_by_name.get(suffix)
        return default if v is None else v

    def summary(self):
        parts = []
        if self.params:
            parts.append("参数:config.c")
        if self.faults:
            parts.append("故障码:foc.h")
        if self.fs:
            parts.append("fs:%gHz" % self.fs)
        return " · ".join(parts)


_cache = None


def load(force=False):
    global _cache
    if _cache is None or force:
        _cache = FwConfig()
    return _cache
