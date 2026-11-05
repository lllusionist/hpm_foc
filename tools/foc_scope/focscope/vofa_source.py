# -*- coding: utf-8 -*-
"""从固件 vofa.c 自动解析波形通道定义.

解析 `vofa_snapshot(float *d)` 里的 `d[i] = 表达式; // 注释`,
生成通道列表 (短名, 显示名, 单位). 这样固件增删/调整通道后, 上位机重启即同步.
找不到文件或解析失败时返回 None, 由调用方回退到内置默认表.
"""
import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))

_CANDIDATES = [
    os.path.join(_HERE, "..", "..", "..", "foc", "core", "outer", "src", "vofa.c"),
    os.path.join(_HERE, "..", "foc", "core", "outer", "src", "vofa.c"),
]


def find_vofa_c():
    env = os.environ.get("FOC_VOFA_C")
    if env and os.path.isfile(env):
        return os.path.abspath(env)
    for c in _CANDIDATES:
        p = os.path.abspath(c)
        if os.path.isfile(p):
            return p
    return None


def _read_text(path):
    for enc in ("utf-8", "gbk", "latin-1"):
        try:
            with open(path, "r", encoding=enc) as f:
                return f.read()
        except UnicodeDecodeError:
            continue
        except OSError:
            return None
    return None


def _split_comment(cmt, i):
    """从注释得到 (显示名, 单位)."""
    if not cmt:
        return "data%d" % (i + 1), ""
    cmt = cmt.strip()
    # 去掉逗号后的补充说明 (如 ", 恒应≈0")
    cmt = re.split(r"[,，;；]", cmt, maxsplit=1)[0].strip()
    unit = ""
    m = re.search(r"[\(（]([^()（）]*)[\)）]\s*$", cmt)
    if m:
        unit = m.group(1).strip()
        cmt = cmt[:m.start()].strip()
    name = cmt.replace(" ", "").replace("　", "")
    return (name or "data%d" % (i + 1)), unit


def _short_from_expr(expr, i):
    for mod in ("FOC", "Controller", "encoder"):
        m = re.search(mod + r"\.(\w+)", expr)
        if m:
            return m.group(1)
    m = re.search(r"(\w+)\s*$", expr)
    return m.group(1) if m else "d%d" % i


def parse(path):
    text = _read_text(path)
    if text is None:
        return None, None
    nch = None
    m = re.search(r"#define\s+VOFA_NCH\s+(\d+)", text)
    if m:
        nch = int(m.group(1))
    m = re.search(r"vofa_snapshot\s*\([^)]*\)\s*\{(.*?)\n\}", text, re.S)
    if not m:
        return None, nch
    entries = {}
    pending = ""
    for line in m.group(1).splitlines():
        # 有些通道的注释写在上一行 (表达式太长), 先记下来给下一条 d[] 用
        mc = re.match(r"\s*/\*\s*(.+?)\s*\*/\s*$", line)
        if mc:
            pending = mc.group(1).strip()
            continue
        # 固件注释统一成 /* */, 这里同时兼容 // 与 /* */ 两种行尾注释
        mm = re.match(
            r"\s*d\[(\d+)\]\s*=\s*(.+?);\s*"
            r"(?://\s*(?P<lc>.*)|/\*\s*(?P<bc>.*?)\s*\*/)?\s*$", line)
        if mm:
            idx = int(mm.group(1))
            cmt = mm.group("lc")
            if cmt is None:
                cmt = mm.group("bc") or ""
            cmt = cmt.strip() or pending
            entries[idx] = (mm.group(2).strip(), cmt)
            pending = ""
    if not entries:
        return None, nch
    count = max(entries) + 1
    if nch:
        count = max(count, nch)
    out = []
    for i in range(count):
        expr, cmt = entries.get(i, ("", ""))
        disp, unit = _split_comment(cmt, i)
        short = _short_from_expr(expr, i) if expr else "data%d" % (i + 1)
        out.append((short, disp, unit))
    return out, count


def load_channels(path=None):
    """返回 [(短名, 显示名, 单位), ...] 或 None."""
    p = path or find_vofa_c()
    if not p:
        return None
    chans, _ = parse(p)
    return chans


def source_info(path=None):
    """返回 (路径 或 None, 通道数 或 0, 是否解析成功)."""
    p = path or find_vofa_c()
    if not p:
        return None, 0, False
    chans, count = parse(p)
    if chans:
        return p, len(chans), True
    return p, (count or 0), False
