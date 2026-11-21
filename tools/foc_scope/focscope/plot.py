# -*- coding: utf-8 -*-
"""自绘波形控件: QPainter 绘制网格/上下时间轴/右侧纵轴/曲线/光标读数/提示框.

交互:
  左键拖动         左右平移 / 上下缩放 Y (跟随时水平拖动会自动暂停跟随)
双击             恢复跟随最新数据
滚轮             时间缩放 (跟随状态下缩放右端)
  Ctrl/Shift+滚轮  或 在左侧刻度区滚轮   幅值缩放
  拖动             平移 (时间 + 幅值)
  双击             自动量程
  悬停             光标读数
"""
import math

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygonF, QBrush
from PySide6.QtWidgets import QWidget

from . import theme
from .channels import NCH

LM = 12          # 左留白
RM = 60          # 右侧纵轴刻度留白
TM = 20          # 顶部时间轴留白
BM = 22          # 底部时间轴留白


def _draw_trace(painter, points):
    """用缺失样本断开曲线，避免把两段有效数据连起来。"""
    poly = QPolygonF()
    for x, y in points:
        if math.isfinite(y):
            poly.append(QPointF(x, y))
        elif not poly.isEmpty():
            if poly.size() >= 2:
                painter.drawPolyline(poly)
            poly.clear()
    if poly.size() >= 2:
        painter.drawPolyline(poly)


class PlotWidget(QWidget):
    def __init__(self, store, model, parent=None):
        super().__init__(parent)
        self.store = store
        self.model = model
        self.channels = []             # 本图可见通道索引
        self.t_span = 0.5
        self.t_right = 0.0
        self.y_min, self.y_max = -0.2, 0.2
        self.follow = True
        self.frozen = False
        self.max_span = store.max_span
        self._cursor = None
        self._drag = None
        self._auto_pending = True
        self.setMouseTracking(True)
        self.setMinimumHeight(140)
        self.setAutoFillBackground(False)

    # ================= 外部接口 =================
    def set_channels(self, chans):
        self.channels = list(chans)
        self._auto_pending = True
        self.update()

    def autoscale(self, follow=True):
        k0, k1 = self.store.k_range(self.t_right - self.t_span, self.t_right)
        lo, hi = float("inf"), float("-inf")
        found = False
        for c in self.channels:
            if k1 <= k0:
                continue
            stride = max(1, (k1 - k0) // 2000)
            k = k0
            while k < k1:
                s = self.store.sample_at(c, k)
                if s is not None:
                    v = s * self.model.gain[c] + self.model.off[c]
                    if math.isfinite(v):
                        if v < lo:
                            lo = v
                        if v > hi:
                            hi = v
                        found = True
                k += stride
        if not found or not (math.isfinite(lo) and math.isfinite(hi)):
            return
        if hi - lo < 1e-9:
            hi += 0.5
            lo -= 0.5
        pad = (hi - lo) * 0.08
        self.y_min, self.y_max = lo - pad, hi + pad
        if follow:
            self.follow = True
        self._auto_pending = False
        self.update()

    def set_frozen(self, frozen):
        self.frozen = frozen
        if not frozen:
            self.follow = True
        self.update()

    def t_left(self):
        return self.t_right - self.t_span

    def reset_view(self):
        """返回正常视图: 恢复默认时长并跟随最新数据."""
        self.t_span = min(0.5, self.max_span)
        self.follow = True
        self._auto_pending = True
        self.autoscale()

    def set_view(self, left, span):
        span = min(max(span, 1e-6), self.max_span)
        self.t_span = span
        self.t_right = left + span
        self.follow = False
        self._auto_pending = False
        self.update()

    # ================= 几何 =================
    def _rect(self):
        w, h = self.width(), self.height()
        return LM, TM, max(LM + 10, w - RM), max(TM + 10, h - BM)

    def _nice_step(self, span, px):
        if not math.isfinite(span) or span <= 0:
            return 1.0
        rough = span / max(2.0, px / 70.0)
        if not math.isfinite(rough) or rough <= 0:
            return 1.0
        p = 10.0 ** math.floor(math.log10(max(1e-12, rough)))
        m = rough / p
        m = 1.0 if m < 1.5 else 2.0 if m < 3 else 5.0 if m < 7 else 10.0
        return m * p

    @staticmethod
    def _fmt(v, step):
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

    # ================= 绘制 =================
    @staticmethod
    def _pen(color, width=1.0):
        pen = QPen(QColor(color))
        pen.setWidthF(width)
        pen.setCosmetic(True)          # 关键: 非 cosmetic 宽笔在自交折线上极慢
        return pen

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PLOT_BG))
        x0, y0, x1, y1 = self._rect()
        pw, ph = x1 - x0, y1 - y0
        if pw < 20 or ph < 20:
            return
        if self.store.total > 0 and self._auto_pending:
            self.autoscale(follow=False)
        if self.follow and not self.frozen:
            self.t_right = self.store.t_right()
        if self.t_span > self.max_span:
            self.t_span = self.max_span
        if not (math.isfinite(self.t_span) and self.t_span > 0):
            self.t_span = 0.5
        if not math.isfinite(self.t_right):
            self.t_right = self.t_span
        tL = self.t_right - self.t_span
        ymin, ymax = self.y_min, self.y_max
        if not (math.isfinite(ymin) and math.isfinite(ymax) and ymax > ymin):
            ymin, ymax = -1.0, 1.0
            self.y_min, self.y_max = ymin, ymax
        yspan = max(1e-12, ymax - ymin)
        ysc = ph / yspan
        xsc = pw / max(1e-12, self.t_span)

        self._draw_grid(p, x0, y0, x1, y1, tL, ymin, ymax)

        k0, k1 = self.store.k_range(tL, self.t_right)
        nvis = max(0, k1 - k0)
        if nvis >= 2:
            envelope = nvis > 2 * pw
            ncol = max(1, int(pw))
            p.save()
            p.setClipRect(QRectF(x0, y0, x1 - x0, y1 - y0))
            for c in self.channels:
                g, o = self.model.gain[c], self.model.off[c]
                p.setPen(self._pen(self.model.color[c], 1.0))
                if envelope:
                    columns = self.store.envelope(c, k0, k1, ncol)
                    samples = (point for imn, vmin, imx, vmax in columns
                               for point in ((imn, vmin), (imx, vmax)))
                else:
                    base_k, vals = self.store.raw(c, k0, k1)
                    samples = enumerate(vals, start=base_k)
                _draw_trace(p, (
                    (x0 + (k / self.store.fs - tL) * xsc,
                     y1 - (value * g + o - ymin) * ysc)
                    for k, value in samples
                ))
            p.restore()

        p.setPen(self._pen(theme.PLOT_AXIS, 1.0))
        p.drawRect(QRectF(x0, y0, x1 - x0, y1 - y0))

        self._draw_cursor(p, x0, y0, x1, y1, tL, xsc, ysc, ymin)

    def _draw_grid(self, p, x0, y0, x1, y1, tL, ymin, ymax):
        pw = x1 - x0
        tstep = self._nice_step(self.t_span, pw)
        p.setPen(QPen(QColor(theme.PLOT_GRID), 1))
        font = QFont(theme.MONO, 8)
        p.setFont(font)
        if math.isfinite(tstep) and tstep > 0 and math.isfinite(tL) \
                and math.isfinite(self.t_right) and self.t_span > 0:
            t = math.ceil(tL / tstep) * tstep
            first = True
            n = 0
            while t <= self.t_right + 1e-12 and n < 200:
                gx = x0 + (t - tL) / self.t_span * pw
                p.setPen(self._pen(theme.PLOT_GRID, 1.0))
                p.drawLine(QPointF(gx, y0), QPointF(gx, y1))
                p.setPen(self._pen(theme.FG_DIM, 1.0))
                label = self._fmt(t, tstep)
                if first:
                    label = label + " s"
                    first = False
                p.drawText(QRectF(gx - 40, y0 - TM + 2, 80, TM - 2),
                           Qt.AlignHCenter | Qt.AlignVCenter, label)
                p.drawText(QRectF(gx - 40, y1 + 2, 80, BM - 2),
                           Qt.AlignHCenter | Qt.AlignVCenter, self._fmt(t, tstep))
                t += tstep
                n += 1

        ystep = self._nice_step(ymax - ymin, y1 - y0)
        if math.isfinite(ystep) and ystep > 0:
            v = math.ceil(ymin / ystep) * ystep
            n = 0
            while v <= ymax + 1e-12 and n < 200:
                gy = y1 - (v - ymin) / (ymax - ymin) * (y1 - y0)
                p.setPen(self._pen(theme.PLOT_GRID, 1.0))
                p.drawLine(QPointF(x0, gy), QPointF(x1, gy))
                p.setPen(self._pen(theme.FG_DIM, 1.0))
                p.drawText(QRectF(x1 + 6, gy - 9, RM - 8, 18),
                           Qt.AlignLeft | Qt.AlignVCenter, self._fmt(v, ystep))
                v += ystep
                n += 1

    def _draw_cursor(self, p, x0, y0, x1, y1, tL, xsc, ysc, ymin):
        if not self._cursor or self.store.total == 0:
            return
        mx, my = self._cursor
        inside = x0 <= mx <= x1 and y0 <= my <= y1
        in_yband = (mx < x0 or mx > x1) and (y0 <= my <= y1)
        in_xband = (my < y0 or my > y1) and (x0 <= mx <= x1)

        # 鼠标在 Y 轴刻度区: 画一条横线并显示该处数值
        if in_yband:
            p.setPen(self._pen(theme.ACCENT, 1.0))
            p.drawLine(QPointF(x0, my), QPointF(x1, my))
            val = ymin + (y1 - my) / max(1e-12, ysc)
            p.setFont(QFont(theme.MONO, 9))
            fm = p.fontMetrics()
            txt = self._fmt(val, 0.0001)
            tw = fm.horizontalAdvance(txt)
            r = QRectF(x1 + 3, my - fm.height() / 2.0 - 1, tw + 8, fm.height() + 2)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(255, 255, 255, 210)))
            p.drawRoundedRect(r, 3, 3)
            p.setPen(self._pen(theme.ACCENT, 1.0))
            p.drawText(QRectF(x1 + 6, my - fm.height() / 2.0, RM - 8, fm.height()),
                       Qt.AlignLeft | Qt.AlignVCenter, txt)
            return

        # 鼠标在 X 轴刻度区: 画一条竖线
        if in_xband:
            xk = min(max(mx, x0), x1)
            p.setPen(self._pen(theme.ACCENT, 1.0))
            p.drawLine(QPointF(xk, y0), QPointF(xk, y1))
            return

        if not inside:
            return

        tcur = tL + (mx - x0) / max(1e-12, xsc)
        k = int(round(tcur * self.store.fs))
        k = max(self.store.base, min(k, self.store.total - 1))
        xk = x0 + (k / self.store.fs - tL) * xsc
        p.setPen(self._pen("#e05555", 1.0))
        p.drawLine(QPointF(xk, y0), QPointF(xk, y1))

        # 读数: 时间行 + 各通道 (按波形颜色)
        entries = [("t = %.5f s" % (k / self.store.fs), theme.FG_DIM)]
        for c in self.channels:
            s = self.store.sample_at(c, k)
            if s is None:
                continue
            vv = s * self.model.gain[c] + self.model.off[c]
            p.setBrush(QBrush(QColor(self.model.color[c])))
            p.setPen(Qt.NoPen)
            yk = y1 - (vv - ymin) * ysc
            p.drawEllipse(QPointF(xk, yk), 2.6, 2.6)
            entries.append(("%-8s %s" % (self.model.short_name(c), self._fmt(vv, 0.0001)),
                            self.model.color[c]))

        font = QFont(theme.MONO, 9)
        p.setFont(font)
        fm = p.fontMetrics()
        w = max(fm.horizontalAdvance(s) for s, _ in entries) + 16
        h = fm.height() * len(entries) + 12
        bx = min(xk + 14, x1 - w - 2)
        by = min(max(my - h // 2, y0 + 2), y1 - h - 2)
        # 透明背景 (不遮挡波形), 只留淡边框
        p.setPen(self._pen(theme.BORDER, 1.0))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(QRectF(bx, by, w, h), 5, 5)
        ty = by + 6
        for s, col in entries:
            p.setPen(QPen(QColor(col)))
            p.drawText(QPointF(bx + 8, ty + fm.ascent()), s)
            ty += fm.height()

        # 暂停跟随时在右上角提示 (否则用户可能不知道波形已经不动了)
        if (not self.follow) and (not self.frozen):
            p.setFont(QFont(theme.MONO, 9))
            tip = "已暂停跟随 · 双击恢复"
            tw = p.fontMetrics().horizontalAdvance(tip) + 12
            p.setPen(self._pen(theme.ACCENT, 1.0))
            p.setBrush(Qt.NoBrush)
            p.drawRoundedRect(QRectF(x1 - tw - 4, y0 + 4, tw, 18), 5, 5)
            p.drawText(QPointF(x1 - tw + 2, y0 + 17), tip)

    # ================= 交互 =================
    def mouseMoveEvent(self, ev):
        self._cursor = (ev.position().x(), ev.position().y())
        if self._drag:
            x0, y0, x1, y1 = self._rect()
            pw = max(1, x1 - x0)
            ph = max(1, y1 - y0)
            dx = ev.position().x() - self._drag[0]
            dy = ev.position().y() - self._drag[1]
            # 水平拖动超过 3px 就退出"跟随最新", 进入平移.
            # (原来跟随状态下左键拖动只能改 Y 缩放, 所以 Ia/Ib/Ic 那个
            #  窗口看着像"不能左右拖动"; 双击恢复跟随)
            if self.follow and not self.frozen and abs(dx) > 3.0:
                self.follow = False
            if self.frozen or not self.follow:
                tr = self._drag[2] - dx / pw * self.t_span
                # 不要拖到没有数据的区域
                lo = self.store.t_left() + self.t_span
                hi = max(lo, self.store.t_right())
                self.t_right = min(max(tr, lo), hi)
            span = self._drag[4] - self._drag[3]
            self.y_min = self._drag[3] + dy / ph * span
            self.y_max = self._drag[4] + dy / ph * span
        self.update()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._drag = (ev.position().x(), ev.position().y(), self.t_right,
                          self.y_min, self.y_max)
            self.setCursor(Qt.ClosedHandCursor)

    def mouseReleaseEvent(self, ev):
        self._drag = None
        self.unsetCursor()

    def mouseDoubleClickEvent(self, ev):
        self.follow = True
        self.autoscale()

    def leaveEvent(self, ev):
        self._cursor = None
        self.update()

    def _axis_zone(self, x, y):
        """返回鼠标所在区域: 'plot' / 'x'(上下时间轴) / 'y'(左右数值轴)."""
        x0, y0, x1, y1 = self._rect()
        if x0 <= x <= x1 and y0 <= y <= y1:
            return "plot"
        if y < y0 or y > y1:
            return "x"
        if x < x0 or x > x1:
            return "y"
        return "plot"

    def wheelEvent(self, ev):
        delta = ev.angleDelta().y()
        if delta == 0:
            return
        x0, y0, x1, y1 = self._rect()
        factor = 1.1 ** (-delta / 120.0)
        mods = ev.modifiers()
        zone = self._axis_zone(ev.position().x(), ev.position().y())
        force_y = bool(mods & Qt.ControlModifier) or bool(mods & Qt.ShiftModifier)

        if force_y or (zone == "y"):
            # 缩放 Y 轴 (以光标处数值为中心)
            my = min(max(ev.position().y(), y0), y1)
            frac = (y1 - my) / max(1, y1 - y0)
            vcur = self.y_min + frac * (self.y_max - self.y_min)
            span = min(max((self.y_max - self.y_min) * factor, 1e-9), 1e9)
            self.y_min = vcur - frac * span
            self.y_max = self.y_min + span
        elif self.follow:
            # 跟随状态下只改变显示时长, 波形继续实时滚动 (不暂停)
            self.t_span = min(max(self.t_span * factor, 1e-6), self.max_span)
        else:
            # 已暂停/平移时, 以光标处时刻为中心缩放
            self._zoom_time_centered(ev.position().x(), x0, x1, factor)
        self.update()

    def _zoom_time_centered(self, mx, x0, x1, factor):
        mx = min(max(mx, x0), x1)
        frac = (mx - x0) / max(1, x1 - x0)
        tcur = (self.t_right - self.t_span) + frac * self.t_span
        span = min(max(self.t_span * factor, 1e-6), self.max_span)
        self.t_right = tcur + (1 - frac) * span
        self.t_span = span
        self.follow = False
        self._auto_pending = False


class OverviewWidget(QWidget):
    """总览: 显示整段缓存的缩略波形 + 可拖动/缩放的视窗 (联动下方明细图)."""

    viewChanged = Signal(float, float)      # (left, span)
    resetRequested = Signal()               # 双击: 返回正常视图

    OLM, ORM, OTM, OBM = 12, 60, 4, 4
    EDGE_PX = 6

    def __init__(self, store, model, parent=None):
        super().__init__(parent)
        self.store = store
        self.model = model
        self.channels = []
        self.win_left = 0.0
        self.win_span = 0.5
        self._cache = None
        self._cache_t = 0.0
        self._last_update = 0.0
        self._drag = None
        self.setMinimumHeight(54)
        self.setMaximumHeight(120)
        self.setMouseTracking(True)
        self.setAutoFillBackground(False)
        self.setToolTip("拖动/滚轮=切换查看位置 · 双击=返回正常波形")

    def set_channels(self, chans):
        self.channels = list(chans)
        self._cache = None
        self.update()

    def set_window(self, left, span, force=False):
        self.win_left = left
        self.win_span = span
        import time as _t
        now = _t.time()
        if force or now - self._last_update >= 0.1:
            self._last_update = now
            self.update()

    # ---------- 内部 ----------
    def _rect(self):
        w, h = self.width(), self.height()
        return self.OLM, self.OTM, max(self.OLM + 10, w - self.ORM), max(self.OTM + 10, h - self.OBM)

    def _full(self):
        """返回 (左端时刻, 总时长). 自动贴合当前缓存到的数据长度."""
        tA = self.store.base / self.store.fs
        full = (self.store.total - self.store.base) / self.store.fs
        if full < 1e-6:
            return 0.0, max(self.store.max_span, 1.0)
        return tA, full

    def _compute(self):
        import time as _t
        now = _t.time()
        chans = tuple(self.channels)
        if self._cache and self._cache[0] == chans and now - self._cache_t < 0.25:
            return self._cache[1]
        x0, y0, x1, y1 = self._rect()
        ncol = max(1, int((x1 - x0) * 0.5))
        tA, full = self._full()
        envs = []
        lo, hi = float("inf"), float("-inf")
        for c in self.channels:
            g, o = self.model.gain[c], self.model.off[c]
            env = self.store.envelope(c, self.store.base, self.store.total, ncol)
            envs.append((self.model.color[c], g, o, env))
            for _imn, vmin, _imx, vmax in env:
                a, b = vmin * g + o, vmax * g + o
                if a < lo:
                    lo = a
                if b > hi:
                    hi = b
        if (lo == float("inf")) or not (math.isfinite(lo) and math.isfinite(hi)):
            lo, hi = -1.0, 1.0
        if hi - lo < 1e-9:
            hi += 1.0
            lo -= 1.0
        data = (envs, lo, hi, tA, full)
        self._cache = (chans, data)
        self._cache_t = now
        return data

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PLOT_BG))
        x0, y0, x1, y1 = self._rect()
        pw, ph = x1 - x0, y1 - y0
        if pw < 10 or ph < 10:
            return
        envs, lo, hi, tA, full = self._compute()
        ysc = ph / max(1e-12, hi - lo)
        xsc = pw / max(1e-12, full)

        def X(k):
            return x0 + (k / self.store.fs - tA) * xsc

        p.save()
        p.setClipRect(QRectF(x0, y0, x1 - x0, y1 - y0))
        for color, g, o, env in envs:
            p.setPen(PlotWidget._pen(color, 1.0))
            _draw_trace(p, (
                (X(k), y1 - (value * g + o - lo) * ysc)
                for imn, vmin, imx, vmax in env
                for k, value in ((imn, vmin), (imx, vmax))
            ))
        p.restore()

        p.setPen(PlotWidget._pen(theme.PLOT_AXIS, 1.0))
        p.drawRect(QRectF(x0, y0, x1 - x0, y1 - y0))

        # 视窗 (win_left/win_span 是秒, 不能走 X() 的样本号映射)
        vx0 = x0 + (self.win_left - tA) * xsc
        vx1 = x0 + (self.win_left + self.win_span - tA) * xsc
        vx0 = max(x0, min(vx0, x1))
        vx1 = max(x0, min(vx1, x1))
        if vx1 - vx0 < 2:
            vx1 = min(x1, vx0 + 2)
        p.setPen(PlotWidget._pen(theme.ACCENT, 1.0))
        p.setBrush(QBrush(QColor(26, 115, 232, 45)))
        p.drawRect(QRectF(vx0, y0, vx1 - vx0, y1 - y0))

        # 当前观看位置的蓝色竖线 (视窗中心)
        vxm = (vx0 + vx1) / 2.0
        p.setPen(PlotWidget._pen(theme.ACCENT, 2.0))
        p.drawLine(QPointF(vxm, y0), QPointF(vxm, y1))

        p.setPen(QPen(QColor(theme.FG_FAINT)))
        p.setFont(QFont(theme.MONO, 8))
        p.drawText(QRectF(x0 + 4, y0 + 1, 60, 14), Qt.AlignLeft | Qt.AlignVCenter, "总览")

    # ---------- 交互 ----------
    def _mode_at(self, x):
        x0, y0, x1, y1 = self._rect()
        tA, full = self._full()
        if full <= 0:
            return "O"
        pw = max(1, x1 - x0)
        vx0 = x0 + (self.win_left - tA) / full * pw
        vx1 = x0 + (self.win_left + self.win_span - tA) / full * pw
        if abs(x - vx0) <= self.EDGE_PX:
            return "L"
        if abs(x - vx1) <= self.EDGE_PX:
            return "R"
        if vx0 < x < vx1:
            return "M"
        return "O"

    def _time_at(self, x):
        x0, y0, x1, y1 = self._rect()
        pw = max(1, x1 - x0)
        tA, full = self._full()
        return tA + max(0.0, min(1.0, (x - x0) / pw)) * full

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            mode = self._mode_at(ev.position().x())
            tA, full = self._full()
            x0, y0, x1, y1 = self._rect()
            self._drag = (mode, ev.position().x(), self.win_left, self.win_span,
                          tA, full, max(1, x1 - x0))
            if mode == "O":
                self._drag = (mode, ev.position().x(), self.win_left, self.win_span,
                              tA, full, max(1, x1 - x0))
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, ev):
        if not self._drag:
            m = self._mode_at(ev.position().x())
            self.setCursor(Qt.SizeHorCursor if m in ("L", "R") else
                           (Qt.OpenHandCursor if m == "M" else Qt.ArrowCursor))
            return
        mode, sx, sl, ss, tA, full, pw = self._drag
        dx = ev.position().x() - sx
        dt = dx / pw * full
        if mode == "M":
            self.viewChanged.emit(sl + dt, ss)
        elif mode == "L":
            new_span = max(1e-4, ss - dt)
            self.viewChanged.emit(sl + (ss - new_span), new_span)
        elif mode == "R":
            self.viewChanged.emit(sl, max(1e-4, ss + dt))
        elif mode == "O":
            t = self._time_at(ev.position().x())
            self.viewChanged.emit(t - ss / 2.0, ss)

    def mouseReleaseEvent(self, ev):
        self._drag = None
        self.setCursor(Qt.ArrowCursor)

    def mouseDoubleClickEvent(self, ev):
        self.resetRequested.emit()

    def wheelEvent(self, ev):
        delta = ev.angleDelta().y()
        if delta == 0:
            return
        factor = 1.1 ** (-delta / 120.0)
        tcur = self._time_at(ev.position().x())
        x0, y0, x1, y1 = self._rect()
        frac = max(0.0, min(1.0, (ev.position().x() - x0) / max(1, x1 - x0)))
        new_span = max(1e-4, self.win_span * factor)
        new_left = tcur - frac * new_span
        self.viewChanged.emit(new_left, new_span)



