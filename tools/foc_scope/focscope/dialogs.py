# -*- coding: utf-8 -*-
"""分析窗口: 频谱 (FFT) 与 矢量图 (dq / αβ)."""
import math

from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPolygonF, QBrush
from PySide6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
)

from . import theme
from . import analysis
from .channels import index_of, NCH

LM, RM, TM, BM = 56, 16, 26, 30


def _recent(store, c, maxn=32768):
    b = store.bufs[c]
    if not b:
        return None
    if len(b) > maxn:
        b = b[-maxn:]
    return analysis.np.asarray(b, dtype=float)


class _Canvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 320)

    def _rect(self):
        w, h = self.width(), self.height()
        return LM, TM, max(LM + 10, w - RM), max(TM + 10, h - BM)


class SpectrumCanvas(_Canvas):
    def __init__(self, store, model, parent=None):
        super().__init__(parent)
        self.store = store
        self.model = model
        self.channels = []
        self.freq_max = 0.0
        self.db_min, self.db_max = -120.0, 20.0

    def compute(self):
        self.channels = self.model.shown_channels()
        if not self.channels:
            self.channels = [i for i in range(NCH) if self.store.bufs[i]][:1]
        return self.channels

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PLOT_BG))
        x0, y0, x1, y1 = self._rect()
        pw, ph = x1 - x0, y1 - y0
        if pw < 20 or ph < 20:
            return

        chans = self.channels
        curves = []
        fmax = 0.0
        dmin, dmax = 10.0, -200.0
        for c in chans:
            x = _recent(self.store, c)
            if x is None or len(x) < 16:
                continue
            g, o = self.model.gain[c], self.model.off[c]
            x = x * g + o
            fr, amp = analysis.fft_magnitude(x, self.store.fs)
            if fr is None:
                continue
            db = analysis.to_db(amp)
            curves.append((c, fr, db))
            fmax = max(fmax, fr[-1])
            dmin = min(dmin, float(db[1:].min()) if len(db) > 1 else -120.0)
            dmax = max(dmax, float(db.max()))
        fmax = fmax or (self.store.fs / 2.0)
        dmax = min(dmax, 60.0)
        dmin = max(dmin, dmax - 120.0)
        self.freq_max, self.db_min, self.db_max = fmax, dmin, dmax

        # grid
        p.setPen(QPen(QColor(theme.PLOT_GRID), 1))
        p.setFont(QFont(theme.MONO, 8))
        for i in range(1, 5):
            gx = x0 + i / 4.0 * pw
            p.drawLine(QPointF(gx, y0), QPointF(gx, y1))
        for i in range(1, 4):
            gy = y0 + i / 4.0 * ph
            p.drawLine(QPointF(x0, gy), QPointF(x1, gy))
        p.setPen(QPen(QColor(theme.PLOT_AXIS), 1))
        p.drawRect(QRectF(x0, y0, pw, ph))

        p.setPen(QPen(QColor(theme.FG_DIM), 1))
        p.drawText(QRectF(x0, y1 + 2, 80, BM - 2), Qt.AlignLeft | Qt.AlignVCenter, "0")
        p.drawText(QRectF(x1 - 80, y1 + 2, 80, BM - 2), Qt.AlignRight | Qt.AlignVCenter,
                   "%.0f Hz" % fmax)
        p.drawText(QRectF(x0 - LM, y0 - TM, LM, TM), Qt.AlignLeft | Qt.AlignVCenter,
                   "dB")
        p.drawText(QRectF(x0 + 6, y0 + 2, 240, 16), Qt.AlignLeft, "频谱 (单边, Hann)")

        dspan = max(1e-6, dmax - dmin)
        for c, fr, db in curves:
            p.setPen(QPen(QColor(self.model.color[c]), 1.0))
            poly = QPolygonF()
            for k in range(len(fr)):
                xx = x0 + fr[k] / fmax * pw
                yy = y1 - (float(db[k]) - dmin) / dspan * ph
                if math.isfinite(yy):
                    poly.append(QPointF(xx, yy))
            if poly.size() >= 2:
                p.drawPolyline(poly)

        # legend
        ty = y0 + 20
        for c, _, _ in curves:
            p.setPen(QPen(QColor(self.model.color[c])))
            p.drawText(QRectF(x1 - 140, ty, 130, 14), Qt.AlignRight, self.model.short_name(c))
            ty += 13


class SpectrumDialog(QDialog):
    def __init__(self, store, model, parent=None):
        super().__init__(parent)
        self.setWindowTitle("频谱")
        self.resize(760, 440)
        v = QVBoxLayout(self)
        self.canvas = SpectrumCanvas(store, model, self)
        v.addWidget(self.canvas, 1)
        row = QHBoxLayout()
        self.lbl = QLabel("对右侧“选择性波形图”勾选的通道做 FFT")
        self.lbl.setObjectName("Dim")
        row.addWidget(self.lbl)
        row.addStretch(1)
        b = QPushButton("刷新")
        b.clicked.connect(self._refresh)
        row.addWidget(b)
        v.addLayout(row)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(800)
        self._refresh()

    def _refresh(self):
        self.canvas.compute()
        self.canvas.update()


class VectorCanvas(_Canvas):
    def __init__(self, store, model, parent=None):
        super().__init__(parent)
        self.store = store
        self.model = model
        self.mode = "dq"   # dq | ab
        self.span = 1.0
        self.i_id = index_of("Id")
        self.i_iq = index_of("Iq")
        self.i_ia = index_of("Ia")
        self.i_ib = index_of("Ib")
        self.i_ic = index_of("Ic")
        self.i_ph = index_of("phase")

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(theme.PLOT_BG))
        x0, y0, x1, y1 = self._rect()
        pw, ph = x1 - x0, y1 - y0
        if pw < 20 or ph < 20:
            return

        xs = ys = None
        xlabel = ylabel = ""
        if self.mode == "dq" and self.i_id is not None and self.i_iq is not None:
            xs = _recent(self.store, self.i_id)
            ys = _recent(self.store, self.i_iq)
            xlabel, ylabel = "Id (A)", "Iq (A)"
            if xs is not None:
                xs = xs * self.model.gain[self.i_id] + self.model.off[self.i_id]
            if ys is not None:
                ys = ys * self.model.gain[self.i_iq] + self.model.off[self.i_iq]
        elif self.mode == "ab" and None not in (self.i_ia, self.i_ib, self.i_ic):
            ia = _recent(self.store, self.i_ia)
            ib = _recent(self.store, self.i_ib)
            ic = _recent(self.store, self.i_ic)
            if ia is not None and ib is not None and ic is not None:
                n = min(len(ia), len(ib), len(ic))
                xs, ys = analysis.clarke(ia[-n:], ib[-n:], ic[-n:])
            xlabel, ylabel = "α (A)", "β (A)"

        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        span = self.span
        if xs is not None and len(xs) > 1:
            m = max(abs(float(xs.min())), abs(float(xs.max())),
                    abs(float(ys.min())), abs(float(ys.max())), 1e-3)
            span = m * 1.1
            self.span = span
        sc = min(pw, ph) / (2.0 * span)

        p.setPen(QPen(QColor(theme.PLOT_GRID), 1))
        for f in (0.5, 1.0):
            r = f * span * sc
            p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QPen(QColor(theme.PLOT_AXIS), 1))
        p.drawLine(QPointF(x0, cy), QPointF(x1, cy))
        p.drawLine(QPointF(cx, y0), QPointF(cx, y1))
        p.setPen(QPen(QColor(theme.FG_DIM), 1))
        p.setFont(QFont(theme.MONO, 8))
        p.drawText(QRectF(x1 - 120, y1 + 2, 120, BM - 2), Qt.AlignRight, xlabel)
        p.drawText(QRectF(x0, y0 - TM, 200, TM), Qt.AlignLeft, ylabel + "  (X-Y)")

        if xs is not None and len(xs) > 1:
            p.setPen(QPen(QColor(theme.ACCENT), 1.0))
            poly = QPolygonF()
            n = len(xs)
            step = max(1, n // 4000)
            k = 0
            while k < n:
                px = cx + float(xs[k]) * sc
                py = cy - float(ys[k]) * sc
                if math.isfinite(px) and math.isfinite(py):
                    poly.append(QPointF(px, py))
                k += step
            if poly.size() >= 2:
                p.drawPolyline(poly)
            lx, ly = float(xs[-1]) * sc, float(ys[-1]) * sc
            if abs(lx) < pw and abs(ly) < ph:
                p.setBrush(QBrush(QColor(theme.RED)))
                p.setPen(Qt.NoPen)
                p.drawEllipse(QPointF(cx + lx, cy - ly), 3.5, 3.5)

        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(x0, y0, pw, ph))


class VectorDialog(QDialog):
    def __init__(self, store, model, parent=None):
        super().__init__(parent)
        self.setWindowTitle("矢量图")
        self.resize(560, 520)
        v = QVBoxLayout(self)
        self.canvas = VectorCanvas(store, model, self)
        v.addWidget(self.canvas, 1)
        row = QHBoxLayout()
        row.addWidget(QLabel("显示"))
        self.cmb = QComboBox()
        self.cmb.addItems(["d-q 电流", "α-β 电流"])
        self.cmb.currentIndexChanged.connect(self._on_mode)
        row.addWidget(self.cmb)
        row.addStretch(1)
        v.addLayout(row)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.canvas.update)
        self._timer.start(200)

    def _on_mode(self, idx):
        self.canvas.mode = "ab" if idx == 1 else "dq"
        self.canvas.span = 1.0
        self.canvas.update()
