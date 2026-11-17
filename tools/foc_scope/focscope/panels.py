# -*- coding: utf-8 -*-
"""界面组件: 标题栏 / 左侧控制面板 / 右侧数据面板 / 底部状态栏 / 通道设置对话框."""
from PySide6.QtCore import Qt, Signal, QRectF, QPointF
from PySide6.QtGui import QColor, QPainter, QPixmap, QPen, QBrush, QFont, QIcon
from PySide6.QtWidgets import (
    QWidget, QFrame, QLabel, QPushButton, QToolButton, QComboBox, QLineEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, QScrollArea, QDialog,
    QTableWidget, QTableWidgetItem, QHeaderView, QCheckBox, QAbstractItemView,
    QSizePolicy, QDoubleSpinBox, QStyle, QToolTip,
)

from . import theme
from .channels import CHANNELS, NCH, STATUS_ITEMS, fault_texts
from .vofa_source import source_info
from .fwconfig import load as _load_fw

_FW = _load_fw()


def num(v, dec=3):
    if v is None:
        return "-"
    if v == 0:
        return "0"
    a = abs(v)
    if a >= 100000 or a < 0.001:
        return "%.3g" % v
    return ("%%.%df" % dec) % v


def dot(color, size=10):
    lb = QLabel()
    lb.setFixedSize(size, size)
    lb.setStyleSheet("background:%s;border-radius:%dpx;" % (color, size // 2))
    return lb


def card(title):
    """返回 (卡片 Frame, 内容垂直布局)."""
    f = QFrame()
    f.setObjectName("Card")
    v = QVBoxLayout(f)
    v.setContentsMargins(10, 8, 10, 10)
    v.setSpacing(6)
    if title:
        t = QLabel(title)
        t.setObjectName("CardTitle")
        v.addWidget(t)
    return f, v


class TitleBar(QFrame):
    request_settings = Signal()
    request_clear = Signal()
    request_spectrum = Signal()
    request_vector = Signal()
    request_replay = Signal()
    request_min = Signal()
    request_max = Signal()
    request_close = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(40)
        self.setStyleSheet("QFrame{background:%s;border:none;}" % theme.TITLEBAR)
        self._drag = None
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 0, 6, 0)
        lay.setSpacing(4)

        icon = QLabel()
        icon.setPixmap(self._make_icon())
        lay.addWidget(icon)

        title = QLabel("FOC 上位机")
        title.setStyleSheet("color:%s;font-size:13px;font-weight:600;background:transparent;" % theme.FG)
        lay.addWidget(title)
        lay.addSpacing(8)

        for text, tip, sig in (
            ("⚙", "通道设置", self.request_settings),
            ("⌫", "清空数据", self.request_clear),
            ("频谱", "FFT 频谱", self.request_spectrum),
            ("矢量", "矢量图 (dq/αβ)", self.request_vector),
            ("回放", "打开波形记录 CSV", self.request_replay),
        ):
            b = QToolButton()
            b.setText(text)
            b.setToolTip(tip)
            b.setFixedSize(28 if len(text) == 1 else 40, 26)
            b.setStyleSheet("color:%s;background:transparent;border:none;border-radius:6px;"
                            "QToolButton:hover{background:#eceff4;}" % theme.FG_DIM)
            if sig:
                b.clicked.connect(sig.emit)
            lay.addWidget(b)

        lay.addStretch(1)

        self._conn = QLabel("未连接")
        self._conn.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        lay.addWidget(self._conn)
        lay.addSpacing(6)

        for text, sig, name in (("–", self.request_min, "min"),
                                ("▢", self.request_max, "max"),
                                ("✕", self.request_close, "close")):
            b = QToolButton()
            b.setText(text)
            b.setFixedSize(34, 26)
            hover = "#e81123" if name == "close" else "#eceff4"
            fg = "#ffffff" if name == "close" else theme.FG_DIM
            b.setStyleSheet("color:%s;background:transparent;border:none;border-radius:6px;"
                            "QToolButton:hover{background:%s;color:%s;}" %
                            (fg, hover, "#ffffff" if name == "close" else theme.FG))
            b.clicked.connect(sig.emit)
            lay.addWidget(b)

    @staticmethod
    def _make_icon():
        pm = QPixmap(22, 22)
        pm.fill(Qt.transparent)
        p = QPainter(pm)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(theme.ACCENT)))
        p.drawRoundedRect(QRectF(1, 1, 20, 20), 5, 5)
        p.setPen(QPen(QColor("#ffffff")))
        f = QFont("Microsoft YaHei UI", 9)
        f.setBold(True)
        p.setFont(f)
        p.drawText(QRectF(0, 0, 22, 22), Qt.AlignCenter, "F")
        p.end()
        return pm

    def set_connected(self, on, text):
        self._conn.setText(text)
        self._conn.setStyleSheet("color:%s;background:transparent;" % (theme.GREEN if on else theme.FG_DIM))

    # 拖动 / 双击最大化
    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._drag = ev.globalPosition().toPoint() - self.window().frameGeometry().topLeft()

    def mouseMoveEvent(self, ev):
        if self._drag and (ev.buttons() & Qt.LeftButton):
            w = self.window()
            if getattr(w, "_is_max", False):
                w.restore_from_max()
                self._drag = ev.globalPosition().toPoint() - w.frameGeometry().topLeft()
            w.move(ev.globalPosition().toPoint() - self._drag)

    def mouseReleaseEvent(self, ev):
        self._drag = None

    def mouseDoubleClickEvent(self, ev):
        self.request_max.emit()


class _DblLabel(QLabel):
    """双击可触发回调的标签."""
    doubleClicked = Signal()

    def mouseDoubleClickEvent(self, ev):
        self.doubleClicked.emit()


class TargetSlider(QSlider):
    """目标值滑条: 悬停显示光标处数值; 点击/拖动直接跳到该位置."""

    jumped = Signal(int)   # 鼠标释放时的最终值 (int, 需 ×scale)

    def __init__(self, parent=None):
        super().__init__(Qt.Horizontal, parent)
        self.scale = 1.0
        self.decimals = 3
        self._dragging = False
        self.setMouseTracking(True)

    def set_scale(self, scale, decimals=3):
        self.scale = float(scale) if scale else 1.0
        self.decimals = decimals

    def _value_at(self, x):
        hw = self.style().pixelMetric(QStyle.PM_SliderLength, None, self)
        span = max(1, self.width() - hw)
        pos = int(x - hw / 2)
        return self.style().sliderValueFromPosition(self.minimum(), self.maximum(), pos, span)

    def _fmt(self, iv):
        return num(iv * self.scale, self.decimals)

    def paintEvent(self, ev):
        super().paintEvent(ev)
        # 0 处的小圆点标记: 内白外蓝圈, 位于进度条下方
        if self.minimum() <= 0 <= self.maximum():
            hw = self.style().pixelMetric(QStyle.PM_SliderLength, None, self)
            span = max(1, self.width() - hw)
            x = hw / 2.0 + self.style().sliderPositionFromValue(
                self.minimum(), self.maximum(), 0, span)
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing, True)
            p.setPen(QPen(QColor(theme.ACCENT), 1.6))
            p.setBrush(QBrush(QColor("#ffffff")))
            r = 3.5
            p.drawEllipse(QPointF(x, self.height() - 6.0), r, r)
            p.end()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._dragging = True
            self.setValue(self._value_at(ev.position().x()))
            ev.accept()
            return
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._dragging:
            self.setValue(self._value_at(ev.position().x()))
            ev.accept()
            return
        v = self._value_at(ev.position().x())
        QToolTip.showText(ev.globalPosition().toPoint(), self._fmt(v), self)
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self.jumped.emit(self.value())
            ev.accept()
            return
        super().mouseReleaseEvent(ev)


class Stepper(QWidget):
    changed = Signal(float)
    sent = Signal(float)
    configChanged = Signal()

    def __init__(self, label, value, lo, hi, step, decimals=3, unit="", parent=None):
        super().__init__(parent)
        self.lo, self.hi, self.step, self.dec = lo, hi, step, decimals
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        self.lbl = _DblLabel(label)
        self.lbl.setObjectName("Dim")
        self.lbl.setFixedWidth(78)
        self.lbl.setCursor(Qt.PointingHandCursor)
        self.lbl.setToolTip("双击修改 最小值/最大值/步长")
        self.lbl.doubleClicked.connect(self.open_range_dialog)
        lay.addWidget(self.lbl)

        self.minus = QPushButton("−")
        self.minus.setObjectName("Flat")
        self.minus.setFixedWidth(20)
        lay.addWidget(self.minus)

        self.edit = QLineEdit()
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText(num(value, decimals))
        self.edit.setFixedHeight(24)
        lay.addWidget(self.edit, 1)

        self.plus = QPushButton("+")
        self.plus.setObjectName("Flat")
        self.plus.setFixedWidth(20)
        lay.addWidget(self.plus)

        self.send = QToolButton()
        self.send.setText("➤")
        self.send.setToolTip("发送")
        self.send.setFixedSize(24, 24)
        self.send.setStyleSheet("color:%s;background:transparent;border:none;"
                                "QToolButton:hover{background:#eceff4;border-radius:6px;}" % theme.ACCENT)
        lay.addWidget(self.send)

        self.minus.clicked.connect(lambda: self._bump(-1))
        self.plus.clicked.connect(lambda: self._bump(1))
        self.send.clicked.connect(self._emit_sent)
        self.edit.editingFinished.connect(self._emit_sent)   # 回车或失焦自动下发

    def _bump(self, sign):
        try:
            v = float(self.edit.text() or 0)
        except ValueError:
            v = 0.0
        v = min(self.hi, max(self.lo, v + sign * self.step))
        self.edit.setText(num(v, self.dec))
        self.changed.emit(v)

    def _emit_sent(self):
        try:
            v = float(self.edit.text() or 0)
        except ValueError:
            return
        self.sent.emit(v)

    def value(self):
        try:
            return float(self.edit.text() or 0)
        except ValueError:
            return 0.0

    def set_range(self, mn, mx, st):
        self.lo, self.hi, self.step = float(mn), float(mx), float(st)
        try:
            v = min(self.hi, max(self.lo, float(self.edit.text() or 0)))
            self.edit.setText(num(v, self.dec))
        except ValueError:
            pass

    def set_value(self, v):
        """回填设备值; 若超出现有量程则自动扩大量程, 不夹死显示."""
        try:
            v = float(v)
        except (TypeError, ValueError):
            return
        if v > self.hi:
            self.hi = v
        if v < self.lo:
            self.lo = v
        self.edit.setText(num(v, self.dec))

    def open_range_dialog(self):
        win = QDialog(self.window())
        win.setWindowTitle("配置 " + self.lbl.text())
        win.setMinimumWidth(270)
        g = QGridLayout(win)
        g.setContentsMargins(14, 14, 14, 14)
        g.setSpacing(8)
        entries = {}
        for r, (text, val) in enumerate((("最小值", self.lo), ("最大值", self.hi), ("步长", self.step))):
            g.addWidget(QLabel(text), r, 0)
            e = QLineEdit(num(val))
            e.setAlignment(Qt.AlignCenter)
            g.addWidget(e, r, 1)
            entries[text] = e
        err = QLabel("")
        err.setStyleSheet("color:%s;background:transparent;" % theme.RED)
        g.addWidget(err, 3, 0, 1, 2)

        def ok():
            try:
                mn = float(entries["最小值"].text())
                mx = float(entries["最大值"].text())
                st = float(entries["步长"].text())
            except ValueError:
                err.setText("请输入数字")
                return
            if mx <= mn or st <= 0:
                err.setText("需满足 最大>最小 且 步长>0")
                return
            self.set_range(mn, mx, st)
            self.configChanged.emit()
            win.accept()

        brow = QHBoxLayout()
        brow.addStretch(1)
        okb = QPushButton("确定")
        okb.setObjectName("Accent")
        okb.clicked.connect(ok)
        cb = QPushButton("取消")
        cb.clicked.connect(win.reject)
        brow.addWidget(cb)
        brow.addWidget(okb)
        g.addLayout(brow, 4, 0, 1, 2)
        entries["最小值"].setFocus()
        entries["最小值"].selectAll()
        win.exec()


class LeftPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(304)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(8)

        # ---- 连接设备 ----
        cf, cv = card("连接设备")
        row = QHBoxLayout()
        row.setSpacing(6)
        row.addWidget(QLabel("设备"))
        self.cmb_port = QComboBox()
        self.cmb_port.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.cmb_port.currentIndexChanged.connect(self._update_port_preview)
        row.addWidget(self.cmb_port, 1)
        cv.addLayout(row)
        self.lbl_port = QLabel("—")
        self.lbl_port.setWordWrap(True)
        self.lbl_port.setStyleSheet("color:%s;background:transparent;" % theme.FG_FAINT)
        cv.addWidget(self.lbl_port)
        self._port_infos = {}
        row = QHBoxLayout()
        row.setSpacing(6)
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setObjectName("Accent")
        self.btn_disconnect = QPushButton("断开连接")
        self.btn_disconnect.setEnabled(False)
        row.addWidget(self.btn_connect)
        row.addWidget(self.btn_disconnect)
        cv.addLayout(row)
        lay.addWidget(cf)

        # ---- 驱动器状态 ----
        cf, cv = card("驱动器状态")
        self.status_widgets = {}
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(6)
        for idx, (key, name, kind) in enumerate(STATUS_ITEMS):
            r, c = divmod(idx, 2)
            cell = QWidget()
            h = QHBoxLayout(cell)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(6)
            d = dot(theme.FG_FAINT)
            h.addWidget(d)
            lb = QLabel(name)
            lb.setObjectName("Dim")
            h.addWidget(lb)
            h.addStretch(1)
            grid.addWidget(cell, r, c)
            self.status_widgets[key] = d
        cv.addLayout(grid)
        lay.addWidget(cf)

        # ---- 驱动器参数 ----
        cf, cv = card("驱动器参数")
        self.steppers = {}
        params = [
            ("curkp", "电流环Kp", _FW.get_float("current_kp", 0.0), 0.0, 100000.0, 0.001, 3),
            ("curki", "电流环Ki", _FW.get_float("current_ki", 0.0), 0.0, 10000000.0, 0.001, 3),
            ("curbw", "电流带宽Hz", _FW.get_float("current_ctrl_bw", 1000.0), 0.0, 5000.0, 0.001, 3),
            ("velkp", "速度环Kp", _FW.get_float("vel_p_gain", 0.10), 0.0, 100.0, 0.001, 3),
            ("velki", "速度环Ki", _FW.get_float("vel_i_gain", 0.01), 0.0, 100.0, 0.001, 3),
            ("poskp", "位置环Kp", _FW.get_float("pos_p_gain", 30.0), 0.0, 1000.0, 0.001, 3),
        ]
        for key, label, default, lo, hi, step, dec in params:
            s = Stepper(label, default, lo, hi, step, dec)
            self.steppers[key] = s
            cv.addWidget(s)
        self.btn_read = QPushButton("从设备读取参数")
        self.btn_read.setToolTip("发送读取命令, 用设备当前参数回填各输入框")
        cv.addWidget(self.btn_read)
        lay.addWidget(cf)

        # ---- 保护阈值 ----
        cf, cv = card("保护阈值")
        for key, label, default, lo, hi, step, dec in (
            ("uv", "欠压 V", _FW.get_float("protect_under_voltage", 9.0), 0.0, 60.0, 0.001, 3),
            ("ov", "过压 V", _FW.get_float("protect_over_voltage", 15.0), 0.0, 60.0, 0.001, 3),
        ):
            s = Stepper(label, default, lo, hi, step, dec)
            self.steppers[key] = s
            cv.addWidget(s)
        self.btn_save = QPushButton("保存到 Flash (3F)")
        cv.addWidget(self.btn_save)
        self.btn_reset = QPushButton("恢复默认参数 (3B)")
        self.btn_reset.setObjectName("Danger")
        self.btn_reset.setToolTip("恢复固件出厂默认参数, 会清掉校准 (需重新校准)")
        cv.addWidget(self.btn_reset)
        lay.addWidget(cf)

        lay.addStretch(1)

        hint = QLabel("滚轮=时间缩放 · Ctrl/Shift+滚轮=幅值\n拖动=平移 · 双击绘图区=自动量程\n总览: 拖动=定位 · 双击=返回正常波形")
        hint.setObjectName("Dim")
        hint.setStyleSheet("color:%s;background:transparent;" % theme.FG_FAINT)
        lay.addWidget(hint)

    def populate_ports(self, infos):
        """填充串口下拉 (USB 优先, 蓝牙最后), 默认选第一个 USB."""
        self._port_infos = {d["device"]: d for d in infos}
        cur = self.cmb_port.currentData()
        self.cmb_port.blockSignals(True)
        self.cmb_port.clear()
        for d in infos:
            self.cmb_port.addItem("%s  ·  %s" % (d["device"], d["desc"]), d["device"])
        self.cmb_port.blockSignals(False)

        idx = None
        if cur:
            for i in range(self.cmb_port.count()):
                if self.cmb_port.itemData(i) == cur:
                    idx = i
                    break
        if idx is None and infos:
            idx = 0
            for i, d in enumerate(infos):
                if d["kind"] == "usb":
                    idx = i
                    break
        if idx is not None:
            self.cmb_port.setCurrentIndex(idx)
        self._update_port_preview()

    def _update_port_preview(self, *args):
        dev = self.cmb_port.currentData()
        d = self._port_infos.get(dev)
        if not d:
            self.lbl_port.setText("—")
            self.lbl_port.setToolTip("")
            return
        tag = {"usb": "USB", "bt": "蓝牙", "debug": "调试器", "other": "串口"}.get(d["kind"], "串口")
        self.lbl_port.setText("%s · %s" % (tag, d["desc"]))
        self.lbl_port.setToolTip(d["hwid"] or d["desc"])

    def current_port(self):
        return self.cmb_port.currentData() or self.cmb_port.currentText().split("  ·  ")[0].strip()

    def stepper_cfg(self):
        return {k: {"min": s.lo, "max": s.hi, "step": s.step} for k, s in self.steppers.items()}

    def apply_stepper_cfg(self, cfg):
        if not isinstance(cfg, dict):
            return
        for k, s in self.steppers.items():
            d = cfg.get(k)
            if not isinstance(d, dict):
                continue
            try:
                mn, mx, st = float(d["min"]), float(d["max"]), float(d["step"])
            except (KeyError, TypeError, ValueError):
                continue
            if mx > mn and st > 0:
                s.set_range(mn, mx, st)


class RightDataPanel(QWidget):
    visChanged = Signal()

    def __init__(self, model, store=None, parent=None):
        super().__init__(parent)
        self.model = model
        self.store = store
        self.stat_mode = False
        self.setFixedWidth(300)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        head = QHBoxLayout()
        title = QLabel("数据")
        title.setObjectName("CardTitle")
        head.addWidget(title)
        head.addStretch(1)
        self.btn_stat = QPushButton("统计")
        self.btn_stat.setCheckable(True)
        self.btn_stat.setObjectName("Toggle")
        self.btn_stat.setFixedWidth(52)
        self.btn_stat.setToolTip("显示 min / max / 均值 (自上次清空起)")
        self.btn_stat.toggled.connect(self._on_stat)
        head.addWidget(self.btn_stat)
        lay.addLayout(head)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        inner = QWidget()
        self.v = QVBoxLayout(inner)
        self.v.setContentsMargins(2, 2, 2, 2)
        self.v.setSpacing(7)

        self.rows = {}
        self.name_labels = {}
        self.dots = {}
        self.checks = {}
        for i in range(NCH):
            self.v.addWidget(self._make_row(i))
        # 派生: 母线功率
        self.power_val = self._add_derived("母线功率", "W", theme.FG)
        self.v.addStretch(1)
        scroll.setWidget(inner)
        lay.addWidget(scroll, 1)

    def _make_row(self, i):
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(7)
        cb = QCheckBox()
        cb.setChecked(self.model.shown[i])
        cb.setFixedWidth(18)
        cb.toggled.connect(lambda on, i=i: self._on_vis(i, on))
        h.addWidget(cb)
        d = dot(self.model.color[i])
        h.addWidget(d)
        name = QLabel(self.model.disp[i])
        name.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        h.addWidget(name)
        h.addStretch(1)
        val = QLabel("-")
        val.setStyleSheet("color:%s;background:transparent;font-family:%s;font-weight:600;" %
                          (self.model.color[i], theme.MONO))
        h.addWidget(val)
        self.rows[i] = val
        self.name_labels[i] = name
        self.dots[i] = d
        self.checks[i] = cb
        return row

    def _on_vis(self, i, on):
        self.model.set_shown(i, on)
        self.visChanged.emit()

    def refresh_names(self):
        for i in range(NCH):
            self.name_labels[i].setText(self.model.disp[i])
            self.checks[i].blockSignals(True)
            self.checks[i].setChecked(self.model.shown[i])
            self.checks[i].blockSignals(False)
            self.rows[i].setStyleSheet(
                "color:%s;background:transparent;font-family:%s;font-weight:600;" %
                (self.model.color[i], theme.MONO))
        self.update()

    def _add_derived(self, name, unit, color):
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(7)
        h.addWidget(dot(theme.FG_DIM))
        lb = QLabel(name)
        lb.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        h.addWidget(lb)
        h.addStretch(1)
        val = QLabel("-")
        val.setStyleSheet("color:%s;background:transparent;font-family:%s;font-weight:600;" %
                          (color, theme.MONO))
        h.addWidget(val)
        self.v.addWidget(row)
        return val

    def _on_stat(self, on):
        self.stat_mode = bool(on)
        self.update()

    def update_values(self, vals):
        power = None
        stat = self.stat_mode and self.store is not None
        for i in range(NCH):
            v = vals[i] if i < len(vals) else None
            sname = CHANNELS[i][0] if i < len(CHANNELS) else ""
            if stat:
                st = self.store.stats(i)
                if st is None:
                    self.rows[i].setText("-")
                else:
                    g, o = self.model.gain[i], self.model.off[i]
                    lo, hi = st[0], st[1]
                    if g < 0:
                        lo, hi = hi, lo
                    self.rows[i].setText("%s/%s/%s" % (num(lo * g + o, 3),
                                                       num(hi * g + o, 3),
                                                       num(st[2] * g + o, 3)))
                continue
            if v is None:
                self.rows[i].setText("-")
            elif sname.startswith("fault"):
                mask = int(round(v))
                self.rows[i].setText("/".join(fault_texts(mask)) if mask else "无")
            elif sname.startswith("sat"):
                self.rows[i].setText("1" if v > 0.5 else "0")
            else:
                self.rows[i].setText(num(v))
            if sname.startswith("Vbus"):
                power = v
            elif sname.startswith("Ibus") and power is not None:
                p = power * v
                self.power_val.setText(num(p, 3))
                power = None


class StatusBarWidget(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(30)
        self.setStyleSheet("QFrame{background:%s;border-top:1px solid %s;}" %
                           (theme.TITLEBAR, theme.BORDER))
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 0, 10, 0)
        lay.setSpacing(10)
        self.conn_dot = dot(theme.FG_FAINT, 9)
        lay.addWidget(self.conn_dot)
        self.lbl_conn = QLabel("未连接")
        self.lbl_conn.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        lay.addWidget(self.lbl_conn)
        lay.addWidget(self._sep())
        self.lbl_info = QLabel("FW: —   HW: —")
        self.lbl_info.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        lay.addWidget(self.lbl_info)
        lay.addWidget(self._sep())
        self.lbl_io = QLabel("TX 0 B  RX 0 B")
        self.lbl_io.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        lay.addWidget(self.lbl_io)
        lay.addStretch(1)
        self.lbl_stat = QLabel("")
        self.lbl_stat.setStyleSheet("color:%s;background:transparent;" % theme.FG_DIM)
        lay.addWidget(self.lbl_stat)

    def _sep(self):
        s = QFrame()
        s.setFixedSize(1, 16)
        s.setStyleSheet("background:%s;border:none;" % theme.BORDER)
        return s

    def set_connected(self, on):
        self.conn_dot.setStyleSheet("background:%s;border-radius:4px;" %
                                    (theme.GREEN if on else theme.FG_FAINT))
        self.lbl_conn.setText("已连接" if on else "未连接")

    def set_io(self, tx, rx):
        self.lbl_io.setText("TX %d B   RX %d B" % (tx, rx))

    def set_stats(self, fs, fps, dropped):
        self.lbl_stat.setText("采样 %.0f Hz · %.0f 帧/s · 丢帧 %d" % (fs, fps, dropped))

    def flash(self, text):
        """非模态的瞬时提示 (会被下一次统计刷新覆盖)."""
        self.lbl_stat.setText(text)


class ChannelDialog(QDialog):
    """通道设置: 显示名/增益/偏置/图1/图2; 以及显示时长/采样率."""

    def __init__(self, model, max_span=10.0, cache_span=60.0, fs=5000.0, parent=None):
        super().__init__(parent)
        self.model = model
        self.setWindowTitle("通道设置")
        self.resize(700, 640)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)

        top = QHBoxLayout()
        top.addWidget(QLabel("最大显示时长(s)"))
        self.ent_span = QLineEdit(num(max_span))
        self.ent_span.setFixedWidth(70)
        top.addWidget(self.ent_span)
        top.addSpacing(12)
        top.addWidget(QLabel("缓存时长(s)"))
        self.ent_cache = QLineEdit(num(cache_span))
        self.ent_cache.setFixedWidth(70)
        top.addWidget(self.ent_cache)
        top.addSpacing(12)
        top.addWidget(QLabel("采样率(Hz)"))
        self.ent_fs = QLineEdit(num(fs, 0))
        self.ent_fs.setFixedWidth(80)
        top.addWidget(self.ent_fs)
        top.addStretch(1)
        lay.addLayout(top)

        path, cnt, ok = source_info()
        if ok:
            src = "已自动绑定固件 vofa.c · %d 通道" % cnt
        else:
            src = "未找到 vofa.c, 使用内置默认通道 · %d 通道" % NCH
        text = "通道来源: " + src
        extra = _FW.summary()
        if extra:
            text += "   |   参数来源: " + extra
        self.lbl_src = QLabel(text)
        self.lbl_src.setWordWrap(True)
        self.lbl_src.setStyleSheet("color:%s;background:transparent;" % theme.FG_FAINT)
        if path:
            self.lbl_src.setToolTip(path)
        lay.addWidget(self.lbl_src)

        self.table = QTableWidget(NCH, 5)
        self.table.setHorizontalHeaderLabels(["短名", "显示名", "增益", "偏置", "显示"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.Stretch)
        hh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(4, QHeaderView.ResizeToContents)

        for i in range(NCH):
            self._fill_row(i)
        lay.addWidget(self.table)

        brow = QHBoxLayout()
        b1 = QPushButton("全选")
        b2 = QPushButton("清空")
        b1.clicked.connect(lambda: self._set_all(True))
        b2.clicked.connect(lambda: self._set_all(False))
        brow.addWidget(b1)
        brow.addWidget(b2)
        brow.addStretch(1)
        lay.addLayout(brow)

        brow = QHBoxLayout()
        brow.addStretch(1)
        ok = QPushButton("确定")
        ok.setObjectName("Accent")
        cancel = QPushButton("取消")
        ok.clicked.connect(self._accept)
        cancel.clicked.connect(self.reject)
        brow.addWidget(cancel)
        brow.addWidget(ok)
        lay.addLayout(brow)

    def _fill_row(self, i):
        it0 = QTableWidgetItem(self.model.short[i])
        it0.setFlags(it0.flags() & ~Qt.ItemIsEditable)
        it0.setForeground(QColor(self.model.color[i]))
        self.table.setItem(i, 0, it0)
        self.table.setItem(i, 1, QTableWidgetItem(self.model.disp[i]))
        self.table.setItem(i, 2, QTableWidgetItem(num(self.model.gain[i])))
        self.table.setItem(i, 3, QTableWidgetItem(num(self.model.off[i])))
        it = QTableWidgetItem()
        it.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        it.setCheckState(Qt.Checked if self.model.shown[i] else Qt.Unchecked)
        self.table.setItem(i, 4, it)

    def _set_all(self, on):
        for i in range(NCH):
            self.table.item(i, 4).setCheckState(Qt.Checked if on else Qt.Unchecked)

    def result_values(self):
        span = cache = fs = None
        for ent, name in ((self.ent_span, "span"), (self.ent_cache, "cache"), (self.ent_fs, "fs")):
            try:
                v = float(ent.text())
            except (TypeError, ValueError):
                continue
            if name == "span":
                span = v
            elif name == "cache":
                cache = v
            else:
                fs = v
        return span, cache, fs

    def _accept(self):
        for i in range(NCH):
            name = self.table.item(i, 1).text().strip()
            if name:
                self.model.disp[i] = name
            try:
                self.model.gain[i] = float(self.table.item(i, 2).text())
            except (TypeError, ValueError):
                pass
            try:
                self.model.off[i] = float(self.table.item(i, 3).text())
            except (TypeError, ValueError):
                pass
            self.model.shown[i] = self.table.item(i, 4).checkState() == Qt.Checked
        self.accept()
