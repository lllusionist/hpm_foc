# -*- coding: utf-8 -*-
"""主窗口: 组装布局、串口/演示数据、主循环、配置持久化."""
import json
import math
import os
import time

from PySide6.QtCore import Qt, QTimer, QPoint, QUrl, QStandardPaths
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSlider,
    QSplitter, QLineEdit, QToolButton, QFrame, QMessageBox, QButtonGroup,
)

from . import theme
from .channels import ChannelModel, FAULT_TEXT, NCH, CHANNELS, index_of
from .buffers import DataStore
from .protocol import CMD, MODES
from .serial_io import SerialIO, HAVE_SERIAL
from .plot import PlotWidget, OverviewWidget
from .panels import TitleBar, LeftPanel, RightDataPanel, StatusBarWidget, ChannelDialog, num
from .recorder import Recorder, write_csv, default_dir, make_filename

CMD_NAME = {v: k for k, v in CMD.items()}

CFG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scope_cfg.json")
DEFAULT_FS = 5000.0        # 固件 VOFA 采样率 = 40kHz/8
EDGE = 5

TARGET_DEFS = {
    # (标签, 命令键, 最小, 最大, 步长) —— 与固件单位/限幅一致
    0: ("目标电流 (A)",    "torque",   -8.0,    8.0,    0.001),  # current_limit 默认 8A
    1: ("目标速度 (rps)",  "velocity", -30.0,   30.0,   0.001),  # velocity_limit 默认 30rps
    2: ("目标位置 (rad)",  "position", 0.0,     6.2832, 0.001),  # 0~2π rad
    3: ("目标位置 (rad)",  "position", 0.0,     6.2832, 0.001),  # 轨迹模式同样走位置
}


class MainWindow(QWidget):
    def __init__(self, demo=False):
        super().__init__()
        self.demo = demo
        self.setWindowTitle("FOC 上位机")
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        self.setMinimumSize(1180, 720)
        self.setObjectName("MainWindow")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setMouseTracking(True)
        self.cfg = self._load_cfg()

        self.model = ChannelModel()
        self.model.load_cfg(self.cfg.get("channels"))
        self.max_span = float(self.cfg.get("max_span", 10.0))
        self.cache_span = float(self.cfg.get("cache_span", 60.0))
        self.fs = float(self.cfg.get("fs", DEFAULT_FS))
        self.store = DataStore(self.fs, self.max_span, self.cache_span)
        self.recorder = Recorder(self.fs)
        self.record_max_sec = float(self.cfg.get("record_max_sec", 120.0))
        self.io = SerialIO()
        self._enabled = False
        self._mode = None
        self._frozen = False
        self._last_tx = 0.0
        self._last_fault = 0
        self._fault_idx = index_of("fault")
        self._demo_t = 0.0
        self._stat_t = time.time()
        self._stat_frames = 0
        self._resize_edge = None
        self._resize_geo = None

        self._build()
        self._wire()
        self.left.apply_stepper_cfg(self.cfg.get("steppers") or {})

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

        self._last_ports_key = None
        self.refresh_ports()
        self._port_timer = QTimer(self)
        self._port_timer.timeout.connect(self._auto_refresh_ports)
        self._port_timer.start(1500)

    # ================= 构建界面 =================
    def _build(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(EDGE, EDGE, EDGE, EDGE)
        outer.setSpacing(0)
        root = QFrame()
        root.setObjectName("Root")
        root.setStyleSheet("QFrame#Root{background:%s;border:1px solid %s;border-radius:8px;}" %
                           (theme.BG, theme.BORDER))
        outer.addWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        self.title = TitleBar()
        v.addWidget(self.title)

        v.addWidget(self._build_toolbar())

        body = QHBoxLayout()
        body.setContentsMargins(8, 8, 8, 8)
        body.setSpacing(8)
        self.left = LeftPanel()
        body.addWidget(self.left)
        body.addWidget(self._build_center(), 1)
        self.right = RightDataPanel(self.model)
        body.addWidget(self.right)
        v.addLayout(body, 1)

        self.statusbar = StatusBarWidget()
        v.addWidget(self.statusbar)

        self._restore_geometry()

    def _build_toolbar(self):
        bar = QFrame()
        bar.setFixedHeight(48)
        bar.setStyleSheet("QFrame{background:transparent;}")
        h = QHBoxLayout(bar)
        h.setContentsMargins(10, 4, 10, 4)
        h.setSpacing(10)

        h.addWidget(QLabel("模式"))
        self.mode_buttons = []
        self.mode_group = QButtonGroup(self)
        self.mode_group.setExclusive(True)
        for i, name in enumerate(MODES):
            b = QPushButton(name)
            b.setObjectName("Toggle")
            b.setCheckable(True)
            b.setFixedWidth(58)
            b.clicked.connect(lambda _=False, idx=i: self._on_mode(idx))
            self.mode_group.addButton(b, i)
            h.addWidget(b)
            self.mode_buttons.append(b)

        h.addStretch(1)

        h.addWidget(QLabel("错误"))
        self.lbl_fault = QLabel("无")
        self.lbl_fault.setStyleSheet("color:%s;background:transparent;font-weight:600;" % theme.GREEN)
        h.addWidget(self.lbl_fault)
        self.btn_clear_fault = QPushButton("清除错误")
        self.btn_clear_fault.setEnabled(False)
        self.btn_clear_fault.setToolTip("固件暂无清故障命令")
        h.addWidget(self.btn_clear_fault)

        self.btn_calib = QPushButton("校准")
        self.btn_calib.setToolTip("开始 R/L 参数校准 (电机需可自由转动)")
        self.btn_enable = QPushButton("使能")
        self.btn_enable.setObjectName("Toggle")
        self.btn_enable.setCheckable(True)
        self.btn_disable = QPushButton("失能")
        self.btn_disable.setObjectName("Toggle")
        self.btn_disable.setCheckable(True)
        self.btn_zero = QPushButton("当前位置置零")
        self.btn_record = QPushButton("● 记录波形")
        self.btn_record.setObjectName("Toggle")
        self.btn_record.setCheckable(True)
        self.btn_record.setToolTip("开始录制全分辨率波形+命令; 再点一下停止并保存到桌面")
        for b in (self.btn_calib, self.btn_enable, self.btn_disable, self.btn_zero, self.btn_record):
            h.addWidget(b)
        return bar

    def _build_center(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(6)

        self.split = QSplitter(Qt.Vertical)
        self.plot_top = PlotWidget(self.store, self.model)
        self.overview = OverviewWidget(self.store, self.model)
        self.plot_main = PlotWidget(self.store, self.model)
        self.split.addWidget(self.plot_top)
        self.split.addWidget(self.overview)
        self.split.addWidget(self.plot_main)
        self.split.setStretchFactor(0, 0)
        self.split.setStretchFactor(1, 0)
        self.split.setStretchFactor(2, 1)
        sizes = self.cfg.get("split", [210, 74, 360])
        if not (isinstance(sizes, list) and len(sizes) == 3):
            sizes = [210, 74, 360]
        self.split.setSizes(sizes)
        v.addWidget(self.split, 1)

        row = QHBoxLayout()
        row.setSpacing(10)
        self.btn_clear = QPushButton("清空数据")
        row.addWidget(self.btn_clear)
        row.addWidget(QLabel("曲线冻结"))
        self.btn_freeze = QPushButton("关闭")
        self.btn_freeze.setCheckable(True)
        self.btn_freeze.setFixedWidth(64)
        row.addWidget(self.btn_freeze)
        row.addStretch(1)
        self.btn_auto = QPushButton("Auto")
        row.addWidget(self.btn_auto)
        v.addLayout(row)

        row2 = QHBoxLayout()
        row2.setSpacing(10)
        self.lbl_target = QLabel(TARGET_DEFS[0][0])
        row2.addWidget(self.lbl_target)
        self.ent_target = QLineEdit("0")
        self.ent_target.setFixedWidth(72)
        self.ent_target.setAlignment(Qt.AlignCenter)
        row2.addWidget(self.ent_target)
        self.sld_target = QSlider(Qt.Horizontal)
        row2.addWidget(self.sld_target, 1)
        self.btn_send_target = QToolButton()
        self.btn_send_target.setText("➤")
        self.btn_send_target.setFixedSize(30, 26)
        self.btn_send_target.setStyleSheet(
            "color:%s;background:#ffffff;border:1px solid #ccd2db;border-radius:6px;"
            "QToolButton:hover{background:#f1f4f9;}" % theme.ACCENT)
        row2.addWidget(self.btn_send_target)
        v.addLayout(row2)

        self._apply_target(None)
        return w

    # ================= 信号 =================
    def _wire(self):
        self.title.request_settings.connect(self.open_settings)
        self.title.request_clear.connect(self.clear_data)
        self.title.request_min.connect(self.showMinimized)
        self.title.request_max.connect(self._toggle_max)
        self.title.request_close.connect(self.close)

        self.left.btn_connect.clicked.connect(self.connect)
        self.left.btn_disconnect.clicked.connect(self.disconnect)
        self.left.btn_save.clicked.connect(lambda: self.send(CMD["save"]))
        self.left.btn_reset.clicked.connect(self.reset_defaults)
        for key, s in self.left.steppers.items():
            s.changed.connect(lambda v, k=key: self._stepper_send(k, v))
            s.sent.connect(lambda v, k=key: self.send(CMD[k], v))
            s.configChanged.connect(self._save_cfg_later)

        self.btn_enable.clicked.connect(self.enable)
        self.btn_disable.clicked.connect(self.disable)
        self.btn_calib.clicked.connect(lambda: self.send(CMD["calib"]))
        self.btn_zero.clicked.connect(lambda: self.send(CMD["home"]))
        self.btn_record.clicked.connect(self.toggle_record)
        self.btn_clear.clicked.connect(self.clear_data)
        self.btn_freeze.clicked.connect(self.toggle_freeze)
        self.btn_auto.clicked.connect(self.autoscale_all)
        self.btn_send_target.clicked.connect(self._send_target_now)
        self.sld_target.valueChanged.connect(self._on_target_slider)
        self.sld_target.sliderReleased.connect(self._send_target_now)
        self.ent_target.returnPressed.connect(self._send_target_now)

        self.right.visChanged.connect(self._on_vis_changed)
        self.overview.viewChanged.connect(self._on_overview_view)
        self.overview.resetRequested.connect(self._reset_view)

        self._sync_plots()

    # ================= 串口 =================
    def refresh_ports(self):
        infos = SerialIO.ports_info()
        self.left.populate_ports(infos)
        self._last_ports_key = self._ports_key(infos)

    @staticmethod
    def _ports_key(infos):
        return tuple((d["device"], d["desc"], d["kind"]) for d in infos)

    def _auto_refresh_ports(self):
        try:
            if self.left.cmb_port.view().isVisible():
                return
        except Exception:
            pass
        infos = SerialIO.ports_info()
        key = self._ports_key(infos)
        if key != self._last_ports_key:
            self.left.populate_ports(infos)
            self._last_ports_key = key

    def connect(self):
        if not HAVE_SERIAL:
            QMessageBox.critical(self, "错误", "未安装 pyserial: pip install pyserial")
            return
        dev = self.left.current_port()
        if not dev:
            QMessageBox.warning(self, "提示", "先选择串口 (可点刷新)")
            return
        try:
            self.io.open(dev)
        except Exception as e:
            QMessageBox.critical(self, "打开失败", str(e))
            return
        self.demo = False
        self._set_conn_ui(True)
        # 连接后把当前选择的运行模式下发给固件 (未选择模式则不发送)
        if self._mode is not None:
            self.send(CMD["opmode"], float(self._mode))

    def disconnect(self):
        self.io.close()
        self._set_conn_ui(False)

    def _set_conn_ui(self, on):
        self.left.btn_connect.setEnabled(not on)
        self.left.btn_disconnect.setEnabled(on)
        self.title.set_connected(on, "已连接" if on else "未连接")
        self.statusbar.set_connected(on)
        if not on:
            self._clear_enable()
            self._clear_mode()

    def _clear_mode(self):
        self._mode = None
        self.mode_group.setExclusive(False)
        for b in self.mode_buttons:
            b.setChecked(False)
        self.mode_group.setExclusive(True)
        self._apply_target(None)

    def send(self, cid, val=None):
        if not self.io.is_open():
            return
        self.io.send(cid, val)
        self._record_event(cid, val)

    def _record_event(self, cid, val):
        """录制中时, 把发出的命令记进时间线 (与波形写同一文件)."""
        if not self.recorder.is_active():
            return
        name = CMD_NAME.get(cid, "0x%02X" % cid)
        if cid == CMD["opmode"] and val is not None:
            try:
                i = int(val)
                val = "%s(%d)" % (MODES[i] if 0 <= i < len(MODES) else "?", i)
            except (TypeError, ValueError):
                pass
        self.recorder.add_event(name, val)

    def _stepper_send(self, key, val):
        self.send(CMD[key], val)

    def reset_defaults(self):
        r = QMessageBox.question(
            self, "恢复默认参数",
            "将恢复固件出厂默认参数并清除校准 (需重新校准), 继续?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r == QMessageBox.Yes:
            self.send(CMD["reset"])

    def enable(self):
        self._set_enabled(True)
        self.send(CMD["enable"])

    def disable(self):
        self._set_enabled(False)
        self.send(CMD["disable"])

    def _set_enabled(self, on):
        self._enabled = on
        self.btn_enable.setChecked(on)
        self.btn_disable.setChecked(not on)

    def _clear_enable(self):
        self._enabled = False
        self.btn_enable.setChecked(False)
        self.btn_disable.setChecked(False)

    # ================= 模式/目标 =================
    def _on_mode(self, idx):
        self._mode = idx
        self.send(CMD["opmode"], float(idx))
        self._apply_target(idx)

    def _apply_target(self, mode):
        if mode is None:
            self._target_key = None
            self.lbl_target.setText("目标值 (请先选择模式)")
            self.sld_target.blockSignals(True)
            self.sld_target.setMinimum(0)
            self.sld_target.setMaximum(1)
            self.sld_target.setValue(0)
            self.sld_target.blockSignals(False)
            self.ent_target.setText("0")
            self.sld_target.setEnabled(False)
            self.ent_target.setEnabled(False)
            self.btn_send_target.setEnabled(False)
            return
        label, key, lo, hi, step = TARGET_DEFS.get(mode, TARGET_DEFS[1])
        self._target_key = key
        self._target_step = step
        self.lbl_target.setText(label)
        self.sld_target.blockSignals(True)
        self.sld_target.setMinimum(int(round(lo / step)))
        self.sld_target.setMaximum(int(round(hi / step)))
        self.sld_target.setValue(0)
        self.sld_target.blockSignals(False)
        self.ent_target.setText("0")
        self.sld_target.setEnabled(True)
        self.ent_target.setEnabled(True)
        self.btn_send_target.setEnabled(True)

    def _on_target_slider(self, iv):
        if self._target_key is None:
            return
        v = iv * self._target_step
        self.ent_target.setText(num(v, 3))
        now = time.time()
        if now - self._last_tx > 0.05:
            self._last_tx = now
            self.send(CMD.get(self._target_key, CMD["velocity"]), v)

    def _send_target_now(self):
        if self._target_key is None:
            return
        try:
            v = float(self.ent_target.text() or 0)
        except ValueError:
            return
        self.send(CMD.get(self._target_key, CMD["velocity"]), v)

    # ================= 显示控制 =================
    def clear_data(self):
        self.store.clear()
        self.plot_top.update()
        self.plot_main.update()
        self.overview.set_window(self.plot_main.t_left(), self.plot_main.t_span)

    def toggle_freeze(self):
        self._frozen = self.btn_freeze.isChecked()
        self.btn_freeze.setText("开启" if self._frozen else "关闭")
        self.btn_freeze.setStyleSheet(
            "QPushButton{background:%s;border:1px solid %s;color:#fff;border-radius:6px;}" %
            (theme.ACCENT_D, theme.ACCENT) if self._frozen else "")
        self.plot_top.set_frozen(self._frozen)
        self.plot_main.set_frozen(self._frozen)

    def autoscale_all(self):
        self.plot_top.autoscale()
        self.plot_main.autoscale()

    # ================= 波形记录 =================
    def toggle_record(self):
        if self.recorder.is_active():
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        self.recorder.start(self.fs)
        self.btn_record.setChecked(True)
        self.btn_record.setText("■ 录制中 0.0s (0点)")
        self.btn_record.setStyleSheet(
            "QPushButton{background:#7f2d2d;border:1px solid #b33a3a;"
            "color:#ffdede;border-radius:6px;}")

    def _stop_record(self, auto=False, notify=True):
        was = self.recorder.is_active()
        self.recorder.stop()
        self.btn_record.setChecked(False)
        self.btn_record.setText("● 记录波形")
        self.btn_record.setStyleSheet("")
        if not was:
            return
        if self.recorder.count == 0:
            if notify:
                self.statusbar.flash("没有录到数据")
            return
        self._save_record(auto=auto, notify=notify)

    def _save_record(self, auto=False, notify=True):
        path = os.path.join(default_dir(), make_filename())
        try:
            write_csv(self.recorder, path, self._record_meta())
        except Exception as e:
            if notify:
                QMessageBox.critical(self, "保存失败", str(e))
            return None
        if notify:
            self.statusbar.flash("已保存: %s" % path)
        return path

    def _record_meta(self):
        vals = self.store.latest_values()

        def rd(name):
            i = index_of(name)
            if i is None or i >= len(vals) or vals[i] is None:
                return "-"
            return num(vals[i], 4)

        fault = 0
        fi = index_of("fault")
        if fi is not None and fi < len(vals) and vals[fi] is not None:
            fault = int(round(vals[fi]))
        st = self.left.steppers

        def sv(k):
            s = st.get(k)
            return "-" if s is None else s.value()

        port = "demo" if self.demo else (self.left.current_port() or "-")
        if self._mode is None:
            mode = "未选择"
        elif 0 <= self._mode < len(MODES):
            mode = MODES[self._mode]
        else:
            mode = str(self._mode)
        return {
            "生成": time.strftime("%Y-%m-%d %H:%M:%S"),
            "端口": port,
            "采样率": "%g Hz" % self.fs,
            "样本数": "%d" % self.recorder.count,
            "时长": "%.3f s" % self.recorder.duration(),
            "运行模式": mode,
            "故障": FAULT_TEXT.get(fault, "?"),
            "读数": "Iq=%s Iq_set=%s vel=%s vel_set=%s Vbus=%s" %
                    (rd("Iq"), rd("Iq_set"), rd("vel"), rd("vel_set"), rd("Vbus")),
            "参数": "电流Kp=%s 电流Ki=%s 电流BW=%s 速度Kp=%s 速度Ki=%s 位置Kp=%s" %
                    (sv("curkp"), sv("curki"), sv("curbw"),
                     sv("velkp"), sv("velki"), sv("poskp")),
        }

    def _on_vis_changed(self):
        self._sync_plots()
        self._save_cfg_later()

    def _on_overview_view(self, left, span):
        self.plot_main.set_view(left, span)
        self.overview.set_window(self.plot_main.t_left(), self.plot_main.t_span, force=True)

    def _reset_view(self):
        self.plot_top.reset_view()
        self.plot_main.reset_view()
        self.overview.set_window(self.plot_main.t_left(), self.plot_main.t_span, force=True)

    def _sync_plots(self):
        shown = self.model.shown_channels()
        self.plot_top.set_channels(list(self.model.TOP_CHANNELS))
        self.plot_main.set_channels(shown)
        self.overview.set_channels(shown)

    def open_settings(self):
        dlg = ChannelDialog(self.model, self.max_span, self.cache_span, self.fs, self)
        if dlg.exec():
            self._sync_plots()
            self.right.refresh_names()
            span, cache, fs = dlg.result_values()
            if span and span > 0:
                self.max_span = span
                self.store.max_span = span
                self.plot_top.max_span = span
                self.plot_main.max_span = span
            if cache and cache > 0:
                self.cache_span = cache
                self.store.cache_span = cache
                self.store.trim()
                self.overview.update()
            if fs and fs > 0:
                self.fs = fs
                self.store.fs = fs
            self._save_cfg_later()

    # ================= 主循环 =================
    def _tick(self):
        if self.io.link_lost and self.io.is_open():
            self.io.close()
            self._set_conn_ui(False)
            self.title.set_connected(False, "已断开 · 可重连")
        frames = self._collect()
        # 录制独立于“曲线冻结”: 冻结只停绘图, 不影响记录
        if self.recorder.is_active():
            for vals in frames:
                self.recorder.append(vals)
            if self.recorder.duration() >= self.record_max_sec:
                self._stop_record(auto=True)
        if not self._frozen:
            for vals in frames:
                self.store.append(vals)
        if frames:
            self.plot_top.update()
            self.plot_main.update()
        self.overview.set_window(self.plot_main.t_left(), self.plot_main.t_span)

        vals = self.store.latest_values()
        self.right.update_values(vals)
        self._update_fault(vals)

        now = time.time()
        if now - self._stat_t >= 0.5:
            fps = (self.store.total - self._stat_frames) / (now - self._stat_t)
            self._stat_frames = self.store.total
            self._stat_t = now
            st = self.io.stats()
            self.statusbar.set_io(st["tx"], st["rx"])
            self.statusbar.set_stats(self.fs, fps, st["dropped"])
            if self.recorder.is_active():
                self.btn_record.setText("■ 录制中 %.1fs (%d点)" %
                                        (self.recorder.duration(), self.recorder.count))

    def _collect(self):
        if self.demo:
            out = []
            dt = 1.0 / self.fs
            n = max(1, int(self.fs * 0.033))
            for _ in range(n):
                t = self._demo_t
                self._demo_t += dt
                ia = 3.0 * math.sin(2 * math.pi * 8 * t)
                ib = 3.0 * math.sin(2 * math.pi * 8 * t - 2.094)
                ic = 3.0 * math.sin(2 * math.pi * 8 * t + 2.094)
                vel = 2.0 + 0.2 * math.sin(2 * math.pi * 0.5 * t)
                out.append((
                    ia, ib, ic,
                    0.15 * math.sin(2 * math.pi * 1.0 * t),      # Id
                    3.0 * math.cos(2 * math.pi * 1.0 * t),       # Iq
                    3.0,                                          # Iq_set
                    2.0,                                          # vel_set
                    vel,                                          # vel
                    0.06 * math.cos(2 * math.pi * 1.0 * t),      # torque
                    23.8,                                         # Vbus
                    12.0 * math.cos(2 * math.pi * 8 * t),        # Vmag
                    0.0,                                          # sat
                    0.2, 11.9,                                    # Vd, Vq
                    (t * 2 * math.pi) % (2 * math.pi),            # phase
                    2 * math.pi * 8,                              # phase_vel
                    0.02 * math.sin(2 * math.pi * 8 * t),         # isum
                    ia, ib, ic,                                   # Va,Vb,Vc
                    0.0,                                          # fault
                    3.2,                                          # Ibus
                ))
            self.io.frames += len(out)
            return out
        if not self.io.is_open():
            return []
        return self.io.poll()

    def _update_fault(self, vals):
        fi = self._fault_idx
        raw = vals[fi] if fi is not None and fi < len(vals) and vals[fi] is not None else 0.0
        code = int(round(raw))
        if code != self._last_fault:
            self._last_fault = code
            txt = FAULT_TEXT.get(code, "?")
            self.lbl_fault.setText(txt)
            self.lbl_fault.setStyleSheet("color:%s;background:transparent;font-weight:600;" %
                                         (theme.GREEN if code == 0 else theme.RED))
        # 状态灯
        w = self.left.status_widgets
        self._set_dot(w["enabled"], theme.GREEN if self._enabled else theme.FG_FAINT)
        self._set_dot(w["ok"], theme.GREEN if code == 0 else theme.FG_FAINT)
        for key, codev in (("f_oc", 1), ("f_ov", 2), ("f_uv", 3), ("f_adc", 4)):
            self._set_dot(w[key], theme.RED if code == codev else theme.FG_FAINT)

    @staticmethod
    def _set_dot(lb, color):
        lb.setStyleSheet("background:%s;border-radius:5px;" % color)

    # ================= 配置 =================
    def _load_cfg(self):
        try:
            with open(CFG_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_cfg_later(self):
        if hasattr(self, "_save_job") and self._save_job:
            self._save_job.stop()
        self._save_job = QTimer(self)
        self._save_job.setSingleShot(True)
        self._save_job.timeout.connect(self.save_cfg)
        self._save_job.start(800)

    def save_cfg(self):
        g = self.geometry()
        cfg = {
            "window": [g.x(), g.y(), g.width(), g.height()],
            "max_span": self.max_span,
            "cache_span": self.cache_span,
            "fs": self.fs,
            "record_max_sec": self.record_max_sec,
            "split": self.split.sizes(),
            "channels": self.model.to_cfg(),
            "steppers": self.left.stepper_cfg(),
        }
        try:
            with open(CFG_PATH, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=1)
        except Exception:
            pass

    def _restore_geometry(self):
        w = self.cfg.get("window")
        if isinstance(w, list) and len(w) == 4:
            try:
                self.setGeometry(int(w[0]), int(w[1]), int(w[2]), int(w[3]))
                return
            except Exception:
                pass
        self.resize(1360, 840)

    # ================= 无边框缩放 =================
    def _toggle_max(self):
        self.showNormal() if self.isMaximized() else self.showMaximized()

    def _edge_at(self, pos):
        r = self.rect()
        e = EDGE + 3
        left = pos.x() <= e
        right = pos.x() >= r.width() - e
        top = pos.y() <= e
        bottom = pos.y() >= r.height() - e
        return (left, right, top, bottom)

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            edges = self._edge_at(ev.position().toPoint())
            if any(edges):
                self._resize_edge = edges
                self._resize_geo = (ev.globalPosition().toPoint(), self.geometry())

    def mouseMoveEvent(self, ev):
        if self._resize_edge and self._resize_geo:
            start, geo = self._resize_geo
            dp = ev.globalPosition().toPoint() - start
            left, right, top, bottom = self._resize_edge
            x, y, w, h = geo.x(), geo.y(), geo.width(), geo.height()
            if left:
                x += dp.x(); w -= dp.x()
            if right:
                w += dp.x()
            if top:
                y += dp.y(); h -= dp.y()
            if bottom:
                h += dp.y()
            if w >= self.minimumWidth() and h >= self.minimumHeight():
                self.setGeometry(x, y, w, h)
            return
        c = Qt.ArrowCursor
        l, r, t, b = self._edge_at(ev.position().toPoint())
        if (l and t) or (r and b):
            c = Qt.SizeFDiagCursor
        elif (r and t) or (l and b):
            c = Qt.SizeBDiagCursor
        elif l or r:
            c = Qt.SizeHorCursor
        elif t or b:
            c = Qt.SizeVerCursor
        self.setCursor(c)

    def mouseReleaseEvent(self, ev):
        self._resize_edge = None
        self._resize_geo = None

    def closeEvent(self, ev):
        if self.recorder.is_active():
            self._stop_record(auto=True, notify=False)
        self.save_cfg()
        self.io.close()
        super().closeEvent(ev)
