# -*- coding: utf-8 -*-
"""主窗口: 组装布局、串口/演示数据、主循环、配置持久化."""
import json
import math
import os
import time

from PySide6.QtCore import Qt, QTimer, QPoint, QUrl, QStandardPaths
from PySide6.QtGui import QDesktopServices, QGuiApplication
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox, QSlider,
    QSplitter, QLineEdit, QToolButton, QFrame, QMessageBox, QButtonGroup, QScrollArea,
    QFileDialog, QCheckBox,
)

from . import theme
from .channels import ChannelModel, NCH, CHANNELS, index_of, STATUS_ITEMS, fault_texts
from .fwconfig import load as _load_fw
from .buffers import DataStore
from .protocol import CMD, HEARTBEAT_INTERVAL_MS, MODES
from .serial_io import SerialIO, HAVE_SERIAL
from .plot import PlotWidget, OverviewWidget
from .panels import TitleBar, LeftPanel, RightDataPanel, StatusBarWidget, ChannelDialog, num, TargetSlider
from .recorder import Recorder, write_csv, default_dir, make_filename

CMD_NAME = {v: k for k, v in CMD.items()}

CFG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scope_cfg.json")
EDGE = 5

# 从固件源码自动读取限幅/采样率 (读不到用默认)
_FW = _load_fw()
_CUR_LIM = _FW.get_float("current_limit", 8.0)
_VEL_LIM = _FW.get_float("velocity_limit", 30.0)
DEFAULT_FS = _FW.fs or 5000.0
PHASE_OC_ISR = _FW.fault_code("PHASE_OC_ISR", 1 << 5)

TARGET_DEFS = {
    # (标签, 命令键, 最小, 最大, 步长) —— 量程取自固件 current_limit / velocity_limit
    0: ("目标电流 (A)",    "torque",   -_CUR_LIM, _CUR_LIM, 0.001),
    1: ("目标速度 (rps)",  "velocity", -_VEL_LIM, _VEL_LIM, 0.001),
    2: ("目标角度 (rad)",  "position", -3.14159, 3.14159, 0.001),   # 单圈角度, 0 居中
    3: ("目标角度 (rad)",  "position", -3.14159, 3.14159, 0.001),
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
        self._enable_requested = False
        self._mode = None
        self._cur_lim = float(self.cfg.get("cur_limit", _CUR_LIM))
        self._vel_lim = float(self.cfg.get("vel_limit", _VEL_LIM))
        self._frozen = False
        self._last_tx = 0.0
        self._last_fault = None
        self._fault_idx = index_of("fault")
        self._cfg_wait_until = 0.0
        self._demo_t = 0.0
        self._stat_t = time.time()
        self._stat_frames = 0
        self._resize_edge = None
        self._resize_geo = None
        self._is_max = False
        self._normal_geo = None

        self._build()
        self._wire()
        self._set_conn_ui(False)
        self.left.apply_stepper_cfg(self.cfg.get("steppers") or {})
        self._update_target_defs()
        self._apply_saved_view()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.timeout.connect(self._send_heartbeat)
        self._heartbeat_timer.start(HEARTBEAT_INTERVAL_MS)

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
        self._outer = outer
        self._root = root
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
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.setWidget(self.left)
        left_scroll.setFixedWidth(318)
        body.addWidget(left_scroll)
        body.addWidget(self._build_center(), 1)
        self.right = RightDataPanel(self.model, self.store)
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
        self.lbl_fault = QLabel("—")
        h.addWidget(self.lbl_fault)
        self.btn_clear_fault = QPushButton("清除错误")
        self.btn_clear_fault.setEnabled(False)
        self.btn_clear_fault.setToolTip("先发送失能，再请求清除固件故障")
        h.addWidget(self.btn_clear_fault)

        self.btn_calib = QPushButton("校准")
        self.btn_calib.setToolTip("开始 R/L 参数校准 (电机需可自由转动)")
        # 两种齿槽标定方法写的是同一张表, 选一个即可
        self.btn_anticog_sweep = QPushButton("齿槽扫描★")
        self.btn_anticog_sweep.setToolTip(
            "【推荐】恒速扫描法 (0x2E): 速度环匀速 0.1 rps 正反各 4 圈,\n"
            "按机械角把电流分箱平均。约 90 s, 噪声比位置保持法低一个数量级,\n"
            "无步进扰动。实测 0.1 rps 转速纹波 -19%, 位置锁定分量 -35%")
        self.btn_anticog = QPushButton("齿槽标定(慢)")
        self.btn_anticog.setToolTip(
            "位置保持法 (0x28): 位置环把转子锁在每 0.7° 一格, 停稳后取 64 点平均,\n"
            "正反各扫一遍。约 2~4 分钟, 结果噪声较大 (单点约 3 mA)。\n"
            "保留作备用/对比: 电机行程受限、无法连续转圈时用这个")
        # 电机配置档 (固件 0x5F): 一键切换标称参数, 切换后会作废标定
        h.addWidget(QLabel("电机"))
        self.cb_motor = QComboBox()
        self.cb_motor.addItems(["4310 云台", "5210 航模", "2312 航模"])
        self.cb_motor.setPlaceholderText("档位待确认")
        self.cb_motor.setCurrentIndex(-1)
        self.cb_motor.setToolTip(
            "电机配置档 (0x5F):\n"
            "  4310 云台: 8.29Ω / 3.4mH / 14 对极 / 0.9A / 带宽 300Hz\n"
            "  5210 航模: 0.03Ω / 9µH / KV300 / 44A / 带宽 1000Hz\n"
            "  2312 航模: 0.144Ω / 28.5µH / 7 对极 / KV960 / 带宽 600Hz\n"
            "⚠ 切换会作废标定 (编码器零位/极对数都变了), 必须重新跑 0x27")
        self.cb_motor.setFixedWidth(96)
        h.addWidget(self.cb_motor)
        self._motor_profile = None
        self._motor_req = None
        # 用 activated(用户操作才触发): 重复选同一项也能重新下发,
        # 程序里 setCurrentIndex 同步状态时不会误发
        self.cb_motor.activated.connect(self._on_motor_profile)

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
        for b in (self.btn_calib, self.btn_anticog, self.btn_anticog_sweep, self.btn_enable, self.btn_disable, self.btn_zero, self.btn_record):
            h.addWidget(b)

        # A/B 调试开关 (固件 0x2A / 0x2B, 运行时即时生效)
        self.cb_anticog = QCheckBox("齿槽补偿")
        self.cb_anticog.setChecked(True)
        self.cb_anticog.setToolTip(
            "齿槽前馈在线开关 (0x2A)。用途: 做 A/B 验证补偿是否有效\n"
            "(低速匀速运行, 开/关各录 20~30 s, 对比通道 7 的转速纹波)。\n"
            "需要先跑过一次齿槽标定 (表在 Flash 里, 读参数看 COG=1)")
        self.cb_ff = QCheckBox("摩擦前馈")
        self.cb_ff.setChecked(True)
        self.cb_ff.setToolTip(
            "摩擦前馈在线开关 (0x2B)。稳态低速实测开关差异很小\n"
            "(0.1 rps: 0.0204 vs 0.0209 rps), 主要作用在启动/突破静摩擦时。\n"
            "出现低速抖动时可关掉排查是不是 BREAK/MOVE 换挡引起的")
        self.btn_dump_cog = QPushButton("导出齿槽表")
        self.btn_dump_cog.setToolTip(
            "0x2C: 把 Flash 里的 512 点齿槽表通过串口逐行打印 (索引,值),\n"
            "可用上位机日志/串口助手保存成 txt 做离线 FFT")
        for w in (self.cb_anticog, self.cb_ff, self.btn_dump_cog):
            h.addWidget(w)
        self.btn_dump_cog.clicked.connect(lambda: self.send(CMD["dump_cog"]))
        self.cb_anticog.toggled.connect(
            lambda on: self.send(CMD["set_anticog"], 1.0 if on else 0.0))
        self.cb_ff.toggled.connect(
            lambda on: self.send(CMD["set_ff"], 1.0 if on else 0.0))
        return bar

    def _build_center(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(6)

        self.split = QSplitter(Qt.Vertical)
        self.plot_top = PlotWidget(self.store, self.model)
        self.plot_top.setToolTip(
            "左键拖动: 左右平移时间 / 上下缩放数值轴\n"
            "（第一次水平拖动会自动暂停跟随, 右上角会提示）\n"
            "双击: 恢复跟随最新数据    滚轮: 缩放时间轴")
        self.overview = OverviewWidget(self.store, self.model)
        self.plot_main = PlotWidget(self.store, self.model)
        self.plot_main.setToolTip(
            "左键拖动: 左右平移时间 / 上下缩放数值轴\n"
            "双击: 恢复跟随最新数据    滚轮: 缩放时间轴")
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
        self.sld_target = TargetSlider()
        self.sld_target.setMinimumHeight(34)
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
        self.title.request_spectrum.connect(self.open_spectrum)
        self.title.request_vector.connect(self.open_vector)
        self.title.request_replay.connect(self.open_replay)
        self.title.request_min.connect(self.showMinimized)
        self.title.request_max.connect(self._toggle_max)
        self.title.request_close.connect(self.close)

        self.left.btn_connect.clicked.connect(self.connect)
        self.left.btn_disconnect.clicked.connect(self.disconnect)
        self.left.btn_save.clicked.connect(self.save_to_flash)
        self.left.btn_reset.clicked.connect(self.reset_defaults)
        self.left.btn_read.clicked.connect(self.request_config)
        for key, s in self.left.steppers.items():
            s.changed.connect(lambda v, k=key: self._stepper_send(k, v))
            s.sent.connect(lambda v, k=key: self.send(CMD[k], v))
            s.configChanged.connect(self._save_cfg_later)

        self.btn_enable.clicked.connect(self.enable)
        self.btn_disable.clicked.connect(self.disable)
        self.btn_clear_fault.clicked.connect(self.clear_fault)
        self.btn_calib.clicked.connect(self._start_calibration)
        self.btn_anticog.clicked.connect(lambda: self.send(CMD["anticog"]))
        self.btn_anticog_sweep.clicked.connect(lambda: self.send(CMD["anticog_sweep"]))
        self.btn_zero.clicked.connect(lambda: self.send(CMD["home"]))
        self.btn_record.clicked.connect(self.toggle_record)
        self.btn_clear.clicked.connect(self.clear_data)
        self.btn_freeze.clicked.connect(self.toggle_freeze)
        self.btn_auto.clicked.connect(self.autoscale_all)
        self.btn_send_target.clicked.connect(self._send_target_now)
        self.sld_target.valueChanged.connect(self._on_target_slider)
        self.sld_target.jumped.connect(self._on_target_jump)
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
            import traceback
            QMessageBox.critical(self, "打开失败", "%s\n\n%s" % (e, traceback.format_exc()))
            return
        if not self.send(CMD["disable"]):
            self.io.close()
            self._set_conn_ui(False)
            return
        self.demo = False
        self._set_conn_ui(True)
        # 连接后把当前选择的运行模式下发给固件 (未选择模式则不发送)
        if self._mode is not None:
            self.send(CMD["opmode"], float(self._mode))
        # 连接后自动读取一次设备参数
        self.request_config()

    def disconnect(self):
        if self.io.is_open() and not self.io.link_lost:
            self.send(CMD["disable"])
        self.io.close()
        self._set_conn_ui(False)

    def _send_heartbeat(self):
        if self.io.is_open() and not self.io.link_lost:
            self.io.send(CMD["heartbeat"])

    def _set_conn_ui(self, on):
        self.left.btn_connect.setEnabled(not on)
        self.left.btn_disconnect.setEnabled(on)
        self.title.set_connected(on, "已连接" if on else "未连接")
        self.statusbar.set_connected(on)
        for button in (self.btn_disable, self.btn_anticog,
                       self.btn_anticog_sweep, self.btn_zero):
            button.setEnabled(on)
        self._last_fault = None
        self.lbl_fault.setToolTip("")
        self._motor_profile = None
        self._motor_req = None
        self.cb_motor.setCurrentIndex(-1)
        if not on:
            self._cfg_wait_until = 0.0
            self._clear_enable()
            self._clear_mode()
        self._show_fault()

    def _clear_mode(self):
        self._mode = None
        self.mode_group.setExclusive(False)
        for b in self.mode_buttons:
            b.setChecked(False)
        self.mode_group.setExclusive(True)
        self._apply_target(None)

    def send(self, cid, val=None):
        if not self.io.send(cid, val):
            self.statusbar.flash("命令发送失败，请检查串口连接")
            return False
        self._record_event(cid, val)
        return True

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
        if r == QMessageBox.Yes and self.send(CMD["reset"]):
            self._motor_profile = None
            self._motor_req = None
            self.cb_motor.setCurrentIndex(-1)
            self._update_calibration_button()
            # 固件恢复默认后是 4310 档, 读回让界面同步
            QTimer.singleShot(500, self.request_config)

    # ================= 读取/保存设备参数 =================
    def save_to_flash(self):
        if not self.io.is_open():
            return
        # 先把界面当前参数全部下发, 再保存, 确保 Flash 与界面一致。
        # 电流带宽会按 R/L 重算 Kp/Ki, 所以先发带宽, 再发电流环 Kp/Ki(手动值优先)。
        keys = list(self.left.steppers.keys())
        order = (["curbw"] if "curbw" in keys else []) + [k for k in keys if k != "curbw"]
        for key in order:
            if not self.send(CMD[key], self.left.steppers[key].value()):
                return
        if self.send(CMD["save"]):
            self.statusbar.flash("保存请求已发送；写入结果见固件日志")

    def request_config(self):
        if not self.io.is_open():
            return
        if self.send(CMD["getcfg"]):
            self._cfg_wait_until = time.monotonic() + 1.5
            self.statusbar.flash("正在读取设备参数…")

    def _update_target_defs(self):
        TARGET_DEFS[0] = ("目标电流 (A)", "torque", -self._cur_lim, self._cur_lim, 0.001)
        TARGET_DEFS[1] = ("目标速度 (rps)", "velocity", -self._vel_lim, self._vel_lim, 0.001)
        if self._mode is not None:
            self._apply_target(self._mode)

    def _on_motor_profile(self, idx):
        prev = self._motor_profile if self._motor_profile is not None else -1
        # 切换会覆盖参数并作废标定, 必须二次确认
        ret = QMessageBox.question(
            self, "切换电机配置档",
            "切换电机配置档会覆盖当前电机参数，并作废标定 / 齿槽表。\n"
            "切换后必须重新跑「校准」(0x27)。\n\n"
            "确定切换到「%s」吗？" % self.cb_motor.currentText(),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret != QMessageBox.Yes:
            self.cb_motor.setCurrentIndex(prev)  # 不触发 activated, 安全
            return
        if not self.send(CMD["motor_profile"], float(idx)):
            self.cb_motor.setCurrentIndex(prev)
            return
        self._motor_req = idx
        self._update_calibration_button()
        self.statusbar.flash(
            "已请求切换电机配置档 → %s，等待读回"
            % self.cb_motor.currentText())
        QTimer.singleShot(600, self.request_config)

    def _apply_device_config(self, line):
        if not line.startswith("$FOCFG"):
            return
        body = line.split(",", 1)[1] if "," in line else ""
        kv = {}
        for part in body.split(","):
            if "=" in part:
                k, v = part.split("=", 1)
                try:
                    value = float(v)
                except ValueError:
                    continue
                if math.isfinite(value):
                    kv[k.strip().upper()] = value
        if not kv:
            return
        if self._motor_req is None or "PROF" in kv:
            self._cfg_wait_until = 0.0
        mapping = {"CURKP": "curkp", "CURKI": "curki", "CURBW": "curbw",
                   "VELKP": "velkp", "VELKI": "velki", "POSKP": "poskp",
                   "UV": "uv", "OV": "ov"}
        got = 0
        for k, sk in mapping.items():
            if k in kv and sk in self.left.steppers:
                self.left.steppers[sk].set_value(kv[k])
                got += 1
        if "CLIM" in kv:
            self._cur_lim = max(0.001, kv["CLIM"])
        if "VLIM" in kv:
            self._vel_lim = max(0.001, kv["VLIM"])
        if "CLIM" in kv or "VLIM" in kv:
            self._update_target_defs()
        self._save_cfg_later()
        # 标定得到的电机参数 / 齿槽表状态 (只在状态栏显示, 不写回输入框)
        extra = []
        if "R" in kv:
            extra.append("R=%.2fΩ" % kv["R"])
        if "L" in kv:
            extra.append("L=%.2fmH" % (kv["L"] * 1000.0))
        if "LAM" in kv:
            extra.append("λ=%.4fWb" % kv["LAM"])
        if "J" in kv and kv["J"] > 0:
            extra.append("J=%.2ge-5" % (kv["J"] * 1e5))
        if "FRIC" in kv and kv["FRIC"] > 0:
            extra.append("摩擦=%.0fmA" % (kv["FRIC"] * 1000.0))
        if "CAL" in kv:
            extra.append("标定%s" % ("有效" if kv["CAL"] > 0.5 else "无效"))
        if "PROF" in kv:
            profile = kv["PROF"]
            if profile.is_integer() and 0 <= profile < self.cb_motor.count():
                prof = int(profile)
                name = self.cb_motor.itemText(prof)
                if self._motor_req is not None and prof != self._motor_req:
                    extra.append(f"切换未确认，固件仍为 {name}")
                    self._motor_profile = None
                else:
                    extra.append(f"固件档位 {name}")
                    self._motor_profile = prof
            else:
                self._motor_profile = None
            self._motor_req = None
            self.cb_motor.setCurrentIndex(
                self._motor_profile if self._motor_profile is not None else -1
            )
            self._update_calibration_button()
        if "COG" in kv:
            extra.append("齿槽表%s" % ("有效" if kv["COG"] > 0.5 else "无效"))
        kvk = kv.get("CURKI")
        msg = "已读取设备参数: %d 项" % got
        if kvk is not None:
            msg += " (设备Ki=%g)" % kvk
        if extra:
            msg += " | " + " ".join(extra)
        self.statusbar.flash(msg)
        if self._last_fault and (self._last_fault & PHASE_OC_ISR):
            self._show_phase_trip(kv)

    def _show_phase_trip(self, kv):
        keys = ("OCIA", "OCIB", "OCIC", "OCLIM", "OCPROF", "OCFR", "OCVR")
        if not all(key in kv for key in keys):
            return
        ia, ib, ic, limit, profile, frame, voltage = (kv[key] for key in keys)
        name = self.cb_motor.itemText(int(profile)) or str(int(profile))
        hint = (
            f"跳闸当拍三相 {ia:.2f} / {ib:.2f} / {ic:.2f} A"
            f"（阈值 {limit:.2f} A）\n"
            f"档位 {name}，使能后第 {int(frame)} 个有效 ADC 样本\n"
            f"电阻标定电压请求 {voltage:.3f} V"
        )
        if "OCVL" in kv:
            hint += f"，电感交流幅值 {kv['OCVL']:.3f} V"
        self.lbl_fault.setToolTip(hint)

    def _update_calibration_button(self):
        confirmed = self._motor_profile is not None and self._motor_req is None
        ready = (self.io.is_open() and not self.io.link_lost and confirmed
                 and self._last_fault in (None, 0))
        self.btn_calib.setEnabled(ready)
        if not confirmed:
            tip = "等待读回固件的电机档位"
        elif self._last_fault not in (None, 0):
            tip = "清除设备故障后可开始校准"
        else:
            name = self.cb_motor.itemText(self._motor_profile)
            tip = f"开始校准 {name}，电机需可自由转动"
        self.btn_calib.setToolTip(tip)

    def _start_calibration(self):
        if self.btn_calib.isEnabled() and self.send(CMD["calib"]):
            self.statusbar.flash("已按固件读回档位发送校准请求")

    def enable(self):
        if self.send(CMD["enable"]):
            self._set_enable_requested(True)
            self.statusbar.flash("已发送使能请求")
        else:
            self.btn_enable.setChecked(self._enable_requested)

    def disable(self):
        if self.send(CMD["disable"]):
            self._set_enable_requested(False)
            self.statusbar.flash("已发送失能请求")
        else:
            self.btn_disable.setChecked(False)

    def clear_fault(self):
        if not self.send(CMD["disable"]):
            return
        self._set_enable_requested(False)
        if self.send(CMD["clear_fault"]):
            self.statusbar.flash("已请求清除故障，等待读回")

    def _set_enable_requested(self, on):
        self._enable_requested = on
        self.btn_enable.setChecked(on)
        self.btn_disable.setChecked(not on)
        self._show_fault()

    def _clear_enable(self):
        self._enable_requested = False
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
        self.sld_target.set_scale(step, 3)
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
            self.statusbar.flash("请输入有效目标值")
            return
        if not math.isfinite(v):
            self.statusbar.flash("请输入有限目标值")
            return
        step = self._target_step or 1.0
        v = min(max(v, self.sld_target.minimum() * step),
                self.sld_target.maximum() * step)
        iv = int(round(v / step))
        v = iv * step
        # 同步滑条位置 (不回触发发送)
        self.sld_target.blockSignals(True)
        self.sld_target.setValue(iv)
        self.sld_target.blockSignals(False)
        self.ent_target.setText(num(v, 3))
        self.send(CMD.get(self._target_key, CMD["velocity"]), v)

    def _on_target_jump(self, iv):
        self.ent_target.setText(num(iv * self._target_step, 3))
        self._send_target_now()

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

        def parameter_value(key):
            stepper = self.left.steppers.get(key)
            return "-" if stepper is None else stepper.value()

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
            "故障": "/".join(fault_texts(fault)) if fault else "无",
            "读数": "Iq=%s Iq_set=%s vel=%s vel_set=%s Vbus=%s" %
                    (rd("Iq"), rd("Iq_set"), rd("vel"), rd("vel_set"), rd("Vbus")),
            "参数": ("电流Kp=%s 电流Ki=%s 电流BW=%s "
                     "速度Kp=%s 速度Ki=%s 位置Kp=%s") % (
                parameter_value("curkp"), parameter_value("curki"),
                parameter_value("curbw"), parameter_value("velkp"),
                parameter_value("velki"), parameter_value("poskp"),
            ),
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

    # ================= 分析窗口 =================
    def open_spectrum(self):
        try:
            from .dialogs import SpectrumDialog
        except Exception as e:
            QMessageBox.critical(self, "频谱不可用", str(e))
            return
        d = getattr(self, "_spec_dlg", None)
        if d is not None:
            d.show()
            d.raise_()
            d.activateWindow()
            return
        self._spec_dlg = SpectrumDialog(self.store, self.model, self)
        self._spec_dlg.show()

    def open_vector(self):
        try:
            from .dialogs import VectorDialog
        except Exception as e:
            QMessageBox.critical(self, "矢量图不可用", str(e))
            return
        d = getattr(self, "_vec_dlg", None)
        if d is not None:
            d.show()
            d.raise_()
            d.activateWindow()
            return
        self._vec_dlg = VectorDialog(self.store, self.model, self)
        self._vec_dlg.show()

    def open_replay(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "打开波形记录", default_dir(), "CSV (*.csv);;所有文件 (*)")
        if not path:
            return
        try:
            from .analysis import load_csv
            fs, names, arr = load_csv(path)
        except Exception as e:
            QMessageBox.critical(self, "载入失败", str(e))
            return
        self.disconnect()
        self.demo = False
        self.store.clear()
        for row in arr:
            self.store.append(row[1:1 + NCH])
        if fs and fs > 0:
            self.fs = fs
            self.store.fs = fs
        dur = self.store.total / self.fs if self.fs > 0 else 0.0
        self.store.max_span = max(self.store.max_span, dur)
        self.store.cache_span = max(self.store.cache_span, dur)
        for p in (self.plot_top, self.plot_main):
            p.max_span = max(p.max_span, dur)
            p.t_span = min(max(dur, 0.1), p.max_span)
            p.follow = False
            p.t_right = dur
            p._auto_pending = True
        self._sync_plots()
        self.autoscale_all()
        self.statusbar.flash("已载入记录: %d 点 / %.2fs" % (self.store.total, dur))

    # ================= 主循环 =================
    def _tick(self):
        try:
            self._tick_impl()
        except Exception:
            import traceback
            traceback.print_exc()

    def _tick_impl(self):
        if self.io.link_lost and self.io.is_open():
            self.io.close()
            self._set_conn_ui(False)
            self.title.set_connected(False, "已断开 · 可重连")
            self.statusbar.flash("串口通信中断，已停止发送心跳")
        for line in self.io.poll_config():
            self._apply_device_config(line)
        if self._cfg_wait_until and time.monotonic() > self._cfg_wait_until:
            self._cfg_wait_until = 0.0
            if self._motor_req is not None:
                self._motor_req = None
                self._motor_profile = None
                self.cb_motor.setCurrentIndex(-1)
                self._update_calibration_button()
            self.statusbar.flash("未收到设备参数回应 (固件可能未烧录带读取命令的版本)")
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
            self._update_fault(frames[-1])
            self.plot_top.update()
            self.plot_main.update()
        self.overview.set_window(self.plot_main.t_left(), self.plot_main.t_span)

        vals = self.store.latest_values()
        self.right.update_values(vals)

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
        if fi is None or fi >= len(vals):
            return
        raw = vals[fi]
        if raw is None or not math.isfinite(raw) or raw < 0 or int(raw) != raw:
            return
        code = int(raw)
        if code == self._last_fault:
            return
        cleared = code == 0 and self._last_fault not in (None, 0)
        self._last_fault = code
        self.lbl_fault.setToolTip("")
        if code:
            self._clear_enable()
        self._show_fault()
        if (code & PHASE_OC_ISR) and self.io.is_open():
            self.request_config()
        if cleared:
            self.statusbar.flash("设备已清除故障")

    def _show_fault(self):
        code = self._last_fault
        active = code not in (None, 0)
        connected = self.io.is_open()
        self.btn_enable.setEnabled(connected and not active)
        self.btn_clear_fault.setEnabled(connected and active)
        self._update_calibration_button()
        if code is None:
            self.lbl_fault.setText("—")
        elif code == 0:
            self.lbl_fault.setText("无故障")
        else:
            # 多故障并发: 展开成文本, 用 / 连接 (如 "母线过压/母线欠压")
            self.lbl_fault.setText("/".join(fault_texts(code)))
        color = theme.RED if active else theme.GREEN if code == 0 else theme.FG_DIM
        self.lbl_fault.setStyleSheet(
            "color:%s;background:transparent;font-weight:600;" % color)
        # 状态灯 (故障码来自固件解析, 位掩码用 & 判断)
        w = self.left.status_widgets
        self._set_dot(w["enabled"],
                      theme.GREEN if self._enable_requested else theme.FG_FAINT)
        self._set_dot(w["ok"], theme.GREEN if code == 0 else theme.FG_FAINT)
        for key, _name, kind in STATUS_ITEMS:
            if isinstance(kind, int):
                self._set_dot(w[key], theme.RED if (code and (code & kind)) else theme.FG_FAINT)

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
        if self._is_max and self._normal_geo is not None:
            g = self._normal_geo
        cfg = {
            "window": [g.x(), g.y(), g.width(), g.height()],
            "max_span": self.max_span,
            "cache_span": self.cache_span,
            "fs": self.fs,
            "cur_limit": self._cur_lim,
            "vel_limit": self._vel_lim,
            "record_max_sec": self.record_max_sec,
            "split": self.split.sizes(),
            "channels": self.model.to_cfg(),
            "steppers": self.left.stepper_cfg(),
            "view": {
                "top": {"t_span": self.plot_top.t_span,
                        "y": [self.plot_top.y_min, self.plot_top.y_max]},
                "main": {"t_span": self.plot_main.t_span,
                         "y": [self.plot_main.y_min, self.plot_main.y_max]},
            },
        }
        try:
            with open(CFG_PATH, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=1)
        except Exception:
            pass

    def _apply_saved_view(self):
        """沿用上次保存的 X 轴缩放(显示时长)与 Y 轴缩放, 启动即生效."""
        v = self.cfg.get("view") or {}
        for key, plot in (("top", self.plot_top), ("main", self.plot_main)):
            d = v.get(key)
            if not isinstance(d, dict):
                continue
            try:
                ts = float(d.get("t_span"))
                if ts > 0:
                    plot.t_span = min(ts, plot.max_span)
            except (TypeError, ValueError):
                pass
            y = d.get("y")
            if isinstance(y, list) and len(y) == 2:
                try:
                    y0, y1 = float(y[0]), float(y[1])
                    if y1 > y0:
                        plot.y_min, plot.y_max = y0, y1
                except (TypeError, ValueError):
                    pass
            plot.follow = True
            plot._auto_pending = False
            plot.update()

    def _restore_geometry(self):
        w = self.cfg.get("window")
        if isinstance(w, list) and len(w) == 4:
            try:
                self.setGeometry(int(w[0]), int(w[1]), int(w[2]), int(w[3]))
                self._clamp_to_screen()
                return
            except Exception:
                pass
        self.resize(1360, 840)
        self._clamp_to_screen()

    def _clamp_to_screen(self):
        """把窗口限制在当前屏幕的工作区内 (不超出任务栏)."""
        g = self.geometry()
        scr = QGuiApplication.screenAt(g.center()) or QGuiApplication.primaryScreen()
        if scr is None:
            return
        a = scr.availableGeometry()
        w = min(g.width(), a.width())
        h = min(g.height(), a.height())
        x = min(max(g.x(), a.x()), a.x() + a.width() - w)
        y = min(max(g.y(), a.y()), a.y() + a.height() - h)
        if (x, y, w, h) != (g.x(), g.y(), g.width(), g.height()):
            self.setGeometry(x, y, w, h)

    # ================= 无边框缩放 =================
    def _toggle_max(self):
        if self._is_max:
            self.restore_from_max()
        else:
            self._normal_geo = self.geometry()
            self._is_max = True
            scr = self.screen()
            if scr is not None:
                self.setGeometry(scr.availableGeometry())   # 工作区, 不含任务栏
            self._outer.setContentsMargins(0, 0, 0, 0)
            self._root.setStyleSheet(
                "QFrame#Root{background:%s;border:none;border-radius:0;}" % theme.BG)

    def restore_from_max(self):
        if not self._is_max:
            return
        self._is_max = False
        self._outer.setContentsMargins(EDGE, EDGE, EDGE, EDGE)
        self._root.setStyleSheet(
            "QFrame#Root{background:%s;border:1px solid %s;border-radius:8px;}" %
            (theme.BG, theme.BORDER))
        if self._normal_geo is not None:
            self.setGeometry(self._normal_geo)
        self._clamp_to_screen()

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
        self.disconnect()
        super().closeEvent(ev)
