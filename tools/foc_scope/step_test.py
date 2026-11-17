# -*- coding: utf-8 -*-
"""临时电流阶跃测试上位机.

用法:
  1. 机械锁住转子;
  2. 运行本脚本, 选 COM 口 -> 连接;
  3. 点"使能(电流0)" (会先切电流模式、目标清零再使能), 一个连接内完成, 不用其他上位机;
  4. 输入阶跃电流(A), 点"阶跃一次";
  5. 固件以 40kHz 注入 Iq 阶跃并回传采样，本脚本画 Iq 并估算带宽;
  6. 测完点"禁用".

协议: 发送 5 字节 [0x60][float32 amp(LE)];
      固件回 "$STEP,FS=..,N=..,CH=3,PRE=..\n" + N*CH 个 float32(LE).
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox

from focscope.protocol import (
    CMD, FLOAT32_MAX, HEARTBEAT_INTERVAL_MS, pack_cmd, read_step_reply,
)

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    serial = None
    list_ports = None


class StepApp:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self._write_lock = threading.Lock()
        self._results = queue.Queue()
        self._capture = None
        self._command_job = None
        self._enable_requested = False
        root.title("FOC 电流阶跃测试 (临时)")
        root.geometry("900x560")
        root.protocol("WM_DELETE_WINDOW", self._exit)

        top = ttk.Frame(root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="串口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=12)
        self.port_cb.pack(side="left", padx=4)
        self.refresh_btn = ttk.Button(top, text="刷新", command=self.refresh_ports)
        self.refresh_btn.pack(side="left")
        self.conn_btn = ttk.Button(top, text="连接", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        self.en_btn = ttk.Button(top, text="使能(电流0)",
                                 command=self.on_enable)
        self.en_btn.pack(side="left", padx=2)
        self.dis_btn = ttk.Button(top, text="禁用", command=self.on_disable)
        self.dis_btn.pack(side="left", padx=2)
        self.clear_btn = ttk.Button(top, text="清除故障", command=self.on_clear)
        self.clear_btn.pack(side="left", padx=2)

        ttk.Label(top, text="基值(A):").pack(side="left", padx=(16, 2))
        self.base_var = tk.StringVar(value="0.0")
        ttk.Entry(top, textvariable=self.base_var, width=6).pack(side="left")
        ttk.Label(top, text="阶跃Δ(A):").pack(side="left", padx=(8, 2))
        self.amp_var = tk.StringVar(value="1.0")
        ttk.Entry(top, textvariable=self.amp_var, width=8).pack(side="left")
        self.step_btn = ttk.Button(top, text="阶跃一次", command=self.on_step)
        self.step_btn.pack(side="left", padx=8)

        row2 = ttk.Frame(root, padding=(8, 0, 8, 4))
        row2.pack(fill="x")
        ttk.Label(row2, text="电流环带宽(Hz):").pack(side="left")
        self.bw_var = tk.StringVar(value="1000")
        ttk.Entry(row2, textvariable=self.bw_var, width=8).pack(side="left", padx=4)
        self.bw_btn = ttk.Button(row2, text="设带宽", command=self.on_set_bw)
        self.bw_btn.pack(side="left")
        self.save_btn = ttk.Button(row2, text="保存(先禁用)", command=self.on_save)
        self.save_btn.pack(side="left", padx=4)
        self.state_lbl = ttk.Label(row2, text="未连接")
        self.state_lbl.pack(side="left", padx=8)

        self.canvas = tk.Canvas(root, bg="white", height=380,
                                highlightthickness=1,
                                highlightbackground="#999")
        self.canvas.pack(fill="both", expand=True, padx=8, pady=6)
        self.canvas.bind("<Configure>", lambda e: self.redraw())

        self.info = ttk.Label(root, text="等待数据……", justify="left",
                              font=("Consolas", 11))
        self.info.pack(fill="x", padx=8, pady=(0, 8))

        self.last = None
        self.refresh_ports()
        self._sync_controls()
        root.after(HEARTBEAT_INTERVAL_MS, self._poll_connection)

    # ---------- 串口 ----------
    def refresh_ports(self):
        ports = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.ser is not None:
            self.close()
            return
        if serial is None:
            messagebox.showerror("错误", "未安装 pyserial: pip install pyserial")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05,
                                     write_timeout=0.05)
            self.ser.reset_input_buffer()
        except OSError as e:
            self.close(request_stop=False)
            messagebox.showerror("打开失败", str(e))
            return
        if not self.send_cmd(CMD["disable"]):
            return
        self.conn_btn.configure(text="断开")
        self.state_lbl.configure(text="已连接 %s" % port)
        self._sync_controls()

    def close(self, request_stop=True):
        self._cancel_command()
        self._cancel_capture()
        if request_stop and self.ser is not None:
            try:
                self._write_cmd(self.ser, CMD["disable"])
            except OSError:
                pass
        with self._write_lock:
            port, self.ser = self.ser, None
            if port is not None:
                try:
                    port.close()
                except OSError:
                    pass
        self._enable_requested = False
        self.conn_btn.configure(text="连接")
        self.state_lbl.configure(text="未连接")
        self._sync_controls()

    def _exit(self):
        self.close()
        self.root.destroy()

    def _sync_controls(self):
        connected = self.ser is not None
        for button in (self.dis_btn, self.clear_btn):
            button.configure(state="normal" if connected else "disabled")
        ready = connected and self._capture is None and self._command_job is None
        for button in (self.en_btn, self.bw_btn, self.save_btn):
            button.configure(state="normal" if ready else "disabled")
        self.step_btn.configure(
            state="normal" if ready and self._enable_requested else "disabled"
        )

    def _poll_connection(self):
        if self.ser is not None:
            try:
                self._write_cmd(self.ser, CMD["heartbeat"])
            except OSError as exc:
                self._link_failed(exc)
        try:
            capture, result = self._results.get_nowait()
        except queue.Empty:
            pass
        else:
            self._capture = None
            if not capture.is_set():
                self.done(result)
            self._sync_controls()
        self.root.after(HEARTBEAT_INTERVAL_MS, self._poll_connection)

    def _link_failed(self, exc):
        self.close(request_stop=False)
        self.state_lbl.configure(text="串口通信中断")
        self.info.configure(text=str(exc))

    # ---------- 使能/禁用 ----------
    def _write_cmd(self, ser, cid, value=None, capture=None):
        payload = pack_cmd(cid, value)
        with self._write_lock:
            if self.ser is not ser or (capture is not None and capture.is_set()):
                return False
            if ser.write(payload) != len(payload):
                raise OSError("串口命令帧发送不完整")
            return True

    def send_cmd(self, cid, value=None):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        try:
            return self._write_cmd(self.ser, cid, value)
        except OSError as exc:
            self._link_failed(exc)
            return False

    def _cancel_command(self):
        if self._command_job is not None:
            self.root.after_cancel(self._command_job)
            self._command_job = None

    def _cancel_capture(self):
        if self._capture is not None:
            self._capture.set()

    def on_enable(self):
        if self.send_cmd(CMD["opmode"], 0.0):
            self._command_job = self.root.after(50, self._finish_enable)
            self._sync_controls()

    def _finish_enable(self):
        self._command_job = None
        if self.send_cmd(CMD["torque"], 0.0) and self.send_cmd(CMD["enable"]):
            self._enable_requested = True
            self.state_lbl.configure(text="已请求使能（电流模式 0 A）")
        self._sync_controls()

    def on_disable(self):
        self._cancel_command()
        self._cancel_capture()
        self._enable_requested = False
        self._sync_controls()
        if self.send_cmd(CMD["disable"]):
            self.state_lbl.configure(text="已发送失能请求")
            return True
        return False

    def on_clear(self):
        if self.on_disable() and self.send_cmd(CMD["clear_fault"]):
            self.state_lbl.configure(text="已发送清除故障请求")

    def on_set_bw(self):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            bw = float(self.bw_var.get())
        except ValueError:
            messagebox.showwarning("提示", "带宽必须是数字")
            return
        if not math.isfinite(bw) or not 50 <= bw <= 2000:
            messagebox.showwarning("提示", "电流环带宽范围为 50–2000 Hz")
            return
        if self.send_cmd(CMD["curbw"], bw):
            self.state_lbl.configure(text="已发送带宽 %.0f Hz（未保存）" % bw)

    def on_save(self):
        if self.on_disable():
            self._command_job = self.root.after(300, self._finish_save)
            self._sync_controls()

    def _finish_save(self):
        self._command_job = None
        if self.send_cmd(CMD["save"]):
            self.state_lbl.configure(text="已发送保存请求，结果见固件日志")
        self._sync_controls()

    # ---------- 阶跃 ----------
    def on_step(self):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        if self._capture is not None or not self._enable_requested:
            return
        try:
            amp = float(self.amp_var.get())
            base = float(self.base_var.get())
        except ValueError:
            messagebox.showwarning("提示", "基值/阶跃必须是数字")
            return
        if not all(-FLOAT32_MAX <= v <= FLOAT32_MAX for v in (amp, base)):
            messagebox.showwarning("提示", "请输入可传输的有限电流值")
            return
        self._capture = threading.Event()
        self._sync_controls()
        self.state_lbl.configure(text="采集中…")
        t = threading.Thread(target=self.worker,
                             args=(self.ser, self._capture, amp, base),
                             daemon=True)
        t.start()

    def worker(self, ser, capture, amp, base):
        try:
            if not self._write_cmd(ser, CMD["step_base"], base, capture=capture):
                raise OSError("发送阶跃基值失败")
            if capture.wait(0.02):
                raise InterruptedError("阶跃已取消")
            ser.reset_input_buffer()
            if not self._write_cmd(ser, CMD["step"], amp, capture=capture):
                raise OSError("发送阶跃命令失败")
            result = ("ok", *read_step_reply(ser, capture.is_set))
        except (OSError, ValueError) as exc:
            result = ("error", str(exc))
        self._results.put((capture, result))

    def done(self, result):
        if result[0] == "error":
            if self.on_disable():
                self.state_lbl.configure(text="采集失败，已发送失能请求")
            self.info.configure(text="错误: " + result[1])
            return
        _, fs, n, ch, pre, vals = result
        iq = list(vals[0::ch])
        ref = list(vals[1::ch])
        self.state_lbl.configure(text="完成 (%d 点 @ %.0fHz)" % (n, fs))
        self.last = (fs, n, pre, iq, ref)
        self.analyze()
        self.redraw()

    def analyze(self):
        fs, n, pre, iq, ref = self.last
        # 基值取预置段最后 1/4 (前面是从 0 爬到基值的过渡)
        p0 = max(0, pre - max(20, pre // 4))
        seg = iq[p0:pre]
        base = sum(seg) / max(1, len(seg))
        tail = max(1, (n - pre) // 5)
        final = sum(iq[-tail:]) / tail
        ref_final = sum(ref[-tail:]) / tail
        delta = final - base
        lines = [
            "基线 Iq = %.3f A   阶跃后 Iq(末段均值) = %.3f A   目标 = %.3f A   跟随比 = %.1f%%"
            % (base, final, ref_final, (100.0 * final / ref_final) if abs(ref_final) > 1e-6 else 0.0),
        ]
        if abs(delta) > 1e-4:
            lo = base + 0.1 * delta
            hi = base + 0.9 * delta
            i10 = self.first_cross(iq[pre:], lo, delta > 0)
            i90 = self.first_cross(iq[pre:], hi, delta > 0)
            if i10 is not None and i90 is not None and i90 > i10:
                tr = (i90 - i10) / fs
                bw = 0.35 / tr if tr > 0 else 0.0
                lines.append(
                    "上升时间 tr(10-90%%) = %.1f us   估算带宽 ≈ %.0f Hz"
                    % (tr * 1e6, bw))
            peak = max(iq[pre:]) if delta > 0 else min(iq[pre:])
            over = (peak - final) / delta * 100.0
            lines.append("峰值 = %.3f A   超调 = %.1f%%" % (peak, over))
        else:
            lines.append("Iq 几乎没变化: 检查电机是否使能/锁轴、电流环增益、阶跃幅度")
        self.info.configure(text="\n".join(lines))

    @staticmethod
    def first_cross(v, level, rising):
        for i, x in enumerate(v):
            if (rising and x >= level) or ((not rising) and x <= level):
                return i
        return None

    # ---------- 画图 ----------
    def redraw(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return
        if not self.last:
            c.create_text(w / 2, h / 2, text="等待阶跃数据…", fill="#888")
            return
        fs, n, pre, iq, ref = self.last
        ml, mr, mt, mb = 60, 16, 16, 40
        pw, ph = w - ml - mr, h - mt - mb

        ymin = min(min(iq), min(ref))
        ymax = max(max(iq), max(ref))
        if ymax - ymin < 1e-3:
            ymax, ymin = ymin + 0.1, ymin - 0.1
        pad = (ymax - ymin) * 0.1
        ymin -= pad
        ymax += pad

        def X(i):
            return ml + pw * i / max(1, n - 1)

        def Y(v):
            return mt + ph * (ymax - v) / (ymax - ymin)

        # 坐标轴
        c.create_line(ml, mt, ml, mt + ph, fill="#333")
        c.create_line(ml, mt + ph, ml + pw, mt + ph, fill="#333")
        for k in range(5):
            v = ymin + (ymax - ymin) * k / 4.0
            y = Y(v)
            c.create_line(ml - 4, y, ml, y, fill="#333")
            c.create_text(ml - 8, y, text="%.2f" % v, anchor="e", fill="#333")
        # 时间刻度 (ms)
        total_ms = n / fs * 1000.0
        for k in range(5):
            i = (n - 1) * k / 4.0
            x = X(i)
            c.create_line(x, mt + ph, x, mt + ph + 4, fill="#333")
            c.create_text(x, mt + ph + 8, text="%.2f" % (i / fs * 1000.0),
                          anchor="n", fill="#333")
        c.create_text(ml + pw / 2, h - 6, text="时间 (ms)  总长 %.2f ms" % total_ms)
        c.create_text(14, mt + ph / 2, text="I (A)", angle=90)

        # 阶跃位置
        xs = X(pre)
        c.create_line(xs, mt, xs, mt + ph, fill="#ccc", dash=(4, 3))

        def draw(vals, color, width):
            pts = []
            for i, v in enumerate(vals):
                pts += [X(i), Y(v)]
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=width)

        draw(ref, "#e08a00", 1)
        draw(iq, "#1565c0", 2)
        # 图例
        c.create_line(ml + pw - 150, mt + 8, ml + pw - 120, mt + 8,
                      fill="#1565c0", width=2)
        c.create_text(ml + pw - 115, mt + 8, text="Iq(实测)", anchor="w")
        c.create_line(ml + pw - 150, mt + 24, ml + pw - 120, mt + 24,
                      fill="#e08a00", width=1)
        c.create_text(ml + pw - 115, mt + 24, text="Iq(目标)", anchor="w")


def main():
    root = tk.Tk()
    StepApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
