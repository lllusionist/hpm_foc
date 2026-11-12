# -*- coding: utf-8 -*-
"""临时电流阶跃测试上位机.

用法:
  1. 机械锁住转子;
  2. 运行本脚本, 选 COM 口 -> 连接;
  3. 点"使能(电流0)" (会先切电流模式、目标清零再使能), 一个连接内完成, 不用其他上位机;
  4. 输入阶跃电流(A), 点"阶跃一次";
  5. 固件以 40kHz 注入 Iq 阶跃并采集 16ms, 回传后本脚本画 Iq 并估算带宽;
  6. 测完点"禁用".

协议: 发送 5 字节 [0x60][float32 amp(LE)];
      固件回 "$STEP,FS=..,N=..,CH=3,PRE=..\n" + N*CH 个 float32(LE).
"""

import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    serial = None
    list_ports = None

CMD_STEP = 0x60
CMD_STEP_BASE = 0x61
CMD_SET_OP_MODE = 0x20
CMD_MOTOR_ENABLE = 0x21
CMD_MOTOR_DISABLE = 0x22
CMD_SET_TORQUE = 0x23
CMD_SET_VELOCITY = 0x24
CMD_SET_CUR_BW = 0x58
CMD_SAVE_CONFIG = 0x3F


class StepApp:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.busy = False
        root.title("FOC 电流阶跃测试 (临时)")
        root.geometry("900x560")

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

        ttk.Label(top, text="基值(A):").pack(side="left", padx=(16, 2))
        self.base_var = tk.StringVar(value="0.0")
        ttk.Entry(top, textvariable=self.base_var, width=6).pack(side="left")
        ttk.Label(top, text="阶跃Δ(A):").pack(side="left", padx=(8, 2))
        self.amp_var = tk.StringVar(value="1.0")
        ttk.Entry(top, textvariable=self.amp_var, width=8).pack(side="left")
        self.step_btn = ttk.Button(top, text="阶跃一次", command=self.on_step)
        self.step_btn.pack(side="left", padx=8)
        self.state_lbl = ttk.Label(top, text="未连接")
        self.state_lbl.pack(side="left", padx=8)

        row2 = ttk.Frame(root, padding=(8, 0, 8, 4))
        row2.pack(fill="x")
        ttk.Label(row2, text="电流环带宽(Hz):").pack(side="left")
        self.bw_var = tk.StringVar(value="1000")
        ttk.Entry(row2, textvariable=self.bw_var, width=8).pack(side="left", padx=4)
        ttk.Button(row2, text="设带宽", command=self.on_set_bw).pack(side="left")
        ttk.Button(row2, text="保存(先禁用)", command=self.on_save).pack(
            side="left", padx=4)
        ttk.Label(row2, text="(试 1500/1800 后重新阶跃; 满意后点保存)"
                  ).pack(side="left", padx=8)

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
            self.ser = serial.Serial(port, 115200, timeout=0.05)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.ser = None
            messagebox.showerror("打开失败", str(e))
            return
        self.conn_btn.configure(text="断开")
        self.state_lbl.configure(text="已连接 %s" % port)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.conn_btn.configure(text="连接")
        self.state_lbl.configure(text="未连接")

    # ---------- 使能/禁用 ----------
    def send_cmd(self, cid, value=None):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        # 固件按 5 字节一帧切分 (1B id + 4B float), 无参命令也必须补齐 5 字节,
        # 否则整包被丢弃
        payload = bytes([cid]) + struct.pack(
            "<f", 0.0 if value is None else float(value))
        self.ser.write(payload)
        return True

    def on_enable(self):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        # 先切电流模式、目标清零, 再使能; 避免位置/速度模式跟锁轴较劲
        self.send_cmd(CMD_SET_OP_MODE, 0.0)
        time.sleep(0.05)
        self.send_cmd(CMD_SET_TORQUE, 0.0)
        time.sleep(0.05)
        self.send_cmd(CMD_MOTOR_ENABLE)
        time.sleep(0.02)
        self.ser.reset_input_buffer()
        self.state_lbl.configure(text="已使能(电流模式 0A)")

    def on_disable(self):
        if self.send_cmd(CMD_MOTOR_DISABLE):
            self.state_lbl.configure(text="已禁用")

    def on_set_bw(self):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            bw = float(self.bw_var.get())
        except ValueError:
            messagebox.showwarning("提示", "带宽必须是数字")
            return
        self.send_cmd(CMD_SET_CUR_BW, bw)
        self.state_lbl.configure(text="电流环带宽设为 %.0f Hz (未保存)" % bw)

    def on_save(self):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        # 保存要求电机未使能: 先禁用, 等任务切到 IDLE, 再存
        self.send_cmd(CMD_MOTOR_DISABLE)
        time.sleep(0.3)
        self.send_cmd(CMD_SAVE_CONFIG)
        self.state_lbl.configure(text="已发送保存 (确认串口打印 cfg save: ok)")

    # ---------- 阶跃 ----------
    def on_step(self):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        if self.busy:
            return
        try:
            amp = float(self.amp_var.get())
            base = float(self.base_var.get())
        except ValueError:
            messagebox.showwarning("提示", "基值/阶跃必须是数字")
            return
        self.busy = True
        self.step_btn.configure(state="disabled")
        self.state_lbl.configure(text="采集中…")
        t = threading.Thread(target=self.worker, args=(amp, base), daemon=True)
        t.start()

    def worker(self, amp, base):
        try:
            self.ser.write(bytes([CMD_STEP_BASE]) + struct.pack("<f", base))
            time.sleep(0.02)
            self.ser.reset_input_buffer()
            self.ser.write(bytes([CMD_STEP]) + struct.pack("<f", amp))
            result = self.read_reply()
        except Exception as e:
            result = ("error", str(e))
        self.root.after(0, self.done, result)

    def read_reply(self):
        deadline = time.time() + 3.0
        buf = b""
        while time.time() < deadline:
            chunk = self.ser.read(512)
            if chunk:
                buf += chunk
                i = buf.find(b"$STEP,")
                if i >= 0:
                    j = buf.find(b"\n", i)
                    while j < 0 and time.time() < deadline:
                        buf += self.ser.read(512)
                        j = buf.find(b"\n", i)
                    if j < 0:
                        return ("error", "等不到头行结束")
                    header = buf[i:j].decode("ascii", "replace")
                    fs, n, ch, pre = self.parse_header(header)
                    need = n * ch * 4
                    data = buf[j + 1:]
                    while len(data) < need and time.time() < deadline:
                        data += self.ser.read(min(2048, need - len(data)))
                    if len(data) < need:
                        return ("error", "数据不完整 (%d/%d)" % (len(data), need))
                    vals = struct.unpack("<%df" % (n * ch), data[:need])
                    return ("ok", fs, n, ch, pre, vals)
            else:
                time.sleep(0.005)
        return ("error", "超时: 没收到 $STEP 数据 (电机使能了吗?)")

    @staticmethod
    def parse_header(line):
        fs, n, ch, pre = 40000.0, 640, 3, 40
        for kv in line[len("$STEP,"):].split(","):
            if "=" not in kv:
                continue
            k, v = kv.split("=", 1)
            k = k.strip().upper()
            try:
                if k == "FS":
                    fs = float(v)
                elif k == "N":
                    n = int(v)
                elif k == "CH":
                    ch = int(v)
                elif k == "PRE":
                    pre = int(v)
            except ValueError:
                pass
        return fs, n, ch, pre

    def done(self, result):
        self.busy = False
        self.step_btn.configure(state="normal")
        if result[0] == "error":
            self.state_lbl.configure(text="失败")
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
        tail = max(1, n // 5)
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
            i10 = self.first_cross(iq, lo, delta > 0)
            i90 = self.first_cross(iq, hi, delta > 0)
            if i10 is not None and i90 is not None and i90 > i10:
                tr = (i90 - i10) / fs
                bw = 0.35 / tr if tr > 0 else 0.0
                lines.append(
                    "上升时间 tr(10-90%%) = %.1f us   估算带宽 ≈ %.0f Hz"
                    % (tr * 1e6, bw))
            peak = max(iq) if delta > 0 else min(iq)
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
