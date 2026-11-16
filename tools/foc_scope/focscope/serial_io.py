# -*- coding: utf-8 -*-
"""串口收发: 后台线程读取 -> Justfloat 解析 -> 队列; 主线程 poll 取走."""
import queue
import threading
import time

from .protocol import JustfloatParser, pack_cmd, CFG_MAGIC

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False


class SerialIO:
    def __init__(self, baud=115200):
        self.baud = baud
        self.ser = None
        self.thread = None
        self.running = False
        self.q = queue.Queue(maxsize=20000)
        self.cfg_q = queue.Queue()
        self._lock = threading.Lock()
        self._parser = JustfloatParser()
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.frames = 0
        self.dropped = 0
        self.link_lost = False      # 设备断开/复位导致读失败

    # ---------- 端口 ----------
    @staticmethod
    def ports_info():
        """返回端口信息列表, 已按 USB串口 -> 其它 -> 调试器 -> 蓝牙 排序."""
        if not HAVE_SERIAL:
            return []
        out = []
        for p in list_ports.comports():
            desc = (p.description or "").strip()
            hwid = (p.hwid or "").strip()
            s = (desc + " " + hwid).lower()
            if any(k in s for k in ("bluetooth", "bth", "蓝牙")):
                kind = "bt"
            elif any(k in s for k in ("jlink", "j-link", "stlink", "st-link",
                                      "segger", "cmsis", "dap")):
                kind = "debug"
            elif ("usb" in s or "串行" in s) and \
                 ("serial" in s or "串行" in s or "uart" in s or "cdc" in s):
                kind = "usb"
            elif "usb" in s:
                kind = "usb"
            else:
                kind = "other"
            out.append({"device": p.device, "desc": desc or "未知设备",
                        "hwid": hwid, "kind": kind})
        order = {"usb": 0, "other": 1, "debug": 2, "bt": 3}
        out.sort(key=lambda d: (order[d["kind"]], d["device"]))
        return out

    @staticmethod
    def ports():
        return [d["device"] for d in SerialIO.ports_info()]

    # ---------- 打开/关闭 ----------
    def open(self, dev):
        if not HAVE_SERIAL:
            raise RuntimeError("未安装 pyserial: pip install pyserial")
        self.close()
        self.link_lost = False
        last = None
        for _ in range(4):
            try:
                self.ser = serial.Serial(dev, self.baud, timeout=0.05,
                                         write_timeout=0.05)
                break
            except Exception as e:
                last = e
                time.sleep(0.15)
        if self.ser is None:
            raise last if last else RuntimeError("打开串口失败")
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if self.thread:
            try:
                self.thread.join(timeout=0.3)
            except Exception:
                pass
            self.thread = None

    def is_open(self):
        return self.ser is not None

    # ---------- 发送 ----------
    def send(self, cid, val=None):
        frame = pack_cmd(cid, val)
        return self.write(frame)

    def write(self, data):
        if not self.ser:
            return False
        try:
            with self._lock:
                self.ser.write(data)
            self.tx_bytes += len(data)
            return True
        except Exception:
            return False

    # ---------- 接收 ----------
    def _feed(self, data):
        for vals in self._parser.feed(data):
            try:
                self.q.put_nowait(vals)
            except queue.Full:
                self.dropped += 1

    def _ingest(self, buf, data):
        """并入新数据, 先剥离 $FOCFG 配置行, 其余按 Justfloat 解析."""
        buf += data
        m = len(CFG_MAGIC)
        while True:
            idx = buf.find(CFG_MAGIC)
            if idx < 0:
                # 仅在末尾可能是 magic 前缀时保留少量字节, 其余立即解析
                hold = 0
                for k in range(min(m - 1, len(buf)), 0, -1):
                    if buf[-k:] == CFG_MAGIC[:k]:
                        hold = k
                        break
                if hold:
                    self._feed(bytes(buf[:-hold]))
                    del buf[-hold:]
                else:
                    if buf:
                        self._feed(bytes(buf))
                        del buf[:]
                break
            if idx > 0:
                self._feed(bytes(buf[:idx]))
                del buf[:idx]
            end = buf.find(b"\n")
            if end < 0:
                break
            line = bytes(buf[:end]).decode("ascii", "ignore").strip()
            del buf[:end + 1]
            if line:
                self.cfg_q.put(line)

    def _reader(self):
        self._parser.reset()
        buf = bytearray()
        while self.running and self.ser:
            try:
                data = self.ser.read(4096)
            except Exception:
                self.link_lost = True
                break
            if not data:
                continue
            self.rx_bytes += len(data)
            self._ingest(buf, data)

    def poll_config(self):
        out = []
        while True:
            try:
                out.append(self.cfg_q.get_nowait())
            except queue.Empty:
                break
        return out

    def poll(self, maxn=20000):
        out = []
        while len(out) < maxn:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                break
        self.frames += len(out)
        return out

    def stats(self):
        return {
            "rx": self.rx_bytes,
            "tx": self.tx_bytes,
            "frames": self.frames,
            "dropped": self.dropped,
        }
