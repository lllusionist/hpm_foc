# -*- coding: utf-8 -*-
import math
import os
import struct
import tempfile
import unittest

from focscope import analysis


class TestFFT(unittest.TestCase):
    def test_tone(self):
        fs = 1000.0
        f0 = 50.0
        n = 2000
        x = [math.sin(2 * math.pi * f0 * i / fs) for i in range(n)]
        fr, amp = analysis.fft_magnitude(x, fs)
        self.assertIsNotNone(fr)
        k = int(max(range(len(amp)), key=lambda i: amp[i]))
        self.assertAlmostEqual(fr[k], f0, delta=2.0)
        self.assertAlmostEqual(float(amp[k]), 1.0, delta=0.15)


class TestClarkePark(unittest.TestCase):
    def test_balanced(self):
        # 幅值不变的平衡三相 + 对应电角度 -> dq 常数
        amp = 2.0
        for th in (0.3, 1.1, 2.7):
            ia = amp * math.cos(th)
            ib = amp * math.cos(th - 2.0943951)
            ic = amp * math.cos(th + 2.0943951)
            al, be = analysis.clarke([ia], [ib], [ic])
            d, q = analysis.park(al, be, [th])
            self.assertAlmostEqual(float(d[0]), amp, delta=1e-3)
            self.assertAlmostEqual(float(q[0]), 0.0, delta=1e-3)


class TestCsv(unittest.TestCase):
    def test_load(self):
        txt = (
            "# FOC Scope 波形记录\n"
            "# 采样率: 5000 Hz\n"
            "# 通道: 0=A相电流(Ia/A)\n"
            "t_s,Ia,Ib,Ic\n"
            "0.000,1.0,2.0,3.0\n"
            "0.001,4.0,5.0,6.0\n"
        )
        fd, path = tempfile.mkstemp(suffix=".csv")
        os.close(fd)
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(txt)
            fs, names, arr = analysis.load_csv(path)
            self.assertEqual(fs, 5000.0)
            self.assertEqual(names, ["t_s", "Ia", "Ib", "Ic"])
            self.assertEqual(arr.shape, (2, 4))
            self.assertAlmostEqual(float(arr[1, 3]), 6.0)
        finally:
            os.remove(path)


if __name__ == "__main__":
    unittest.main()
