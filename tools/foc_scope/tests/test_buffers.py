# -*- coding: utf-8 -*-
import math
import unittest

from focscope.buffers import DataStore
from focscope.channels import NCH


class TestDataStore(unittest.TestCase):
    def test_append_count_total(self):
        s = DataStore(fs=1000.0, max_span=100.0)
        s.append(tuple(range(NCH)))
        s.append(tuple(range(NCH)))
        self.assertEqual(s.count(), 2)
        self.assertEqual(s.total, 2)
        self.assertEqual(s.base, 0)

    def test_latest_and_sample_at(self):
        s = DataStore(fs=1000.0, max_span=100.0)
        for k in range(5):
            s.append((float(k),) + (0.0,) * (NCH - 1))
        self.assertEqual(s.latest(0), 4.0)
        self.assertEqual(s.sample_at(0, 2), 2.0)
        self.assertIsNone(s.sample_at(0, 99))

    def test_trim_advances_base(self):
        s = DataStore(fs=1000.0, max_span=0.1)   # keep = 100
        for k in range(1000):
            s.append((float(k),) + (0.0,) * (NCH - 1))
        self.assertGreater(s.base, 0)
        self.assertLessEqual(s.count(), 250)

    def test_trim_uses_cache_span(self):
        s = DataStore(fs=1000.0, max_span=1.0, cache_span=0.05)   # keep = 50
        for k in range(500):
            s.append((float(k),) + (0.0,) * (NCH - 1))
        self.assertGreater(s.base, 0)
        self.assertLess(s.count(), 200)

    def test_raw_slice(self):
        s = DataStore(fs=1000.0, max_span=100.0)
        for k in range(10):
            s.append((float(k),) + (0.0,) * (NCH - 1))
        k0, vals = s.raw(0, 2, 6)
        self.assertEqual(k0, 2)
        self.assertEqual(vals, [2.0, 3.0, 4.0, 5.0])

    def test_envelope_minmax(self):
        s = DataStore(fs=1000.0, max_span=100.0)
        for k in range(1000):
            s.append((math.sin(2 * math.pi * k / 100.0),) + (0.0,) * (NCH - 1))
        cols = s.envelope(0, 0, 1000, 50)
        self.assertEqual(len(cols), 50)
        lo = min(c[1] for c in cols)
        hi = max(c[3] for c in cols)
        self.assertLess(lo, -0.95)
        self.assertGreater(hi, 0.95)

    def test_k_range_empty(self):
        s = DataStore(fs=1000.0, max_span=10.0)
        k0, k1 = s.k_range(0.0, 1.0)
        self.assertEqual(k0, k1)


if __name__ == "__main__":
    unittest.main()
