# -*- coding: utf-8 -*-
import struct
import unittest

from focscope.protocol import CMD, TAIL, NCH, pack_cmd, JustfloatParser


class TestPackCmd(unittest.TestCase):
    def test_len_and_id(self):
        b = pack_cmd(CMD["velocity"])
        self.assertEqual(len(b), 5)
        self.assertEqual(b[0], 0x24)
        self.assertEqual(b[1:5], b"\x00\x00\x00\x00")

    def test_value_roundtrip(self):
        b = pack_cmd(CMD["torque"], 1.5)
        self.assertEqual(struct.unpack("<f", b[1:5])[0], 1.5)


class TestJustfloatParser(unittest.TestCase):
    def _frame(self, *vals):
        return struct.pack("<%df" % len(vals), *vals) + TAIL

    def test_single_frame(self):
        p = JustfloatParser()
        out = p.feed(self._frame(1.0, 2.0, 3.0))
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0], (1.0, 2.0, 3.0))

    def test_multiple_frames_one_feed(self):
        p = JustfloatParser()
        out = p.feed(self._frame(1.0, 2.0) + self._frame(3.0, 4.0))
        self.assertEqual(out, [(1.0, 2.0), (3.0, 4.0)])

    def test_split_across_feeds(self):
        p = JustfloatParser()
        data = self._frame(1.0, 2.0, 3.0)
        self.assertEqual(p.feed(data[:5]), [])
        out = p.feed(data[5:])
        self.assertEqual(out, [(1.0, 2.0, 3.0)])

    def test_nch_constant(self):
        self.assertEqual(NCH, 22)


if __name__ == "__main__":
    unittest.main()
