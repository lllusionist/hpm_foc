# -*- coding: utf-8 -*-
import struct
import unittest
from unittest.mock import Mock, patch

from focscope.protocol import (
    CMD, TAIL, NCH, JustfloatParser, pack_cmd, read_step_reply,
)


class TestPackCmd(unittest.TestCase):
    def test_len_and_id(self):
        b = pack_cmd(CMD["velocity"])
        self.assertEqual(len(b), 5)
        self.assertEqual(b[0], 0x24)
        self.assertEqual(b[1:5], b"\x00\x00\x00\x00")

    def test_value_roundtrip(self):
        b = pack_cmd(CMD["torque"], 1.5)
        self.assertEqual(struct.unpack("<f", b[1:5])[0], 1.5)

    def test_heartbeat_frame(self):
        self.assertEqual(pack_cmd(CMD["heartbeat"]), b"\x29\x00\x00\x00\x00")


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


class TestStepReply(unittest.TestCase):
    def test_split_header_and_payload_after_waveform_bytes(self):
        samples = (0.0, 0.5, 1.0, 0.4, 0.5, 1.1)
        payload = struct.pack("<6f", *samples)
        port = Mock()
        port.read.side_effect = [
            b"\x00\x00\x80\x7f$ST",
            b"EP,FS=40000,N=2,CH=3,",
            b"PRE=1\n" + payload[:6],
            payload[6:],
        ]
        fs, n, ch, pre, data = read_step_reply(port, lambda: False)
        self.assertEqual((fs, n, ch, pre), (40000, 2, 3, 1))
        self.assertEqual(data, struct.unpack("<6f", payload))

    def test_rejects_invalid_header_and_nonfinite_samples(self):
        port = Mock()
        port.read.return_value = b"$STEP,FS=40000,N=2,CH=3,PRE=2\n"
        with self.assertRaises(ValueError):
            read_step_reply(port, lambda: False)
        port.read.return_value = (b"$STEP,FS=40000,N=2,CH=3,PRE=1\n"
                                  + struct.pack("<6f", 0, 0, 0, 0, 0, float("nan")))
        with self.assertRaises(ValueError):
            read_step_reply(port, lambda: False)

    def test_times_out_on_truncated_payload(self):
        port = Mock()
        port.read.return_value = b"$STEP,FS=40000,N=2,CH=3,PRE=1\n\x00"
        with patch("focscope.protocol.time.monotonic", side_effect=[0, 0, 4]):
            with self.assertRaisesRegex(TimeoutError, "1/24"):
                read_step_reply(port, lambda: False)

    def test_cancelled_before_read(self):
        port = Mock()
        with self.assertRaises(InterruptedError):
            read_step_reply(port, lambda: True)
        port.read.assert_not_called()


if __name__ == "__main__":
    unittest.main()
