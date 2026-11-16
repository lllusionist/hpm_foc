import unittest
from unittest.mock import Mock

from focscope.protocol import CMD, pack_cmd
from focscope.serial_io import SerialIO


class TestSerialIO(unittest.TestCase):
    def test_complete_write_updates_tx_count(self):
        io = SerialIO()
        io.ser = Mock()
        frame = pack_cmd(CMD["heartbeat"])
        io.ser.write.return_value = len(frame)

        self.assertTrue(io.send(CMD["heartbeat"]))
        io.ser.write.assert_called_once_with(frame)
        self.assertEqual(io.tx_bytes, len(frame))
        self.assertFalse(io.link_lost)

    def test_short_write_marks_link_lost_and_stops_sending(self):
        io = SerialIO()
        io.ser = Mock()
        io.ser.write.return_value = 2

        self.assertFalse(io.send(CMD["enable"]))
        self.assertTrue(io.link_lost)
        self.assertFalse(io.send(CMD["heartbeat"]))
        self.assertEqual(io.tx_bytes, 2)
        self.assertEqual(io.ser.write.call_count, 1)

    def test_close_drops_pending_frames_and_configuration(self):
        io = SerialIO()
        port = io.ser = Mock()
        io.q.put((1.0, 2.0))
        io.cfg_q.put("$FOCFG,PROF=2")

        io.close()

        port.close.assert_called_once_with()
        self.assertFalse(io.is_open())
        self.assertEqual(io.poll(), [])
        self.assertEqual(io.poll_config(), [])

    def test_late_reader_error_does_not_close_the_new_port(self):
        io = SerialIO()
        old_port = io.ser = Mock()
        new_port = Mock()

        def old_port_read(_size):
            io.ser = new_port
            raise OSError("previous port closed")

        old_port.read.side_effect = old_port_read
        io._reader(old_port)

        self.assertFalse(io.link_lost)
        self.assertIs(io.ser, new_port)


if __name__ == "__main__":
    unittest.main()
