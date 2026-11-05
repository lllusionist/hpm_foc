# -*- coding: utf-8 -*-
import unittest

from focscope.channels import ChannelModel, NCH, CHANNELS, FAULT_TEXT


class TestChannelModel(unittest.TestCase):
    def test_default_visibility(self):
        m = ChannelModel()
        self.assertFalse(m.shown[0])
        self.assertTrue(m.shown[6] and m.shown[7])
        self.assertEqual(m.shown_channels(), [6, 7])
        self.assertEqual(m.TOP_CHANNELS, (0, 1, 2))

    def test_value_gain_offset(self):
        m = ChannelModel()
        m.gain[0], m.off[0] = 2.0, 1.0
        self.assertEqual(m.value(0, 3.0), 7.0)

    def test_cfg_roundtrip(self):
        m = ChannelModel()
        m.disp[0] = "自定义"
        m.gain[5] = 12.5
        m.off[5] = -1.0
        m.shown[4] = True
        m2 = ChannelModel()
        m2.load_cfg(m.to_cfg())
        self.assertEqual(m2.disp[0], "自定义")
        self.assertEqual(m2.gain[5], 12.5)
        self.assertEqual(m2.off[5], -1.0)
        self.assertTrue(m2.shown[4])

    def test_firmware_order(self):
        self.assertEqual(NCH, len(CHANNELS))
        self.assertEqual(CHANNELS[0][0], "Ia")
        self.assertIn("母线电压", CHANNELS[9][1])
        self.assertIn("母线电流", CHANNELS[21][1])
        self.assertEqual(FAULT_TEXT[0], "无")

    def test_names_not_generic(self):
        """每个通道都要有可读短名和显示名, 不能退化成 dataN/dN."""
        import re

        self.assertEqual(NCH, 27)  # 22 个固件通道 + 5 个模型/观测通道
        for i, chan in enumerate(CHANNELS):
            short, disp = chan[0], chan[1]
            self.assertFalse(re.fullmatch(r"(data|ch|d)\d+", short),
                             "channel %d 短名为通用占位: %r" % (i, short))
            self.assertTrue(disp, "channel %d 没有显示名" % i)
        self.assertEqual(CHANNELS[22][0], "smo_rpm")
        self.assertEqual(CHANNELS[23][0], "smo_theta")
        self.assertEqual(CHANNELS[24][0], "model_state")
        self.assertEqual(CHANNELS[25][0], "ang_err")
        self.assertEqual(CHANNELS[26][0], "spd_ref")


if __name__ == "__main__":
    unittest.main()
