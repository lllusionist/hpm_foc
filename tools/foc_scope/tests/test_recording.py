import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from PySide6.QtWidgets import QApplication

from focscope import main_window
from focscope.analysis import load_csv


class TestRecording(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(
            ["foc-scope-test", "-platform", "offscreen"]
        )

    def test_stop_recording_exports_waveforms_and_gain_values(self):
        with tempfile.TemporaryDirectory() as output_dir:
            config_path = Path(output_dir) / "scope_cfg.json"
            with (
                patch.object(main_window, "CFG_PATH", config_path),
                patch.object(main_window, "default_dir", return_value=output_dir),
                patch.object(main_window.QMessageBox, "critical") as error,
            ):
                window = main_window.MainWindow(demo=True)
                try:
                    window.left.steppers["curkp"].set_value(0.125)
                    window.left.steppers["curbw"].set_value(300.0)
                    window.btn_record.click()
                    legacy = (0.0,) * 22
                    angles = (0.75, 0.5, 0.25)
                    with patch.object(window, "_collect", return_value=[legacy + angles, legacy]):
                        window._tick_impl()
                    window.btn_record.click()

                    error.assert_not_called()
                    self.assertFalse(window.recorder.is_active())
                    self.assertGreater(window.recorder.count, 0)
                    files = list(Path(output_dir).glob("foc_wave_*.csv"))
                    self.assertEqual(len(files), 1)
                    csv = files[0].read_text(encoding="utf-8-sig")
                    self.assertIn("电流Kp=0.125", csv)
                    self.assertIn("电流BW=300.0", csv)
                    rows = [line for line in csv.splitlines()
                            if not line.startswith("#")]
                    self.assertIn("Iq", rows[0].split(","))
                    self.assertEqual(len(rows) - 1, window.recorder.count)
                    _, names, samples = load_csv(files[0])
                    self.assertEqual(names[-3:], ["pos_target", "pos_actual", "pos_error"])
                    self.assertEqual(tuple(samples[0, -3:]), angles)
                    self.assertTrue(all(math.isnan(x) for x in samples[1, -3:]))
                    with patch.object(main_window.QFileDialog, "getOpenFileName",
                                      return_value=(str(files[0]), "")):
                        window.open_replay()
                    self.assertEqual(window.store.sample_at(22, 0), angles[0])
                    self.assertIsNone(window.store.sample_at(22, 1))
                finally:
                    window.close()
