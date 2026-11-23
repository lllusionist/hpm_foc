import pathlib
import shutil
import subprocess
import tempfile
import unittest


TESTS = pathlib.Path(__file__).resolve().parent
OUTER = TESTS.parent / "core/outer"
HOST_CC = shutil.which("gcc") or shutil.which("clang")
SCENARIOS = (
    "5210 success",
    "4310 low-current resistance",
    "resistance with no response",
    "inductance below resistive impedance followed by a retry",
    "pole-pair measurement with no rotation",
    "saturation in the first flux measurement window",
    "soft overcurrent in a required stage",
    "soft overcurrent during optional inertia measurement",
    "flash write failure",
    "hard overcurrent during inductance measurement",
    "saturation only in the diagnostic resistance window",
)


@unittest.skipUnless(HOST_CC, "A native GCC or Clang is required")
class CalibrationTests(unittest.TestCase):
    def test_native_calibration_lifecycle(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = pathlib.Path(temporary)
            executable = directory / "calibration_test.exe"
            (directory / "hpm_common.h").write_text(
                "#include <stdint.h>\n#include <stdlib.h>\n",
                encoding="utf-8",
            )
            (directory / "task.h").write_text(
                "#define taskENTER_CRITICAL() ((void)0)\n"
                "#define taskEXIT_CRITICAL() ((void)0)\n",
                encoding="utf-8",
            )
            for header in (
                "hpm_crc32.h", "sys.h", "tim.h", "spi.h", "FreeRTOS.h",
                "hpm_soc.h", "hpm_spi_drv.h", "hpm_clock_drv.h", "hpm_gpio_drv.h",
            ):
                (directory / header).touch()
            result = subprocess.run(
                [
                    HOST_CC, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(directory), "-I", str(OUTER / "inc"),
                    "-I", str(OUTER / "src"), str(TESTS / "calibration_test.c"),
                    "-lm", "-o", str(executable),
                ],
                capture_output=True,
                text=True,
                timeout=20,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            for index, scenario in enumerate(SCENARIOS):
                with self.subTest(scenario=scenario):
                    result = subprocess.run(
                        [str(executable), str(index)],
                        capture_output=True,
                        text=True,
                        timeout=10,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
