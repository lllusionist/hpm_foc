import pathlib
import shutil
import subprocess
import tempfile
import unittest


TESTS = pathlib.Path(__file__).resolve().parent
OUTER = TESTS.parent / "core/outer"
HOST_CC = shutil.which("gcc") or shutil.which("clang")


@unittest.skipUnless(HOST_CC, "A native GCC or Clang is required")
class PositionProfileTests(unittest.TestCase):
    def test_native_trajectory(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = pathlib.Path(temporary)
            executable = directory / "position_profile_test.exe"
            (directory / "hpm_common.h").write_text(
                "#include <stdint.h>\n", encoding="utf-8"
            )
            compile_result = subprocess.run(
                [
                    HOST_CC, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(directory), "-I", str(OUTER / "inc"),
                    str(TESTS / "position_profile_test.c"),
                    str(OUTER / "src/position_profile.c"),
                    "-lm", "-o", str(executable),
                ],
                capture_output=True,
                text=True,
                timeout=20,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
            result = subprocess.run(
                [str(executable)], capture_output=True, text=True, timeout=10
            )
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
