# -*- coding: utf-8 -*-
"""FOC 上位机入口 (PySide6).

用法:
    python foc_scope.py           # 正常连接串口
    python foc_scope.py --demo    # 演示数据 (无需硬件)
"""
import sys


def main():
    demo = "--demo" in sys.argv
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print("未安装 PySide6, 请执行: pip install -r requirements.txt")
        return 1
    from focscope import theme
    from focscope.main_window import MainWindow

    app = QApplication(sys.argv)
    theme.apply(app)
    win = MainWindow(demo=demo)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
