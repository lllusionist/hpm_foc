# -*- coding: utf-8 -*-
"""浅色主题: 调色板 + 全局 QSS."""

BG        = "#e9ecf1"
PANEL     = "#ffffff"
PANEL2    = "#f7f8fa"
CARD      = "#ffffff"
BORDER    = "#d8dce2"
BORDER_LT = "#bcc3cd"
FG        = "#1f2328"
FG_DIM    = "#5b6470"
FG_FAINT  = "#9aa3ae"
ACCENT    = "#1a73e8"
ACCENT_D  = "#1558b0"
GREEN     = "#1a9d4b"
RED       = "#d93025"
AMBER     = "#e37400"
TITLEBAR  = "#ffffff"
PLOT_BG   = "#ffffff"
PLOT_GRID = "#e8ebef"
PLOT_AXIS = "#9aa3ae"

CURVE = ["#0072bd", "#d95319", "#edb120", "#7e2f8e", "#77ac30", "#4dbeee",
         "#a2142f", "#e377c2", "#17becf", "#8c564b", "#2ca02c", "#ff7f0e",
         "#1f77b4", "#9467bd", "#bcbd22", "#7f7f7f", "#c00000", "#00a0a0",
         "#b8860b", "#6a5acd", "#2e8b57", "#cd5c5c"]

FONT = "Microsoft YaHei UI"
MONO = "Consolas"

QSS = """
* {
    font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif;
    font-size: 12px;
    color: #1f2328;
    outline: none;
}
QWidget { background: transparent; }

QLabel { background: transparent; }

QWidget#MainWindow { background: #e9ecf1; }
QFrame#Root { background: #e9ecf1; border: 1px solid #d8dce2; border-radius: 8px; }
QFrame#Card { background: #ffffff; border: 1px solid #d8dce2; border-radius: 8px; }
QLabel#CardTitle { color: #1f2328; font-size: 13px; font-weight: 600; background: transparent; }
QLabel#Dim { color: #5b6470; background: transparent; }
QLabel#Accent { color: #1a73e8; background: transparent; }

QPushButton {
    background: #ffffff;
    border: 1px solid #ccd2db;
    border-radius: 6px;
    padding: 4px 12px;
    color: #1f2328;
}
QPushButton:hover { background: #f1f4f9; border-color: #aab3bf; }
QPushButton:pressed { background: #e6eaf0; }
QPushButton:disabled { color: #a9b0ba; background: #f2f4f7; border-color: #e0e4ea; }
QPushButton#Accent { background: #1a73e8; border-color: #1a73e8; color: #ffffff; }
QPushButton#Accent:hover { background: #2b80ef; }
QPushButton#Danger { background: #fdecea; border-color: #f0b4ae; color: #b3261e; }
QPushButton#Danger:hover { background: #f9d9d6; }
QPushButton#Toggle { background: #ffffff; border: 1px solid #ccd2db; color: #1f2328; }
QPushButton#Toggle:hover { background: #f1f4f9; }
QPushButton#Toggle:checked { background: #1a73e8; border-color: #1a73e8; color: #ffffff; }
QPushButton#Toggle:checked:hover { background: #2b80ef; }
QPushButton#Flat { background: transparent; border: none; padding: 3px 8px; }
QPushButton#Flat:hover { background: #eceff4; border-radius: 6px; }

QToolButton {
    background: transparent; border: none; border-radius: 6px; padding: 4px;
    color: #5b6470;
}
QToolButton:hover { background: #eceff4; }
QToolButton:pressed { background: #e0e4ea; }

QComboBox, QLineEdit, QDoubleSpinBox, QSpinBox {
    background: #ffffff;
    border: 1px solid #ccd2db;
    border-radius: 6px;
    padding: 4px 8px;
    selection-background-color: #1a73e8;
    selection-color: #ffffff;
}
QComboBox:hover, QLineEdit:hover { border-color: #aab3bf; }
QComboBox::drop-down { border: none; width: 20px; }
QComboBox::down-arrow {
    image: none; width: 0; height: 0;
    border-left: 4px solid transparent; border-right: 4px solid transparent;
    border-top: 5px solid #5b6470; margin-right: 6px;
}
QComboBox QAbstractItemView {
    background: #ffffff; border: 1px solid #d8dce2;
    selection-background-color: #1a73e8; selection-color: #ffffff; padding: 2px;
}

QCheckBox { background: transparent; spacing: 6px; }
QCheckBox::indicator {
    width: 14px; height: 14px; border-radius: 4px;
    border: 1px solid #b3bac4; background: #ffffff;
}
QCheckBox::indicator:hover { border-color: #1a73e8; }
QCheckBox::indicator:checked { background: #1a73e8; border-color: #1a73e8; }

QSlider::groove:horizontal { height: 6px; background: #dfe3e8; border-radius: 3px; }
QSlider::sub-page:horizontal { background: #1a73e8; border-radius: 3px; }
QSlider::handle:horizontal {
    width: 14px; height: 14px; margin: -5px 0; border-radius: 7px;
    background: #ffffff; border: 2px solid #1a73e8;
}
QSlider::handle:horizontal:hover { border-color: #1558b0; }

QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: transparent; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #ccd2db; border-radius: 5px; min-height: 30px; }
QScrollBar::handle:vertical:hover { background: #aab3bf; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 0; }
QScrollBar::handle:horizontal { background: #ccd2db; border-radius: 5px; min-width: 30px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }

QSplitter::handle { background: #e9ecf1; }
QSplitter::handle:vertical { height: 6px; }
QSplitter::handle:hover { background: #1a73e8; }

QDialog { background: #e9ecf1; }
QHeaderView::section {
    background: #f2f4f7; color: #5b6470; border: none;
    border-bottom: 1px solid #d8dce2; padding: 4px;
}
QTableWidget { background: #ffffff; border: 1px solid #d8dce2; gridline-color: #eceff3; }
QTableWidget::item:selected { background: #1a73e8; color: #ffffff; }

QToolTip {
    background: #ffffff; color: #1f2328;
    border: 1px solid #bcc3cd; padding: 4px;
}
"""


def apply(app):
    """应用全局样式与字体."""
    from PySide6.QtGui import QFont
    app.setStyle("Fusion")
    app.setFont(QFont(FONT, 9))
    app.setStyleSheet(QSS)
