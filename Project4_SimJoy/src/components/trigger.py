"""
扳机组件
实现左右扳机（LT, RT）
"""
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, pyqtSignal, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QLinearGradient
import math


class Trigger(QWidget):
    """
    扳机组件
    支持点击触发，可选压力模拟
    """

    # 信号：轴值改变时发射
    axis_changed = pyqtSignal(int, float)  # (axis_id, value)

    def __init__(self, axis_id: int, label: str,
                 width: int = 100, height: int = 50, parent=None):
        """
        初始化扳机

        Args:
            axis_id: 轴ID (2=LT, 5=RT)
            label: 扳机标签
            width: 扳机宽度
            height: 扳机高度
            parent: 父组件
        """
        super().__init__(parent)
        self.axis_id = axis_id
        self.label = label

        self.is_pressed = False
        self.is_hovered = False
        self.pressure = 0.0  # 压力值 [0, 1]

        self.setFixedSize(width, height)

    def set_pressed(self, pressed: bool):
        """设置按下状态"""
        if self.is_pressed != pressed:
            self.is_pressed = pressed
            if pressed:
                self.pressure = 1.0
            else:
                self.pressure = 0.0
            self.axis_changed.emit(self.axis_id, self.pressure)
            self.update()

    def set_pressure(self, pressure: float):
        """
        设置压力值（用于模拟压力）

        Args:
            pressure: 压力值 [0, 1]
        """
        self.pressure = max(0.0, min(1.0, pressure))
        self.is_pressed = self.pressure > 0
        self.axis_changed.emit(self.axis_id, self.pressure)
        self.update()

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.is_pressed = True
            # 根据点击位置计算压力
            rel_y = event.pos().y()
            pressure = 1.0 - (rel_y / self.height())
            self.pressure = max(0.0, min(1.0, pressure))
            self.axis_changed.emit(self.axis_id, self.pressure)
            self.update()

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.is_pressed = False
            self.pressure = 0.0
            self.axis_changed.emit(self.axis_id, 0.0)
            self.update()

    def enterEvent(self, event):
        """鼠标进入事件"""
        self.is_hovered = True
        self.update()

    def leaveEvent(self, event):
        """鼠标离开事件"""
        self.is_hovered = False
        self.update()

    def paintEvent(self, event):
        """绘制扳机"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 绘制背景阴影
        shadow_rect = QRectF(3, 3, self.width(), self.height())
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 80))
        painter.drawRoundedRect(shadow_rect, 15, 15)

        # 绘制扳机主体
        rect = QRectF(0, 0, self.width(), self.height())

        # 根据压力值调整颜色
        if self.pressure > 0:
            # 压力越大，颜色越深
            intensity = int(150 + self.pressure * 80)
            color = QColor(intensity, intensity, intensity)
        else:
            # 正常状态
            if self.is_hovered:
                color = QColor(220, 220, 220)
            else:
                color = QColor(200, 200, 200)

        # 创建渐变
        gradient = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        gradient.setColorAt(0, color.lighter(130))
        gradient.setColorAt(0.5, color)
        gradient.setColorAt(1, color.darker(110))

        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(gradient)
        painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 15, 15)

        # 绘制压力条
        if self.pressure > 0:
            pressure_rect = QRectF(
                5,
                self.height() * (1 - self.pressure) + 5,
                self.width() - 10,
                self.height() * self.pressure - 10
            )
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(0, 150, 255, 150))
            painter.drawRoundedRect(pressure_rect, 10, 10)

        # 绘制标签
        painter.setPen(QColor(60, 60, 60))
        font = painter.font()
        font.setFamily("Arial")
        font.setBold(True)
        font.setPointSize(12)
        painter.setFont(font)

        fm = painter.fontMetrics()
        text_width = fm.width(self.label)
        text_x = (self.width() - text_width) // 2
        text_y = self.height() // 2 + fm.height() // 4

        painter.drawText(text_x, text_y, self.label)

        # 绘制压力值
        if self.pressure > 0:
            painter.setPen(QColor(0, 100, 200))
            font.setPointSize(8)
            painter.setFont(font)

            pressure_text = f"{int(self.pressure * 100)}%"
            pressure_width = fm.width(pressure_text)
            pressure_x = (self.width() - pressure_width) // 2
            pressure_y = self.height() - 8

            painter.drawText(pressure_x, pressure_y, pressure_text)
