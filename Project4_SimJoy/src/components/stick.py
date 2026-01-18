"""
摇杆组件
实现可拖拽的双轴摇杆
"""
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, pyqtSignal, QPoint, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QRadialGradient
import math


class AnalogStick(QWidget):
    """
    模拟摇杆组件
    支持鼠标拖拽，自动回弹到中心
    """

    # 信号：轴值改变时发射
    axis_changed = pyqtSignal(int, float)  # (axis_id, value)

    def __init__(self, axis_x_id: int, axis_y_id: int,
                 radius: int = 50, parent=None):
        """
        初始化摇杆

        Args:
            axis_x_id: X轴ID
            axis_y_id: Y轴ID
            radius: 底座半径
            parent: 父组件
        """
        super().__init__(parent)
        self.axis_x_id = axis_x_id
        self.axis_y_id = axis_y_id
        self.radius = radius
        self.stick_radius = radius * 0.4  # 摇杆头半径

        # 当前摇杆位置（相对于中心）
        self.stick_pos = QPoint(0, 0)

        # 拖拽状态
        self.is_dragging = False
        self.drag_start_pos = QPoint(0, 0)

        # 定时器用于回弹动画
        self.return_timer = QTimer()
        self.return_timer.timeout.connect(self._return_to_center)
        self.return_timer.setInterval(16)  # ~60fps

        # 设置固定大小
        size = radius * 2 + 10
        self.setFixedSize(size, size)

        # 启用鼠标追踪
        self.setMouseTracking(True)

    def _get_center(self) -> QPoint:
        """获取中心点坐标"""
        return QPoint(self.width() // 2, self.height() // 2)

    def _return_to_center(self):
        """平滑回弹到中心"""
        center = self._get_center()
        target_pos = center

        # 计算当前摇杆的实际位置
        current_stick_pos = center + self.stick_pos

        # 线性插值回弹
        step = 0.2
        new_x = current_stick_pos.x() + (target_pos.x() - current_stick_pos.x()) * step
        new_y = current_stick_pos.y() + (target_pos.y() - current_stick_pos.y()) * step

        # 更新摇杆位置
        self.stick_pos = QPoint(int(new_x - center.x()), int(new_y - center.y()))

        # 检查是否接近中心
        distance = math.sqrt(self.stick_pos.x() ** 2 + self.stick_pos.y() ** 2)
        if distance < 2:
            self.stick_pos = QPoint(0, 0)
            self.return_timer.stop()

        # 更新轴值
        self._update_axis_values()

        # 重绘
        self.update()

    def _update_axis_values(self):
        """更新轴值并发射信号"""
        # 计算归一化值 [-1, 1]
        max_distance = self.radius - self.stick_radius
        if max_distance > 0:
            x_value = self.stick_pos.x() / max_distance
            y_value = self.stick_pos.y() / max_distance

            # 限制在[-1, 1]范围内
            x_value = max(-1.0, min(1.0, x_value))
            y_value = max(-1.0, min(1.0, y_value))
        else:
            x_value = 0.0
            y_value = 0.0

        # 发射信号
        self.axis_changed.emit(self.axis_x_id, x_value)
        self.axis_changed.emit(self.axis_y_id, y_value)

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            center = self._get_center()
            click_pos = event.pos()

            # 检查是否点击在摇杆头上
            stick_pos = center + self.stick_pos
            distance = math.sqrt(
                (click_pos.x() - stick_pos.x()) ** 2 +
                (click_pos.y() - stick_pos.y()) ** 2
            )

            if distance <= self.stick_radius:
                self.is_dragging = True
                self.drag_start_pos = click_pos - self.stick_pos
                self.return_timer.stop()

    def mouseMoveEvent(self, event):
        """鼠标移动事件"""
        if self.is_dragging:
            center = self._get_center()
            mouse_pos = event.pos()

            # 计算新的摇杆位置
            new_pos = mouse_pos - self.drag_start_pos - center

            # 限制在最大移动范围内
            max_distance = self.radius - self.stick_radius
            distance = math.sqrt(new_pos.x() ** 2 + new_pos.y() ** 2)

            if distance > max_distance:
                # 归一化并缩放到最大距离
                scale = max_distance / distance
                new_pos = QPoint(
                    int(new_pos.x() * scale),
                    int(new_pos.y() * scale)
                )

            self.stick_pos = new_pos
            self._update_axis_values()
            self.update()

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.is_dragging = False
            # 启动回弹动画
            self.return_timer.start()

    def paintEvent(self, event):
        """绘制摇杆"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center = self._get_center()

        # 1. 绘制底座
        # 外圈阴影
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 80))
        painter.drawEllipse(
            center.x() - self.radius - 2,
            center.y() - self.radius - 2,
            self.radius * 2 + 4,
            self.radius * 2 + 4
        )

        # 底座主体 - 使用径向渐变
        gradient = QRadialGradient(center, self.radius)
        gradient.setColorAt(0, QColor(60, 60, 60))
        gradient.setColorAt(1, QColor(40, 40, 40))

        painter.setPen(QPen(QColor(20, 20, 20), 2))
        painter.setBrush(gradient)
        painter.drawEllipse(
            center.x() - self.radius,
            center.y() - self.radius,
            self.radius * 2,
            self.radius * 2
        )

        # 底座内圈
        inner_radius = self.radius * 0.8
        painter.setPen(QPen(QColor(30, 30, 30), 1))
        painter.setBrush(QColor(35, 35, 35))
        painter.drawEllipse(
            int(center.x() - inner_radius),
            int(center.y() - inner_radius),
            int(inner_radius * 2),
            int(inner_radius * 2)
        )

        # 2. 绘制摇杆头
        stick_center = center + self.stick_pos

        # 摇杆头阴影
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 100))
        painter.drawEllipse(
            int(stick_center.x() - self.stick_radius + 2),
            int(stick_center.y() - self.stick_radius + 2),
            int(self.stick_radius * 2),
            int(self.stick_radius * 2)
        )

        # 摇杆头主体 - 使用径向渐变
        offset_x = int(self.stick_radius * 0.3)
        offset_y = int(self.stick_radius * 0.3)

        if self.is_dragging:
            # 拖拽时颜色稍深
            stick_gradient = QRadialGradient(
                stick_center - QPoint(offset_x, offset_y),
                self.stick_radius
            )
            stick_gradient.setColorAt(0, QColor(120, 120, 120))
            stick_gradient.setColorAt(1, QColor(80, 80, 80))
        else:
            stick_gradient = QRadialGradient(
                stick_center - QPoint(offset_x, offset_y),
                self.stick_radius
            )
            stick_gradient.setColorAt(0, QColor(140, 140, 140))
            stick_gradient.setColorAt(1, QColor(90, 90, 90))

        painter.setPen(QPen(QColor(50, 50, 50), 2))
        painter.setBrush(stick_gradient)
        painter.drawEllipse(
            int(stick_center.x() - self.stick_radius),
            int(stick_center.y() - self.stick_radius),
            int(self.stick_radius * 2),
            int(self.stick_radius * 2)
        )

        # 摇杆头高光
        highlight_radius = self.stick_radius * 0.4
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 60))
        painter.drawEllipse(
            int(stick_center.x() - highlight_radius - self.stick_radius * 0.3),
            int(stick_center.y() - highlight_radius - self.stick_radius * 0.3),
            int(highlight_radius * 2),
            int(highlight_radius * 2)
        )

        # 3. 绘制中心十字标记
        painter.setPen(QPen(QColor(70, 70, 70), 1))
        painter.drawLine(
            center.x() - 10, center.y(),
            center.x() + 10, center.y()
        )
        painter.drawLine(
            center.x(), center.y() - 10,
            center.x(), center.y() + 10
        )

    def set_position(self, x: float, y: float):
        """
        直接设置摇杆位置（用于外部控制）

        Args:
            x: X轴值 [-1, 1]
            y: Y轴值 [-1, 1]
        """
        max_distance = self.radius - self.stick_radius
        self.stick_pos = QPoint(
            int(x * max_distance),
            int(y * max_distance)
        )
        self._update_axis_values()
        self.update()
