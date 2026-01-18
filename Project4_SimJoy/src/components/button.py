"""
按钮组件
用于实现手柄上的各种按钮（A, B, X, Y, LB, RB, START, SELECT）
"""
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, pyqtSignal, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont


class JoystickButton(QWidget):
    """
    手柄按钮组件
    圆形按钮，支持按下和释放状态
    """

    # 信号：按钮状态改变时发射
    state_changed = pyqtSignal(int, bool)  # (button_id, state)

    def __init__(self, button_id: int, label: str, color: QColor,
                 radius: int = 30, parent=None):
        """
        初始化按钮

        Args:
            button_id: 按键ID (0-7)
            label: 按钮标签文本
            color: 按钮颜色
            radius: 按钮半径
            parent: 父组件
        """
        super().__init__(parent)
        self.button_id = button_id
        self.label = label
        self.base_color = color
        self.radius = radius

        self.is_pressed = False
        self.is_hovered = False

        # 设置固定大小
        size = radius * 2
        self.setFixedSize(size, size)

    def set_pressed(self, pressed: bool):
        """
        设置按钮状态

        Args:
            pressed: 是否按下
        """
        if self.is_pressed != pressed:
            self.is_pressed = pressed
            self.state_changed.emit(self.button_id, pressed)
            self.update()

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.set_pressed(True)

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.set_pressed(False)

    def enterEvent(self, event):
        """鼠标进入事件"""
        self.is_hovered = True
        self.update()

    def leaveEvent(self, event):
        """鼠标离开事件"""
        self.is_hovered = False
        self.update()

    def paintEvent(self, event):
        """绘制按钮"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 计算中心和半径
        center_x = self.width() / 2
        center_y = self.height() / 2

        # 根据状态调整半径（按下时稍微缩小）
        current_radius = self.radius * 0.9 if self.is_pressed else self.radius

        # 绘制外圈阴影
        shadow_offset = 3
        shadow_color = QColor(0, 0, 0, 80)
        painter.setPen(Qt.NoPen)
        painter.setBrush(shadow_color)
        painter.drawEllipse(
            int(center_x - current_radius + shadow_offset),
            int(center_y - current_radius + shadow_offset),
            int(current_radius * 2),
            int(current_radius * 2)
        )

        # 绘制按钮主体
        if self.is_pressed:
            # 按下状态：颜色变深
            button_color = self.base_color.darker(130)
        elif self.is_hovered:
            # 悬停状态：颜色变亮
            button_color = self.base_color.lighter(120)
        else:
            # 正常状态
            button_color = self.base_color

        # 渐变填充
        gradient_color = button_color.lighter(110)

        painter.setPen(QPen(QColor(0, 0, 0, 100), 2))
        painter.setBrush(gradient_color)
        painter.drawEllipse(
            int(center_x - current_radius),
            int(center_y - current_radius),
            int(current_radius * 2),
            int(current_radius * 2)
        )

        # 绘制内圈高光
        inner_radius = current_radius * 0.7
        highlight_color = QColor(255, 255, 255, 50)
        painter.setPen(Qt.NoPen)
        painter.setBrush(highlight_color)
        painter.drawEllipse(
            int(center_x - inner_radius),
            int(center_y - inner_radius),
            int(inner_radius * 2),
            int(inner_radius * 2)
        )

        # 绘制标签文字
        painter.setPen(QColor(255, 255, 255))
        font = QFont("Arial", 14, QFont.Bold)
        painter.setFont(font)

        # 文字居中
        fm = painter.fontMetrics()
        text_width = fm.width(self.label)
        text_height = fm.height()
        text_x = int(center_x - text_width / 2)
        text_y = int(center_y + text_height / 4)

        painter.drawText(text_x, text_y, self.label)


class ShoulderButton(QWidget):
    """
    肩键组件（LB, RB）
    横向椭圆形按钮
    """

    state_changed = pyqtSignal(int, bool)

    def __init__(self, button_id: int, label: str, width: int = 80,
                 height: int = 30, parent=None):
        """
        初始化肩键

        Args:
            button_id: 按键ID
            label: 按钮标签
            width: 按钮宽度
            height: 按钮高度
            parent: 父组件
        """
        super().__init__(parent)
        self.button_id = button_id
        self.label = label

        self.is_pressed = False
        self.is_hovered = False

        self.setFixedSize(width, height)

    def set_pressed(self, pressed: bool):
        """设置按钮状态"""
        if self.is_pressed != pressed:
            self.is_pressed = pressed
            self.state_changed.emit(self.button_id, pressed)
            self.update()

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.set_pressed(True)

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.set_pressed(False)

    def enterEvent(self, event):
        """鼠标进入事件"""
        self.is_hovered = True
        self.update()

    def leaveEvent(self, event):
        """鼠标离开事件"""
        self.is_hovered = False
        self.update()

    def paintEvent(self, event):
        """绘制肩键"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 肩键颜色
        base_color = QColor(200, 200, 200)

        if self.is_pressed:
            color = base_color.darker(130)
        elif self.is_hovered:
            color = base_color.lighter(120)
        else:
            color = base_color

        # 绘制圆角矩形
        rect = self.rect().adjusted(2, 2, -2, -2)
        painter.setPen(QPen(QColor(0, 0, 0, 100), 2))
        painter.setBrush(color)
        painter.drawRoundedRect(rect, 10, 10)

        # 绘制标签
        painter.setPen(QColor(50, 50, 50))
        font = QFont("Arial", 10, QFont.Bold)
        painter.setFont(font)

        fm = painter.fontMetrics()
        text_width = fm.width(self.label)
        text_x = (self.width() - text_width) // 2
        text_y = self.height() // 2 + fm.height() // 4

        painter.drawText(text_x, text_y, self.label)


class SmallButton(QWidget):
    """
    小型按钮组件（START, SELECT）
    用于中间功能键
    """

    state_changed = pyqtSignal(int, bool)

    def __init__(self, button_id: int, label: str, radius: int = 20,
                 parent=None):
        """
        初始化小型按钮

        Args:
            button_id: 按键ID
            label: 按钮标签
            radius: 按钮半径
            parent: 父组件
        """
        super().__init__(parent)
        self.button_id = button_id
        self.label = label
        self.radius = radius

        self.is_pressed = False
        self.is_hovered = False

        size = radius * 2
        self.setFixedSize(size, size)

    def set_pressed(self, pressed: bool):
        """设置按钮状态"""
        if self.is_pressed != pressed:
            self.is_pressed = pressed
            self.state_changed.emit(self.button_id, pressed)
            self.update()

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.set_pressed(True)

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.set_pressed(False)

    def enterEvent(self, event):
        """鼠标进入事件"""
        self.is_hovered = True
        self.update()

    def leaveEvent(self, event):
        """鼠标离开事件"""
        self.is_hovered = False
        self.update()

    def paintEvent(self, event):
        """绘制小型按钮"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        center_x = self.width() / 2
        center_y = self.height() / 2

        # 根据状态调整半径
        current_radius = self.radius * 0.9 if self.is_pressed else self.radius

        # 绘制阴影
        shadow_offset = 2
        shadow_color = QColor(0, 0, 0, 60)
        painter.setPen(Qt.NoPen)
        painter.setBrush(shadow_color)
        painter.drawEllipse(
            int(center_x - current_radius + shadow_offset),
            int(center_y - current_radius + shadow_offset),
            int(current_radius * 2),
            int(current_radius * 2)
        )

        # 绘制按钮主体
        base_color = QColor(180, 180, 180)

        if self.is_pressed:
            color = base_color.darker(130)
        elif self.is_hovered:
            color = base_color.lighter(120)
        else:
            color = base_color

        painter.setPen(QPen(QColor(0, 0, 0, 100), 2))
        painter.setBrush(color)
        painter.drawEllipse(
            int(center_x - current_radius),
            int(center_y - current_radius),
            int(current_radius * 2),
            int(current_radius * 2)
        )

        # 绘制标签
        painter.setPen(QColor(50, 50, 50))
        font = QFont("Arial", 8, QFont.Bold)
        painter.setFont(font)

        # 使用多行绘制长标签
        if len(self.label) > 5:
            lines = self.label.split()
            for i, line in enumerate(lines):
                fm = painter.fontMetrics()
                text_width = fm.width(line)
                text_x = int(center_x - text_width / 2)
                text_y = int(center_y + fm.height() / 2 + (i - 0.5) * fm.height())
                painter.drawText(text_x, text_y, line)
        else:
            fm = painter.fontMetrics()
            text_width = fm.width(self.label)
            text_x = int(center_x - text_width / 2)
            text_y = int(center_y + fm.height() / 4)
            painter.drawText(text_x, text_y, self.label)
