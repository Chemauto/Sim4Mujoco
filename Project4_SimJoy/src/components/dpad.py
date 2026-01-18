"""
方向键组件
实现十字方向键（D-PAD）
"""
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, pyqtSignal, QPoint, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF
from PyQt5.QtCore import QPointF


class DPad(QWidget):
    """
    十字方向键组件
    支持上下左右四个方向
    """

    # 信号：方向键状态改变时发射
    hat_changed = pyqtSignal(int, int, int)  # (hat_id, x, y)

    def __init__(self, hat_id: int = 0, size: int = 120, parent=None):
        """
        初始化方向键

        Args:
            hat_id: 方向键ID
            size: 组件大小
            parent: 父组件
        """
        super().__init__(parent)
        self.hat_id = hat_id
        self.size = size

        # 当前方向状态
        self.hat_x = 0  # -1(左), 0(中), 1(右)
        self.hat_y = 0  # -1(下), 0(中), 1(上)

        # 各方向的按下状态
        self.up_pressed = False
        self.down_pressed = False
        self.left_pressed = False
        self.right_pressed = False

        # 悬停状态
        self.up_hovered = False
        self.down_hovered = False
        self.left_hovered = False
        self.right_hovered = False

        self.setFixedSize(size, size)

    def _update_hat_state(self):
        """更新方向键状态并发射信号"""
        self.hat_x = 0
        self.hat_y = 0

        if self.left_pressed:
            self.hat_x = -1
        elif self.right_pressed:
            self.hat_x = 1

        if self.up_pressed:
            self.hat_y = 1
        elif self.down_pressed:
            self.hat_y = -1

        self.hat_changed.emit(self.hat_id, self.hat_x, self.hat_y)
        self.update()

    def _get_button_rects(self):
        """
        获取各方向按钮的矩形区域

        Returns:
            dict: 包含上下左右四个方向的矩形
        """
        center = self.size // 2
        button_width = self.size // 3
        button_height = self.size // 3

        return {
            'up': QRectF(
                center - button_width // 2,
                0,
                button_width,
                button_height
            ),
            'down': QRectF(
                center - button_width // 2,
                self.size - button_height,
                button_width,
                button_height
            ),
            'left': QRectF(
                0,
                center - button_height // 2,
                button_width,
                button_height
            ),
            'right': QRectF(
                self.size - button_width,
                center - button_height // 2,
                button_width,
                button_height
            ),
            'center': QRectF(
                center - button_width // 2,
                center - button_height // 2,
                button_width,
                button_height
            )
        }

    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            pos = event.pos()
            rects = self._get_button_rects()

            if rects['up'].contains(pos):
                self.up_pressed = True
            elif rects['down'].contains(pos):
                self.down_pressed = True
            elif rects['left'].contains(pos):
                self.left_pressed = True
            elif rects['right'].contains(pos):
                self.right_pressed = True

            self._update_hat_state()

    def mouseReleaseEvent(self, event):
        """鼠标释放事件"""
        if event.button() == Qt.LeftButton:
            self.up_pressed = False
            self.down_pressed = False
            self.left_pressed = False
            self.right_pressed = False
            self._update_hat_state()

    def mouseMoveEvent(self, event):
        """鼠标移动事件（支持拖拽切换方向）"""
        if self.up_pressed or self.down_pressed or self.left_pressed or self.right_pressed:
            pos = event.pos()
            rects = self._get_button_rects()

            # 重置所有状态
            self.up_pressed = False
            self.down_pressed = False
            self.left_pressed = False
            self.right_pressed = False

            # 根据当前位置设置状态
            if rects['up'].contains(pos):
                self.up_pressed = True
            elif rects['down'].contains(pos):
                self.down_pressed = True
            elif rects['left'].contains(pos):
                self.left_pressed = True
            elif rects['right'].contains(pos):
                self.right_pressed = True

            self._update_hat_state()

    def enterEvent(self, event):
        """鼠标进入事件"""
        pass

    def leaveEvent(self, event):
        """鼠标离开事件"""
        # 清除悬停状态
        self.up_hovered = False
        self.down_hovered = False
        self.left_hovered = False
        self.right_hovered = False

    def paintEvent(self, event):
        """绘制方向键"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rects = self._get_button_rects()

        # 绘制中心连接块
        painter.setPen(QPen(QColor(60, 60, 60), 2))
        painter.setBrush(QColor(80, 80, 80))
        painter.drawRoundedRect(rects['center'], 5, 5)

        # 绘制各个方向按钮
        self._draw_direction_button(
            painter, rects['up'],
            self.up_pressed, self.up_hovered,
            "▲", "up"
        )
        self._draw_direction_button(
            painter, rects['down'],
            self.down_pressed, self.down_hovered,
            "▼", "down"
        )
        self._draw_direction_button(
            painter, rects['left'],
            self.left_pressed, self.left_hovered,
            "◄", "left"
        )
        self._draw_direction_button(
            painter, rects['right'],
            self.right_pressed, self.right_hovered,
            "►", "right"
        )

    def _draw_direction_button(self, painter, rect, pressed, hovered, symbol, direction):
        """
        绘制单个方向按钮

        Args:
            painter: QPainter对象
            rect: 按钮矩形区域
            pressed: 是否按下
            hovered: 是否悬停
            symbol: 方向符号
            direction: 方向（用于调整阴影）
        """
        # 阴影偏移
        shadow_offset = 2

        # 绘制阴影
        shadow_rect = rect.adjusted(shadow_offset, shadow_offset,
                                    shadow_offset, shadow_offset)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 80))
        painter.drawRoundedRect(shadow_rect, 8, 8)

        # 确定颜色
        if pressed:
            base_color = QColor(100, 100, 100)
        elif hovered:
            base_color = QColor(150, 150, 150)
        else:
            base_color = QColor(130, 130, 130)

        # 绘制按钮主体
        painter.setPen(QPen(QColor(60, 60, 60), 2))
        painter.setBrush(base_color)
        painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 8, 8)

        # 绘制高光
        highlight_rect = rect.adjusted(5, 5, -5, -5)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 30))
        painter.drawRoundedRect(highlight_rect, 5, 5)

        # 绘制方向符号
        painter.setPen(QColor(255, 255, 255))
        font = painter.font()
        font.setFamily("Arial")
        font.setBold(True)
        font.setPointSize(16)
        painter.setFont(font)

        fm = painter.fontMetrics()
        symbol_width = fm.width(symbol)
        symbol_x = int(rect.center().x() - symbol_width / 2)
        symbol_y = int(rect.center().y() + fm.height() / 4)

        painter.drawText(symbol_x, symbol_y, symbol)

    def get_hat_value(self):
        """获取当前方向键值"""
        return (self.hat_x, self.hat_y)
