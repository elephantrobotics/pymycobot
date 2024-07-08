from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *


class SwitchButton(QCheckBox):
    status_button = Signal(bool)

    def __init__(
        self,
        width=60,
        bg_color="#777",
        circle_color="#ddd",
        active_color="#409eff",
        animation_curve=QEasingCurve.OutQuint

    ):
        QCheckBox.__init__(self)

        self.setFixedSize(width, 28)
        self.setCursor(Qt.PointingHandCursor)

        self._bg_corlor = bg_color
        self._circle_color = circle_color
        self._active_color = active_color

        self.stateChanged.connect(self.start_transition)

        self._circle_position = 3
        self.animation = QPropertyAnimation(self, b"circle_position", self)
        self.animation.setEasingCurve(animation_curve)
        self.animation.setDuration(500)

    @Property(float)
    def circle_position(self):
        return self._circle_position

    @circle_position.setter
    def circle_position(self, pos):
        self._circle_position = pos
        self.update()
        
    def reset_state(self):
        self.setChecked(False)
        self.animation.setEndValue(3)

    def start_transition(self, value):
        # print(f'status:{self.isChecked()}')

        # print(value, "value")

        self.animation.stop()

        if value:
            self.animation.setEndValue(self.width()-25)
            self.status_button.emit(True)
        else:
            self.animation.setEndValue(3)
            self.status_button.emit(False)

        self.animation.start()

    def hitButton(self, pos: QPoint):
        return self.contentsRect().contains(pos)

    def paintEvent(self, e):
        # pass
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        p.setPen(Qt.NoPen)

        rect = QRect(0, 0, self.width(), self.height())

        if self.isChecked():

            p.setBrush(QColor(self._active_color))
            p.drawRoundedRect(0, 0, rect.width(), self.height(),
                              self.height()/2, self.height()/2)

            p.setBrush(QColor(self._circle_color))
            p.drawEllipse(self._circle_position, 3, 22, 22)

        else:

            p.setBrush(QColor(self._bg_corlor))
            p.drawRoundedRect(0, 0, rect.width(), self.height(),
                              self.height()/2, self.height()/2)

            p.setBrush(QColor(self._circle_color))

            p.drawEllipse(self.circle_position, 3, 22, 22)

        p.end()
