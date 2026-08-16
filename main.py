#!/usr/bin/env python3
import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.hello = ["Hallo Welt", "Hei maailma", "Hola Mundo", "Привет мир"]

        self.button = QtWidgets.QPushButton("Click me!")
        self.text = QtWidgets.QLabel("Hello World",
                                     alignment=QtCore.Qt.AlignCenter)

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.button)

        self.button.clicked.connect(self.magic)
        self.button_pen = self.generate_button("Drive Pen")
        self.button_servo_on = self.generate_button("Servo On")
        self.button_servo_off = self.generate_button("Servo Off")

    @QtCore.Slot()
    def magic(self):
        self.text.setText(random.choice(self.hello))

    def generate_button(self, name, xpos=100, ypos=100, width=100, height=50):
        button = QtWidgets.QPushButton()
        button.setObjectName(name)
        button.setText(name)
        self.layout.addWidget(button)
        return button

if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = MyWidget()
    widget.resize(800, 600)
    widget.show()

    sys.exit(app.exec())
