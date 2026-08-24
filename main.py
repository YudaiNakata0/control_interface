#!/usr/bin/env python3
import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        # layout: grid style
        self.layout = QtWidgets.QGridLayout(self)
        self.button_pen = self.generate_button(name="Drive Pen", row=0, column=0)
        # sub layout for servo on/off
        self.servo_layout = QtWidgets.QHBoxLayout()
        self.button_servo_on = self.generate_button(name="Servo On", sub_layout=self.servo_layout)
        self.button_servo_off = self.generate_button(name="Servo Off", sub_layout=self.servo_layout)
        self.layout.addLayout(self.servo_layout, 0, 1)

    # @QtCore.Slot()

    # button settings
    def generate_button(self, name, sub_layout=False, row=0, column=0, height=1, width=1):
        button = QtWidgets.QPushButton()
        button.setObjectName(name)
        button.setText(name)
        if sub_layout == False:
            self.layout.addWidget(button, row, column, height, width)
        else:
            sub_layout.addWidget(button)
        return button

if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = MyWidget()
    widget.resize(800, 600)
    widget.show()

    sys.exit(app.exec())
