#!/usr/bin/env python3
import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui
import rospy
from std_msgs.msg import Empty, Bool

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        # ROS publishers
        self.pub_drive_pen = rospy.Publisher("/drive_pen", Empty, queue_size=1)
        self.pub_servo = rospy.Publisher("/servo", Bool, queue_size=1)

        # layout: grid style
        self.layout = QtWidgets.QGridLayout(self)
        self.button_pen = self.generate_button(name="Drive Pen", row=0, column=0)
        self.button_pen.clicked.connect(self.on_drive_pen)
        # sub layout for servo on/off
        self.servo_layout = QtWidgets.QHBoxLayout()
        self.button_servo_on = self.generate_button(name="Servo On", sub_layout=self.servo_layout)
        self.button_servo_off = self.generate_button(name="Servo Off", sub_layout=self.servo_layout)
        self.button_servo_on.clicked.connect(lambda: self.on_servo(True))
        self.button_servo_off.clicked.connect(lambda: self.on_servo(False))
        self.layout.addLayout(self.servo_layout, 0, 1)

    # @QtCore.Slot()

    def on_drive_pen(self):
        self.pub_drive_pen.publish(Empty())

    def on_servo(self, state):
        self.pub_servo.publish(Bool(data=state))

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
    rospy.init_node("control_interface", anonymous=True)

    app = QtWidgets.QApplication([])

    widget = MyWidget()
    widget.resize(800, 600)
    widget.show()

    sys.exit(app.exec())
