#!/usr/bin/env python3
import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui
import rospy
from std_msgs.msg import Empty, Bool
from aerial_robot_msgs.msg import FlightNav

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        # ROS publishers
        self.pub_drive_pen = rospy.Publisher("/drive_pen", Empty, queue_size=1)
        self.pub_servo = rospy.Publisher("/servo", Bool, queue_size=1)

        # ROS publishers for keyboard teleop (see reference/keyboard_command.py)
        self.pub_start = rospy.Publisher("/teleop_command/start", Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher("/teleop_command/takeoff", Empty, queue_size=1)
        self.pub_land = rospy.Publisher("/teleop_command/land", Empty, queue_size=1)
        self.pub_force_landing = rospy.Publisher("/teleop_command/force_landing", Empty, queue_size=1)
        self.pub_halt = rospy.Publisher("/teleop_command/halt", Empty, queue_size=1)
        self.pub_nav = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.xy_vel = rospy.get_param("~xy_vel", 0.05)
        self.z_vel = rospy.get_param("~z_vel", 0.05)
        self.yaw_vel = rospy.get_param("~yaw_vel", 0.05)

        # receive key events on the window regardless of which child widget has focus
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # layout: grid style
        self.layout = QtWidgets.QGridLayout(self)
        # text instruction
        self.setup_instruction()

        # self.layout.addWidget()
        # button for driving pen
        self.button_pen = self.generate_button(name="Drive Pen", row=1, column=0)
        self.button_pen.clicked.connect(self.on_drive_pen)
        # button for driving servo
        # sub layout for servo on/off
        self.servo_layout = QtWidgets.QHBoxLayout()
        self.button_servo_on = self.generate_button(name="Servo On", sub_layout=self.servo_layout)
        self.button_servo_off = self.generate_button(name="Servo Off", sub_layout=self.servo_layout)
        self.button_servo_on.clicked.connect(lambda: self.on_servo(True))
        self.button_servo_off.clicked.connect(lambda: self.on_servo(False))
        self.layout.addLayout(self.servo_layout, 1, 1)

    # @QtCore.Slot()

    def setup_instruction(self):
        msg = """
        <b>Instruction:</b><br>
        ---------------------------<br>
        <b>r</b>:  arming motor (please do before takeoff)<br><br>
        <b>t</b>:  takeoff<br>
        <b>l</b>:  land<br>
        <b>f</b>:  force landing<br>
        <b>h</b>:  halt (force stop motor)<br><br>
        <b>q        w           e           [</b><br>
        (turn left)  (forward)  (turn right)  (move up)<br>
        <b>a        s           d           ]</b><br>
        (move left)  (backward) (move right) (move down)<br>
        Please don't have caps lock on.<br>
        CTRL+c to quit<br>
        ---------------------------<br>
        """
        text_instruction = QtWidgets.QLabel(msg)
        text_instruction.setWordWrap(True)
        text_instruction.setStyleSheet("""
        QLabel {
        background-color: #f4f4f4;
        border: 3px solid #cccccc;
        border-radius: 5px;
        padding: 10px;
        font-size: 16px;
        }
        """)
        self.layout.addWidget(text_instruction, 0, 0, 1, 1)

    def on_drive_pen(self):
        self.pub_drive_pen.publish(Empty())

    def on_servo(self, state):
        self.pub_servo.publish(Bool(data=state))

    # keyboard teleop (see reference/keyboard_command.py)
    def keyPressEvent(self, event):
        key = event.text()

        if key == "r":
            self.pub_start.publish(Empty())
        elif key == "t":
            self.pub_takeoff.publish(Empty())
        elif key == "l":
            self.pub_land.publish(Empty())
        elif key == "f":
            self.pub_force_landing.publish(Empty())
        elif key == "h":
            self.pub_halt.publish(Empty())
        elif key in ("w", "s", "a", "d", "q", "e", "[", "]"):
            self.publish_nav_key(key)
        else:
            super().keyPressEvent(event)

    def publish_nav_key(self, key):
        nav_msg = FlightNav()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        if key == "w":
            nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_vel_x = self.xy_vel
        elif key == "s":
            nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_vel_x = -self.xy_vel
        elif key == "a":
            nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_vel_y = self.xy_vel
        elif key == "d":
            nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_vel_y = -self.xy_vel
        elif key == "q":
            nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_omega_z = self.yaw_vel
        elif key == "e":
            nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_omega_z = -self.yaw_vel
        elif key == "[":
            nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_vel_z = self.z_vel
        elif key == "]":
            nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
            nav_msg.target_vel_z = -self.z_vel

        self.pub_nav.publish(nav_msg)

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
    widget.resize(960, 720)
    widget.show()

    sys.exit(app.exec())
