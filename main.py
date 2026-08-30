#!/usr/bin/env python3
import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import Image, CompressedImage

class MyWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        # ROS publishers
        self.pub_drive_pen = rospy.Publisher("/drive_pen", Empty, queue_size=1)
        self.pub_servo = rospy.Publisher("/servo", Bool, queue_size=1)

        # ROS publishers for keyboard teleop (see reference/keyboard_command.py)
        self.robot_ns = "gimbalrotor"
        self.setup_robot_controller(self.robot_ns)
        # receive key events on the window regardless of which child widget has focus
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # layout: grid style
        self.layout = QtWidgets.QGridLayout(self)
        # text instruction
        self.setup_instruction(row=1, column=0)

        # self.layout.addWidget()
        # button for driving pen
        self.pen_servo_layout = QtWidgets.QVBoxLayout()
        self.button_pen = self.generate_button(name="Drive Pen", sub_layout=self.pen_servo_layout, row=2, column=0)
        self.button_pen.clicked.connect(self.on_drive_pen)
        # button for driving servo
        # sub layout for servo on/off
        self.servo_layout = QtWidgets.QHBoxLayout()
        self.button_servo_on = self.generate_button(name="Servo On", sub_layout=self.servo_layout)
        self.button_servo_off = self.generate_button(name="Servo Off", sub_layout=self.servo_layout)
        self.button_servo_on.clicked.connect(lambda: self.on_servo(True))
        self.button_servo_off.clicked.connect(lambda: self.on_servo(False))
        self.pen_servo_layout.addLayout(self.servo_layout)
        self.layout.addLayout(self.pen_servo_layout, 2, 0)

        # input line for selecting robot
        self.setup_robot_selection(row=0, column=0)

        # state viewer
        self.setup_state_viewer(row=2, column=1)
        # onboard camera viewer
        self.setup_image_viewer(row=1, column=1)

        # grid ratio
        self.layout.setColumnStretch(0, 1)
        self.layout.setColumnStretch(1, 1)
        self.layout.setRowStretch(0, 1)
        self.layout.setRowStretch(1, 2)
        self.layout.setRowStretch(2, 1)

    def setup_instruction(self, row=1, column=0, width=1, height=1):
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
        self.layout.addWidget(text_instruction, row, column, width, height)

    def setup_robot_controller(self, namespace):
        self.pub_start = rospy.Publisher(namespace + "/teleop_command/start", Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher(namespace + "/teleop_command/takeoff", Empty, queue_size=1)
        self.pub_land = rospy.Publisher(namespace + "/teleop_command/land", Empty, queue_size=1)
        self.pub_force_landing = rospy.Publisher(namespace + "/teleop_command/force_landing", Empty, queue_size=1)
        self.pub_halt = rospy.Publisher(namespace + "/teleop_command/halt", Empty, queue_size=1)
        self.pub_nav = rospy.Publisher(namespace + "/uav/nav", FlightNav, queue_size=1)
        self.xy_vel = rospy.get_param("~xy_vel", 0.05)
        self.z_vel = rospy.get_param("~z_vel", 0.05)
        self.yaw_vel = rospy.get_param("~yaw_vel", 0.05)

    def setup_robot_selection(self, text=False, row=0, column=0, width=1, height=1):
        self.input_label = QtWidgets.QLabel("Robot namespace: " + self.robot_ns)
        self.input_layout = QtWidgets.QHBoxLayout()
        self.input_layout.addWidget(self.input_label)
        self.input_line = QtWidgets.QLineEdit(self)
        self.input_layout.addWidget(self.input_line)
        self.input_line.returnPressed.connect(self.returnPressedLineedit)
        self.layout.addLayout(self.input_layout, row, column, width, height)

    def returnPressedLineedit(self):
        # re-create publisher with new name
        self.pub_start.unregister()
        self.pub_takeoff.unregister()
        self.pub_land.unregister()
        self.pub_force_landing.unregister()
        self.pub_halt.unregister()
        self.pub_nav.unregister()
        self.setup_robot_controller(self.input_line.text())
        self.robot_ns = self.input_line.text()
        self.input_label.setText("Robot namespace: " + self.robot_ns)

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

    def on_drive_pen(self):
        self.pub_drive_pen.publish(Empty())

    def on_servo(self, state):
        self.pub_servo.publish(Bool(data=state))

    def setup_state_viewer(self, row=1, column=1, width=1, height=1):
        self.sub_state = rospy.Subscriber(self.robot_ns + "/uav/cog/odom", Odometry, self.cb_odom)
        msg = """
        <b>COG</b><br>
        x:     ---<br>
        y:     ---<br>
        z:     ---<br>
        roll:  ---<br>
        pitch: ---<br>
        yaw:   ---<br>
        """
        self.state_text = QtWidgets.QLabel(msg)
        self.state_text.setStyleSheet("QLabel { font-family: monospace; }")
        self.state_text.setFixedWidth(200)
        self.layout.addWidget(self.state_text, row, column, width, height)

    def cb_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        rotation = Rotation.from_quat(np.array([msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w]))
        roll, pitch, yaw = rotation.as_euler("xyz")

        msg = """
        <b>COG</b><br>
        x:     {:+7.3f}<br>
        y:     {:+7.3f}<br>
        z:     {:+7.3f}<br>
        roll:  {:+7.3f}<br>
        pitch: {:+7.3f}<br>
        yaw:   {:+7.3f}<br>
        """.format(x, y, z, roll, pitch, yaw)
        self.state_text.setText(msg)

    def setup_image_viewer(self, row=2, column=1, width=1, height=1):
        self.sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, self.cb_image)
        self.sub_compressed_image = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.cb_compressed_image)

        # dropdown to choose which of the two topics is shown in the viewer below
        self.image_source = "raw"
        self.image_source_combo = QtWidgets.QComboBox()
        self.image_source_combo.addItem("Image (raw)", "raw")
        self.image_source_combo.addItem("CompressedImage", "compressed")
        self.image_source_combo.currentIndexChanged.connect(self.on_image_source_changed)

        self.image_viewer = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("image.png")
        self.image_viewer.setPixmap(pixmap)

        image_layout = QtWidgets.QVBoxLayout()
        image_layout.addWidget(self.image_source_combo)
        image_layout.addWidget(self.image_viewer)
        self.layout.addLayout(image_layout, row, column, width, height)

    def on_image_source_changed(self, index):
        self.image_source = self.image_source_combo.itemData(index)

    # sensor_msgs/Image encoding -> QImage format (raw, uncompressed pixel data)
    IMAGE_ENCODING_TO_QT_FORMAT = {
        "rgb8": QtGui.QImage.Format_RGB888,
        "bgr8": QtGui.QImage.Format_BGR888,
        "rgba8": QtGui.QImage.Format_RGBA8888,
        "bgra8": QtGui.QImage.Format_ARGB32,
        "mono8": QtGui.QImage.Format_Grayscale8,
    }

    def cb_image(self, msg):
        if self.image_source != "raw":
            return
        qt_format = self.IMAGE_ENCODING_TO_QT_FORMAT.get(msg.encoding)
        if qt_format is None:
            rospy.logwarn_throttle(5, "cb_image: unsupported encoding '%s'" % msg.encoding)
            return
        # .copy() so the QImage owns its pixel data once msg.data goes out of scope
        image = QtGui.QImage(msg.data, msg.width, msg.height, msg.step, qt_format).copy()
        self.image_viewer.setPixmap(QtGui.QPixmap.fromImage(image))

    def cb_compressed_image(self, msg):
        if self.image_source != "compressed":
            return
        # msg.data holds an encoded image (jpeg/png/...); Qt's built-in codecs decode it directly
        image = QtGui.QImage.fromData(bytes(msg.data))
        if image.isNull():
            rospy.logwarn_throttle(5, "cb_compressed_image: failed to decode image (format='%s')" % msg.format)
            return
        self.image_viewer.setPixmap(QtGui.QPixmap.fromImage(image))

if __name__ == "__main__":
    rospy.init_node("control_interface", anonymous=True)

    app = QtWidgets.QApplication([])

    widget = MyWidget()
    widget.resize(960, 720)
    widget.show()

    sys.exit(app.exec())
