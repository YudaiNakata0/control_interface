#!/usr/bin/env python3
import sys
import os
import glob
import random
import yaml
from PySide6 import QtCore, QtWidgets, QtGui
import rospy
import numpy as np
from std_msgs.msg import Empty, Bool
from aerial_robot_msgs.msg import FlightNav, SimpleFlightNav
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import Image, CompressedImage

class MyWidget(QtWidgets.QWidget):
    # rospy delivers subscriber callbacks on background threads, but Qt widgets
    # may only be touched from the GUI thread. These signals are emitted from
    # the callback threads and, since sender/receiver threads differ, Qt
    # auto-queues the connection so the connected slot below runs on the GUI
    # thread instead.
    odom_updated = QtCore.Signal(str, float, float, float, float, float, float)
    image_updated = QtCore.Signal(QtGui.QImage)

    def __init__(self):
        super().__init__()
        # ROS publishers
        self.pub_drive_pen = rospy.Publisher("/pen_switch", Empty, queue_size=1)
        self.pub_servo = rospy.Publisher("/servo_switch", Bool, queue_size=1)

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
        # toggle button for driving servo (checked = on)
        self.button_servo = self.generate_button(name="Servo", sub_layout=self.pen_servo_layout, checkable=True)
        self.button_servo.clicked.connect(self.on_servo)
        self.update_toggle_text(self.button_servo, "Servo", False)
        self.layout.addLayout(self.pen_servo_layout, 2, 0)

        # input line for selecting robot
        self.setup_robot_selection(row=0, column=0)
        # indicator showing whether key events currently reach keyPressEvent
        self.setup_teleop_indicator(row=0, column=1)

        # state viewer, with target pose control (tabbed per topic) placed beside it
        self.setup_state_viewer()
        self.setup_target_pose_control()
        state_row_layout = QtWidgets.QHBoxLayout()
        state_row_layout.addWidget(self.state_text)
        state_row_layout.addWidget(self.target_tabs)
        self.layout.addLayout(state_row_layout, 2, 1)
        # onboard camera viewer
        self.setup_image_viewer(row=1, column=1)

        # control panel column: impedance settings (tabbed, so modes that are never
        # used at the same time can share the space) and attitude command below it
        control_layout = QtWidgets.QVBoxLayout()
        self.control_tabs = QtWidgets.QTabWidget()
        self.control_tabs.addTab(self.create_impedance_panel(), "Impedance")
        control_layout.addWidget(self.control_tabs)
        control_layout.addWidget(self.create_attitude_panel())
        control_layout.addStretch()
        self.layout.addLayout(control_layout, 1, 2, 2, 1)

        # grid ratio: instruction : viewer : control panel = 2 : 5 : 3
        self.layout.setColumnStretch(0, 2)
        self.layout.setColumnStretch(1, 5)
        self.layout.setColumnStretch(2, 3)
        self.layout.setRowStretch(0, 0)
        self.layout.setRowStretch(1, 2)
        self.layout.setRowStretch(2, 1)

        # keyboard teleop only works while no text-entry widget holds focus:
        # watch focus changes for the indicator, and catch Esc app-wide to release focus
        self.latest_position = None
        app = QtWidgets.QApplication.instance()
        app.focusChanged.connect(self.on_focus_changed)
        app.installEventFilter(self)

        # preset selector is set up last: loading a preset writes into the input widgets created above
        self.setup_preset_selector(row=0, column=2)
        self.load_startup_preset()

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
        <b>Esc</b>: leave input field (re-enable keys)<br>
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
        self.pub_target_pose = rospy.Publisher(namespace + "/target_pose", PoseStamped, queue_size=10)
        self.pub_simple_nav = rospy.Publisher(namespace + "/simple_nav", SimpleFlightNav, queue_size=1)
        self.pub_impedance_flag = rospy.Publisher(namespace + "/impedance_flag", Bool, queue_size=1)
        self.pub_impedance_direction = rospy.Publisher(namespace + "/impedance_direction", Vector3, queue_size=1)
        self.pub_impedance_pos = rospy.Publisher(namespace + "/desire_pos_for_impedance", Vector3, queue_size=1)
        self.pub_target_rpy = rospy.Publisher(namespace + "/final_target_baselink_rpy", Vector3Stamped, queue_size=1)
        self.xy_vel = rospy.get_param("~xy_vel", 0.02)
        self.z_vel = rospy.get_param("~z_vel", 0.02)
        self.yaw_vel = rospy.get_param("~yaw_vel", 0.02)

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
        self.pub_target_pose.unregister()
        self.pub_simple_nav.unregister()
        self.pub_impedance_flag.unregister()
        self.pub_impedance_direction.unregister()
        self.pub_impedance_pos.unregister()
        self.pub_target_rpy.unregister()
        self.setup_robot_controller(self.input_line.text())
        self.robot_ns = self.input_line.text()
        self.input_label.setText("Robot namespace: " + self.robot_ns)
        # re-subscribe odom of the new robot and clear values of the previous one
        self.sub_state.unregister()
        self.subscribe_state(self.robot_ns)
        self.reset_state_text()
        self.latest_position = None

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
    def generate_button(self, name, sub_layout=False, row=0, column=0, height=1, width=1, checkable=False):
        button = QtWidgets.QPushButton()
        button.setObjectName(name)
        button.setText(name)
        if checkable:
            # toggle button: checked state = last state sent from this GUI
            button.setCheckable(True)
            button.setStyleSheet("QPushButton:checked { background-color: #4caf50; color: white; font-weight: bold; }")
        if sub_layout == False:
            self.layout.addWidget(button, row, column, height, width)
        else:
            sub_layout.addWidget(button)
        return button

    def update_toggle_text(self, button, name, state):
        button.setText("{}: {}".format(name, "ON" if state else "OFF"))

    def on_drive_pen(self):
        self.pub_drive_pen.publish(Empty())

    def on_servo(self, state):
        self.pub_servo.publish(Bool(data=state))
        self.update_toggle_text(self.button_servo, "Servo", state)

    # keyboard focus handling: text-entry widgets swallow teleop keys while focused
    TEXT_INPUT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox)

    def setup_teleop_indicator(self, row=0, column=1, height=1, width=1):
        self.teleop_indicator = QtWidgets.QLabel()
        self.teleop_indicator.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.teleop_indicator, row, column, height, width)
        self.update_teleop_indicator(False)

    def update_teleop_indicator(self, active):
        if active:
            text = "Keyboard teleop: ACTIVE"
            color = "#4caf50"
        else:
            text = "Keyboard teleop: INACTIVE (press Esc or click empty space)"
            color = "#e53935"
        self.teleop_indicator.setText(text)
        self.teleop_indicator.setStyleSheet(
            "QLabel { background-color: %s; color: white; font-weight: bold; padding: 4px; border-radius: 4px; }" % color)

    def on_focus_changed(self, old, new):
        active = (new is not None
                  and (new is self or self.isAncestorOf(new))
                  and not isinstance(new, self.TEXT_INPUT_WIDGETS))
        self.update_teleop_indicator(active)

    def release_input_focus(self):
        self.setFocus(QtCore.Qt.OtherFocusReason)

    def eventFilter(self, watched, event):
        # Esc is caught app-wide because focused input widgets would otherwise consume it
        if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Escape:
            focus = QtWidgets.QApplication.focusWidget()
            if focus is not None and self.isAncestorOf(focus):
                self.release_input_focus()
                return True
        return super().eventFilter(watched, event)

    def mousePressEvent(self, event):
        # clicks on empty space (or labels such as the image viewer) end up here
        self.release_input_focus()
        super().mousePressEvent(event)

    def setup_state_viewer(self):
        self.subscribe_state(self.robot_ns)
        self.odom_updated.connect(self.update_state_text)
        self.state_text = QtWidgets.QLabel()
        self.state_text.setStyleSheet("QLabel { font-family: monospace; }")
        self.state_text.setFixedWidth(200)
        self.reset_state_text()

    def subscribe_state(self, namespace):
        # namespace is passed to the callback so odom still in flight from a previous robot can be dropped
        self.sub_state = rospy.Subscriber(namespace + "/uav/cog/odom", Odometry, self.cb_odom, callback_args=namespace)

    def reset_state_text(self):
        msg = """
        <b>COG</b><br>
        x:     ---<br>
        y:     ---<br>
        z:     ---<br>
        roll:  ---<br>
        pitch: ---<br>
        yaw:   ---<br>
        """
        self.state_text.setText(msg)

    def cb_odom(self, msg, namespace):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        rotation = Rotation.from_quat(np.array([msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w]))
        roll, pitch, yaw = rotation.as_euler("xyz")
        # emit instead of touching the widget here: this callback runs on rospy's
        # subscriber thread, not the GUI thread
        self.odom_updated.emit(namespace, x, y, z, roll, pitch, yaw)

    def update_state_text(self, namespace, x, y, z, roll, pitch, yaw):
        # a message from the previous robot may arrive right after a namespace change
        if namespace != self.robot_ns:
            return
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
        # kept for "Use Current" in the impedance panel (updated on the GUI thread)
        self.latest_position = (x, y, z)

    # number of preset x/y/z input+send panels shown side by side in each tab
    NUM_TARGET_POSE_PANELS = 2
    # tab key -> (tab label, panel title, send button name); each tab publishes to its own topic
    TARGET_POSE_MODES = {
        "target_pose": ("Target Pose", "Target Pose", "Send Pose"),
        "simple_nav": ("Simple Nav", "Simple Nav", "Send Nav"),
    }

    def setup_target_pose_control(self):
        # only one of the target topics is used at a time, so they share the space as tabs
        self.target_tabs = QtWidgets.QTabWidget()
        self.target_pose_inputs = {}
        for mode, (tab_label, _, _) in self.TARGET_POSE_MODES.items():
            tab = QtWidgets.QWidget()
            panels_layout = QtWidgets.QHBoxLayout(tab)
            self.target_pose_inputs[mode] = []
            for index in range(self.NUM_TARGET_POSE_PANELS):
                panels_layout.addWidget(self.create_target_pose_panel(mode, index))
            self.target_tabs.addTab(tab, tab_label)

    def create_target_pose_panel(self, mode, index):
        _, title, button_name = self.TARGET_POSE_MODES[mode]
        panel = QtWidgets.QWidget()
        pose_layout = QtWidgets.QVBoxLayout(panel)
        pose_layout.setContentsMargins(0, 0, 0, 0)

        pose_layout.addWidget(QtWidgets.QLabel("<b>{} {}</b>".format(title, index + 1)))

        inputs = {}
        for axis in ("x", "y", "z"):
            axis_layout = QtWidgets.QHBoxLayout()
            axis_layout.addWidget(QtWidgets.QLabel(axis + ":"))
            line_edit = QtWidgets.QLineEdit("0.0")
            line_edit.setFixedWidth(80)
            axis_layout.addWidget(line_edit)
            pose_layout.addLayout(axis_layout)
            inputs[axis] = line_edit
        self.target_pose_inputs[mode].append(inputs)

        button_send_pose = self.generate_button(name="{} {}".format(button_name, index + 1), sub_layout=pose_layout)
        button_send_pose.clicked.connect(lambda checked=False, mode=mode, index=index: self.on_send_target_pose(mode, index))
        return panel

    def on_send_target_pose(self, mode, index):
        inputs = self.target_pose_inputs[mode][index]
        try:
            x = float(inputs["x"].text())
            y = float(inputs["y"].text())
            z = float(inputs["z"].text())
        except ValueError:
            rospy.logwarn("Invalid target pose input. Enter numeric x y z")
            return

        if z < 0.7:
            rospy.logwarn("too low z value")
            return

        if mode == "simple_nav":
            self.publish_simple_nav(x, y, z)
        else:
            self.publish_target_pose(x, y, z)

    def publish_target_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now() + rospy.Duration(10.0)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        self.pub_target_pose.publish(msg)
        rospy.loginfo("Published PoseStamped: (%.3f, %.3f, %.3f) time=now+10s", x, y, z)

    def publish_simple_nav(self, x, y, z):
        msg = SimpleFlightNav()
        msg.x_control_mode = SimpleFlightNav.POS_MODE
        msg.y_control_mode = SimpleFlightNav.POS_MODE
        msg.z_control_mode = SimpleFlightNav.POS_MODE
        msg.pos_x = x
        msg.pos_y = y
        msg.pos_z = z

        self.pub_simple_nav.publish(msg)
        rospy.loginfo("Published SimpleFlightNav (POS_MODE): (%.3f, %.3f, %.3f)", x, y, z)

    # row of labeled spin boxes (e.g. x/y/z); returns (layout, {axis: QDoubleSpinBox})
    def create_vector_inputs(self, axes, minimum, maximum, step, default=0.0, decimals=3, unit=""):
        vector_layout = QtWidgets.QHBoxLayout()
        inputs = {}
        for axis in axes:
            vector_layout.addWidget(QtWidgets.QLabel(axis + ":"))
            spin_box = QtWidgets.QDoubleSpinBox()
            spin_box.setRange(minimum, maximum)
            spin_box.setSingleStep(step)
            spin_box.setDecimals(decimals)
            spin_box.setValue(default.get(axis, 0.0) if isinstance(default, dict) else default)
            if unit:
                spin_box.setSuffix(" " + unit)
            vector_layout.addWidget(spin_box)
            inputs[axis] = spin_box
        return vector_layout, inputs

    def create_impedance_panel(self):
        panel = QtWidgets.QWidget()
        impedance_layout = QtWidgets.QVBoxLayout(panel)

        self.button_impedance = self.generate_button(name="Impedance", sub_layout=impedance_layout, checkable=True)
        self.button_impedance.clicked.connect(self.on_impedance_flag)
        self.update_toggle_text(self.button_impedance, "Impedance", False)

        direction_box = QtWidgets.QGroupBox("Direction")
        direction_layout = QtWidgets.QVBoxLayout(direction_box)
        axes_layout, self.impedance_direction_inputs = self.create_vector_inputs(
            ("x", "y", "z"), -10.0, 10.0, 0.1, default={"x": 1.0})
        direction_layout.addLayout(axes_layout)
        button_direction = self.generate_button(name="Set Direction", sub_layout=direction_layout)
        button_direction.clicked.connect(self.on_impedance_direction)
        impedance_layout.addWidget(direction_box)

        pos_box = QtWidgets.QGroupBox("Desired Position")
        pos_layout = QtWidgets.QVBoxLayout(pos_box)
        axes_layout, self.impedance_pos_inputs = self.create_vector_inputs(
            ("x", "y", "z"), -10.0, 10.0, 0.05, unit="m")
        pos_layout.addLayout(axes_layout)
        pos_button_layout = QtWidgets.QHBoxLayout()
        button_use_current = self.generate_button(name="Use Current", sub_layout=pos_button_layout)
        button_use_current.clicked.connect(self.on_use_current_position)
        button_pos = self.generate_button(name="Set Pos", sub_layout=pos_button_layout)
        button_pos.clicked.connect(self.on_impedance_pos)
        pos_layout.addLayout(pos_button_layout)
        impedance_layout.addWidget(pos_box)

        impedance_layout.addStretch()
        return panel

    def on_impedance_flag(self, state):
        self.pub_impedance_flag.publish(Bool(data=state))
        self.update_toggle_text(self.button_impedance, "Impedance", state)

    def on_impedance_direction(self):
        inputs = self.impedance_direction_inputs
        msg = Vector3(x=inputs["x"].value(), y=inputs["y"].value(), z=inputs["z"].value())
        self.pub_impedance_direction.publish(msg)
        rospy.loginfo("Published impedance direction: (%.3f, %.3f, %.3f)", msg.x, msg.y, msg.z)

    def on_use_current_position(self):
        if self.latest_position is None:
            rospy.logwarn("No odometry received yet")
            return
        for axis, value in zip(("x", "y", "z"), self.latest_position):
            self.impedance_pos_inputs[axis].setValue(value)

    def on_impedance_pos(self):
        inputs = self.impedance_pos_inputs
        msg = Vector3(x=inputs["x"].value(), y=inputs["y"].value(), z=inputs["z"].value())
        self.pub_impedance_pos.publish(msg)
        rospy.loginfo("Published impedance desired position: (%.3f, %.3f, %.3f)", msg.x, msg.y, msg.z)

    # safety limit [rad] for each of roll/pitch/yaw sent to final_target_baselink_rpy
    RPY_LIMIT = 0.6

    def create_attitude_panel(self):
        attitude_box = QtWidgets.QGroupBox("Attitude (RPY)")
        attitude_layout = QtWidgets.QVBoxLayout(attitude_box)

        self.rpy_inputs = {}
        for axis in ("roll", "pitch", "yaw"):
            axis_layout, inputs = self.create_vector_inputs(
                (axis,), -self.RPY_LIMIT, self.RPY_LIMIT, 0.05, unit="rad")
            attitude_layout.addLayout(axis_layout)
            self.rpy_inputs.update(inputs)

        rpy_button_layout = QtWidgets.QHBoxLayout()
        button_rpy = self.generate_button(name="Send RPY", sub_layout=rpy_button_layout)
        button_rpy.clicked.connect(self.on_send_rpy)
        button_level = self.generate_button(name="Level (r,p=0)", sub_layout=rpy_button_layout)
        button_level.clicked.connect(self.on_level_rpy)
        attitude_layout.addLayout(rpy_button_layout)
        return attitude_box

    def on_send_rpy(self):
        roll = self.rpy_inputs["roll"].value()
        pitch = self.rpy_inputs["pitch"].value()
        yaw = self.rpy_inputs["yaw"].value()
        # the spin boxes are already range-limited; check again so nothing out of range is ever published
        if max(abs(roll), abs(pitch), abs(yaw)) > self.RPY_LIMIT:
            rospy.logwarn("RPY out of range (limit: +-%.2f rad)", self.RPY_LIMIT)
            return

        msg = Vector3Stamped()
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        self.pub_target_rpy.publish(msg)
        rospy.loginfo("Published target baselink rpy: (%.3f, %.3f, %.3f)", roll, pitch, yaw)

    def on_level_rpy(self):
        # keep yaw as entered, only level roll/pitch
        self.rpy_inputs["roll"].setValue(0.0)
        self.rpy_inputs["pitch"].setValue(0.0)
        self.on_send_rpy()

    # presets of input values: one YAML file per preset in this directory (see presets/default.yaml)
    PRESET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "presets")

    def setup_preset_selector(self, row=0, column=2, height=1, width=1):
        preset_layout = QtWidgets.QHBoxLayout()
        preset_layout.addWidget(QtWidgets.QLabel("Preset:"))
        self.preset_combo = QtWidgets.QComboBox()
        self.preset_combo.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        preset_layout.addWidget(self.preset_combo)
        button_load = self.generate_button(name="Load", sub_layout=preset_layout)
        button_load.clicked.connect(self.on_load_preset)
        # re-scan PRESET_DIR so files added/renamed while the GUI is running show up
        button_refresh = self.generate_button(name="Refresh", sub_layout=preset_layout)
        button_refresh.clicked.connect(self.refresh_preset_list)
        self.layout.addLayout(preset_layout, row, column, height, width)
        self.refresh_preset_list()

    def refresh_preset_list(self):
        current = self.preset_combo.currentText()
        self.preset_combo.clear()
        for path in sorted(glob.glob(os.path.join(self.PRESET_DIR, "*.yaml"))):
            self.preset_combo.addItem(os.path.splitext(os.path.basename(path))[0], path)
        index = self.preset_combo.findText(current)
        if index >= 0:
            self.preset_combo.setCurrentIndex(index)

    def load_startup_preset(self):
        # ~preset: name of the preset (file name without .yaml) applied at startup
        name = rospy.get_param("~preset", "default")
        index = self.preset_combo.findText(name)
        if index < 0:
            if rospy.has_param("~preset"):
                rospy.logwarn("Preset '%s' not found in %s", name, self.PRESET_DIR)
            return
        self.preset_combo.setCurrentIndex(index)
        self.load_preset(self.preset_combo.itemData(index))

    def on_load_preset(self):
        path = self.preset_combo.currentData()
        if path is None:
            rospy.logwarn("No preset file in %s", self.PRESET_DIR)
            return
        self.load_preset(path)
        # the combo box took focus; hand it back so keyboard teleop works again
        self.release_input_focus()

    def load_preset(self, path):
        # only fills the input widgets; nothing is published until a send button is pressed
        try:
            with open(path) as f:
                preset = yaml.safe_load(f)
            namespace, line_edit_values, spin_box_values = self.parse_preset(preset)
        except (OSError, yaml.YAMLError, ValueError) as e:
            rospy.logwarn("Failed to load preset %s: %s", path, e)
            QtWidgets.QMessageBox.warning(self, "Preset", "Failed to load preset:\n{}\n\n{}".format(path, e))
            return False

        for line_edit, text in line_edit_values:
            line_edit.setText(text)
        for spin_box, value in spin_box_values:
            spin_box.setValue(value)
        if namespace is not None and namespace != self.robot_ns:
            self.input_line.setText(namespace)
            self.returnPressedLineedit()
        rospy.loginfo("Loaded preset: %s", path)
        return True

    def parse_preset(self, preset):
        # the whole file is validated before anything is applied, so a typo never leaves
        # the inputs half-updated; keys left out of the file keep the current values
        if preset is None:
            preset = {}
        self.check_preset_keys(preset, ("namespace", "target_pose", "impedance", "rpy"), "preset")
        line_edit_values = []
        spin_box_values = []

        namespace = preset.get("namespace")
        if namespace is not None and not isinstance(namespace, str):
            raise ValueError("namespace: must be a string")

        targets = preset.get("target_pose") or {}
        self.check_preset_keys(targets, tuple(self.TARGET_POSE_MODES), "target_pose")
        for mode, panels in targets.items():
            name = "target_pose.{}".format(mode)
            if not isinstance(panels, list) or len(panels) > self.NUM_TARGET_POSE_PANELS:
                raise ValueError("{}: must be a list of at most {} entries".format(name, self.NUM_TARGET_POSE_PANELS))
            for index, values in enumerate(panels):
                # null entry keeps that panel as is
                if values is None:
                    continue
                inputs = self.target_pose_inputs[mode][index]
                for axis, value in self.parse_preset_vector(values, ("x", "y", "z"), "{}[{}]".format(name, index)).items():
                    line_edit_values.append((inputs[axis], str(value)))

        impedance = preset.get("impedance") or {}
        self.check_preset_keys(impedance, ("direction", "desired_pos"), "impedance")
        if "direction" in impedance:
            spin_box_values += self.parse_preset_spin_boxes(impedance["direction"], self.impedance_direction_inputs, "impedance.direction")
        if "desired_pos" in impedance:
            spin_box_values += self.parse_preset_spin_boxes(impedance["desired_pos"], self.impedance_pos_inputs, "impedance.desired_pos")

        if "rpy" in preset:
            spin_box_values += self.parse_preset_spin_boxes(preset["rpy"], self.rpy_inputs, "rpy")
        return namespace, line_edit_values, spin_box_values

    def check_preset_keys(self, values, keys, name):
        if not isinstance(values, dict):
            raise ValueError("{}: must be a mapping".format(name))
        unknown = set(values) - set(keys)
        if unknown:
            raise ValueError("{}: unknown key(s) {} (allowed: {})".format(name, sorted(unknown, key=str), ", ".join(keys)))

    def parse_preset_vector(self, values, axes, name):
        self.check_preset_keys(values, axes, name)
        result = {}
        for axis, value in values.items():
            # bool is a subclass of int, but "x: true" is certainly a mistake
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("{}.{}: must be a number, got {!r}".format(name, axis, value))
            result[axis] = float(value)
        return result

    def parse_preset_spin_boxes(self, values, inputs, name):
        spin_box_values = []
        for axis, value in self.parse_preset_vector(values, tuple(inputs), name).items():
            spin_box = inputs[axis]
            # rejected rather than silently clamped by the spin box (this also keeps rpy within RPY_LIMIT)
            if not spin_box.minimum() <= value <= spin_box.maximum():
                raise ValueError("{}.{}: {} is out of range [{}, {}]".format(
                    name, axis, value, spin_box.minimum(), spin_box.maximum()))
            spin_box_values.append((spin_box, value))
        return spin_box_values

    def setup_image_viewer(self, row=2, column=1, width=1, height=1):
        self.sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, self.cb_image)
        self.sub_compressed_image = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.cb_compressed_image)
        self.image_updated.connect(self.update_image_pixmap)

        # dropdown to choose which of the two topics is shown in the viewer below
        self.image_source = "raw"
        self.image_source_combo = QtWidgets.QComboBox()
        self.image_source_combo.addItem(QtGui.QIcon(), "Image (raw)", "raw")
        self.image_source_combo.addItem(QtGui.QIcon(), "CompressedImage", "compressed")
        self.image_source_combo.currentIndexChanged.connect(self.on_image_source_changed)

        self.image_viewer = QtWidgets.QLabel()
        # let the label grow/shrink with its column instead of sizing to the pixmap
        self.image_viewer.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        # QLabel's minimum size hint equals the pixmap size even with scaled contents,
        # which would stop the window from shrinking below the camera resolution
        self.image_viewer.setMinimumSize(1, 1)
        # scaled by Qt at paint time (cheap; aspect ratio is not kept)
        self.image_viewer.setScaledContents(True)
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
        # emit instead of touching the widget here: this callback runs on rospy's
        # subscriber thread, not the GUI thread
        self.image_updated.emit(image)

    def cb_compressed_image(self, msg):
        if self.image_source != "compressed":
            return
        # msg.data holds an encoded image (jpeg/png/...); Qt's built-in codecs decode it directly
        image = QtGui.QImage.fromData(bytes(msg.data))
        if image.isNull():
            rospy.logwarn_throttle(5, "cb_compressed_image: failed to decode image (format='%s')" % msg.format)
            return
        self.image_updated.emit(image)

    def update_image_pixmap(self, image):
        self.image_viewer.setPixmap(QtGui.QPixmap.fromImage(image))

if __name__ == "__main__":
    rospy.init_node("control_interface", anonymous=True)

    app = QtWidgets.QApplication([])

    widget = MyWidget()
    widget.resize(960, 720)
    widget.show()

    sys.exit(app.exec())
