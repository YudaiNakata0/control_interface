# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 設定
- 日本語でチャットを行う

## プロジェクトの説明

- PySide6を用いたGUI開発のプロジェクトであり、ロボットを操作するためのインターフェースを作成したい
- 今後ROS1と接続し、ボタンを押してトピックを配信するなどの機能を遂次追加する予定
- 配信されているトピックを拾って表示などの機能も後々追加予定

## ディレクトリ構成の説明
- main.pyがインターフェースの開発用のソースコード
- referece/にはインターフェースに実装したい機能をもった参考のソースコードなどを置く
  - keyboard_command.pyはキーボードを押してロボットの操作用のトピックを配信するスクリプト

## Environment

- `main.py` imports both PySide6 (GUI) and `rospy`/`std_msgs` (ROS1).
- PySide6 is installed in the local venv at `env/` (Python 3.10). ROS1 (`rospy`, `std_msgs`, `genpy`, etc.) is **not** in the venv — it comes from a system ROS install at `/opt/ros/one`.
- To run anything that touches `rospy`, source the ROS environment first, then use a Python that has both ROS's dist-packages and the venv's PySide6 on its path:
  ```bash
  source /opt/ros/one/setup.bash
  source env/bin/activate
  python main.py
  ```
- `rospy` also transitively needs `PyYAML` (via `genpy`), which is not currently installed anywhere in this environment — expect `ModuleNotFoundError: No module named 'yaml'` until it's added to the venv.
- `main.py` also imports `aerial_robot_msgs.msg.FlightNav` for keyboard teleop (see below). That package is **not** installed in this ROS environment (`rospack find aerial_robot_msgs` fails) — `main.py` cannot currently be run end-to-end until it's added to the ROS workspace. This was a deliberate choice (the user plans to add `aerial_robot_msgs` to the workspace later), not an oversight — don't "fix" it by swapping in a different message type.
- `libraries.md` tracks pinned GUI library versions (currently just PySide6 6.11.1) — update it when bumping PySide6.
- `env/` is a local virtualenv and is gitignored; don't edit files under it.

## Architecture notes

- `MyWidget` (in `main.py`) builds its layout by hand in `__init__` using a top-level `QtWidgets.QGridLayout`, with a nested `QHBoxLayout` (`servo_layout`) placed into one grid cell for grouped on/off controls.
- Buttons are created through the single helper `generate_button(name, sub_layout=False, row=0, column=0, height=1, width=1)`: pass `sub_layout` to add the button into a nested layout instead of directly into the grid. Follow this pattern when adding new controls rather than calling `QPushButton` / `addWidget` directly, to keep button creation and naming (`setObjectName`) consistent.
- `rospy.init_node("control_interface", anonymous=True)` is called once in `main.py`'s `__main__` block, before `MyWidget` is constructed. `rospy.Publisher` instances are created in `MyWidget.__init__` and stored as `self.pub_*` attributes; each button's `clicked` signal is wired to a small `on_*` handler method that calls `.publish(...)`. Follow this publisher-per-topic + `on_*` handler pattern when adding new ROS-connected controls.
- Current button topics: `/drive_pen` (`std_msgs/Empty`, published by the "Drive Pen" button) and `/servo` (`std_msgs/Bool`, `data=True`/`False` published by "Servo On"/"Servo Off").
- Keyboard teleop is implemented as a `keyPressEvent` override on `MyWidget` (the widget uses `QtCore.Qt.StrongFocus` so it receives key events), mirroring the key layout of `reference/keyboard_command.py`: `r`/`t`/`l`/`f`/`h` publish `std_msgs/Empty` on `/teleop_command/{start,takeoff,land,force_landing,halt}`, and `w`/`s`/`a`/`d`/`q`/`e`/`[`/`]` build an `aerial_robot_msgs/FlightNav` message (via `publish_nav_key`) published on `/uav/nav`. Button-triggered topics (`/drive_pen`, `/servo`) are unaffected by this and keep their existing click-only behavior. Velocity magnitudes come from ROS params `~xy_vel`/`~z_vel`/`~yaw_vel` (default `0.05`), same as the reference script.
- Topic subscription (to display incoming topic data in the GUI) is planned but not yet implemented.
