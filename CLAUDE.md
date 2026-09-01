# CLAUDE.md

このファイルは、このリポジトリのコードを扱う際にClaude Code (claude.ai/code) が参照するガイドです。

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

## 環境について

- `main.py`はPySide6（GUI）と`rospy`/`std_msgs`（ROS1）の両方をimportしている。
- PySide6はローカルのvenv `env/`（Python 3.10）にインストールされている。ROS1（`rospy`, `std_msgs`, `genpy`など）はvenvには**入っておらず**、システムのROSインストール `/opt/ros/one` から提供される。
- `rospy`を使うものを実行する場合は、先にROS環境をsourceしてから、ROSのdist-packagesとvenvのPySide6の両方がパスに通ったPythonを使うこと。
  ```bash
  source /opt/ros/one/setup.bash
  source env/bin/activate
  python main.py
  ```
- `rospy`は（`genpy`経由で）`PyYAML`にも依存しているが、現状この環境のどこにもインストールされていない。venvに追加するまでは`ModuleNotFoundError: No module named 'yaml'`が発生する想定。
- `main.py`はキーボード操作（後述）のために`aerial_robot_msgs.msg.FlightNav`もimportしている。このパッケージは現在のROS環境には**入っておらず**（`rospack find aerial_robot_msgs`が失敗する）、ROSワークスペースに追加するまで`main.py`はエンドツーエンドでは実行できない。これは意図的な選択（ユーザーが後で`aerial_robot_msgs`をワークスペースに追加する予定）であり、見落としではない — 別のメッセージ型に差し替えて「修正」しないこと。
- `libraries.md`にGUIライブラリの固定バージョン（現状PySide6 6.11.1のみ）を記録している — PySide6をバージョンアップする際は更新すること。
- `env/`はローカルのvirtualenvでgitignore対象。この配下のファイルは編集しないこと。

## アーキテクチャに関するメモ

- `main.py`内の`MyWidget`は、`__init__`の中でトップレベルの`QtWidgets.QGridLayout`を使い、手動でレイアウトを組み立てている。servo on/offのコントロールをまとめるため、ネストした`QHBoxLayout`（`servo_layout`）を1つのグリッドセルに配置している。
- ボタンは単一のヘルパー`generate_button(name, sub_layout=False, row=0, column=0, height=1, width=1)`を通して生成する。`sub_layout`を渡すと、グリッドに直接ではなくネストしたレイアウトにボタンを追加できる。ボタンの生成・命名（`setObjectName`）を統一するため、新しいコントロールを追加する際は`QPushButton`/`addWidget`を直接呼ぶのではなく、このパターンに従うこと。
- `rospy.init_node("control_interface", anonymous=True)`は`main.py`の`__main__`ブロック内で、`MyWidget`を生成する前に一度だけ呼ばれる。`rospy.Publisher`のインスタンスは`MyWidget.__init__`内で生成され、`self.pub_*`属性として保持される。各ボタンの`clicked`シグナルは、`.publish(...)`を呼ぶ小さな`on_*`ハンドラメソッドに接続されている。新しいROS連携コントロールを追加する際は、この「トピックごとにpublisherを持ち、`on_*`ハンドラで処理する」パターンに従うこと。
- 現在のボタン紐付けトピック: `/pen_switch`（`std_msgs/Empty`、"Drive Pen"ボタンからpublish）と`/servo_switch`（`std_msgs/Bool`、"Servo On"/"Servo Off"から`data=True`/`False`をpublish）。
- キーボード操作は`MyWidget`の`keyPressEvent`オーバーライドとして実装されている（ウィジェットは`QtCore.Qt.StrongFocus`を使っているのでキーイベントを受け取れる）。`reference/keyboard_command.py`のキー配置を踏襲しており、`r`/`t`/`l`/`f`/`h`は`std_msgs/Empty`を`/teleop_command/{start,takeoff,land,force_landing,halt}`にpublishし、`w`/`s`/`a`/`d`/`q`/`e`/`[`/`]`は（`publish_nav_key`経由で）`aerial_robot_msgs/FlightNav`メッセージを組み立てて`/uav/nav`にpublishする。ボタン起点のトピック（`/drive_pen`, `/servo`）はこれの影響を受けず、従来通りクリック時のみの挙動を保つ。速度の大きさはROSパラメータ`~xy_vel`/`~z_vel`/`~yaw_vel`（デフォルト`0.05`）から取得しており、参考スクリプトと同じ。
- トピックの購読は、状態表示ビューア（`setup_state_viewer`/`cb_odom`、`<namespace>/uav/cog/odom`を購読）と画像ビューア（`setup_image_viewer`/`cb_image`/`cb_compressed_image`、`/usb_cam/image_raw`と`/usb_cam/image_raw/compressed`を購読）に実装済み。
- **サブスクライバコールバックのスレッドセーフティ:** `rospy.Subscriber`のコールバックは、`rospy.spin()`を呼んでいるかどうかに関わらず、Qtのメインスレッドではなくrospyが持つバックグラウンドスレッドで呼び出される。Qtウィジェットの読み書きはGUIスレッドからしか行えないため、コールバック内でウィジェットのメソッド（`setText`, `setPixmap`など）を直接呼んではならない。これを行うと、PySide6環境で実際に発生する断続的な`Segmentation fault`の原因になる。タイミング依存のクラッシュなので、毎回再現するとは限らない。代わりに、`MyWidget`のクラス属性として`QtCore.Signal`を定義し、サブスクライバコールバックからは受信したデータを添えて`.emit()`するだけにし、それを（`__init__`や`setup_*`メソッド内で）実際のウィジェット更新を行う通常のメソッドに接続すること。発信元スレッドと接続先スロットのスレッドが異なる場合、Qtが自動的にその接続をqueued connectionに昇格させるため、スロットの処理本体はGUIスレッド上で実行される。`main.py`内の`odom_updated`/`update_state_text`と`image_updated`/`update_image_pixmap`がこのパターンのリファレンス実装なので、GUIに新しいサブスクライバを追加する際はこれに従うこと。
