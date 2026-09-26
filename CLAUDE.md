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
- `rospy`は（`genpy`経由で）`PyYAML`にも依存している。venvには入っていないが、システムの`/usr/lib/python3/dist-packages`のものが上記の手順で読み込まれる。`main.py`もプリセットファイルの読み込みに`yaml`を使っている。
- `main.py`はキーボード操作（後述）と目標位置送信のために`aerial_robot_msgs.msg`の`FlightNav`/`SimpleFlightNav`もimportしている。このパッケージは`/opt/ros/one`には**入っておらず**、`~/ros/jsk_aerial_robot_ws`（`src/jsk_aerial_robot/aerial_robot_msgs`）にある。実行時は`source ~/ros/jsk_aerial_robot_ws/devel/setup.bash`も必要。importエラーが出ても、別のメッセージ型に差し替えて「修正」しないこと。
- `libraries.md`にGUIライブラリの固定バージョン（現状PySide6 6.11.1のみ）を記録している — PySide6をバージョンアップする際は更新すること。
- `env/`はローカルのvirtualenvでgitignore対象。この配下のファイルは編集しないこと。

## アーキテクチャに関するメモ

- `main.py`内の`MyWidget`は、`__init__`の中でトップレベルの`QtWidgets.QGridLayout`を使い、手動でレイアウトを組み立てている。列は左から「キー操作説明・Drive Pen/Servo」「画像ビューア・COG状態・目標位置タブ（`target_tabs`）」「制御パネル」で、幅の比率は2:5:3。0行目にはnamespace入力、キーボード操作の有効/無効インジケータ、プリセット選択（後述）を左から置いている。制御パネル列は`QTabWidget`（`control_tabs`、現状は"Impedance"タブのみ）と、その下の姿勢指令`QGroupBox`で構成される。同時に使わないモード（将来追加予定のcontrol_mode切り替えなど）は、`control_tabs`に新しいタブとして追加すること。目標位置の送信も同様に`target_tabs`（`QTabWidget`）で配信先トピックごとにタブを分けており、タブの種類は`TARGET_POSE_MODES`で定義している（各タブにx/y/z入力パネルが`NUM_TARGET_POSE_PANELS`個並ぶ）。
- ボタンは単一のヘルパー`generate_button(name, sub_layout=False, row=0, column=0, height=1, width=1, checkable=False)`を通して生成する。`sub_layout`を渡すと、グリッドに直接ではなくネストしたレイアウトにボタンを追加できる。ボタンの生成・命名（`setObjectName`）を統一するため、新しいコントロールを追加する際は`QPushButton`/`addWidget`を直接呼ぶのではなく、このパターンに従うこと。ON/OFFを切り替えるコントロールは`generate_button(..., checkable=True)`でトグルボタンとして作り、`clicked(bool)`を`on_*`に接続し、`update_toggle_text`でラベルを"Name: ON/OFF"に更新する（チェック状態は、このGUIから最後に送った値を表す）。数値入力には`create_vector_inputs`（`QDoubleSpinBox`の行）を使う。
- `rospy.init_node("control_interface", anonymous=True)`は`main.py`の`__main__`ブロック内で、`MyWidget`を生成する前に一度だけ呼ばれる。`rospy.Publisher`のインスタンスは`MyWidget.__init__`内で生成され、`self.pub_*`属性として保持される。各ボタンの`clicked`シグナルは、`.publish(...)`を呼ぶ小さな`on_*`ハンドラメソッドに接続されている。新しいROS連携コントロールを追加する際は、この「トピックごとにpublisherを持ち、`on_*`ハンドラで処理する」パターンに従うこと。
- 現在のボタン紐付けトピック: `/pen_switch`（`std_msgs/Empty`、"Drive Pen"ボタンからpublish）、`/servo_switch`（`std_msgs/Bool`、"Servo"トグルボタンから）。ロボットnamespace付きのものは、`<ns>/impedance_flag`（`std_msgs/Bool`、"Impedance"トグル）、`<ns>/impedance_direction`・`<ns>/desire_pos_for_impedance`（`geometry_msgs/Vector3`）、`<ns>/final_target_baselink_rpy`（`geometry_msgs/Vector3Stamped`、`vector`にroll/pitch/yawをラジアンで設定し、headerは空）、`<ns>/target_pose`（`PoseStamped`、"Target Pose"タブ）、`<ns>/simple_nav`（`aerial_robot_msgs/SimpleFlightNav`、"Simple Nav"タブ。x/y/zとも`POS_MODE`で`pos_*`を設定）。どちらもz<0.7の値はpublishしない。安全のため、rpyは`RPY_LIMIT`（±0.6 rad）を超える値をpublishしない。spinboxの範囲制限と`on_send_rpy`での再チェックの二重で守っているので、どちらも外さないこと。namespace付きのpublisherを追加する場合は、`setup_robot_controller`で生成し、`returnPressedLineedit`で`unregister`すること。
- キーボード操作は`MyWidget`の`keyPressEvent`オーバーライドとして実装されている（ウィジェットは`QtCore.Qt.StrongFocus`を使っているのでキーイベントを受け取れる）。`reference/keyboard_command.py`のキー配置を踏襲しており、`r`/`t`/`l`/`f`/`h`は`std_msgs/Empty`を`/teleop_command/{start,takeoff,land,force_landing,halt}`にpublishし、`w`/`s`/`a`/`d`/`q`/`e`/`[`/`]`は（`publish_nav_key`経由で）`aerial_robot_msgs/FlightNav`メッセージを組み立てて`/uav/nav`にpublishする。ボタン起点のトピック（`/drive_pen`, `/servo`）はこれの影響を受けず、従来通りクリック時のみの挙動を保つ。`QLineEdit`/`QAbstractSpinBox`/`QComboBox`にフォーカスがある間は、キーがそのウィジェットに取られてキーボード操作が効かない。そこで、`QApplication`に`eventFilter`を仕掛けてEscを捕まえ、`mousePressEvent`で空いている場所のクリックを受けて、`release_input_focus`でフォーカスを`MyWidget`に戻している。状態は`on_focus_changed`がインジケータに反映する。速度の大きさはROSパラメータ`~xy_vel`/`~z_vel`/`~yaw_vel`（デフォルト`0.05`）から取得しており、参考スクリプトと同じ。
- **入力値のプリセット:** 各入力欄の値は`presets/*.yaml`（1ファイル=1プリセット、`PRESET_DIR`）から読み込める。0行目右のコンボボックスで選んで"Load"で適用し、"Refresh"でディレクトリを再スキャンする。起動時はROSパラメータ`~preset`（拡張子なしのファイル名、デフォルト`default`）のプリセットを適用する。対応するキーは`namespace`/`target_pose`（`TARGET_POSE_MODES`のキーごとのリスト）/`impedance`（`direction`, `desired_pos`）/`rpy`で、書式は`presets/default.yaml`のコメントを参照。書かれていないキーは現在値のまま残る。`parse_preset`でファイル全体を検証してから適用するため、エラー（未知のキー、数値以外、spinboxの範囲外）があれば何も変更せずダイアログを出す。範囲外の値はspinboxに丸めさせず拒否する（rpyの`RPY_LIMIT`を守るため）。プリセットの読み込みは入力欄を埋めるだけで、publishはしないこと。新しい入力欄を追加したら、`parse_preset`と`presets/default.yaml`にも対応を追加すること。
- トピックの購読は、状態表示ビューア（`setup_state_viewer`/`cb_odom`、`<namespace>/uav/cog/odom`を購読。namespaceを変更すると`returnPressedLineedit`が購読し直して表示をリセットする。購読直後に古いnamespaceのメッセージが遅れて届くことがあるため、`callback_args`でnamespaceをシグナルに乗せ、`update_state_text`側で現在の`robot_ns`と一致しないものは捨てている）と画像ビューア（`setup_image_viewer`/`cb_image`/`cb_compressed_image`、`/usb_cam/image_raw`と`/usb_cam/image_raw/compressed`を購読。画像の拡大縮小は軽さを優先して`setScaledContents(True)`でQtに任せており、縦横比は保たない。`QLabel`の最小サイズはpixmapのサイズになってしまうため、`setMinimumSize(1, 1)`でウィンドウをカメラ解像度より小さくできるようにしている）に実装済み。
- **サブスクライバコールバックのスレッドセーフティ:** `rospy.Subscriber`のコールバックは、`rospy.spin()`を呼んでいるかどうかに関わらず、Qtのメインスレッドではなくrospyが持つバックグラウンドスレッドで呼び出される。Qtウィジェットの読み書きはGUIスレッドからしか行えないため、コールバック内でウィジェットのメソッド（`setText`, `setPixmap`など）を直接呼んではならない。これを行うと、PySide6環境で実際に発生する断続的な`Segmentation fault`の原因になる。タイミング依存のクラッシュなので、毎回再現するとは限らない。代わりに、`MyWidget`のクラス属性として`QtCore.Signal`を定義し、サブスクライバコールバックからは受信したデータを添えて`.emit()`するだけにし、それを（`__init__`や`setup_*`メソッド内で）実際のウィジェット更新を行う通常のメソッドに接続すること。発信元スレッドと接続先スロットのスレッドが異なる場合、Qtが自動的にその接続をqueued connectionに昇格させるため、スロットの処理本体はGUIスレッド上で実行される。`main.py`内の`odom_updated`/`update_state_text`と`image_updated`/`update_image_pixmap`がこのパターンのリファレンス実装なので、GUIに新しいサブスクライバを追加する際はこれに従うこと。
