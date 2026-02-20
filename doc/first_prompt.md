# u-blox GNSS ROS2ドライバパッケージの作成

## 概要
既存のROS1スクリプト2つを参照し、同等の機能を持つROS2パッケージを作成してください。

## 参照スクリプト（ROS1）
以下の2つのスクリプトを参照してください。
- `ros2_ws/src/ublox_gnss_driver/reference/ntrip_pvthp.py`
- `ros2_ws/src/ublox_gnss_driver/reference/moving_base.py`

## カスタムメッセージ定義
以下のディレクトリにあるメッセージ定義を読み取り、フィールドに合わせてPublishしてください。
- `ros2_ws/src/bme_common_msgs/msg/GnssSolution.msg`
- `ros2_ws/src/bme_common_msgs/msg/HPPOSLLH.msg`
- `ros2_ws/src/bme_common_msgs/msg/PVT.msg`
- `ros2_ws/src/bme_common_msgs/msg/RELPOSNED.msg`

## 作成するノード

### 1. ntrip_pvthp_node
ntrip_pvthp.py の機能をROS2ノードとして再実装する。
- u-bloxレシーバからシリアル通信でUBXプロトコルデータを受信
- UBX-NAV-PVT を解析し、PVT.msg トピックとして配信
- UBX-NAV-HPPOSLLH を解析し、HPPOSLLH.msg トピックとして配信
- 解析結果を GnssSolution.msg として統合・配信
- sensor_msgs/NavSatFix、nav_msgs/Odometry（UTM座標）も配信
- RTCM補正データ（StringトピックまたはUInt8MultiArrayなど適切な型）をSubscribeし、シリアルポートに書き込む
- NMEA GNGGAセンテンスを検出しStringトピックとして配信

### 2. moving_base_node
moving_base.py の機能をROS2ノードとして再実装する。
- u-bloxレシーバ（Moving Base Rover）からUBX-NAV-RELPOSNEDを受信・解析
- RELPOSNED.msg トピックとして配信
- 相対位置ベクトルからヘディングを算出
- sensor_msgs/Imu（ヘディングをクォータニオンで）を配信
- ntrip_pvthp_node の HPPOSLLH トピックをSubscribeし、UTM座標＋ヘディングを統合した nav_msgs/Odometry を配信

## 技術要件

### ビルドシステム
- ament_python
- 依存パッケージ: rclpy, sensor_msgs, nav_msgs, std_msgs, bme_common_msgs, pyserial, numpy, pyproj

### パラメータ（ROS2 Parameter）
- `port`: シリアルポートパス（ノードごとにデフォルト値を設定）
- `baudrate`: ボーレート（ntrip_pvthp: 115200, moving_base: 38400）
- `frame_id`: 各メッセージのframe_id

### コード品質
- 参照スクリプトには改善の余地がある箇所が多いため、機能を維持しつつ以下の点を改善すること：
  - UBXパースの共通化・構造化（struct.unpackの繰り返しをヘルパー関数やdataclassで整理）
  - Python 3対応（binascii.b2a_hexの戻り値がbytesである点への対応など）
  - rclpy.Node を継承したクラス設計
  - シリアル読み取りをブロッキングloop内で行う場合、スレッドやタイマーによるROS2スピンとの共存を考慮
  - 型アノテーションの付与
  - 適切なロギング（self.get_logger()）
  - エラーハンドリング（シリアル切断時の再接続やtry-exceptなど）

### パッケージ構成（想定）
```
ublox_gnss_driver/
├── ublox_gnss_driver/
│   ├── __init__.py
│   ├── ntrip_pvthp_node.py
│   ├── moving_base_node.py
│   └── ubx_parser.py          # UBXパース共通ロジック
├── launch/
│   └── ublox_gnss.launch.py
├── reference/                  # 参照用の元スクリプト
│   ├── ntrip_pvthp.py
│   └── moving_base.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

### launchファイル
- 両ノードを起動するlaunchファイルを作成
- シリアルポートやボーレートをlaunch引数で指定可能にする

## 作業手順
1. まず bme_common_msgs/msg/ 内のメッセージ定義を読み取る
2. 参照スクリプトの処理内容とメッセージフィールドの対応を整理する
3. UBXパーサの共通モジュールを作成する
4. 各ノードを実装する
5. launchファイルを作成する
6. package.xml, setup.py を整備する
7. ビルドが通ることを確認する（colcon build）

## 注意点
pyprojはros2_ws/.venv/bin/activateを行ったあとでないと実行できないと思います。
