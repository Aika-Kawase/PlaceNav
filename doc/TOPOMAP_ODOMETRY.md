# オドメトリ付きトポロジカルマップ

従来の画像マップと、新しいノード間相対姿勢付きマップを、同じ生成スクリプトで切り替える。
ナビゲーション側で保存オドメトリを利用する処理は、今回の実装には含めない。

## 生成モードと距離指定

コンテナ内の `/home/user/ws/src/PlaceNav/src` で実行する。

```bash
# 新モード。直線区間のノード間隔を2.0mにする。
./create_topomap_whill.sh tsukuba_odom_2.0 \
  --map-mode odometry --straight-distance 2.0 --odom-topic /odom
```

直線距離は、最後に保存した画像の位置と現在位置の
`sqrt((x2-x1)^2 + (y2-y1)^2)`。走行した道のりの累積長ではない。
画像の到着間隔などにより、距離がしきい値を超えた最初の処理タイミングで保存するため、
厳密に2.000m間隔になるわけではない。
旋回時には従来の適応的保存を維持するため、2mより短い間隔でも保存される。

```bash
# 従来の2引数も引き続き有効。第2引数は秒数であり、距離ではない。
./create_topomap_whill.sh old_route 2.0

# 従来形式の出力で、距離だけ2mに指定することもできる。
./create_topomap_whill.sh old_route_new_name --map-mode legacy --straight-distance 2.0
```

| オプション | 既定値・意味 |
|---|---|
| `--map-mode` | `legacy`。新しいJSONを保存するときは `odometry` |
| `--straight-distance` | 3.0m。直線区間の保存距離 |
| `--sampling-mode` | シェルでは `adaptive`。`time` も指定可能 |
| 第2位置引数／`--dt` | 3.0秒。`time` モードの時間間隔 |
| `--odom-topic` | ロボット設定の値。WHILLではホイール由来の `/odom` を使用する想定 |
| `--odom-max-gap` | 0.2秒。画像を挟む2つのオドメトリの最大時間間隔 |
| `--max-node-distance` | 新モードの `adaptive` では `2*straight-distance` m。これを超えるノード間接続は保存前にエラー停止。`time` では明示指定時のみ適用 |
| `--idle-timeout` | 無画像時の終了待ち秒数。新モードでは0＝Ctrl+Cまで待機。従来モードは既定で `2*dt` |
| `--topomap_directory` | 既定は `src/topomaps/images` |

新モードは既存の同名ディレクトリを上書きしない。別のルート名を指定する。
従来モードの同名マップ置き換え動作は維持している。

## 指定bagから作る例

以下はコンテナ内。先にROS masterを起動する。

```bash
roscore
```

別ターミナルで、生成ノードより前にsim timeを有効にする。

```bash
rosparam set /use_sim_time true
rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw
```

別ターミナルで上記の `create_topomap_whill.sh` を起動し、
最後にさらに別ターミナルで入力だけを再生する。

```bash
rosbag play --clock --delay=3 \
  /home/user/bagfiles/tsukuba/tsukuba.bag \
  --topics /odom /usb_cam/image_raw/compressed
```

bag再生終了後、生成ノードのターミナルでCtrl+Cを押す。
JSONは各ノード保存時に更新済みで、終了時に従来の `traj_data.pkl` も保存される。
深度推定・TEB・車体ドライバの起動は不要。
実機のライブ入力で生成するときは、生成ノードを起動する前に `rosparam set /use_sim_time false` に戻す。

## 保存内容と座標系

```text
tsukuba_odom_2.0/
  0.jpg
  1.jpg
  ...
  traj_data.pkl
  node_odometry.json
```

JSONは `schema_version: 1`。共通情報として入力トピック、オドメトリの
`frame_id` と `child_frame_id`、保存条件、単位（m、rad）を記録する。

- `nodes`: 連番ID、画像名、画像のROS時刻、画像時刻での `pose: {x, y, yaw}`、
  補間に使った前後のオドメトリ時刻。
- `edges`: 隣接する保存ノードの `from`、`to`、`dx`、`dy`、`dyaw`、直線距離 `distance`。
- 時刻は精度を落とさない `{secs, nsecs}` 形式。
- `pose` は入力オドメトリの親座標系での座標。先頭ノードをゼロには変換しない。
- `dx,dy` は出発ノードの車体座標系。前向きが+x、左向きが+y。
  `dyaw` は出発姿勢からの角度差を±πの範囲に正規化した値。

例えばid:1→id:2の相対位置は、`edges` の `from: 1, to: 2` に保存する。
`nodes[].pose` は各画像のodom座標系での姿勢であり、相対位置とは別の補助情報。
`odom_before_stamp` と `odom_after_stamp` は、その画像の `stamp` を挟む
2つのodomメッセージの時刻で、ノード間の始点・終点の時刻ではない。
ノード間の始点・終点の時刻は、`from` と `to` に対応する各ノードの `stamp` を参照する。

画像時刻を挟むオドメトリでx,yを線形補間し、yawは角度の折り返しを考慮して補間する。
同じ時刻のオドメトリがあれば、その値を使用する。外挿はしない。
前後データ不足・大きすぎる時間間隔ではノードを保存しない。
入力復帰時の画像が最後の保存ノードから `--max-node-distance` より遠ければ、
その画像・接続を追加する前に異常終了し、保存済みの区間だけを残す。
例えば直線間隔2mでは既定上限4m。これは離散的な画像取得による超過を許すための安全上限であり、
すべての区間が正確に2mになる保証ではない。より厳しい運用では `--max-node-distance 2.2` などを指定する。
異常終了した出力はルート途中までの地図であり、全ルート完成として扱わない。
欠けた区間の画像を複製したり、架空の画像ノードを生成したりはしない。
座標系の変更やオドメトリ時刻の巻き戻りがあれば、座標を混ぜず生成を停止する。
同じ座標系のまま位置だけをリセットする入力は、利用者側で避け、新しいマップとして再開する。

画像書き込みの成功後にJSONを更新する。一時ファイルへ書いてから置換するため、
更新途中のJSONを通常の読み手が読むことを防ぐ。
新モードの `traj_data.pkl` も保存ノード間の相対姿勢と整合させる。
従来モードは既存形式を維持し、JSONは追加しない。

## オドメトリ入力の変更

```bash
./create_topomap_whill.sh route_from_vo --map-mode odometry \
  --straight-distance 2.0 --odom-topic /vo/odom
```

入力は `nav_msgs/Odometry`。トピック名だけでホイール／VOを判別する処理はない。
VOを使用する場合も同じ車体基準の連続したオドメトリを用意する。
カメラ基準のVOから車体姿勢への変換やVO推定器自体の実装は対象外。
この記録はオドメトリに対する整合性を持つもので、実際の位置精度や累積誤差の改善を保証しない。

## 検証

ROSをsourceしたコンテナ内で実行する。

```bash
cd /home/user/ws/src/PlaceNav
python3 -B -m unittest discover -s tests -p test_topomap_odometry.py -v
python3 -B tests/check_topomap_odometry_ros.py \
  --output "$(mktemp -d /tmp/topomap-odometry.XXXXXX)"
```

後者は専用のROS master（既定11339）で、実際のシェルと生成ノードを使い、
画像時刻補間、旋回、保存を飛ばした場合の差分、逐次JSON保存、
既存マップ保護、従来CLI／形式、別トピック入力、欠損と座標系変更を確認する。
通常のROS masterや実機の制御トピックには接続しない。
