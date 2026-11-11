# rwrc26_bringup：PlaceNav・深度・TEBの統合起動

PlaceNavの`src/rwrc26_bringup`と`src/source_costmaps`をビルドし、
`navigation.launch`から深度ROS adapter、点群処理、TEB、PlaceNav、bag記録、RVizを起動する。
TEBが参照する内部コストマップに、選択した統合方式を直接読み込む。

## 配置とビルド

```text
PlaceNav/src/
├── placenav_teb_bridge/
├── source_costmaps/
└── rwrc26_bringup/
```

実機の既存ROSコンテナ内で実行する。STVL・TEB・`unidepth_ros`の依存環境は既存のものを使う。
PlaceNav全体を古いチェックアウトで上書きしない。

```bash
cd /home/user/ws
bash src/PlaceNav/scripts/build_obstacle_stack_ros1.sh
source install/setup.bash
rospack find source_costmaps
rospack find rwrc26_bringup
```

ビルドスクリプトには両パッケージを登録済み。`--base-paths`で探索範囲を限定する。
旧配置の同名パッケージと旧`pointcloud_fusion`は残さない。
配置を移した場合は、そのパッケージの旧build/install生成物を整理してから再ビルドする。

## 起動

事前に実機ドライバ、ROS master、odom、取り付けTF、UniK3D GPUサーバーを起動する。
NoMaD/SALADなどのモデルとtopomapも準備しておく。
既存のPlaceNav、深度adapter、TEB、単独コストマップ、bag recorderは終了してから統合launchを起動する。

```bash
source /home/user/ws/install/setup.bash
roslaunch rwrc26_bringup navigation.launch
```

速度出力は既定で`/fusion_validation/cmd_vel`。
実機への速度出力は`drive_robot:=true`を付けると`/local_path/cmd_vel`へ切り替わる。
この引数はTEBの出力先を選択する。車体ドライバ・速度mux・非常停止系は別途必要。

通常の既定は`layers + current`。方式比較は両方に`matched`を明示して、一方を終了してから次を起動する。

```bash
# 各点群→個別STVL→コストマップをmax統合
roslaunch rwrc26_bringup navigation.launch \
  fusion_mode:=layers comparison_profile:=matched \
  reference_mode:=latest path_timeout:=1.0 shrink_horizon_backup:=false

# 同じ観測区間の点群を結合→単一STVL
roslaunch rwrc26_bringup navigation.launch \
  fusion_mode:=points comparison_profile:=matched \
  reference_mode:=latest path_timeout:=1.0 shrink_horizon_backup:=false
```

## 開発PCでbagを再生する

深度推定は既存の別コンテナを使用する。ROSコンテナにUniK3Dや別のPython環境を追加しない。
初回はホスト側で `UniDepth` と同じ階層に `UniK3D` を配置し、
[UniK3Dの起動手順](../../../UniDepth/docs/UNIK3D_ROS1_RUNTIME.md)に従ってビルド・起動する。

```bash
cd ~/rwrc26_ws/src
# UniK3D がまだない場合のみ実行する。
git clone https://github.com/lpiccinelli-eth/UniK3D.git

cd UniDepth
docker compose -f docker/unidepth-v2/compose.yaml build
DEPTH_SERVER=/workspace/UniDepth/runtime/unik3d_server.py \
DEPTH_MODEL=lpiccinelli/unik3d-vits \
docker compose -f docker/unidepth-v2/compose.yaml up -d
docker compose -f docker/unidepth-v2/compose.yaml logs --tail=30
```

`model ready` と `listening on 127.0.0.1:5557` を確認してから、
ROSコンテナ内で実行する。深度コンテナは `restart: unless-stopped` で継続稼働する。
debug launch 自体はこのコンテナを起動しない。

```bash
roslaunch rwrc26_bringup navigation_debug.launch \
  topomap_dir:=whill_short_route_adaptive_c2_tsukuba_3.5 \
  bag:=/home/user/bagfiles/tsukuba/tsukuba_lidar.bag \
  bag_pause:=false
```

bagからC2画像、D435点群、odom、TFを再生し、C2深度・点群、PlaceNav経路、
コストマップ、TEB経路を再計算する。既定の画像入力は
`/usb_cam/image_raw/compressed`。非圧縮画像のbagは `bag_camera_transport:=raw` を指定する。
`use_sim_time` とbagの `/clock` はlaunchが設定する。
実機向け速度出力とbag記録は無効。

## 元のbridge.launchとの対応

| 引数 | 既定値 | 反映先・意味 |
|---|---|---|
| `reference_mode` | `state_machine` | TEBブリッジの参照方式。`latest`も選択可能 |
| `path_timeout` | `0.0` | 有効なPathの受信間隔上限、秒。0は無効 |
| `shrink_horizon_backup` | `true` | `TebLocalPlannerROS/shrink_horizon_backup` |
| `global_frame` | `odom` | 点群gate・観測区間・コストマップ・TEB・経路adapter・RVizの共通フレーム |
| `robot_base_frame` | `base_link` | gateの高さ基準・コストマップ・経路adapterの基準 |
| `odom_topic` | `/odom` | TEBのodometry入力 |
| `bridge_config` | ブリッジの`config/bridge.yaml` | 元launchの`config`に相当 |
| `robot_cmd_vel_topic` | `/local_path/cmd_vel` | `drive_robot:=true`時の速度出力 |

元launchと同じく、明示的なlaunch引数は読み込んだYAMLより優先する。
引数にしていないTEBの速度・回避設定などは`bridge_config`で設定する。
`stvl_topic`は単一入力用なので使わず、入力点群は`gates_config`内の各`input_topic`で設定する。
元の入力トピック自体は変更しない。

`path_timeout`は有効なPathを受信してからの壁時計時間で判定する。
画像取得時刻の鮮度検査ではなく、制御タイマーが停止した場合の独立した監視機構でもない。
`reference_mode`の詳細と診断項目は`placenav_teb_bridge/README.md`を参照。

フレーム引数を変更する場合は、対応するTFとodomを用意する。
C2には追加のTF補正を行わず、既存の `c2_camera_optical_frame` を使用する。カメラから車体へのTFは外部で配信する。
bag記録のトピック一覧は自動改名しないため、トピックを変更した場合は`record_topics`も合わせる。

## PlaceNavの引数

`navigate_whill.sh`から`placenav/parser.py`へ渡す。

| launch引数 | 既定値 |
|---|---|
| `topomap_dir` | `whill_short_route_adaptive_c2_tsukuba_2.0` |
| `wp_model` / `pr_model` | `nomad` / `salad` |
| `subgoal_mode` / `filter_mode` | `place_recognition` / `bayesian` |
| `window_radius` | `3` |
| `start_node_idx` / `goal_node_idx` | `0` / `-1`（末尾） |
| `lookahead` / `close_threshold` | `1` / `3` |
| `target_waypoint_idx` / `nomad_seed` | `2` / `0` |
| `device` | `cuda` |
| `transition_model_window_lower` / `transition_model_window_upper` | `-1` / `2` |
| `filter_delta` | `10` |
| `recompute_place_recognition_db` | `false` |
| `img_size` | `[]`（モデルの既定サイズ）。例：`'img_size:=[96,96]'` |
| `navigation_visualization` | `true` |
| `placenav_src` | `/home/user/ws/src/PlaceNav/src` |

モデル・ロボット設定ファイルとモデル重み・topomapの基底ディレクトリは、既存の
`navigate_whill.sh`と`src/config`の設定を使用する。
`image_topic`は深度adapterの入力を指定する。PlaceNavのカメラ入力は`config/robots.yaml`で設定する。
別の配置では`placenav_src`を実際の`navigate_whill.sh`があるディレクトリに変更する。
`navigate_whill.sh`には経路adapterのフレーム指定だけを追加し、推論・目標選択ロジックは変更していない。

## コストマップ・記録・表示

| 引数 | 既定値 |
|---|---|
| `fusion_mode` / `comparison_profile` | `layers` / `current`。`points`時のprofile既定は`matched` |
| `batch_interval` / `common_voxel_size` / `common_decay` | `0.1 s` / `0.05 m` / `0.8 s`（matched時） |
| `gates_config` / `costmap_config` | `source_costmaps/config/{gates,costmap}.yaml` |
| `image_topic` / `server_host` / `server_port` | `/usb_cam/image_raw` / `127.0.0.1` / `5557` |
| `max_input_width` / `cloud_stride` | `640` / `4` |
| `record` / `diagnostics` | `true` / `false` |
| `bag_dir` | `/home/user/ws/src/cache/noetic/bags` |
| `bag_prefix` / `bag_buffer_mb` | 空文字 / `512` |
| `record_topics` | `config/record_topics.yaml` |
| `rviz` / `debug_costmaps` | `true` / `false` |

bagと`.session.json`の名前は日本時間の`YYYYMMDD_HHMMSS_ffffff`。
JSONには有効な起動設定、TEB設定、統合パッケージとブリッジのソース・実行ファイルのSHA256を保存する。
既存ファイルを上書きせず、`tsukuba`内とそこへのシンボリックリンクは保存先として拒否する。
`matched`時は比較用点群・メタデータも記録する。Ctrl+C後はbagの確定を待つ。

`diagnostics:=true`ではTEBの`/placenav_teb_bridge/diagnostics`、状態、gate診断などを追加記録する。
現在のPlaceNavは`--trace-navigation`に未対応なので、その引数は渡さず警告を出す。
対応する別版を`placenav_src`で指定した場合だけ、trace引数と`/rwrc26/*`の記録を追加する。
診断の有無はモデル・参照方式を切り替えない。最新版TEBの診断配信自体は常時有効。

`debug_costmaps:=true`は表示用の別コストマップを追加する。
通常のRViz表示は実際のTEB内部の`/placenav_teb_bridge/local_costmap/costmap`。

## 検証

`test/test_session_runner.py`で実引数parser・シェル・経路adapterへの受け渡しと記録処理を検証する。
`test/test_navigation_launch.py`はROS環境でlaunchを展開し、両統合方式・両参照方式・フレーム・既定値を検証する。
点群・STVLの実ノード検証は`source_costmaps/test/comparison{,_frames}.test`を使用する。
実機への転送と実走行は未実施。
