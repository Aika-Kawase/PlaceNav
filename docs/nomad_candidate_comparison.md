# NoMaD 1本生成／複数候補選択のbag比較

基準は `97d0bbe`（追跡ファイルは `aceb0c4` と同じ）。
新ブランチは `feature/nomad-candidate-selection`。
目標画像を使わない探索への切替はない。GNM/ViNTの推論は変更しない。
WHILLのXY換算は元の `0.4 / 5 = 0.08` のまま。パス延長は追加していない。

## AとCの違い

- A: `--nomad-selection single`（デフォルト）。従来の1本を無加工で使う。
- C: `--nomad-selection costmap --nomad-candidates 8`。
  同じ観測履歴と同じVPRサブゴールから8本生成し、コストマップで選択する。
- 観測画像のエンコードは共有する。拡散デコードは候補ごとに順次実行する。
  seedは `--nomad-seed + 候補番号`。候補0はAと同じ推論結果になる。
  GPUバッチ並列化ではないため、候補数に応じて処理時間が増える。
- 複数のパスを平均せず、必ず生成した1本を選ぶ。

## Cの判定

1. 最新のcostmap（部分更新も反映）と、画像時刻のbase_link→costmapフレームのTFを取得。
2. bridgeの`footprint`を包含する長方形に`footprint_padding`と
   `--nomad-clearance`（初期0.10 m）を加える。セルの半対角も判定余裕に加える。
   **多角形そのものより保守的**な判定。
3. 原点→1点目も含め、並進・旋回を補間し機体の掃引領域を検査。
   OccupancyGridで99以上、未知(-1)、地図外を禁止。
   地図に出ていない芝生・段差・物体を保証するものではない。
4. 有効候補のスコアが小さいものを選ぶ：

   `2 × 平均footprint内最大コスト/100`
   `+ continuity_weight × 前回経路への平均最近傍距離[m]`
   `+ 0.1 × 総旋回角[rad] - 0.2 × 末尾の機体前方成分[m]`

   同点なら小さい候補番号。連続性は同一costmap座標系で比較する。
   サブゴール変更、時計巻戻り、長い時間差では履歴を解除。
   前進量は**画像サブゴールへの実距離ではない**。曲がり角を正しく評価できるかは要検証。
5. 全候補不適切、TF欠落、古い画像/地図などでは選択しない。
   空のViz→空のPathでbridgeの通常・回避計画を破棄してゼロ速度にする。
   直前の良い経路へ自動フォールバックはしない。

最終の速度は従来通りTEBが計算する。TEB最適化後の軌道は選んだ候補と一致するとは限らない。
選択は幾何的な候補評価であり、動的障害物の未来位置予測や機体の制動保証ではない。

## ビルド（コンテナ内）

空経路での無効化と任意の入力期限チェックをbridgeに追加したのでC++再ビルドが必要。
既存navigation/bridgeを停止して行う。Docker再作成は不要。

```bash
cd /home/user/ws
source /opt/ros/noetic/setup.bash
source install/setup.bash
colcon build --base-paths src/PlaceNav/src/placenav_teb_bridge \
  --packages-select placenav_teb_bridge --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## PC上の比較

実機の速度購読系には接続しない。A/Cは**同時起動しない**。
同じbag区間・速度・マップ・VPR・フィルタ・seed・TEB設定で別々に実行する。
bagを巻き戻すたびにnavigationとbridgeを再起動し、VPR状態と地図をリセットする。

各ノード起動前：

```bash
rosparam set /use_sim_time true
```

入力だけを再生する（`BAG_FILE.bag`は実際のファイルへ変更）：

```bash
rosbag play BAG_FILE.bag --clock --pause --topics \
  /usb_cam/image_raw/compressed /usb_cam/camera_info \
  /c2/depth/points /odom /tf /tf_static
```

別ターミナル：

```bash
rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw
```

bridgeはA/Cで**同じコマンド**。既定の`path_timeout=0`は旧挙動維持用だが、
比較では1秒を指定し、推論停止・古いパスで走り続けないようにする。
これはパスの期限監視のみで、時計停止中の独立した制動監視ではない。

```bash
roslaunch placenav_teb_bridge bridge.launch \
  stvl_topic:=/c2/depth/points cmd_vel_topic:=/test_teb_path/cmd_vel \
  global_frame:=odom robot_base_frame:=base_link odom_topic:=/odom \
  path_timeout:=1.0
```

### A：現行の1本

```bash
cd /home/user/ws/src/PlaceNav/src
./navigate_whill.sh --topomap-dir ROUTE \
  --wp-model nomad --pr-model salad \
  --subgoal-mode place_recognition --filter-mode bayesian \
  --nomad-selection single --nomad-seed 0 --no-visualization
```

### C：8本から選択

```bash
cd /home/user/ws/src/PlaceNav/src
./navigate_whill.sh --topomap-dir ROUTE \
  --wp-model nomad --pr-model salad \
  --subgoal-mode place_recognition --filter-mode bayesian \
  --nomad-selection costmap --nomad-candidates 8 --nomad-seed 0 \
  --nomad-clearance 0.10 --nomad-continuity-weight 1.0 \
  --nomad-input-timeout 1.0 --no-visualization
```

`ROUTE`は実際の事前マップ名へ置換。画像が必要なので、起動後bagのpauseを解除する。
`rostopic info /toponav/viz_info` のPublisherがPlaceNavNodeだけであることを確認。
入力不足で起動直後に候補が拒否されることはある。継続する場合は診断理由を確認する。

## RVizと記録

- MarkerArray `/toponav/nomad_candidates`：緑＝選択、青＝有効だが非選択、
  赤＝除外。Aは緑1本だが**安全性検査済みという意味ではない**。
  表示は画像時刻のTFで配置。表示寿命1秒。
- `/toponav/nomad_selection`：std_msgs/StringのJSON。
  mode、query_timestamp、subgoal_idx、seed、候補数、選択番号、
  各候補のXY（base_link・メートル）、各スコア、除外理由、処理時間。
  選択なしは`selected_index=-1`。
- `/placenav/gnm_path`：選択後の参照経路。原点追加で通常9点。拒否時は空。
- `/placenav_teb_bridge/TebLocalPlannerROS/local_plan`：TEBの局所経路。
- `/test_teb_path/cmd_vel`：PC評価用出力。実機へ接続しない。

```bash
rosbag record --lz4 -O nomad_C_results.bag \
  /clock /toponav/nomad_candidates /toponav/nomad_selection \
  /toponav/viz_info /toponav/localization_diagnostics \
  /placenav/gnm_path /placenav_teb_bridge/active_global_path \
  /placenav_teb_bridge/TebLocalPlannerROS/local_plan \
  /placenav_teb_bridge/navigation_state \
  /placenav_teb_bridge/local_costmap/costmap \
  /placenav_teb_bridge/local_costmap/costmap_updates \
  /test_teb_path/cmd_vel
```

Aは保存名を変える。入力bagとコードcommit、実行コマンド、`rosparam dump`も保存する。
**記録bagの時計とヘッダ時刻を、保存後の壁時計と混同しないこと。**

比較指標：有効候補の存在率、選択の左右反転、衝突余裕、TEBの有効速度生成、
ゼロ速度の割合と理由、推論/選択時間、パスの遅延。
サブゴールとquery_timestampが一致する結果を優先比較する。
速度差で取りこぼす画像/VPR更新が変わるとサブゴールも変わり得るため、
同じbagを再生しただけで完全に同一入力の比較になるとは限らない。
まず全候補の中の候補0と選択候補を同一フレームで比較できる。

bagは開ループ。新しい経路へ走行した後の画像は生成されないので、
実際の回避成功やコース走破を証明するものではない。
短いパス問題・VPR誤認識・点群に現れない障害物は別問題。

## テスト

```bash
python3 -m unittest discover -s tests -p 'test_waypoint_backends.py' -v
python3 -m unittest discover -s tests -p 'test_nomad_selection*.py' -v
python3 -m unittest discover -s tests -p 'test_gnm_path_adapter.py' -v
python3 src/verify_waypoint.py --models nomad --device cuda --nomad-candidates 8
```

実モデル検証は既定では合成画像。shape、有限値、再現性、候補0一致を確認するもので、
実際の走行安全性の確認ではない。公式checkpointと既存requirements-waypoint.txtが必要。

ROS統合試験（独立localhost master、ロボットの制御系は起動しない）：

```bash
python3 src/verify_waypoint_ros.py --wp-model nomad --nomad-selection costmap \
  --images src/topomaps/images/whill_short_route_adaptive_c2_6 --port 11339
python3 src/verify_waypoint_ros.py --wp-model nomad --nomad-selection single \
  --images src/topomaps/images/whill_short_route_adaptive_c2_6 --port 11339
python3 tests/check_nomad_bridge_ros.py \
  --bridge /home/user/ws/install/placenav_teb_bridge/lib/placenav_teb_bridge/placenav_teb_bridge_node
```

実装時のPC検証結果：

- 単体テスト24件成功（生成4、選択/ROSメッセージ15、既存adapter5）。
- C++を一時ディレクトリでコンパイル成功。通常のinstallは上書きしていない。
- 実NoMaD重みの合成入力：1本約0.086秒、8本約0.541秒（当該PC・当該試行のみ）。
- CosPlace→NoMaD→選択→8点Viz→9点Pathを単一/複数の両モードで確認。
- 全面障害物による空Path、bridgeでのゼロ速度維持、新規有効パスでの再開、
  参照入力期限切れによる停止を独立ROS masterで確認。
- **つくばbagの比較、実機走行、安全性保証は未実施。**
