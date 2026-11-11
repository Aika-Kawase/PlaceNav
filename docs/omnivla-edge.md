# OmniVLA-edge（waypointバックエンド）

`--wp-model omnivla_edge`でNoMaDの代わりに選べます。VPR・ベイズフィルタ・サブゴール選択・TEB・STVLは変更なしです。
OmniVLA-edgeは**画像ゴールのみ**のモード（上流のmodality 6）で使い、GPS・言語・衛星地図の入力は使いません。

## 準備と実行

```bash
cd /home/user/ws/src/PlaceNav
python3 src/prepare_waypoint.py --models omnivla_edge   # 重み(HF, MIT)とソースをコミット固定で取得
python3 src/verify_waypoint.py --models omnivla_edge --device cuda
cd src && ./navigate_whill.sh --topomap-dir ROUTE --wp-model omnivla_edge --pr-model salad
```

`model_weights/waypoint/`（重み434 MB＋`sources/OmniVLA`）はGit管理外です。オフライン機へはディレクトリごとコピーします。
重みはSHA256を固定値と照合し、`weights_only=True`で読み込みます。

## 仕様

| 項目 | 値 |
| --- | --- |
| 入力 | 96×96、現在＋過去5フレームと、サブゴール画像（ViNT/NoMaDと同じPILリサイズ） |
| 出力 | 8点、各`[dx, dy, cosθ, sinθ]`。x前方・y左。単位は**0.1 m**（`metric_scale: 0.1`で固定） |
| 学習時の時間刻み | 3 Hz（8点で2.4 s） |
| 推論 | 約10 ms（RTX 3080）、約108Mパラメータ、決定的（`--nomad-seed`は無関係） |
| 制約 | **CUDA必須**。上流モデルが`Tensor.get_device()`を使うためCPUでは動かない。CLIP・GPSは不要 |

未使用の入力は上流モデル内でマスクされるため、ゼロで置き換えて同じ出力になります
（`tests/test_waypoint_backends.py`で、姿勢を使うモードとの対照付きで確認）。

## bag再生での検証（ROSなし・読み取り専用）

`rwrc26_ws/tools/waypoint_bag_replay.py`（PlaceNavリポジトリの外、`rosbags`が必要）：bagの時刻iの画像と、odom上でD m先のフレーム（ゴール）を入力し、予測軌道を実際に走った軌跡と比較します。
`dkan_1syu.bag`と`20261103_7.bag`（datasets内）の741サンプル。結果は`rwrc26_ws/results/omnivla_edge_replay/`。

| ゴール方位が15°以上ずれたサンプル（n=143） | OmniVLA-edge | NoMaD | 常に直進 |
| --- | --- | --- | --- |
| 最終点方向の角度誤差 中央値 / p90 | 12.5° / 58.8° | 10.4° / 83.0° | 27.5° / - |
| 旋回側（左右）の一致率 | 0.93 | 0.89 | - |

- 方向選択はNoMaDと同等で、悪化はない。優位とまでは言えない（サンプル数が少なく、同一走行内のゴールのため両者に楽観的）。
- 文脈フレームを毎フレームにしても3フレームおきにしても差はない（navigate.pyの現状のままでよい）。
- **予測経路が短い**：OmniVLA-edgeの8点の長さは中央値0.54 m、NoMaDは1.54 m（ロボットの実速度は0.36〜0.47 m/s、
  OmniVLAの予測は約0.2 m/s相当）。`/placenav/gnm_path`が短くなるため、TEBの終端到達判定・減速に影響しうる。

## 未検証・残課題

- **ROS上のnavigate.py全体（rosbag再生、経路アダプタ、TEB）は未実行**。`navigate.py`のモデル準備（スケール・文脈・8点出力）は
  ROSをスタブにして実コードで確認済み。実機コンテナでは`python3 src/verify_waypoint_ros.py --wp-model omnivla_edge --images DIR`を先に実行すること。
- 障害物回避・衝突は評価していない。TEB＋STVLは外さない。
- 短い参照経路がTEBに与える影響。必要なら`metric_scale`の調整（現在は上流の0.1に固定）や`--target-waypoint-idx`（上流は4）を試す。
- 学習データに車いす型の機体は含まれない（FrodoBots・車載映像など）。WHILLのカメラ高さ・画角との差は未評価。
