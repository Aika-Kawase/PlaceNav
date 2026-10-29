# VPRモデル切替（WHILL）

`--pr-model cosplace|megaloc|salad|anyloc`で選択します。GNMは従来の85×64、
新VPRは元のRGB画像から322×322へ変換します。新VPRは正方形へのリサイズで、
左右の切り落としはありませんが縦横比は変わります。AnyLocも今回の固定解像度
構成であり、公式デモの可変解像度とは異なります。

## 初回準備

PyTorch 2.1.2 / torchvision 0.16.2 / CUDA 11.8を使用します。
変更を取得したホストのPlaceNavディレクトリでコンテナを再ビルドします。
既存コンテナへ`exec`するだけでは新しいイメージへ切り替わりません。

```bash
docker compose -f .devcontainer/docker-compose.yml build ros-noetic-docker-ws
docker compose -f .devcontainer/docker-compose.yml up -d --force-recreate ros-noetic-docker-ws
docker compose -f .devcontainer/docker-compose.yml exec ros-noetic-docker-ws bash
```

以降はコンテナ内です。インターネット接続中に使用するモデルを準備します。
この処理はROSの走行ノードを起動しません。コードはコミット固定で取得します。
公式モデルのライセンスはダウンロード先`model_weights/vpr/sources`に保持されます。

```bash
cd /home/user/ws/src/PlaceNav/src
python3 prepare_vpr.py --model megaloc
python3 prepare_vpr.py --model salad
```

AnyLocも次の1コマンドで、公式配布の都市用語彙とDINOv2重みを取得します。

```bash
python3 prepare_vpr.py --model anyloc
```

既存の公式語彙を使う場合は`--centers /path/to/c_centers.pt`で指定できます。
別語彙は別の実験条件になるため、出所を確認してください。
準備時に重み・語彙を組み込んだ`model_weights/vpr/{megaloc,salad,anyloc}.pt`を
生成します。公式推論と保存モデルの出力を2入力で照合してから保存します。
CPU準備は時間がかかります。`--device cuda`も指定できますが、AnyLoc ViT-gは
大きいためメモリに注意してください。使用するモデルだけ準備できます。
既存の保存モデルを更新する場合は`--force`を指定します。

別PCで準備した場合は、この`.pt`ファイルを実機の同じ相対パスにコピーします。
重みはGitに含めません。実行時には公式Pythonコードやネット接続は不要です。

## navigation

画像トピックとTFなどの既存起動手順はそのままです。

```bash
cd /home/user/ws/src/PlaceNav/src
./navigate_whill.sh \
  --topomap-dir whill_short_route_adaptive_c2_4 \
  --wp-model gnm_large \
  --pr-model megaloc \
  --subgoal-mode place_recognition \
  --filter-mode bayesian
```

モデル名を`salad`、`anyloc`、`cosplace`に変更するだけで切り替わります。
このコマンドはPD Controllerや車体ドライバを起動しません。
既存のTEB bridgeは別端末で起動します。

同じ事前マップ画像ディレクトリ内に`global-feats-<モデル名>.h5`を作ります。
モデル、重み、前処理、画像内容の署名が一致すれば再使用します。
従来の署名なしDBは初回に再生成されます。DB生成中はnavigation開始を待ちます。
強制再生成は`--recompute-place-recognition-db`です。生成は一時ファイルで行い、
正常終了時に置き換えます。同じモデル・マップの同時起動はしないでください。

| CLI名 | 特徴量次元 | 構成 |
|---|---:|---|
| cosplace | 512 | 既存EfficientNetチェックポイント |
| megaloc | 8448 | 公式MegaLoc |
| salad | 8448 | 公式DINOv2 ViT-B + SALAD |
| anyloc | 49152 | DINOv2 ViT-g layer31 value + 都市32クラスタVLAD |

既存bagとの互換性のため診断メッセージの`cosplace_*`フィールド名は維持しますが、
値は選択中VPRの検索結果です。GUIは選択モデル名を表示します。
Bayesian filterは共通です。モデル変更で類似度分布が変わるため、最初は同じ条件で
比較し、その後必要に応じてfilter deltaなどを調整します。

## 確認

```bash
cd /home/user/ws/src/PlaceNav
python3 -m unittest discover -s tests -p 'test_vpr_backends.py'
```

実モデルとGNMの推論確認（速度指令は配信しません）：

```bash
cd /home/user/ws/src/PlaceNav/src
python3 verify_vpr.py --models cosplace megaloc salad anyloc \
  --image topomaps/images/whill_short_route_adaptive_c2_4/0.jpg --device cuda --check-db
```

未準備モデルは`--models`から外します。
画像パスは存在する画像に置き換えてください。`--check-db`は一時ディレクトリで
DB生成・署名確認・Bayesian検索を確認し、既存マップのDBは変更しません。

最初のbag試験では画像と必要なTF/odomだけを再生し、記録済みのPlaceNav出力は
再生しません。選択ノード、GNM経路、処理時間を確認します。
AnyLocは大きいモデルなのでUniK3D同時実行時のVRAMと入力から経路出力までの遅延を
実測してください。モデル準備成功は実機走行性能の保証ではありません。

TEBの`bridge.yaml`は`1a25c02`と同一です。

## 実施済み検証（2026-10-07）

- 単体テスト7件、Python・シェル構文検証、`git diff --check`：成功。
- このPCのRTX 3080で4モデルの実画像GPU推論・有限な正規化特徴量を確認。
- 各モデルの一時DB生成・署名照合・Bayesian検索を確認。
- GNM入力85×64と5点の経路出力を確認。
- 新モデル3種の公式実装と保存モデルの出力一致を確認。

検証は既存コンテナ内の分離したPyTorch 2.1.2環境で実施しました。
更新Dockerfileのフルビルド、bag全区間の評価、実機走行は未実施です。
今回生成した3モデルの`.pt`はこのPCの`src/model_weights/vpr/`にあります。
Gitには含まれないため、実機へコピーするか実機で準備コマンドを実行してください。
