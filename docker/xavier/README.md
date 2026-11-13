# Jetson AGX Xavier への PlaceNav 推論環境の導入

Xavier はインターネットに接続しない前提。必要なものはすべて parrot（インターネット接続あり）で用意し、
USB（書き込み）と parrot–Xavier 間の有線 LAN（`192.168.1.146` ↔ `192.168.1.246`）で渡す。

```
parrot (x86, Internet)                         Xavier (AGX, JetPack 5.1.2, offline)
 10 BSP/rootfs 取得 ─┐
 20 rootfs 作り込み ─┼─ USB-C ── 30 書き込み ──▶ L4T R35.4.1 + Docker + nvidia runtime
 40 arm64 イメージ ──┘                           + chrony(parrot) + 固定IP + ユーザ
 50 配布 ───────────── LAN ──────────────────▶ docker load / ~/PlaceNav (src, 重み, topomap)
                                                 run_xavier.sh → bench / navigate
```

| 項目 | 内容 |
|---|---|
| JetPack | 5.1.2 = Jetson Linux R35.4.1（Xavier 対応の最終系列）。sha256 は `common.sh` に固定 |
| 推論イメージ | `dustynv/l4t-pytorch:r35.4.1`（digest 固定）: Ubuntu 20.04, CUDA 11.4, **PyTorch 2.0.0**, torchvision 0.15.1（sm_72 ビルド済み）+ ROS Noetic（rospy/msgs/cv_bridge のみ） |
| ホスト側に入るもの | docker.io, nvidia-container（JetPack 5.1.2 版）, chrony。CUDA 等はイメージ内にあるのでホストには入れない |
| 設定値 | IP・ユーザ名・ホスト名は `common.sh`。環境変数で上書き可 |

## 前提

- parrot: sudo 可能、インターネット接続あり、空き容量 30 GB 程度（BSP+rootfs 展開 約 10 GB、イメージ tar 数 GB）。
  R35.4.1 の書き込みホストは公式には Ubuntu 18.04/20.04。22.04 でも通常動くが公式外。
- parrot–Xavier 間 LAN の parrot 側が `192.168.1.146/24` で設定済みであること。
- USB-C ケーブル（Xavier 前面の USB-C ↔ parrot）。
- **書き込みで Xavier の eMMC は全消去される。**

## 手順

すべて parrot 上、このディレクトリ（`PlaceNav/docker/xavier`）で実行する。作業ディレクトリは既定で `~/xavier_jp512`。

```bash
./10_download_bsp.sh          # BSP と sample rootfs（計 2.2 GB）を取得し sha256 を確認
sudo ./20_prepare_rootfs.sh   # 展開・apply_binaries・ユーザ作成・Docker 等を qemu chroot で導入（パスワードを聞かれる）
# Xavier をリカバリモードにする: 電源 OFF → 中央の Force Recovery ボタンを押したまま電源 ON → 2 秒後に離す
sudo ./30_flash.sh            # 'flash' と入力すると書き込み（十数分）
```

初回起動後（parrot から）:

```bash
ssh amsl@192.168.1.246 'sudo nvpmodel -m 0 && sudo jetson_clocks'   # MAXN（nvpmodel は再起動後も保持）
```

`jetson_clocks` は再起動で戻るので、計測や走行の前に毎回実行する。

### 時刻同期（parrot 側、初回のみ）

Xavier の chrony は parrot だけを参照する。parrot で chrony を時刻サーバにする:

```bash
sudo apt-get install -y chrony
printf 'allow 192.168.1.0/24\nlocal stratum 10\n' | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
# Xavier 側で確認: ssh amsl@192.168.1.246 chronyc tracking
```

### 推論イメージと PlaceNav の配布

```bash
./40_build_image.sh   # linux/arm64 を qemu でビルド（コンパイルなし、数十分）→ ~/xavier_jp512/placenav-xavier.tar.gz
./50_deploy.sh        # イメージ読込（未読込時のみ）と src/・docker/ の rsync（重み・topomap を含む）
```

`50_deploy.sh` は parrot 上の `src/model_weights` と `src/topomaps` をそのまま送る。
**先に parrot 側で使うモデルの準備（`prepare_waypoint.py`, `prepare_vpr.py`, 重みのダウンロード）を済ませておくこと。**
`global-feats-*.h5` は送らず、Xavier 上で初回起動時に作り直す（GPU/torch の差で特徴量が変わるため）。

### 推論時間の計測（Xavier 上）

```bash
ssh amsl@192.168.1.246
~/PlaceNav/docker/xavier/run_xavier.sh python3 bench_inference.py \
    --pr-models cosplace salad megaloc --wp-models omnivla_edge nomad gnm_large
```

5 Hz で回すには VPR + ウェイポイント推論の合計が 200 ms を十分下回る必要がある。
この結果を見て Xavier に載せるモデルを決める。

## 未検証・注意点

- 本ディレクトリのスクリプトは parrot と実機 Xavier ではまだ一度も実行していない（作成した PC では sudo も Xavier も使えなかった）。
  確認済みなのは、ダウンロード元の存在と sha256、BSP 内のスクリプト名・オプション、rootfs の内容、aarch64 wheel の有無、計測スクリプトの x86 での動作のみ。
- イメージの PyTorch は 2.0.0（x86 イメージは 2.1.2）。x86 で書き出した VPR の TorchScript（`model_weights/vpr/*.pt`）を
  2.0 で読めない場合は、Xavier 上で `prepare_vpr.py --model <名前>` を再実行して書き出し直す。
  その場合、上流ソース（`model_weights/vpr/sources`）と重みのキャッシュが Xavier 側にも必要（Xavier はダウンロードできない）。
- `navigate.py` を Xavier で動かしてロボットを走らせるには、別途 `navigation.launch` の分割（parrot 側で
  `navigate.py` を起動しない）、カメラ画像の縮小・間引き、`path_timeout:=1.0` の必須化が必要。未実装。
- eMMC は 32 GB。イメージ（数 GB）・重み・topomap を置くと余裕は大きくない。
