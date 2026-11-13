# Jetson AGX Xavier への PlaceNav 推論環境の導入

Xavier はインターネットに接続しない前提。必要なものはすべて parrot（インターネット接続あり）で用意し、
parrot–Xavier 間の有線 LAN（`192.168.1.146` ↔ `192.168.1.246`）で渡す。

**現在の方針: Xavier の OS（JetPack 5.0.2 / L4T R35.1）と既存の Docker には手を付けない。**
eMMC には既存の環境があり空きも 3 GB しかないため、PlaceNav 用の Docker デーモンを別に立て、
イメージとデータは追加した NVMe SSD（`/mnt/ssd`）に置く。

```
parrot (x86, Internet)                          Xavier (JetPack 5.0.2 のまま, offline)
 40 arm64 イメージ作成 ─┐                        docker.service（既存, eMMC）… 変更しない
 45 runtime deb 取得 ───┼── LAN ── 45 で導入 ──▶ fstab に SSD / nvidia-container-toolkit
                        │                         docker-placenav.service（SSD, 別ソケット）
 50 配布 ───────────────┘── LAN ──────────────▶ /mnt/ssd/PlaceNav (src, 重み, topomap)
                                                  run_xavier.sh → bench / navigate
```

| 項目 | 内容 |
|---|---|
| 推論イメージ | `dustynv/l4t-pytorch:r35.4.1`（digest 固定）: Ubuntu 20.04, CUDA 11.4, **PyTorch 2.0.0**, torchvision 0.15.1（sm_72）+ ROS Noetic（rospy/msgs/cv_bridge のみ） |
| Xavier に追加するもの | `/etc/fstab` に SSD 1 行、NVIDIA コンテナ runtime の deb 4 つ（R35.1 用、`nvidia-docker2` は入れない）、`/etc/docker/placenav-daemon.json`、`docker-placenav.service` |
| 変更しないもの | 既存の `docker.service`、`/etc/docker/daemon.json`（存在しない）、eMMC 上のイメージ・コンテナ |
| PlaceNav 用 Docker | ソケット `unix:///run/docker-placenav.sock`、データ `/mnt/ssd/docker-placenav`、containerd 名前空間 `placenav`、既定 runtime nvidia |
| 設定値 | IP・ユーザ名・パスは `common.sh`。環境変数で上書き可 |

## 手順（parrot 上、このディレクトリで実行）

```bash
./40_build_image.sh    # linux/arm64 を qemu でビルド（コンパイルなし、数十分）→ ~/xavier_jp512/placenav-xavier.tar.gz
./45_setup_xavier.sh   # deb 取得（sha256 確認）→ Xavier へ転送 → Xavier で sudo 実行（パスワードを聞かれる）
./50_deploy.sh         # PlaceNav 用 Docker へイメージ読込（未読込時のみ）と src/・docker/ を /mnt/ssd/PlaceNav へ rsync
```

`45_setup_xavier.sh` の最後に、PlaceNav 用 Docker の情報（`root=/mnt/ssd/docker-placenav`, `default=nvidia`）と、
既存 Docker が変わっていないこと（`active`, root `/var/lib/docker`）が表示される。

`50_deploy.sh` は parrot 上の `src/model_weights` と `src/topomaps` をそのまま送る。
**先に parrot 側で使うモデルの準備（`prepare_waypoint.py`, `prepare_vpr.py`, 重みのダウンロード）を済ませておくこと。**
`global-feats-*.h5` は送らず、Xavier 上で初回起動時に作り直す。

### 動作確認と推論時間の計測（Xavier 上）

```bash
ssh amsl@192.168.1.246
sudo jetson_clocks   # 計測・走行前に毎回（再起動で戻る）。電力モードは sudo nvpmodel -q で確認
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 -c \
    "import torch; print(torch.__version__, torch.cuda.is_available()); print(torch.randn(2, device='cuda').sum())"
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 bench_inference.py \
    --pr-models cosplace salad megaloc --wp-models omnivla_edge nomad gnm_large
```

5 Hz で回すには VPR + ウェイポイント推論の合計が 200 ms を十分下回る必要がある。
この結果を見て Xavier に載せるモデルを決める。PlaceNav 用 Docker を手で使うときは
`docker -H unix:///run/docker-placenav.sock ...`（既定の `docker` は既存のデーモンを指す）。

### 元に戻す

```bash
sudo systemctl disable --now docker-placenav
sudo rm /etc/systemd/system/docker-placenav.service /etc/docker/placenav-daemon.json
sudo rm -rf /mnt/ssd/docker-placenav /mnt/ssd/PlaceNav
# 必要なら: /etc/fstab の SSD の行を削除（バックアップ /etc/fstab.before-placenav.* あり）
# 必要なら: sudo dpkg -r nvidia-container-toolkit libnvidia-container-tools libnvidia-container1 libnvidia-container0
```

## 未検証・注意点

- **R35.4.1 用のイメージを R35.1 のホストで動かす**（CUDA 11.4 は共通、ドライバはホストのものを使用）。
  互換性の見込みはあるが未確認。上の `torch.cuda.is_available()` が `False` やエラーになる場合はベースイメージの変更が必要。
- スクリプトは parrot と実機 Xavier ではまだ実行していない。確認済みは、ダウンロード元と sha256、
  `nvidia-container-toolkit` の deb の中身（インストール時に Docker 設定を触らない）、`placenav-daemon.json` の
  `dockerd --validate`（Docker 29.8 で）、aarch64 wheel の有無、計測スクリプトの x86 での動作のみ。
- イメージの PyTorch は 2.0.0（x86 イメージは 2.1.2）。x86 で書き出した VPR の TorchScript（`model_weights/vpr/*.pt`）を
  2.0 で読めない場合は、Xavier 上で `prepare_vpr.py --model <名前>` を再実行して書き出し直す
  （上流ソースと重みのキャッシュも Xavier に必要）。
- `navigate.py` を Xavier で動かしてロボットを走らせるには、別途 `navigation.launch` の分割（parrot 側で
  `navigate.py` を起動しない）、カメラ画像の縮小・間引き、`path_timeout:=1.0` の必須化が必要。未実装。

## 参考: Xavier を消去して JetPack 5.1.2 を書き込む場合（現在は使わない）

既存環境を消してよくなった場合のみ。`10_download_bsp.sh` → `sudo 20_prepare_rootfs.sh` →
（USB-C で接続しリカバリモードにして）`sudo 30_flash.sh`。**eMMC は全消去される。**
書き込み後は既定の Docker が nvidia runtime を持つため、`45_setup_xavier.sh` は使わず、
`PLACENAV_DOCKER_HOST=unix:///var/run/docker.sock XAVIER_PLACENAV_DIR=/home/amsl/PlaceNav ./50_deploy.sh` のように既定の Docker を指定する。
