# source_costmaps：入力別コストマップ

D435 / UniK3D の gate、補正 TF、二つの STVL と inflation による地図を管理する。
座標変換・点群生成・C2圧縮画像の復号も本パッケージに含む。旧点群結合機能と
`pointcloud_fusion` への依存は廃止した。通常は二レイヤで統合し、比較用の点群結合は
`fusion_mode:=points comparison_profile:=matched` で選択できる。

```bash
roslaunch source_costmaps costmaps.launch rviz:=true
```

この単独 launch は UniK3D 推論・ROS adapter・PlaceNav・TEB を起動しない。
詳細は [入力別コストマップの手順](../../../docs/SOURCE_COSTMAPS.md) を参照。
比較時は両方式とも `comparison_profile:=matched` を指定する。
[方式・条件・同じbagでの評価](../../../docs/FUSION_COMPARISON.md)を参照。

統合ナビゲーション起動、日時付き bag 記録、実機への転送手順は
[rwrc26_bringup](../rwrc26_bringup/README.md) に分離した。

```bash
roslaunch rwrc26_bringup navigation.launch
```

統合 launch と `costmaps.launch` を同時に起動しない。
