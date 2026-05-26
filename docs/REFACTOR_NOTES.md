# 重构笔记

记录这次整理做了什么、为什么，以及踩过的坑。给未来的自己看。

## 整理前的问题

仓库里堆了**三个时期、三条路线**的脚本，但只有最后一条是活的：

1. **早期 — ICP 自融合**：直接从 bag 读点云，两段式 ICP 拼帧。精度不够（漂移、无回环）。
2. **中期 — KISS-ICP pose 融合**：外部 SLAM 出 KITTI pose，手动按 pose 融合 / TSDF。
3. **现在 — FAST-LIVO**：紧耦合 LIVO，直接输出全局彩色点云。

三条路线的中间产物路径、坐标约定、pose 格式互不通用，全堆在一个 `scripts/`，所以乱。

## 关键认识：FAST-LIVO 让事情大幅简化

FAST-LIVO 的 `pcd_save_en=1` + `img_enable=1` 直接吐出**已融合、已上色、
在全局坐标系**的点云（`PCD/rgb_map*.ply`）。这意味着：

- 不需要 pose 对齐（FAST-LIVO 内部已做）
- 不需要手动融合彩色帧
- 不需要 README 里那套 `robosense_ac_postprocess`（那是另一条独立上色路径，重复了）

所以主线从「读 bag → 抽帧 → 融合 → 清理 → 建网格」缩短成「清理 → 建网格」。

## 做了什么

| 动作 | 文件 |
|---|---|
| 主线保留两步 | `src/clean_pointcloud.py`, `src/make_colored_mesh.py` |
| 抽公用 IO（含 packed RGB 回退） | `src/pc_io.py` |
| 合并 4 个 bag 检查脚本 | `tools/inspect_bag.py` |
| ICP 自融合归档 | `archive/icp_selffuse/` |
| KISS-ICP pose 融合归档 | `archive/slam_pose_fusion/` |
| 删除 | `convert_format.py`（危险读法 + 已无用） |

## 修掉的 bug（在归档脚本里，仅记录）

1. **PointCloud2 危险读法**：`convert_format.py` 和 `merge_with_pose.py` 用
   `point_step//4` / `data[:,0:4].view(float32)` 硬当 float32 读前几列。
   RoboSense 点带 intensity/ring/timestamp 时 offset 不连续，会读错位。
   正确做法（按 `field.offset` 逐字段读）已在 `src/pc_io.py` 和归档的
   `bag_icp_fusion.py` 里。

2. **merge_with_pose.py 丢颜色**：只读 xyz，跟「要彩色 mesh」矛盾。现在
   主线不用它（FAST-LIVO 直接给彩色）。

3. **make_mesh.py 参数不匹配**：旧版 `VOXEL=0.08` + Poisson `depth=10`，
   但 FAST-LIVO 数据已 3cm 下采样。新 `make_colored_mesh.py` 默认 depth=11，
   体素与数据匹配。

## 待验证（没跑，整理时未确认）

- Open3D 能否直接读 `rgb_map*.ply` 的颜色。`pc_io.py` 加了手动解 packed RGB
  的回退，但只覆盖 ASCII PCD；binary PCD 没颜色时建议改用 .ply。
  跑 `tools/diagnose.py` 可确认。
- Poisson depth / density_quantile 的具体值要看实际数据微调。

## README 与旧仓库对不上的历史遗留（已在新 README 修正）

- 旧 README 第 12 节目录结构用 `colored_bag/` `mesh_output/` 等，实际仓库用
  `data/` `outputs/`。新结构统一了。
- 旧 README 提到 `fuse_colored_ply_icp.py` `clean_smooth_colored_cloud.py`
  `transfer_color_to_mesh.py` 三个脚本，仓库里从来没有。新的
  `clean_pointcloud.py` + `make_colored_mesh.py` 覆盖了它们的功能。
