# Point Cloud → Colored Mesh

**中文** | [English](README.md)

把 **FAST-LIVO**（RoboSense AC 版）输出的全局彩色点云，转成干净的彩色 mesh。

> 这不是一个 SLAM 项目。SLAM/建图由 [RS-FAST-LIVO](https://github.com/hku-mars/FAST-LIVO) 完成，
> 本仓库只做**后处理**：清理点云 → 建网格 → 导出彩色 mesh。

---

## 它解决什么

FAST-LIVO（LiDAR-惯性-视觉紧耦合）跑完后，在 `PCD/` 目录直接输出
**已全局融合、已上色**的点云，例如 `rgb_map_voxel_0.030000.ply`。

这个仓库接手之后的两步：

```text
FAST-LIVO 的 rgb_map*.ply   （已融合已上色）
        ↓  src/clean_pointcloud.py
清理 + 法线 + 可选平面平滑
        ↓  src/make_colored_mesh.py
Poisson / Ball Pivoting 建网格 + 保顶点色
        ↓
colored_mesh.ply
```

因为 FAST-LIVO 已经做完融合和上色，**不再需要**任何 pose 对齐、
帧融合、读 bag 抽帧的步骤。那些早期/中期的尝试都归档在 `archive/`。

---

## 目录结构

```text
.
├── src/                      主线（两步）
│   ├── pc_io.py              公用点云读写（处理 FAST-LIVO packed RGB）
│   ├── clean_pointcloud.py   第1步：清理 + 平面平滑
│   └── make_colored_mesh.py  第2步：建网格 + 保顶点色
├── tools/                    检查/查看小工具
│   ├── diagnose.py           诊断点云（颜色/点数/尺度）
│   ├── inspect_bag.py        检查 ROS bag（topic/字段/IMU）
│   └── view.py               查看点云或 mesh
├── archive/                  归档：不再用的早期/中期路线
│   ├── icp_selffuse/         早期 ICP 自融合
│   └── slam_pose_fusion/     中期 KISS-ICP pose 融合
└── docs/
    └── REFACTOR_NOTES.md     整理记录 + 踩过的坑
```

---

## 快速开始

```bash
pip install -r requirements.txt

# 0. （可选）先诊断 FAST-LIVO 的点云，确认颜色读得到
python tools/diagnose.py PCD/rgb_map_voxel_0.030000.ply

# 1. 清理点云（室内加 --plane_smooth 让墙/地更平）
python src/clean_pointcloud.py \
    --input PCD/rgb_map_voxel_0.030000.ply \
    --out_dir outputs/processed \
    --voxel 0.03 --plane_smooth

# 2. 建彩色 mesh（室内 Poisson depth 11）
python src/make_colored_mesh.py \
    --input outputs/processed/global_colored_clean.ply \
    --out outputs/mesh/colored_mesh.ply \
    --method poisson --depth 11

# 查看结果
python tools/view.py outputs/mesh/colored_mesh.ply
```

---

## 选哪个输入文件

FAST-LIVO 的 `PCD/` 里通常有这些（`img_enable=1` 时才有 rgb 组）：

| 文件 | 说明 | 用途 |
|---|---|---|
| `rgb_map.ply` | 全分辨率彩色全局图 | 想要最高细节时用 |
| `rgb_map_voxel_0.030000.ply` | 3cm 下采样彩色图 | **推荐**，建网格够用又不卡 |
| `intensity_map*.ply` | 强度灰度图（`img_enable=0`） | 不要彩色 mesh 时用 |

PLY 版通常比 PCD 版的颜色字段更标准、Open3D 更好读，优先用 `.ply`。

---

## 算法选择

| 算法 | 适合 | 命令 |
|---|---|---|
| **Poisson** | 密集点云、要封闭表面、室内 | `--method poisson --depth 11` |
| **Ball Pivoting** | 表面型点云、不想造假面 | `--method bpa --voxel 0.03` |

室内场景先试 Poisson depth 11。太糊就调高 depth，假面多就调高 `--density_quantile`。

---

## 给 Unity / Unreal 用

加 `--simplify` 输出简化版（减面）：

```bash
python src/make_colored_mesh.py --input ... --simplify 200000
# 额外得到 colored_mesh_lite.ply
```

导出用 `.ply`（带顶点色）。**不要用 .stl**（不存颜色），
OBJ 除非配好 UV+贴图否则也不推荐——本工作流用顶点色，不用纹理贴图。

---

## 致谢

建图基于 [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO) /
[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) 及 RoboSense AC 适配版。
