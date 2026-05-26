# Point Cloud → Colored Mesh

[English](README.md) | [中文](README_CN.md)

Turn the global colored point cloud produced by **FAST-LIVO** (RoboSense AC build)
into a clean, colored mesh.

> This is not a SLAM project. Mapping is done by
> [RS-FAST-LIVO](https://github.com/hku-mars/FAST-LIVO). This repo only handles
> **post-processing**: clean the cloud → reconstruct mesh → export colored mesh.

---

## What it does

After FAST-LIVO (tightly-coupled LiDAR-Inertial-Visual) finishes, it writes an
**already-fused, already-colored** point cloud to `PCD/`, e.g.
`rgb_map_voxel_0.030000.ply`.

This repo picks up from there in two steps:

```text
FAST-LIVO's rgb_map*.ply   (already fused, already colored)
        ↓  src/clean_pointcloud.py
clean + normals + optional plane smoothing
        ↓  src/make_colored_mesh.py
Poisson / Ball Pivoting reconstruction + keep vertex colors
        ↓
colored_mesh.ply
```

Because FAST-LIVO already handles fusion and coloring, **no** pose alignment,
frame fusion, or bag-reading/extraction is needed. Earlier and intermediate
attempts that did those things are kept under `archive/`.

---

## Directory layout

```text
.
├── src/                      Main pipeline (two steps)
│   ├── pc_io.py              Shared point cloud IO (handles FAST-LIVO packed RGB)
│   ├── clean_pointcloud.py   Step 1: clean + plane smoothing
│   └── make_colored_mesh.py  Step 2: reconstruct mesh + keep vertex colors
├── tools/                    Inspection / viewing utilities
│   ├── diagnose.py           Diagnose a point cloud (color / count / scale)
│   ├── inspect_bag.py        Inspect a ROS bag (topics / fields / IMU)
│   └── view.py               View a point cloud or mesh
├── archive/                  Archived: earlier/intermediate routes, no longer used
│   ├── icp_selffuse/         Early ICP self-fusion
│   └── slam_pose_fusion/     Intermediate KISS-ICP pose fusion
└── docs/
    └── REFACTOR_NOTES.md     Cleanup notes + lessons learned
```

---

## Quick start

```bash
pip install -r requirements.txt

# 0. (Optional) Diagnose the FAST-LIVO cloud first, confirm colors are readable
python tools/diagnose.py PCD/rgb_map_voxel_0.030000.ply

# 1. Clean the cloud (indoors, add --plane_smooth to flatten walls/floors)
python src/clean_pointcloud.py \
    --input PCD/rgb_map_voxel_0.030000.ply \
    --out_dir outputs/processed \
    --voxel 0.03 --plane_smooth

# 2. Reconstruct the colored mesh (indoors: Poisson depth 11)
python src/make_colored_mesh.py \
    --input outputs/processed/global_colored_clean.ply \
    --out outputs/mesh/colored_mesh.ply \
    --method poisson --depth 11

# View the result
python tools/view.py outputs/mesh/colored_mesh.ply
```

---

## Which input file to use

FAST-LIVO's `PCD/` usually contains these (the rgb group only exists when
`img_enable=1`):

| File | Description | Use for |
|---|---|---|
| `rgb_map.ply` | Full-resolution colored global map | Maximum detail |
| `rgb_map_voxel_0.030000.ply` | 3cm-downsampled colored map | **Recommended** — enough for meshing, not too heavy |
| `intensity_map*.ply` | Intensity grayscale map (`img_enable=0`) | When you don't need a colored mesh |

The PLY versions usually have more standard color fields and are read more
reliably by Open3D than PCD — prefer `.ply`.

---

## Choosing an algorithm

| Algorithm | Best for | Command |
|---|---|---|
| **Poisson** | Dense clouds, watertight surfaces, indoors | `--method poisson --depth 11` |
| **Ball Pivoting** | Surface-like clouds, avoids fake faces | `--method bpa --voxel 0.03` |

For indoor scenes, try Poisson depth 11 first. Too blurry → raise depth; too
many fake faces → raise `--density_quantile`.

---

## For Unity / Unreal

Add `--simplify` to also output a decimated version:

```bash
python src/make_colored_mesh.py --input ... --simplify 200000
# also produces colored_mesh_lite.ply
```

Export as `.ply` (carries vertex colors). **Do not use .stl** (no color), and
OBJ is not recommended unless UV + textures are set up — this workflow uses
vertex colors, not texture mapping.

---

## Acknowledgments

Mapping is based on [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO) /
[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) and the RoboSense AC build.
