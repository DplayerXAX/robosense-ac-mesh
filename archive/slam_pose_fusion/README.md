# Archived: KISS-ICP / external pose fusion route

An intermediate attempt: use an external SLAM (KISS-ICP) to produce poses
(KITTI txt), then manually fuse cloud frames by pose, or feed a TSDF.

**Why archived**: the project later switched to FAST-LIVO, a tightly-coupled
LiDAR-Inertial-Visual system that directly outputs a global colored cloud, so
manual pose-based fusion is no longer needed. This route is no longer the main line.

Known issues (kept for reference):
- merge_with_pose.py reads only xyz and drops color, and uses an unsafe
  point_step//4 reading; correct reading is in src/pc_io.py
- it assumes frame index == pose index, with no timestamp alignment
- tsdf_fusion.py's coordinate convention (LiDAR local) expects per-frame local
  PLY input, not a fused global map

- merge_with_pose.py — fuse clouds by KITTI pose
- scan.py           — derive normal orientation from poses + Poisson
- tsdf_fusion.py    — project per-frame PLY + pose into virtual RGBD for TSDF
