# 归档：KISS-ICP / 外部 pose 融合路线

中期尝试：用外部 SLAM（KISS-ICP）出 pose（KITTI txt），再手动按 pose
融合点云帧、或喂 TSDF。

**为什么归档**：后来改用 FAST-LIVO，它是 LiDAR-惯性-视觉紧耦合，直接输出
全局彩色点云，不再需要手动按 pose 融合。这套不再是主线。

已知问题（保留备查）：
- merge_with_pose.py 只读 xyz 丢颜色，且用危险的 point_step//4 读法
- 按 index 套 pose 假设帧数与 pose 数严格一致，未做时间戳对齐
- tsdf_fusion.py 的坐标系约定（LiDAR local）需输入是每帧局部 PLY，非全局图

- merge_with_pose.py — 按 KITTI pose 融合点云
- scan.py           — 读 pose 算法线方向 + Poisson
- tsdf_fusion.py    — 每帧 PLY + pose 投影成虚拟 RGBD 做 TSDF
