# 归档：早期 ICP 自融合路线

这是项目最早的尝试：不依赖任何外部 pose，直接从 bag 读点云、
用两段式 ICP（粗+细）把帧拼成全局图。

**为什么归档**：精度不够（ICP 会累积漂移、无回环），后来改用 FAST-LIVO。
保留作精度对比和参考。

- `bag_icp_fusion.py`     — 从 bag 读 + intensity 灰度上色 + 两段 ICP + 平面平滑
- `extract_colored_ply_frames.py` — 从彩色 bag 抽每帧 PLY（postprocess 路线用）
