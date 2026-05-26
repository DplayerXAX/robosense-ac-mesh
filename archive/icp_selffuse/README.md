# Archived: early ICP self-fusion route

The project's earliest attempt: with no external pose, read clouds directly from
the bag and stitch frames with two-stage ICP (coarse + fine).

**Why archived**: accuracy was insufficient (ICP drifts over time, no loop
closure), and the project later switched to FAST-LIVO. Kept for accuracy
comparison and reference.

- `bag_icp_fusion.py`             — read from bag + intensity grayscale + two-stage ICP + plane smoothing
- `extract_colored_ply_frames.py` — extract per-frame PLY from a colored bag (postprocess route)

Known issue (recorded, not fixed): the correct, safe PointCloud2 reading
(field-by-field via `field.offset`) now lives in `src/pc_io.py`.
