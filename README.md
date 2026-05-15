# Mesh Reconstruction from Point Cloud Frames

## Project Overview

This document summarizes the workflow for reconstructing a colored mesh from RoboSense AC point cloud frames.

The current data source is a RoboSense ROS1 `.bag` file containing synchronized sensor streams:

- `/rs_lidar/points`: LiDAR point cloud frames
- `/rs_camera/rgb`: RGB camera images
- `/rs_imu`: IMU measurements

The final goal is to generate a complete colored 3D mesh from frame-based LiDAR point clouds.

---

## 1. RoboSense Bag File Structure

A RoboSense `.bag` file stores frame-based sensor data. Each LiDAR frame is stored as a `sensor_msgs/PointCloud2` message.

A typical raw LiDAR point cloud frame contains:

- `x`, `y`, `z`: 3D point coordinates
- `intensity`: laser reflection intensity
- `ring`: LiDAR scan channel index
- optional timestamp or other point-level attributes

The raw bag does not directly store a complete mesh or a complete global point cloud map.

Instead, it stores independent frames:

```text
ROS bag
  ↓
/rs_lidar/points frame 0
/rs_lidar/points frame 1
/rs_lidar/points frame 2
...
```

### Key Insight

To reconstruct a complete environment, all point cloud frames must be transformed into the same coordinate system and merged into a global point cloud map.

```text
Point cloud frames
  ↓
Frame alignment
  ↓
Global point cloud
  ↓
Mesh reconstruction
```

### Main Issue

During data recording, the camera and LiDAR sensor move and rotate.

Therefore, points from different frames are not naturally aligned in one coordinate system.

If all frames are directly stacked together without pose correction, the merged point cloud may contain:

- duplicated surfaces
- blurred geometry
- bent walls
- distorted floors
- misaligned structures
- poor mesh reconstruction results

---

## 2. Frame Alignment and Point Cloud Fusion

Because the original bag does not contain `/tf`, `/odom`, `/pose`, or `/map`, the global pose of each frame is not directly available.

Therefore, frame alignment must be estimated from point cloud geometry.

The current approach is:

```text
Colored point cloud frames
  ↓
Per-frame cleanup
  ↓
ICP registration
  ↓
Frame transformation
  ↓
Global colored point cloud
```

### Recommended Fusion Method

Use ICP-based registration between each incoming frame and the current global map.

A two-stage ICP strategy is preferred:

```text
Coarse ICP
  ↓
Fine ICP
  ↓
Transform current frame
  ↓
Merge into global map
```

### Why Two-Stage ICP?

Coarse ICP uses a larger correspondence distance to roughly align the point cloud frame.

Fine ICP uses a smaller correspondence distance to refine the alignment.

This improves stability compared with a single ICP pass.

### Important Note

ICP-based fusion can still drift over time because it does not perform full SLAM optimization or loop closure.

However, it is useful for generating a first global map when only raw LiDAR, RGB, and IMU topics are available.

---

## 3. Color Information in RoboSense Point Clouds

The raw RoboSense LiDAR topic usually does not directly include RGB color fields.

The raw `/rs_lidar/points` topic may contain:

```text
x
y
z
intensity
ring
```

It may not contain:

```text
rgb
rgba
r
g
b
```

However, RoboSense AC Viewer can display colored point clouds because it uses the RGB camera stream and calibration parameters to project LiDAR points onto the camera image.

### Color Generation Principle

The color of each LiDAR point can be obtained using:

```text
LiDAR point cloud
+
RGB camera image
+
Camera intrinsics
+
Camera-LiDAR extrinsics
        ↓
Colored point cloud
```

For each LiDAR point:

```text
1. Transform LiDAR point into camera coordinate system
2. Project the 3D point onto the RGB image plane
3. Sample the RGB value from the image
4. Assign the RGB value to the LiDAR point
```

---

## 4. RoboSense Postprocess for Colored Point Clouds

The recommended way to recover the RGB-colored point cloud is to use RoboSense's official `robosense_ac_postprocess` pipeline.

The postprocess node takes:

- RGB image topic
- LiDAR point cloud topic
- IMU topic
- calibration configuration

and outputs colored point cloud topics such as:

```text
/rslidar_points_motion
/rslidar_points_motion_rgb
/rslidar_points_motion_stereo_rgb
```

The most important output topic for this workflow is:

```text
/rslidar_points_motion_rgb
```

This topic contains point cloud frames with fields such as:

```text
x
y
z
rgb
```

### Postprocess Pipeline

```text
Original RoboSense bag
  ↓
robosense_ac_postprocess
  ↓
/rslidar_points_motion_rgb
  ↓
colored_bag/ac_colored_points.bag
  ↓
colored_ply_frames/*.ply
```

### Why Record a Colored Bag?

The original bag does not contain `/rslidar_points_motion_rgb`.

This topic is generated at runtime by the postprocess node.

Therefore, the recommended workflow is:

```text
Original bag
  ↓
Run postprocess
  ↓
Record /rslidar_points_motion_rgb
  ↓
Generate colored bag
```

After the colored bag is generated, each colored point cloud frame can be extracted directly to `.ply` without running postprocess again.

---

## 5. Colored Point Cloud Frame Export

After recording the colored point cloud topic into a new bag, extract each frame as a `.ply` file.

Example output:

```text
colored_ply_frames/
├── frame_000000.ply
├── frame_000001.ply
├── frame_000002.ply
└── ...
```

Each `.ply` frame should contain:

- point coordinates
- RGB colors

A valid extracted frame should report:

```text
has_color=True
```

### Invalid Frames

Some frames may be invalid or empty. For example, a `PointCloud2` message may have missing `x`, `y`, or `z` fields.

These frames should be skipped.

Recommended handling:

```text
If frame is invalid:
  skip frame
else:
  save as PLY
```

This prevents one bad frame from interrupting the full export process.

---

## 6. Colored Point Cloud Fusion

After exporting all colored frames, use ICP to merge them into a global colored point cloud.

Input:

```text
colored_ply_frames/*.ply
```

Output:

```text
colored_icp_fusion_output2/global_colored_map.ply
```

### Fusion Steps

```text
Read colored PLY frame
  ↓
Light cleanup
  ↓
Estimate normals
  ↓
Coarse ICP
  ↓
Fine ICP
  ↓
Transform frame into global coordinates
  ↓
Merge into global map
  ↓
Voxel downsample global map
```

### Before-Merging Cleanup

Before merging each frame, apply only light cleanup:

- remove invalid points
- voxel downsampling
- light statistical outlier removal
- normal estimation

This improves ICP stability without destroying geometric details.

### After-Merging Cleanup

After creating the global point cloud, apply stronger cleanup and smoothing:

- statistical outlier removal
- radius outlier removal
- plane-based smoothing
- normal re-estimation

---

## 7. Global Colored Point Cloud Cleanup

The merged global point cloud may still contain noise or uneven surfaces.

Noise can affect:

- mesh quality
- surface smoothness
- vertex normal estimation
- color consistency
- final model appearance

Recommended cleanup workflow:

```text
Global colored point cloud
  ↓
Remove NaN / Inf points
  ↓
Voxel downsample
  ↓
Statistical outlier removal
  ↓
Radius outlier removal
  ↓
Plane smoothing
  ↓
Normal estimation
```

### Clean Output

```text
processed_cloud/global_colored_clean.ply
```

### Smooth Output

```text
processed_cloud/global_colored_smooth.ply
```

Use `global_colored_smooth.ply` for mesh reconstruction if the smoothing result looks correct.

Use `global_colored_clean.ply` if smoothing damages edges or fine details.

---

## 8. Plane-Based Smoothing

For structured environments such as rooms, hallways, floors, walls, and ceilings, many surfaces can be approximated as planes.

Plane smoothing uses RANSAC plane detection:

```text
Detect large plane
  ↓
Check if plane normal is close to X/Y/Z axis
  ↓
Find points close to the plane
  ↓
Project those points onto the plane
```

This helps reduce uneven surfaces.

### When to Use Plane Smoothing

Use it after merging the global point cloud.

Do not apply strong plane smoothing before ICP, because moving points before registration may damage alignment.

### Recommended Parameters

```text
PLANE_DISTANCE_THRESHOLD = 0.03 ~ 0.06
PLANE_AXIS_ANGLE_DEG    = 8 ~ 12 degrees
MIN_PLANE_POINTS        = 3000 or higher
```

### Effects

Good effects:

- smoother walls
- flatter floors
- cleaner surfaces

Possible problems:

- damaged edges
- over-flattened geometry
- loss of fine detail

---

## 9. Mesh Reconstruction

After the global colored point cloud is cleaned and smoothed, reconstruct a mesh.

Two common algorithms are:

- Ball Pivoting
- Poisson Surface Reconstruction

---

## 9.1 Ball Pivoting Algorithm

Ball Pivoting reconstructs a mesh by rolling a virtual ball over the point cloud surface.

If the ball touches three nearby points, a triangle face is created.

```text
Point cloud vertices
  ↓
Virtual ball rolls over surface
  ↓
Nearby points are connected
  ↓
Triangle mesh is generated
```

### Advantages

- good for surface-like point clouds
- preserves local structures
- less likely to create artificial closed surfaces
- suitable for LiDAR point clouds if the density is acceptable

### Issues

- sensitive to ball radius
- small radius creates holes
- large radius may create wrong faces
- can be slow on large point clouds

### Radius Selection

Ball radius should be related to point spacing.

If:

```text
VOXEL_SIZE = 0.04 m
```

Try:

```text
0.08 m
0.12 m
0.16 m
```

This corresponds approximately to:

```text
2 × voxel size
3 × voxel size
4 × voxel size
```

---

## 9.2 Poisson Surface Reconstruction

Poisson reconstruction uses point positions and normals to generate a continuous surface.

```text
Point cloud
+
Oriented normals
        ↓
Poisson reconstruction
        ↓
Closed mesh
```

### Advantages

- produces smooth surfaces
- fills small holes
- works well for dense point clouds
- useful for complete object-like reconstruction

### Issues

- strongly depends on normal quality
- may generate artificial surfaces
- may close open boundaries incorrectly
- edges and boundaries may become unclear
- may not be ideal for sparse LiDAR scans

### Post-Processing Needed

After Poisson reconstruction, the mesh often needs:

- density filtering
- bounding box cropping
- removal of floating surfaces
- mesh simplification

---

## 10. Mesh Color Preservation

Mesh reconstruction does not always preserve point cloud colors automatically.

If the reconstructed mesh has no color, transfer colors from the colored point cloud to the mesh vertices.

Recommended method:

```text
Colored point cloud
+
Generated mesh
        ↓
Nearest-neighbor vertex color transfer
        ↓
Colored mesh
```

### Vertex Color vs Texture

There are two ways to color a mesh:

| Method | Description | Recommended |
|---|---|---|
| Vertex Color | RGB stored directly on mesh vertices | Yes |
| Texture / UV | Image texture mapped to mesh surface | Not recommended for this workflow |

For this workflow, use vertex colors.

Do not use MeshLab texture transfer unless the mesh has valid UV coordinates.

### Common MeshLab Error

```text
target mesh does not have Per-Wedge texture coordinates
```

This means MeshLab is trying to use UV / texture mapping, but the mesh does not have texture coordinates.

The solution is:

```text
Do not use texture transfer
Use vertex color transfer instead
```

### Recommended Export Format

Use:

```text
colored_mesh.ply
```

Avoid:

```text
.stl
```

because STL does not store colors.

OBJ is also not recommended unless UV coordinates and texture images are properly generated.

---

## 11. Recommended Final Pipeline

The complete recommended pipeline is:

```text
Original RoboSense bag
  ↓
Run RoboSense AC postprocess
  ↓
Record /rslidar_points_motion_rgb into colored bag
  ↓
Extract colored bag into colored PLY frames
  ↓
Light cleanup on each frame
  ↓
ICP-based frame alignment
  ↓
Merge into global colored point cloud
  ↓
Clean global point cloud
  ↓
Smooth global point cloud
  ↓
Estimate normals
  ↓
Ball Pivoting or Poisson reconstruction
  ↓
Transfer point cloud colors to mesh vertices
  ↓
Export final colored mesh as PLY
```

---

## 12. Recommended Working Directory Structure

```text
mesh/
├── super_sensor_2026_05_11_14_46_41.bag
├── colored_bag/
│   └── ac_colored_points.bag
├── colored_ply_frames/
│   ├── frame_000000.ply
│   ├── frame_000001.ply
│   └── ...
├── colored_icp_fusion_output2/
│   └── global_colored_map.ply
├── processed_cloud/
│   ├── global_colored_clean.ply
│   └── global_colored_smooth.ply
├── mesh_output/
│   ├── mesh_ball_pivot.ply
│   └── colored_mesh.ply
└── scripts/
    ├── bag_to_ply.py
    ├── fuse_colored_ply_icp.py
    ├── clean_smooth_colored_cloud.py
    └── transfer_color_to_mesh.py
```

---

## 13. Practical Recommendations

### For Point Cloud Fusion

Use colored PLY frames as input:

```text
colored_ply_frames/*.ply
```

Then output:

```text
colored_icp_fusion_output2/global_colored_map.ply
```

### For Cleanup and Smoothing

Use:

```text
processed_cloud/global_colored_clean.ply
processed_cloud/global_colored_smooth.ply
```

Compare both in CloudCompare before mesh reconstruction.

### For Mesh Reconstruction

Try Ball Pivoting first.

Start with radii:

```text
0.08
0.12
0.16
```

Then test Poisson reconstruction if the global point cloud is dense and normals are reliable.

### For Colored Mesh Export

Use vertex color transfer and export as:

```text
colored_mesh.ply
```

---

## 14. Current Status

The current workflow has already achieved:

- original bag parsing
- RoboSense postprocess setup
- RGB-colored point cloud topic generation
- colored bag recording
- colored PLY frame extraction
- ICP-based colored point cloud fusion

The next key steps are:

```text
1. Clean and smooth global_colored_map.ply
2. Generate mesh from processed point cloud
3. Transfer vertex color from point cloud to mesh
4. Export final colored mesh as PLY
```

---

## 15. Key Notes

- Do not directly stack raw frames without pose alignment.
- Do not use texture transfer unless UV coordinates exist.
- Use PLY for colored point clouds and colored meshes.
- Clean lightly before ICP.
- Clean and smooth more strongly after global fusion.
- Use vertex color instead of UV texture for this workflow.
- Ball Pivoting is preferred first for LiDAR-based surface reconstruction.
- Poisson reconstruction can be tested later if the point cloud is dense enough.
