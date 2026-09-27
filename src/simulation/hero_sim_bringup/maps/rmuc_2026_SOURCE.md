# RMUC 2026 static map provenance and calibration

- User-provided file: `/home/apocalypse_linux_00/Downloads/rmuc_2026_edit.pgm`
- Repository copy: `rmuc_2026.pgm`
- SHA-256: `fe6ef7857d4d1bfd448baff9023af34f2fa547c8671e0f8bc376bf175032d7f9`
- Format: binary PGM (`P5`), 583 × 321 pixels
- Source comment: `Created by GIMP version 2.10.30 PNM plug-in`

The Gazebo STL spans 29.15 m × 16.05 m after its 0.1 scale. The PGM dimensions
match those extents exactly at 0.05 m/pixel, so the map resolution is 0.05 m.
The scaled STL lower-left world bound is `(-14.575, -6.4006561)`.

`sim_ground_truth_bridge` currently captures the first robot pose as the
`map`/`odom` origin. The RMUC 2026 simulator spawns the robot at
`(-10.0, 0.0, yaw=0)`, so world coordinates are translated by `(10, 0)` in the
algorithm map frame. The YAML origin is therefore:

```text
(-14.575, -6.4006561) - (-10.0, 0.0) = (-4.575, -6.4006561)
```

The spawn maps to a free pixel in the supplied PGM. This establishes a
geometrically consistent initial calibration, but it remains provisional until
the RViz static-map edges are visually compared with several Gazebo landmarks.
Changing the simulator spawn or disabling first-odometry zeroing requires a
corresponding origin update.

The asset's redistribution licence has not been established. Do not push the
PGM to a public remote until its source and licence have been confirmed.
