# RMUC 2027 provisional STL-derived map

Source: user-provided `/home/apocalypse_linux_00/Downloads/rmuc_2027.stl`.
STL SHA-256: `f002f5eef870caa721ee2d600eddebff8278487bc564b62d38388bba7331cb67`.
The simulator repository preserves all 378666 triangles, including 32 degenerate
faces; no 2026 decoration-removal rules or 0.1 scale are reused.

Generator in the paired simulator repository: `scripts/build_rmuc_2027_map.py`.
Exact inputs, plane fits, spawn rays, raster parameters and output hash are in
`rmuc_2027_projection.json`. The PGM is generated, NOT a blank placeholder.

## Coordinates

- Mesh scale: 1. World translation:
  `(-0.14910008013248444, -0.06378698348999023, -0.16228600648401698)`.
- World XY lower bound becomes (0,0). Z translation places the main floor TOP
  near world Z=0 at spawn (2,2); the STL's lowest face is an underside, not the road.
- The main floor slopes about 1.5 degrees toward the centre from each side.
  It is preserved, not flattened. Two fitted planes have max residual <0.4 mm.
- Robot spawn world XY=(2,2), Z=0.28, yaw=0. Centre and four corners 40 cm away
  have ground/no overhead obstruction in vertical ray checks. This is geometric
  screening, not a Gazebo settling or collision acceptance test.
- The existing first-planar-odometry origin convention maps this spawn to (0,0).
  Thus YAML origin=(-2,-2,0), where the third component is YAW, not height.
- PGM: 560x321 at 0.05 m/cell, covers 28x16.05 m. Image rows are flipped from
  bottom-up world Y. dynamic_only uses a 28x17 m fixed window and the existing
  35 cm inward rim, with any positive-Y padding occupied.

## Interpretation and limits

The generator clips steep surfaces to a 0.03..0.24 m band above the fitted LOCAL
main floor, excludes slopes below 20 degrees, and fills enclosed obstacle
contours. It does not simply project all roof points downward. It does not add
inflation; Nav2 still does that separately. Sampling can shift edges by a cell.

This is a provisional geometric map, NOT a certified traversability/semantic
map. Raised platforms, small steps, moving mechanisms, overhangs and narrow
openings require review/real driving. combined/static_only may be conservative
or miss details; dynamic_only is recommended for initial tests, where this PGM
is a visual reference and geometry source, not internal static-wall costs.

Point-LIO pose/cloud and DOG filters are unchanged. In particular,
high_column_min_z is still ABSOLUTE map Z, not height above this sloped floor.
The old 0.20 m setting needs scene-specific evaluation; do not interpret it as
20 cm above every part of the new floor. No rules, decision logic or relocation
are added. No success milestone is claimed before user acceptance.

Redistribution licence is unverified; do not publish these assets to a public
remote without confirming the source/licence.
