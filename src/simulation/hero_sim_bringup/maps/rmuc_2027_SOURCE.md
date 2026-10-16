# RMUC 2027 provisional STL-derived map

Source: user-provided `/home/apocalypse_linux_00/Downloads/rmuc_2027.stl`.
STL SHA-256: `f002f5eef870caa721ee2d600eddebff8278487bc564b62d38388bba7331cb67`.
The simulator repository preserves all 378666 triangles, including 32 degenerate
faces; no 2026 decoration-removal rules or 0.1 scale are reused.

Generator in the paired simulator repository: `scripts/build_rmuc_2027_map.py`.
Exact inputs, plane fits, spawn rays, raster parameters and output hash are in
`rmuc_2027_projection.json`. The PGM is generated, NOT a blank placeholder.

Known mapping error: this first PGM fills the central highland, which is actually
driveable. Do not treat it as an accepted static-navigation map. The paired
simulator repository has a corrected review candidate under
`maps/candidates/rmuc_2027/`; that candidate is NOT deployed here. The spawn update
below changes coordinates only, not either map's occupancy pixels.

## Coordinates

- Mesh scale: 1. World translation:
  `(-0.14910008013248444, -0.06378698348999023, -0.27757187763365515)`.
- World XY lower bound becomes (0,0). Z translation places the main floor TOP
  near world Z=0 at spawn (4.6,6.4); the STL's lowest face is an underside, not the road.
- The main floor slopes about 1.5 degrees toward the centre from each side.
  It is preserved, not flattened. Two fitted planes have max residual <0.4 mm.
- Robot spawn world XY=(4.6,6.4), Z=0.28, yaw=0. It is in the open left-side area,
  near the 2026 spawn's offset from its map lower bound (4.575,6.4006561), NOT
  a claim of an officially confirmed 2027 spawn. Centre and four corners 40 cm away
  have ground/no overhead obstruction in vertical ray checks. This is geometric
  screening, not a Gazebo settling or collision acceptance test.
- The existing first-planar-odometry origin convention maps this spawn to (0,0).
  Thus YAML origin=(-4.6,-6.4,0), where the third component is YAW, not height.
  Restart both simulation and algorithm after updating; the one-time Point-LIO
  alignment must not be reused from the old (2,2) spawn. No TF/Point-LIO code changes.
  The field is lowered by 0.1152858711 m to keep the new spawn floor near Z=0;
  floor grade and every source triangle are unchanged. Thin decorations elsewhere
  have NOT been removed by this coordinate update.
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
