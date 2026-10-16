# RMUC 2027 multilevel support-surface map (applied 2026-10-16)

The active `rmuc_2027.pgm` is now the redrawn map from the paired simulator:
`maps/candidates/rmuc_2027/rmuc_2027_surface_candidate.pgm`.
It is byte-identical, not a resampled or manually whitened version.

- PGM SHA-256: `43d2a27e52b85fe8e5e0b19794fc022f65b5f9eff84fff6a7bb8c677ae8802d3`.
- Size: 1120x642; resolution: 0.025 m/cell; extent: 28x16.05 m.
- YAML origin: (-4.6,-6.4,0), where the third component is yaw, not height.
- Generator: `scripts/build_rmuc_2027_surface_map.py` in the simulator repository.
- Source STL: user-provided `/home/apocalypse_linux_00/Downloads/rmuc_2027.stl`.
- STL SHA-256: `f002f5eef870caa721ee2d600eddebff8278487bc564b62d38388bba7331cb67`.

The old 560x321, 5 cm map incorrectly filled the central highland and is no
longer active. Its source and old bytes remain recoverable from Git at
algorithm commit `ac6aae2`; restoring it requires a new deliberate change,
not resetting the branch. No legacy PGM or extra launch selector was added.

## Geometry and coordinates

The new method finds multiple upward-facing support surfaces per XY cell,
checks 24 cm headroom and steep-wall intersections in the 2..24 cm local body
band, then keeps support states reachable from spawn. Slope limit is 20 degrees;
neighbor height allowance is 2 cm plus the height change of a 20 degree slope
over one cell. There is no blind hole filling or obstacle inflation. A separate 0.20 m inward
occupied rim (8 cells) is now painted on all four PGM edges to keep static-mode
navigation away from the unenclosed arena edge. Interior pixels are unchanged.
The dynamic-only generated navigation rim remains 0.35 m, not 0.20 m.

The central highland's reachable upper floor stays free; the central mechanism
remains occupied. Mapping assumptions and numerical checks are recorded in
`rmuc_2027_projection.json`. Its fitted main-floor planes are retained ONLY
for the simulator spawn-Z calibration, not used as the new mapping model.
Reported support heights are above the STL minimum, NOT absolute map Z.

The simulator model/spawn are unchanged by this map deployment:
scale=1; world translation=(-0.14910008013248444,-0.06378698348999023,
-0.27757187763365515); spawn=(4.6,6.4,0.28), yaw=0. The first planar odometry
sample zeroes this spawn in map/odom. The local floor is near world Z=0.
The ~1.5 degree field grade is retained. The paired simulator now uses
`rmuc_2027_strip_repaired.stl` for BOTH visual and collision geometry. This fixes
one reviewed ~1 mm welded ground marking near world X=0..3.319,
Y=3.27555..3.73026: 16 raised top/side faces removed, four existing underside
faces reversed to retain ground. All other 378646 source face records are
unchanged; the original STL is preserved. The repair is not a global decoration
cleanup or terrain flattening. Its derivative SHA is
`1d6c472f901e13df0396a18884da48d86b76d80b9e4b4bb0b557647dbf3f9318`.
These geometric checks do not certify Gazebo contact behavior.

## Consumers and resolution coupling

- `combined` / `static_only`: the original map_server topic now supplies this
  map, so its internal walls participate as static obstacles.
- `dynamic_only`: the reference RViz map displays this map; navigation still
  uses the existing free-interior + 0.35 m rim map and sensed FastLayer obstacles.
  The global window stays 28x17 m, with occupied positive-Y padding; its raster
  becomes 1120x680 at 2.5 cm. No interior PGM walls are merged in this mode.
- DOG map reads XY resolution from this same YAML: 5 cm becomes 2.5 cm, with
  approximately four times as many XY cells. Its +/-10 m processing range,
  Z resolution, decay, high-column filters and all code/thresholds are unchanged.
  This is an inherent map-resolution effect, NOT an independent tuning change.
- Point-LIO, planner, MINCO/ESDF/MPC and behavior-tree implementations are unchanged.
  Nav2 retains footprint/inflation processing; the PGM contains no inflation.

Use `./build_hero2026_safe.sh --jobs 4` to synchronize installation. The original
`field:=rmuc_2027` launch command is unchanged. Restart the algorithm to reload
the map, boundary and DOG grids. The PGM rim alone needs only an algorithm restart, but the separate strip
repair REQUIRES rebuilding/restarting the paired 2027 simulator using its usual
startup script. Then restart the algorithm after settling; do not reuse the
old Point-LIO alignment across a Gazebo restart. The 2026 image is unaffected.
A PGM boundary is a planning constraint, not a physical fence for manual driving.

No full navigation/clearance acceptance is claimed before driving tests.
2.5 cm sampling, multilevel 2-D projection and robot dynamics remain limitations;
check runtime CPU load, DOG occupancy and tunnel behavior after deployment.
2025/2026 maps and launch defaults are untouched.

Redistribution licence is unverified; do not publish the assets to a public remote.
