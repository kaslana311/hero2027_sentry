# RMUC 2027 multilevel support-surface map (applied 2026-10-24)

The active `rmuc_2027.pgm` is the user's manually edited version of the redrawn
map from the paired simulator. The unchanged generated baseline remains at
`maps/candidates/rmuc_2027/rmuc_2027_surface_candidate.pgm`; it is no longer
byte-identical to the active map. Do not copy/regenerate it over the manual edits.

- Active PGM SHA-256: `c53fc95b7ad4154078cc216f5c2d4a567d67a13e2847b63adb2d9261a4e88e45`.
- Generated baseline SHA-256: `43d2a27e52b85fe8e5e0b19794fc022f65b5f9eff84fff6a7bb8c677ae8802d3`.
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
occupied rim (8 cells) was painted on all four PGM edges to keep static-mode
navigation away from the unenclosed arena edge. That generation step left the
interior unchanged; the later user edits intentionally modify interior pixels.
The dynamic-only generated navigation rim remains 0.35 m, not 0.20 m.

The central highland's checked upper-floor points stay free; the central
mechanism's checked point remains occupied. Original generation assumptions and
geometric checks are retained in `rmuc_2027_projection.json` as baseline data,
not a new STL clearance certification of the manual edits. Its fitted main-floor planes are retained ONLY
for the simulator spawn-Z calibration, not used as the new mapping model.
Reported support heights are above the STL minimum, NOT absolute map Z.

## First user edit applied on 2026-10-16

The user edited the source PGM and asked to apply it. Its 1120x642 dimensions,
0.025 m resolution, origin and all four 8-cell black rims are unchanged. Compared
with the generated baseline, 38297 pixels changed: 35065 white-to-black and
3232 black-to-white. First-edit totals were 171685 occupied and 547355 free pixels;
there are no gray/unknown pixels. The user's image bytes are preserved exactly.
The simulator's generated baseline, STL, spawn and Docker image are not changed
by this manual-map deployment. Review manual passage clearances during driving;
only file/coordinate/configuration checks were performed here.

## Second user edit applied on 2026-10-16

The user revised the source PGM again. Compared with first-edit commit `451ea7e`
(SHA-256 `ed7331d69bc59231752468a962c9b9ecfaf9e8927f0cd1fa9209fa133e5c4a58`),
14615 pixels changed: 2367 white-to-black and 12248 black-to-white. Second-edit totals
were 161804 occupied and 557236 free pixels. Dimensions, resolution, origin,
20 cm rims and the existing central check pixels remain unchanged; the image
still contains only 0/255 values. This revision is installed without rewriting
the user's PGM, changing algorithm logic, or rebuilding the simulator image.

## Third user edit applied on 2026-10-24

Source: `/home/apocalypse_linux_00/Downloads/rmuc_2027.pgm`, copied byte-for-byte.
Compared with algorithm commit `ff9421e` (previous active SHA-256
`e98bc31e0719deb36bdcf2acb876bfa1d4ff3758dc782553563e49590db4c204`),
3824 pixels changed, all white-to-black. Third-edit totals were 165628 occupied and
553412 free pixels (345.8825 square metres). Size, YAML bytes, resolution,
origin, 20 cm occupied rims and central check pixels are unchanged.
No geometry, spawn, TF, filter, planner or costmap-mode logic is changed.
This is an asset-only deployment: the source PGM and installed PGM are replaced
with the exact same user bytes; provenance documents are synchronized as well.
No code compilation or Docker rebuild is needed. The existing safe build
script's `--verify-only` mode audits the installation without a cold rebuild.
Restart the algorithm to reload the map; the simulator need not restart.

## Fourth user edit applied on 2026-10-24

The user widened the passages and requested another exact-byte deployment of
`/home/apocalypse_linux_00/Downloads/rmuc_2027.pgm`. Relative to commit `cf04c36`
(SHA-256 `0a8a6740905f2e53ec917c5beb41154ef60873c3b2270a911b1bc1ed8f336ca7`),
1655 pixels changed, all black-to-white. Fourth-edit totals were 163973 occupied and
555067 free pixels (346.916875 square metres). The two reviewed passage
cross-sections at raster rows 520 and 120 were 0.60 m and 0.625 m wide,
respectively; these are sampled widths, not a full navigation acceptance test.
Dimensions, YAML bytes, origin, resolution, 20 cm occupied rims and central
check pixels remain unchanged. Source and installed assets are synchronized;
no planner weights, robot radius, inflation, TF, filters, simulator geometry or
launch commands are changed. Restart only the algorithm to reload this map.

## Fifth user edit applied on 2026-10-24

Source: `/home/apocalypse_linux_00/Downloads/rmuc_2027.pgm`, copied byte-for-byte
to the source and installed map. Relative to commit `5c66f18` (previous SHA-256
`75aac7aa13987a13995cd09157fec25ee8b03d2d6bde817306bac2163978bf9d`),
3651 pixels changed, all white-to-black. Fifth-edit totals were 167624 occupied and
551416 free pixels (344.635 square metres). The map remains binary 0/255 at
1120x642; YAML bytes, origin, resolution, 20 cm occupied rims and existing
central check pixels are unchanged. Provenance and checksum tests follow the
new user image; no planning, perception, simulator, or mode parameters change.
This is a data-only deployment: restart the algorithm using the original
command to reload it; no compilation or Docker rebuild is required.
File/configuration checks do not certify physical passage clearance or routing.

## Sixth user edit applied on 2026-10-24

Source: `/home/apocalypse_linux_00/Downloads/rmuc_2027.pgm`, copied byte-for-byte
to the source and installed map. Relative to commit `b488c9d` (previous SHA-256
`426eefebfa9fd594f2de7ff9665f2c7a791ff22d65238c9b7ff76431e11b4dde`),
488 pixels changed, all white-to-black. Current totals are 168112 occupied and
550928 free pixels (344.33 square metres). Dimensions, YAML bytes, resolution,
origin, 20 cm occupied rims and existing central check pixels are unchanged.
Only the user image, provenance and corresponding checksum/count tests change;
no planning/perception parameters, launch logic or simulator assets are altered.
Restart the algorithm to reload the map; no code or Docker rebuild is required.
File/configuration checks are not a driving or shortest-route acceptance test.

The simulator model/spawn are unchanged by this map deployment:
scale=1; world translation=(-0.14910008013248444,-0.06378698348999023,
-0.27757187763365515); spawn=(4.6,6.4,0.28), yaw=0. The first planar odometry
sample zeroes this spawn in map/odom. The local floor is near world Z=0.
The ~1.5 degree field grade is retained. The earlier paired simulator used
`rmuc_2027_strip_repaired.stl` for both visual and collision geometry. This fixed
one reviewed ~1 mm welded ground marking near world X=0..3.319,
Y=3.27555..3.73026: 16 raised top/side faces removed, four existing underside
faces reversed to retain ground. All other 378646 source face records are
unchanged; the original STL is preserved. The repair is not a global decoration
cleanup or terrain flattening. Its derivative SHA is
`1d6c472f901e13df0396a18884da48d86b76d80b9e4b4bb0b557647dbf3f9318`.
These geometric checks do not certify Gazebo contact behavior.

On 2026-10-17 the simulator source selects `rmuc_2027_clean.stl` for BOTH visual
and collision geometry. Its SHA-256 is
`625f1444f36098995e26fbe85cdc133ebbd00311505d96a60d226b16ed91793c`.
The explicit simulator `ground_decorations.json` reviews 44 CAD marking groups
and 79 embossed-rim patches (not 123 separate objects). It removes 3081 faces,
modifies 3269, and leaves 372316 original records byte-identical. Where deletion
would expose a ground hole, 744 underside faces instead become upward support.
1503 local vertex Z edits are bounded by 1.4 mm; XY, field grade, model pose and
spawn remain unchanged. No real wall, ramp or step is removed. Both the original
STL and historical one-strip derivative remain available. The 7420 support samples
and 68 simulator checks passed, but no Gazebo contact acceptance is claimed;
the noninteractive Docker build still requires the user's sudo password. Rebuild
with the usual 2027 simulator startup script, then restart Point-LIO/navigation.
This geometry cleanup does not regenerate/alter the user's hand-edited PGM,
change algorithm implementations/parameters, or alter the 2026 assets.

## Consumers and resolution coupling

- `combined` / `static_only`: the original map_server topic now supplies this
  map, so its internal walls participate as static obstacles.
- `dynamic_only`: the reference RViz map displays this map; navigation still
  uses the existing free-interior + 0.35 m rim map and sensed FastLayer obstacles.
  The global window stays 28x17 m, with occupied positive-Y padding; its raster
  becomes 1120x680 at 2.5 cm. No interior PGM walls are merged in this mode.
- `split`: A* uses the prior static PGM, while the smoother retains the existing
  dynamic-only global costmap composition. This edit changes the prior map and
  RViz reference, not the dynamic obstacle pipeline or mode configuration.
- DOG XY voxel resolution is independently selected by `dog_resolution_xy`
  (default 0.05 m on this branch); only an explicit value of 0 follows the PGM
  resolution. The PGM remains 0.025 m/cell. Neither value is changed by this
  deployment. Processing range, Z resolution, decay and filters are unchanged.
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
