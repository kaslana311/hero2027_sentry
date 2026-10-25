# Pinned upstream subset

- Source: https://github.com/mozihe/z-localization
- Commit: 76e126328b670adc92981d9b4900cc059a6f2343 (2026-10-22).
- Imported: src/backend, src/localization_interfaces, config/backend.yaml,
  cmake/Warnings.cmake, LICENSE, README.md, docs/design.md.
- No frontend, Livox driver or bringup package is imported: existing Point-LIO remains in use.
- Upstream sources are unmodified. GPL-2.0-only notices are retained.
- HERO-specific configuration, launch isolation, CAD prior preparation and observer live in
  the adjacent hero_localization_backend package; see its README.md.
- GTSAM 4.2: 4f66a491ffc83cf092d0d818b11dc35135521612, built by the workspace safe script.
