# Pinned upstream subset

- Source: https://github.com/mozihe/z-localization
- Commit: 76e126328b670adc92981d9b4900cc059a6f2343 (2026-10-22).
- Imported: src/backend, src/localization_interfaces, config/backend.yaml,
  cmake/Warnings.cmake, LICENSE, README.md, docs/design.md.
- No frontend, Livox driver or bringup package is imported: existing Point-LIO remains in use.
- Imported upstream sources were unmodified through 2026-11-04. GPL-2.0-only notices are retained.
- Local fixes on 2026-11-05: safely retire fully expired disconnected graph components;
  constrain SUSPECT scan-to-map recovery by the last trusted position/orientation and
  the existing tracking correction limits. Regression tests cover both faults.
  These changes do not modify the upstream default YAML, GTSAM, or the frontend.
  See ../hero_localization_backend/BACKEND_RECOVERY_FIX_20261105.md.
- HERO-specific configuration, launch isolation, CAD prior preparation and observer live in
  the adjacent hero_localization_backend package; see its README.md.
- GTSAM 4.2: 4f66a491ffc83cf092d0d818b11dc35135521612, built by the workspace safe script.
