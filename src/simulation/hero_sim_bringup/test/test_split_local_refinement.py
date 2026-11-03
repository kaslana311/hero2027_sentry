"""Compile exact deployed core + protect opt-in split-only configuration."""
from pathlib import Path
import subprocess

import pytest
import yaml

from test_split_mode import BASE, PACKAGE, factory, generated

PLANNER = PACKAGE.parents[1] / "planning/hero_static_prior_planner"


@pytest.fixture(scope="module")
def native_refiner(tmp_path_factory):
    binary = tmp_path_factory.mktemp("local_refiner") / "test_refiner"
    result = subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(PLANNER / "include"), str(PLANNER / "src/local_refiner.cpp"),
        str(PLANNER / "test/test_local_refiner.cpp"), "-o", str(binary)],
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    return binary


@pytest.mark.parametrize("case", ("drift_sweep", "wall_gap", "yaw_mismatch", "one_side_rejected",
    "lintel_is_soft", "no_observations", "budget", "hard_rim", "invalid", "window"))
def test_native_refiner(native_refiner, case):
    run = subprocess.run([str(native_refiner), case], capture_output=True, text=True, timeout=15)
    assert run.returncode == 0, run.stdout + run.stderr


def experimental(mode="split", field="rmuc_2027"):
    return Path(factory()._make_sim_params(str(BASE), str(PACKAGE / "maps" / f"{field}.yaml"),
        "/tmp/test_bt.xml", mode, "red_standard_robot1", False,
        "pointlio/cloud_registered_map", True, .20,
        split_local_refinement=True))


def test_only_opt_in_planner_parameters_change():
    path = experimental()
    try:
        actual = yaml.safe_load(path.read_text())
        with generated("split", "rmuc_2027") as baseline:
            grid = actual["planner_server"]["ros__parameters"]["GridBased"]
            added = {key: grid.pop(key) for key in list(grid) if key.startswith("local_refinement_")}
            assert added["local_refinement_enabled"] is True
            assert len(added["local_refinement_regions"]) == 30
            assert actual == baseline
    finally:
        path.unlink()


@pytest.mark.parametrize("mode,field", (("combined", "rmuc_2027"),
    ("static_only", "rmuc_2027"), ("dynamic_only", "rmuc_2027"),
    ("split", "rmuc_2026"), ("split", "rmuc_2025")))
def test_wrong_mode_or_field_rejected(mode, field):
    with pytest.raises(ValueError, match="requires split mode and rmuc_2027"):
        experimental(mode, field)
