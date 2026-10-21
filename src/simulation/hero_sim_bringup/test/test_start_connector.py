"""Compile/run the exact C++ connector helper, not a Python reimplementation."""
from pathlib import Path
import shutil
import subprocess

import pytest

PLANNER = Path(__file__).parents[3] / "planning/hero_static_prior_planner"


@pytest.fixture(scope="module")
def native_connector(tmp_path_factory):
    compiler = shutil.which("g++")
    assert compiler, "The safe-build compiler is required for native connector checks"
    output = tmp_path_factory.mktemp("split_connector") / "test_connector"
    result = subprocess.run([compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(PLANNER / "include"), str(PLANNER / "test/test_start_connector.cpp"),
        "-o", str(output)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    return output


@pytest.mark.parametrize("case", ("nearest_exit", "new_goal", "float_map_geometry", "radius_limit",
    "occupied_start", "blocked_connection", "unknown_connection", "thin_wall",
    "alternative_exit", "invalid_input"))
def test_native_start_connector(native_connector, case):
    result = subprocess.run([str(native_connector), case], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
