"""Protect simulation BT tolerance without changing the real-robot BT."""

from pathlib import Path
import xml.etree.ElementTree as ET


TREE = Path(__file__).resolve().parents[1] / "behavior_trees/navigate_to_pose_sim.xml"


def test_original_retry_tolerance_is_restored():
    root = ET.parse(TREE).getroot()
    assert root.find(".//RecoveryNode[@name='NavigateRecovery']").get("number_of_retries") == "150"
    assert root.find(".//RecoveryNode[@name='FollowPath']").get("number_of_retries") == "10"


def test_humble_wait_is_a_positive_integer_second():
    waits = ET.parse(TREE).getroot().findall(".//Wait")
    assert len(waits) == 1
    assert waits[0].get("wait_duration") == "1"
    assert int(waits[0].get("wait_duration")) > 0


def test_physical_stall_detection_and_conservative_backup_remain():
    root = ET.parse(TREE).getroot()
    stuck = root.find(".//Sequence[@name='StuckRecovery']/IsStuckCondition")
    assert stuck.get("cmd_vel_topic") == "cmd_vel_nav2_result"
    assert stuck.get("odom_topic") == "odometry/filtered"
    assert stuck.get("stall_duration") == "1.5"
    backup = root.find(".//Sequence[@name='StuckRecovery']/BackUp")
    assert float(backup.get("backup_dist")) == .30
    assert float(backup.get("backup_speed")) == .15
    assert float(backup.get("time_allowance")) == 5.
