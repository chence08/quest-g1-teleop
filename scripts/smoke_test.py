#!/usr/bin/env python3
"""Offline installation checks. No DDS, robot controller, camera, or XR server."""

import argparse
from datetime import datetime, timezone
import importlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

from preview_quest import REPO_ROOT, ik_residuals, model_workspace, pose_is_se3, prepare_imports


def dependencies():
    results = {}
    for module_name in ("numpy", "pinocchio", "pinocchio.casadi", "casadi", "torch", "meshcat", "cv2", "logging_mp", "dex_retargeting", "televuer", "teleimager", "yaml", "sshkeyboard", "rerun"):
        module = importlib.import_module(module_name)
        results[module_name] = str(getattr(module, "__version__", "import OK"))
    # Discovery only: do not import SDK/DDS/controller modules to test availability.
    results["unitree_sdk2py discoverable"] = importlib.util.find_spec("unitree_sdk2py") is not None
    if not results["unitree_sdk2py discoverable"]:
        raise AssertionError("Unitree SDK2 Python package is missing")
    results["inspire_sdkpy discoverable (FTP optional)"] = importlib.util.find_spec("inspire_sdkpy") is not None
    return results


def transforms():
    import numpy as np
    from televuer.tv_wrapper import (
        T_ROBOT_OPENXR, T_OPENXR_ROBOT, get_Brobot_world_head_yaw_rot,
        safe_mat_update, transform_IPunitree_Brobot_world_arm_to_head_then_waist,
    )
    np.testing.assert_allclose(T_ROBOT_OPENXR @ T_OPENXR_ROBOT, np.eye(4), atol=1e-12)
    head, arm = np.eye(4), np.eye(4)
    head[:3, 3], arm[:3, 3] = [1.0, 2.0, 1.5], [1.4, 2.2, 1.1]
    target = transform_IPunitree_Brobot_world_arm_to_head_then_waist(arm, head, "head_yaw")
    np.testing.assert_allclose(target[:3, 3], [0.55, 0.2, 0.05], atol=1e-12)
    yaw = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    world = np.eye(4)
    world[:3, :3], world[:3, 3] = yaw, [2., -3., 0.2]
    rotated = transform_IPunitree_Brobot_world_arm_to_head_then_waist(world @ arm, world @ head, "head_yaw")
    np.testing.assert_allclose(rotated, target, atol=1e-12)
    pitched = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    np.testing.assert_allclose(get_Brobot_world_head_yaw_rot(pitched), np.eye(3), atol=1e-12)
    fallback, valid = safe_mat_update(np.eye(4), np.zeros((4, 4)))
    assert valid is False
    np.testing.assert_array_equal(fallback, np.eye(4))
    assert pose_is_se3(target)
    assert not pose_is_se3(np.diag([2., 1., 1., 1.]))
    invalid = np.eye(4)
    invalid[0, 3] = np.nan
    assert not pose_is_se3(invalid)
    return {"basis_inverse": True, "fixed_offset_m": [0.15, 0.0, 0.45], "common_world_yaw_invariance": True,
            "invalid_input_detection": True, "limitation": "Stock fallback is a fixed pose, not a freshness watchdog."}


def g1_ik():
    import numpy as np
    import pinocchio as pin
    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

    ik = G1_29_ArmIK(Visualization=False)
    model = ik.reduced_robot.model
    assert model.nq == model.nv == 14, f"Expected 14 arm joints, got {model.nq}/{model.nv}"
    q_goal = pin.neutral(model)
    for name, value in {
        "left_shoulder_pitch_joint": 0.15, "right_shoulder_pitch_joint": 0.15,
        "left_shoulder_roll_joint": 0.15, "right_shoulder_roll_joint": -0.15,
        "left_elbow_joint": 0.6, "right_elbow_joint": 0.6,
    }.items():
        assert model.existJointName(name), name
        q_goal[model.joints[model.getJointId(name)].idx_q] = value
    q_goal = np.clip(q_goal, model.lowerPositionLimit, model.upperPositionLimit)
    data = model.createData()
    pin.framesForwardKinematics(model, data, q_goal)
    targets = tuple(data.oMf[index].homogeneous.copy() for index in (ik.L_hand_id, ik.R_hand_id))
    q_seed = np.clip(q_goal + 0.01, model.lowerPositionLimit, model.upperPositionLimit)
    started = time.perf_counter()
    q, torque = ik.solve_ik(*targets, q_seed, np.zeros(model.nv))
    elapsed = time.perf_counter() - started
    stats = ik.opti.stats()
    assert stats.get("success", False), f"IPOPT did not converge: {stats.get('return_status')}"
    assert q is not None and q.shape == (14,) and np.isfinite(q).all()
    assert torque.shape == (14,) and np.isfinite(torque).all()
    assert (q >= model.lowerPositionLimit - 1e-6).all() and (q <= model.upperPositionLimit + 1e-6).all()
    errors = ik_residuals(ik, q, targets)
    for error in errors:
        assert error["position_m"] < 0.03, f"FK position residual too large: {errors}"
        assert error["orientation_rad"] < 0.15, f"FK rotation residual too large: {errors}"
    return {"nq": model.nq, "solver_status": str(stats.get("return_status")), "solve_seconds": elapsed,
            "residuals_left_right": errors, "thresholds": {"position_m": 0.03, "orientation_rad": 0.15},
            "limitation": "One reachable synthetic FK target pair; not collision, calibration, or real-time certification."}


def inspire_retargeting():
    import numpy as np
    from teleop.robot_control.hand_retargeting import HandRetargeting, HandType

    mapping = HandRetargeting(HandType.INSPIRE_HAND)
    result = {}
    for side, prefix in (("left", "L"), ("right", "R")):
        seq = getattr(mapping, f"{side}_retargeting")
        order = getattr(mapping, f"{side}_dex_retargeting_to_hardware")
        expected = [f"{prefix}_{part}" for part in (
            "pinky_proximal_joint", "ring_proximal_joint", "middle_proximal_joint",
            "index_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint",
        )]
        assert [seq.joint_names[index] for index in order] == expected
        assert len(order) == len(set(order)) == 6
        robot = seq.optimizer.robot
        q_reference = seq.get_qpos()
        if seq.optimizer.adaptor is not None:
            q_reference = seq.optimizer.adaptor.forward_qpos(q_reference)
        robot.compute_forward_kinematics(q_reference)
        # Make a deterministic 25-keypoint fixture from this model's own FK.
        # Only wrist and fingertips are used by the shipped DexPilot config.
        keypoints = np.zeros((25, 3))
        links = [f"{prefix}_hand_base_link"] + [f"{prefix}_{finger}_tip" for finger in ("thumb", "index", "middle", "ring", "pinky")]
        for index, link in zip((0, 4, 9, 14, 19, 24), links):
            keypoints[index] = robot.get_link_pose(robot.get_link_index(link))[:3, 3]
        indices = getattr(mapping, f"{side}_indices")
        ref = keypoints[indices[1]] - keypoints[indices[0]]
        q = seq.retarget(ref)
        assert np.isfinite(q).all()
        hardware = q[order]
        assert hardware.shape == (6,) and np.isfinite(hardware).all()
        joint_limits = robot.joint_limits[order]
        assert (hardware >= joint_limits[:, 0] - 1e-6).all() and (hardware <= joint_limits[:, 1] + 1e-6).all()
        status = int(seq.optimizer.opt.last_optimize_result())
        assert status > 0, f"Retarget optimizer failed: {status}"
        result[side] = {"hardware_order": expected, "output_rad": hardware.tolist(), "optimizer_status": status}
    result["limitation"] = "Tests hand retargeting math, not DFX/FTP communications or actuator calibration."
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, help="also save a JSON report at this path")
    args = parser.parse_args(argv)
    report_path = args.report.expanduser().resolve() if args.report else None
    report = {"time_utc": datetime.now(timezone.utc).isoformat(), "python": sys.version,
              "platform": platform.platform(), "checks": [], "scope": "Offline only; no robot/XR/camera services started."}
    prepare_imports()
    report["upstream_commit"] = subprocess.check_output(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"], text=True).strip()
    with model_workspace():
        for name, check in (("dependencies", dependencies), ("synthetic_transforms", transforms),
                            ("g1_forward_inverse_kinematics", g1_ik), ("inspire_hand_retargeting", inspire_retargeting)):
            try:
                detail = check()
                report["checks"].append({"name": name, "status": "pass", "detail": detail})
                print(f"PASS {name}", flush=True)
            except Exception as exc:
                report["checks"].append({"name": name, "status": "fail", "error": str(exc), "traceback": traceback.format_exc()})
                print(f"FAIL {name}: {exc}", flush=True)
    forbidden = [name for name in sys.modules if name.startswith(("cyclonedds", "unitree_sdk2py.core.channel", "teleop.robot_control.robot_arm", "teleop.robot_control.robot_hand_")) and name != "teleop.robot_control.robot_arm_ik"]
    report["checks"].append({"name": "no_robot_transport_imports", "status": "fail" if forbidden else "pass", "modules": forbidden})
    report["passed"] = all(item["status"] == "pass" for item in report["checks"])
    if report_path:
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        print(f"Report: {report_path}", flush=True)
    print("Offline checks passed." if report["passed"] else "Offline checks failed; see details above/report.", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
