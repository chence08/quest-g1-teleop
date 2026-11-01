#!/usr/bin/env python3
"""Quest tracking preview with optional numerical IK; no robot transport.

This script never imports a robot controller or initializes DDS. The upstream
tracking readiness bit is latched and is NOT a tracking freshness signal.
"""

import argparse
from contextlib import contextmanager
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from teleop_config import load_config, validate_host_ip

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT / "repos" / "xr_teleoperate"


def prepare_imports():
    if not (REPO_ROOT / "teleop").is_dir():
        raise RuntimeError(f"Missing upstream checkout: {REPO_ROOT}")
    sys.path.insert(0, str(REPO_ROOT))
    os.environ.setdefault("MPLBACKEND", "Agg")


@contextmanager
def model_workspace():
    """Resolve upstream relative asset paths without leaving model caches in Git."""
    old_cwd = Path.cwd()
    with tempfile.TemporaryDirectory(prefix="quest-g1-preview-") as directory:
        root = Path(directory)
        (root / "assets").symlink_to(REPO_ROOT / "assets", target_is_directory=True)
        (root / "teleop").mkdir()
        os.chdir(root / "teleop")
        try:
            yield
        finally:
            os.chdir(old_cwd)


def pose_is_se3(pose):
    """Stricter offline/display check than the upstream determinant-only check."""
    import numpy as np

    pose = np.asarray(pose)
    return bool(
        pose.shape == (4, 4)
        and np.isfinite(pose).all()
        and np.allclose(pose[3], [0, 0, 0, 1], atol=1e-4)
        and np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-3)
        and np.isclose(np.linalg.det(pose[:3, :3]), 1.0, atol=1e-3)
    )


def pose_text(pose):
    import numpy as np

    rotation = pose[:3, :3]
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    xyz = ", ".join(f"{v:+.3f}" for v in pose[:3, 3])
    angles = ", ".join(f"{math.degrees(v):+.1f}" for v in (roll, pitch, yaw))
    return f"xyz_m=[{xyz}] rpy_deg=[{angles}]"


def ik_residuals(ik, q, targets):
    import numpy as np
    import pinocchio as pin

    model = ik.reduced_robot.model
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    errors = []
    for frame_id, target in zip((ik.L_hand_id, ik.R_hand_id), targets):
        actual = data.oMf[frame_id]
        errors.append({
            "position_m": float(np.linalg.norm(actual.translation - target[:3, 3])),
            "orientation_rad": float(np.linalg.norm(pin.log3(actual.rotation @ target[:3, :3].T))),
        })
    return errors


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ik", action="store_true", help="also solve G1_29 numerical IK and print joints/errors")
    parser.add_argument("--input-mode", choices=("hand", "controller"), default="hand")
    parser.add_argument("--arm-reference", choices=("head_yaw", "head_position"), default="head_yaw")
    parser.add_argument("--host-ip", default=load_config()['HOST_LAN_IP'], help="workstation address shown in the Quest connection URL")
    parser.add_argument("--rate", type=float, default=2.0, help="preview samples/second (default 2; not robot control rate)")
    parser.add_argument("--duration", type=float, default=0.0, help="exit after this many seconds; 0 means until Ctrl-C")
    parser.add_argument("--cert", type=Path, default=Path(os.environ.get("XR_TELEOP_CERT", PROJECT_ROOT / "certs" / "cert.pem")))
    parser.add_argument("--key", type=Path, default=Path(os.environ.get("XR_TELEOP_KEY", PROJECT_ROOT / "certs" / "key.pem")))
    args = parser.parse_args(argv)
    validate_host_ip(parser, args.host_ip)
    if not math.isfinite(args.rate) or not 0 < args.rate <= 30:
        parser.error("--rate must be finite and in (0, 30]")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be a finite nonnegative number")
    args.cert, args.key = args.cert.expanduser().resolve(), args.key.expanduser().resolve()
    return args


def main(argv=None):
    args = parse_args(argv)
    for path in (args.cert, args.key):
        if not path.is_file():
            raise RuntimeError(f"Missing TLS file: {path}. Generate the project certificates first (see README.md).")
    prepare_imports()
    import numpy as np
    from televuer import TeleVuerWrapper

    print("PREVIEW ONLY: no robot transport, controller, or image server is started.", flush=True)
    print("Readiness means an event arrived once. It does NOT prove current tracking or freshness.", flush=True)
    print("Head is in the converted world frame; wrists are targets in the mapped waist frame.", flush=True)
    print(f"Arm reference: {args.arm_reference}; stock fixed offsets: +0.15 m forward, +0.45 m up.", flush=True)
    print(f"Quest browser: https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012", flush=True)
    print("Accept this host's certificate, enter the XR session, allow tracking. Ctrl-C closes preview.", flush=True)
    wrapper = None
    with model_workspace():
        ik = None
        if args.ik:
            from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
            ik = G1_29_ArmIK(Visualization=False)
            q = np.clip(np.zeros(14), ik.reduced_robot.model.lowerPositionLimit, ik.reduced_robot.model.upperPositionLimit)
            print("IK uses the stock arm model; printed joints are not hardware commands or a collision check.", flush=True)
        try:
            wrapper = TeleVuerWrapper(
                use_hand_tracking=args.input_mode == "hand", display_mode="pass-through",
                zmq=False, webrtc=False, cert_file=str(args.cert), key_file=str(args.key),
                arm_reference_mode=args.arm_reference,
            )
            started = time.monotonic()
            while args.duration == 0 or time.monotonic() - started < args.duration:
                if not wrapper.tvuer.process.is_alive():
                    raise RuntimeError("The Vuer server process exited; check TLS files and whether port 8012 is in use.")
                data = wrapper.get_tele_data()
                if not data.motion_data_ready:
                    print("WAITING: no hand/controller event received. Fallback poses are not displayed as tracking.", flush=True)
                else:
                    raw_valid = all(pose_is_se3(p) for p in (
                        wrapper.tvuer.head_pose, wrapper.tvuer.left_arm_pose, wrapper.tvuer.right_arm_pose,
                    ))
                    print(f"EVENT SEEN (latched; freshness unknown); raw SE(3) valid={raw_valid}", flush=True)
                    print("  head world: " + pose_text(data.head_pose), flush=True)
                    print("  left target: " + pose_text(data.left_wrist_pose), flush=True)
                    print("  right target: " + pose_text(data.right_wrist_pose), flush=True)
                    if args.input_mode == "hand":
                        print(f"  thumb-index pinch distance_cm: left={data.left_hand_pinchValue:.2f}, right={data.right_hand_pinchValue:.2f}", flush=True)
                    if ik is not None and raw_valid:
                        targets = (data.left_wrist_pose, data.right_wrist_pose)
                        result, _ = ik.solve_ik(*targets, q, np.zeros(14))
                        success = bool(ik.opti.stats().get("success", False))
                        if success and result is not None and np.isfinite(result).all():
                            q = result
                            errors = ik_residuals(ik, q, targets)
                            print("  numerical IK q_rad: " + np.array2string(q, precision=3), flush=True)
                            print(f"  IK residuals L/R: {errors}", flush=True)
                        else:
                            print("  IK failed; numerical state retained. No robot is connected by this script.", flush=True)
                    elif ik is not None:
                        print("  IK skipped because raw poses failed validation.", flush=True)
                time.sleep(1.0 / args.rate)
        except KeyboardInterrupt:
            print("\nClosing Quest preview.", flush=True)
        finally:
            if wrapper is not None:
                wrapper.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Preview error: {exc}", file=sys.stderr)
        raise SystemExit(1)
