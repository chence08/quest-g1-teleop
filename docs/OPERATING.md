# Operating notes

This build favours continuous, full-range motion over automatic stops, following upstream xr_teleoperate. **Software no longer stops the robot for you.** Keep a person on the physical Unitree remote whenever `--execute` is running.

## Prepare the wired workstation

Use the Ethernet interface's LAN address in `HOST_LAN_IP`, the server certificate and the Quest URL. Avoid guest/client-isolated networks. Keep the robot's internal DDS interface unchanged: workstation Ethernet carries HTTPS and SSH, not robot-internal DDS discovery.

Clock offset between the workstation and robot no longer matters: the bridges do not compare wall-clock timestamps. `scripts/check_robot_clock.py` is informational only.

The session launcher runs for up to `--session-duration` (default 4 hours). Each run lasts until `p` or `q` unless you pass `--duration SECONDS`.

## Running

1. Put the robot in the native mode you want (for example standing, FSM 500). The software reads the mode for telemetry but neither checks nor changes it.
2. Start `./full_teleop.sh --execute --host-ip YOUR_WORKSTATION_LAN_IP` (add `--arms-only` to skip the fingers).
3. Enter **Pass-through** in Quest Browser with both bare hands visible.
4. Hold a comfortable neutral pose and type `r` + Enter. If you have not calibrated, `r` calibrates first against the measured robot arm pose, then starts streaming.
5. Move. Tracking dropouts, IK failures and late packets hold the last target; streaming resumes when tracking returns.

| Key + Enter | Action |
| --- | --- |
| `c` | Recalibrate at the current wrist and measured robot pose; allowed while running |
| `r` | Start a run (calibrates first if needed) |
| `p` | Pause: ramp the arm overlay out to the native controller; calibration is kept, `r` resumes |
| `q` | Quit and clean up |

Looking around after calibration does not rotate wrist targets: the head-yaw reference is frozen at calibration.

## Motion parameters

| Setting | Default | Flag |
| --- | --- | --- |
| Wrist translation scale | 1.0 | `--translation-scale` |
| Max wrist offset / rotation from calibration | 1 m / π rad | `--translation-limit`, `--rotation-limit` |
| Hand packet age before holding | 0.5 s | `--tracking-timeout` |
| IK / command rate | 30 Hz | `--frequency` |
| Robot joint velocity limit | 1 → 10 rad/s over 3 s after engaging | `--max-velocity`, `--speed-ramp` |
| Arm gains | kp 80 / kd 3, wrists kp 40 / kd 1.5 (upstream) | — |
| Feedforward | IK model gravity torque, clipped to ±25 Nm | `--zero-feedforward` |
| Overlay weight ramp | 1 s in, 1 s out | robot bridge `--ramp-up`, `--release-time` |
| Fingers | full normalized range, 5 units/s slew | DFX bridge `--slew-rate` |

Joint targets are only clipped to the URDF limits. The IK model uses the stock G1 hand inertia, not the DFX hand, so feedforward torque is approximate; use `--zero-feedforward` if the arms drift up or down.

## DFX mapping

DFX wire order is **right six, then left six**. Each side is pinky, ring, middle, index, thumb bend, thumb rotation. The patched driver currently opens `/dev/ttyUSB1` for right and `/dev/ttyUSB2` for left at 115200 baud, ID 1. Check device identity after a reboot or USB change.

The hand bridge republishes the latest target at 20 Hz. Serial-loss counters are reported in telemetry but do not stop commands. If the DFX service fails to start, the launcher continues arms-only; if the hand SSH bridge exits, the sender restarts it every 2 seconds.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page unavailable | Confirm the session is running. Check the Ethernet address, certificate SAN and LAN reachability. |
| Certificate warning | Resolve trust on the headset for your own freshly generated certificate. |
| Page loads but no tracking | Keep the full `?ws=wss://ADDRESS:8012` query. Select **Pass-through** and allow hand tracking. |
| Arms freeze in place | `tracking_fresh` is false in the status line: a hand left view or packets are late. The robot holds until tracking returns. Raise `--tracking-timeout` if brief gaps cause stutter. |
| `calibration_rejected` | Both hands must be visible, and with `--execute` the robot must have sent arm feedback. |
| Arms jerky or overshoot | Lower `--max-velocity`, or try `--zero-feedforward`. |
| Arms too slow or lag | Raise `--max-velocity`; check `arm_dropped` in the status line for SSH backpressure. |
| `WARNING: another arm controller` | Something else publishes arm commands. Stop it, or the two will fight. |
| DFX service fails | Check serial identity/ownership (`fuser /dev/ttyUSB1 /dev/ttyUSB2`), the patched driver build and the loopback namespace. |

## Optional robot video

Quest Pass-through needs no robot RGB camera. Optional robot video is implemented but has **not** been physically validated in this setup. Add `--camera-device /dev/EXPLICIT_DEVICE_PATH` to the launcher; prefer a stable `/dev/v4l/by-id/` name. If the camera cannot start, the session continues in Pass-through.

The optional stream is monocular 640×480 at 15 fps using the pinned TeleImager JPEG/ZMQ protocol.
