# Operating notes

## Prepare the wired workstation

Use the Ethernet interface's LAN address in `HOST_LAN_IP`, the server certificate and the Quest URL. Avoid guest/client-isolated networks. Keep the robot's internal DDS interface unchanged: workstation Ethernet carries HTTPS and SSH, not robot-internal DDS discovery.

Run the read-only clock probe before commissioning:

```bash
./run_python.sh scripts/check_robot_clock.py --report reports/clock.json
```

Its result is a sampled compatibility check, not a latency guarantee. The robot accepts command timestamps only within its freshness bounds; clock offset consumes that budget. Resolve large offsets using the normal time-sync configuration of both computers. Do not extend stale-data limits to conceal transport failures.

The normal session launcher defaults to 600 seconds and supports at most 900 seconds. For longer calibration preparation **without starting a DFX service**, use the direct arms-only interface:

```bash
./run_python.sh scripts/quest_upper_body.py --execute --arms-only \
  --host-ip YOUR_WORKSTATION_LAN_IP --wait-timeout 3600 --duration 20 \
  --translation-limit .04 --rotation-limit .15
```

No publisher starts until explicit `c` then `r` and successful qualification. Each armed attempt retains its own duration limit. Quit with `q`; the preview is not a permanent background service.

## Neutral calibration

1. Secure the robot, clear both arms/hands, and have an observer hold the physical remote. Establish exclusive motion-controller ownership.
2. Use the normal native controls to reach the expected stationary posture and mode. Read-only feedback must be ready.
3. Enter **Pass-through** in Quest Browser. Keep both bare hands visible; controllers are not the input. Avoid opening headset menus during a run.
4. Hold a comfortable neutral wrist pose and type `c` + Enter. Calibration records measured robot joint/FK poses; it does not home or move the robot.
5. After `calibrated` and a valid IK preview, type `r` + Enter. Start with small movements of one wrist at a time. Confirm actual movement and correct direction with the observer.

Looking around after calibration does not rotate wrist targets: the head-yaw reference is frozen. Robot posture changes require a new calibration. A first command is not sent until tracking and arm/hand feedback remain ready for 0.5 seconds; the arm overlay then ramps for two seconds holding the measured initial targets.

Default translation scale is 0.5. The regular commissioning workspace is 8 cm / 0.25 rad per wrist; the README's first-arm example uses 4 cm / 0.15 rad. Every joint request is additionally restricted to ±0.15 rad around the measured starting posture, with 0.15 rad/s slew. These are target bounds, not guaranteed physical displacement.

## DFX mapping and stopping

DFX wire order is **right six, then left six**. Each side is pinky, ring, middle, index, thumb bend, thumb rotation. The patched driver currently opens `/dev/ttyUSB1` for right and `/dev/ttyUSB2` for left at 115200 baud, ID 1. Check device identity and orientation after a reboot or USB change before any live test.

The hand bridge bounds targets to ±0.15 normalized units around measured starting values, with 0.15 units/s slew. It requires fresh finite state and non-increasing serial-error counters. It does not change the hand's force or speed registers and performs no automatic homing.

`p` releases the arm overlay to native control, stops hand-target submission and clears calibration. Native handover can move the arms. The DFX controller may retain or finish its last target; stopping publications does not make it torque-free. Use the physical remote for unexpected motion. After any command attempt, faults latch and require a new `c → r`; there is no automatic rearm.

Reports distinguish targets written to SSH from robot DDS publications. Neither proves correct physical following. Preserve operator observations separately; `reports/` is intentionally untracked.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Page unavailable | Confirm the preview process is running; session expiry closes port 8012. Check the Ethernet address, certificate SAN and LAN reachability. |
| Certificate warning | Verify that it is your own workstation and freshly generated certificate. Resolve trust on the headset; keep private keys off Git and other devices. |
| Page loads but no tracking | Keep the full `?ws=wss://ADDRESS:8012` query. Select the page's **Pass-through** button and allow tracking. A browser page or one head event is not an immersive tracking session. |
| Grid/vector/hand overlays disappear | Grid and rendered hand models are hidden by this configuration. This alone is not a session failure; inspect incoming event sequences and ages. |
| `tracking_invalid_or_stale` | Inspect event ages and pose diagnostics. Both hands must remain visible. Separate browser/menu pauses from network delay/loss. Use preview to establish stable delivery before live tests. |
| Native mode/feedback rejected | Check the physical remote, motor state, expected firmware mode and competing controllers. The software does not switch native modes. |
| IK residual rejected | Recalibrate at a comfortable neutral; reduce movement and rotation. Do not assume full-range mirroring is commissioned. |
| DFX service fails | Check serial identity/ownership, SDK libraries, patched driver build, loopback namespace and error counters. Preserve other users' processes. |

## Optional robot video

Quest Pass-through needs no robot RGB camera. Optional robot video is implemented but has **not** been physically validated in this setup. After obtaining exclusive access to a specific RGB device, configure its Python runtime and add `--camera-device /dev/EXPLICIT_DEVICE_PATH` to the launcher. Prefer a verified stable `/dev/v4l/by-id/` or `/dev/v4l/by-path/` name. The launcher checks real JPEG delivery before using the owned SSH tunnel; it does not stop another camera owner or reconfigure depth capture.

The optional stream is monocular 640×480 at 15 fps using the pinned TeleImager JPEG/ZMQ protocol. Validate viewpoint alignment and end-to-end latency before relying on it for remote manipulation.
