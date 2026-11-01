# Architecture

Quest 3 camera-based hand tracking drives G1 29-DoF arm IK and Inspire RH56DFX finger retargeting through bounded project-local bridges. See [upstream.lock.json](../upstream.lock.json) for source pins and the [README](../README.md) for deployment commands.

## Deployment topology

```text
Workstation ── Ethernet ── LAN switch / access point
    ▲                          ├── Wi-Fi ── Quest 3
    │ HTTPS / WebSocket        └── Wi-Fi ── G1 development computer
    │                                      ▲
    └──────── timestamped targets over SSH ─┘
                                           ├── internal DDS → arm overlay
                                           └── loopback DDS → DFX USB service
```

Connect the workstation by Ethernet to the Quest access point's LAN. The robot remains on Wi-Fi/SSH; it needs no external Ethernet cable. Allow Quest-to-workstation traffic and workstation-to-robot SSH; guest/client isolation can block either path. This Ethernet arrangement is a deployment target, not a validated cure for earlier tracking interruptions.

| Component | Responsibility |
| --- | --- |
| Quest Browser | Secure WebXR head, wrist and finger tracking; Pass-through view |
| Workstation | HTTPS/WSS interface, measured-posture calibration, G1 IK, DFX retargeting and SSH target submission |
| Robot arm bridge | Read native state; validate fresh targets; publish only arm motor slots and arm-overlay weight on `rt/arm_sdk` |
| Robot DFX bridge/service | Bound finger targets; use an isolated DDS namespace on loopback; communicate with the USB hands |

DDS stays on the robot's internal interface for arms and loopback for hands; LAN DDS multicast and ROS 2 are unnecessary. Align workstation/robot clocks for timestamp checks. SSH write success does not prove DDS publication or movement.

## Calibration and control

Calibration aligns neutral operator wrists to the **measured robot posture**. Head yaw and origin are then frozen; looking around does not rotate arm targets.

| Terminal input, followed by Enter | Action |
| --- | --- |
| `c` | Recalibrate at the current wrist and measured robot pose, also while running |
| `r` | Start streaming (calibrates first if needed); requires `--execute` |
| `p` | Ramp the arm overlay out and stop finger targets; calibration is kept |
| `q` | Exit and clean up the session's owned services |

The sender solves IK at 30 Hz, warm-started from its previous solution, and streams the latest target every tick. When hand packets are older than `--tracking-timeout`, or IK or retargeting fails, it resends the last target. SSH backpressure drops a frame. Only `p`, `q`, an arm-bridge exit or `--duration` end a run.

The 250 Hz robot bridge holds the measured pose while the overlay weight ramps in, then tracks the latest target through an upstream-style joint velocity limit (scaling the whole move so no joint exceeds it) that ramps from 1 to 10 rad/s. Targets are clipped to URDF limits and feedforward torque to ±25 Nm. Out-of-order or malformed packets are skipped; stdin EOF ramps the weight out. Mode, FSM, measured motion, torque, drift and wall-clock checks are not enforced.

Release returns control to the native controller and moves the arms to its pose; DFX keeps its last target. There are no automatic safety stops: keep an attended physical stop.

## Optional video

The current view is Quest Pass-through. An optional bounded RGB service feeds JPEG/ZMQ through SSH to the pinned TeleImager client. Synthetic protocol tests passed; **real robot video remains unvalidated**. Coordinate device ownership before enabling capture. See [COMMISSIONING.md](COMMISSIONING.md) for outstanding physical tests.
