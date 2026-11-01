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
| `c` | Capture neutral while paused with stable tracking and appropriate feedback |
| `r` | Start a bounded run after calibration, only when execution was explicitly enabled |
| `p` | Release the arm overlay and stop finger-target submission; discard calibration |
| `q` | Exit and clean up the session's owned services |

Before its first write, the sender requires 0.5 seconds of valid tracking and ready feedback. It rechecks age after IK. After any write attempt, tracking/feedback loss or transport failure stops both streams and requires recalibration/rearming, including blocked or partial writes.

Default arm limits include 0.5 translation scaling, 8 cm wrist translation, 0.25 rad rotation, 0.15 rad joint excursion and 0.15 rad/s slew. The 100 Hz robot bridge checks freshness, mode, motion and effort. It expects `mode_machine=6`/native FSM 500 without selecting modes or commanding waist, legs or head. Feedforward torque is zero because hand inertia is uncalibrated.

Release returns control to the native controller and can move the arms; DFX can retain its last target. It is not a physical stop or guaranteed pose hold. Initial tests need support, clearance and an attended physical stop. Software bounds are not complete collision or force protection.

## Optional video

The current view is Quest Pass-through. An optional bounded RGB service feeds JPEG/ZMQ through SSH to the pinned TeleImager client. Synthetic protocol tests passed; **real robot video remains unvalidated**. Coordinate device ownership before enabling capture. See [COMMISSIONING.md](COMMISSIONING.md) for outstanding physical tests.
