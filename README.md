# Quest 3 → G1 arms + Inspire DFX

[![Offline checks](https://github.com/chence08/quest-g1-teleop/actions/workflows/check.yml/badge.svg)](https://github.com/chence08/quest-g1-teleop/actions/workflows/check.yml)

A reproducible upper-body teleoperation integration built on **Unitree xr_teleoperate**. Quest camera-based hand tracking drives calibrated wrist targets and DFX finger retargeting. Arm and hand commands travel over SSH; DDS stays on the robot.

**Status:** bounded DFX-only motion was physically confirmed. Neutral arm calibration and numerical IK passed; **physical arm and combined operation remain uncommissioned** because intermittent Quest tracking stopped the live attempts before any arm motor commands were published. See [commissioning results](docs/COMMISSIONING.md).

## Deployment topology

```text
Quest 3 ── Wi-Fi ── access point/router ── Ethernet ── workstation
                              │                         │
                              └── robot LAN/Wi-Fi ◀── SSH
                                      ├── arm SDK overlay (internal DDS)
                                      └── DFX service (loopback DDS + USB)
```

Wiring the workstation removes one wireless hop; it does not guarantee continuous Quest tracking. Keep the Quest, workstation and robot mutually reachable on a private LAN. This project does not expose robot control to the public internet.

## Hardware and versions

- Unitree **G1 29-DoF**, **Inspire RH56DFX** hands, Quest 3 with hand tracking.
- First live tests require mechanical support, clear arm/hand space and an observer with the physical Unitree remote.
- Workstation: Ubuntu 24.04 x86-64, Git, OpenSSL and Conda; CPU-only IK is supported.
- Robot: Ubuntu 22.04 aarch64, a Python environment with Unitree SDK2, and SDK2 C++/CycloneDDS build dependencies for the DFX service.

| Component | Recorded version |
| --- | --- |
| Python | 3.10 (validated with 3.10.22 workstation / 3.10.20 robot) |
| NumPy / Pinocchio / CasADi | 1.26.4 / 3.1.0 / 3.6.7 |
| PyTorch | 2.3.0 CPU |
| Vuer / params-proto | 0.0.60 / 2.13.0 |
| CycloneDDS Python | 0.10.2 |
| xr_teleoperate and companion repositories | Exact commits in [upstream.lock.json](upstream.lock.json) |

Package constraints and an original-machine Conda lock are included. The portable installer and robot preparation are described below; no upstream repositories or binary environments are bundled.

## Install

```bash
git clone https://github.com/chence08/quest-g1-teleop.git
cd quest-g1-teleop
python3 scripts/fetch_upstream.py
./rebuild_environment.sh
cp config/example.env config/local.env
# Edit config/local.env for your Ethernet LAN address and robot SSH/Python paths.
```

Generate **new** local certificates for the workstation's Ethernet address, then run the offline installation check:

```bash
./run_python.sh scripts/make_certs.py --ip YOUR_WORKSTATION_LAN_IP
./run_python.sh scripts/smoke_test.py
```

No installation command starts robot motion. Follow [robot preparation](docs/ROBOT_SETUP.md) before using robot feedback or execute mode.

## Connect Quest and calibrate

Start a read-only arm preview after robot preparation:

```bash
./full_teleop.sh --robot-feedback --arms-only --host-ip YOUR_WORKSTATION_LAN_IP
```

Open the **complete** URL shown in the terminal in Quest Browser:

```text
https://YOUR_WORKSTATION_LAN_IP:8012/?ws=wss://YOUR_WORKSTATION_LAN_IP:8012
```

For the certificate generated on your own workstation, resolve the local trust warning on the headset. Select the page's **Pass-through** button and allow hand tracking. Both hands must be visible. Hidden grid/hand graphics are intentional; the terminal's fresh tracking status determines readiness.

The robot must already be in the expected native mode: **FSM 500, mode_machine 6** on the observed configuration. Use the normal Unitree remote procedure; the helper does not change modes. Coordinate exclusive control: the conflict scan blocks known competing controllers but cannot discover every possible DDS publisher.

All controls are typed in the workstation terminal and require Enter:

| Key | Action |
| --- | --- |
| `c` | Capture a comfortable neutral wrist pose against measured robot joint positions |
| `r` | Request one bounded run; requires execute mode, calibration and fresh inputs |
| `p` | Release the arm overlay to the native controller, stop hand targets and clear calibration |
| `q` | Exit and clean up owned processes |

Calibration causes no motion. Head yaw and origin are fixed at calibration so looking around does not steer the arms. After release, a new `c → r` is required. **`p` is a native-controller handover, not an emergency stop or pose hold.**

## Commission arms, then combine fingers

First use the restricted arms-only profile. Keep both hands visible and move one wrist a few centimetres at a time:

```bash
./full_teleop.sh --execute --arms-only --host-ip YOUR_WORKSTATION_LAN_IP \
  --duration 20 --translation-limit .04 --rotation-limit .15
```

After measured feedback and the observer confirm correct arm motion and handover, repeat calibration for the combined test:

```bash
./full_teleop.sh --execute --host-ip YOUR_WORKSTATION_LAN_IP --duration 30
```

`--execute` enables `c → r`; it never auto-arms. The launcher owns a passive DFX service for a combined session. It starts target streams only after explicit arming. The service deadline defaults to 10 minutes; use the [operating guide](docs/OPERATING.md) for longer unarmed preparation, finger ordering, limits, troubleshooting and optional camera setup.

## What is bounded

- Only 14 arm joints through `rt/arm_sdk`, plus DFX fingers. No locomotion, waist trajectory or balance controller is provided.
- First commands require 0.5 seconds of fresh tracking and ready feedback; engagement ramps over two seconds at the measured starting posture.
- Arm targets: ±0.15 rad per joint, 0.15 rad/s slew. Tracking ages: 250 ms hands / 300 ms head. Robot-local feedback, torque-estimate, mode and body-drift checks remain active.
- Any command write attempt activates latched fault handling. Tracking loss, stale input, invalid IK or transport failure releases both command streams; there is no automatic rearm.
- Feedforward model torque is zero. DFX inertia, full-range motion, contact control and freestanding balance are not validated. These software guards are not a certified safety system or a full collision model.

## Development

```bash
./run_python.sh -m unittest discover -s tests -v
```

Tests use synthetic data and mocked transports; camera protocol tests bind only local sockets. Never use a successful unit test or DDS write as proof of physical commissioning. Generated certificates, local configuration, recordings and reports are ignored by Git.

The publication build passed a fresh CPU environment install, all 92 Python tests, synthetic arm IK and finger retargeting, and six serial-emulator scenarios including 100 delayed/fragmented write-read cycles. GitHub Actions runs the lightweight transport, lifecycle, configuration, bootstrap and serial checks; the full numerical smoke check runs locally with the installed environment.

[Architecture](docs/ARCHITECTURE.md) · [Commissioning record](docs/COMMISSIONING.md) · [Third-party notices](THIRD_PARTY_NOTICES.md) · [Apache-2.0](LICENSE)
