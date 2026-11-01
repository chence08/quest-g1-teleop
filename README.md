# Quest 3 → G1 arms + Inspire DFX

[![Offline checks](https://github.com/chence08/quest-g1-teleop/actions/workflows/check.yml/badge.svg)](https://github.com/chence08/quest-g1-teleop/actions/workflows/check.yml)

A reproducible upper-body teleoperation integration built on **Unitree xr_teleoperate**. Quest camera-based hand tracking drives calibrated wrist targets and DFX finger retargeting. Arm and hand commands travel over SSH; DDS stays on the robot.

**Status:** this build follows upstream xr_teleoperate behaviour for real, full-range motion: tracking loss, IK failure and late packets hold the last target instead of stopping. **It has no automatic safety stops; keep a person on the physical Unitree remote.** Bounded DFX-only motion was physically confirmed on the earlier, restricted build; full-range arm motion with this build has not yet been run on hardware. See [commissioning results](docs/COMMISSIONING.md).

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

## Run

Put the robot in the native mode you want (for example standing, FSM 500) with the Unitree remote; the software does not change modes. Then:

```bash
./full_teleop.sh --execute --host-ip YOUR_WORKSTATION_LAN_IP
```

Add `--arms-only` to leave the fingers out, or drop `--execute` for a hardware-free preview. Open the **complete** URL shown in the terminal in Quest Browser:

```text
https://YOUR_WORKSTATION_LAN_IP:8012/?ws=wss://YOUR_WORKSTATION_LAN_IP:8012
```

Resolve the local certificate warning, select the page's **Pass-through** button and allow hand tracking. All controls are typed in the workstation terminal and require Enter:

| Key | Action |
| --- | --- |
| `r` | Start a run. Calibrates first if needed: your current wrist pose maps to the measured robot arm pose |
| `c` | Recalibrate at the current pose, also while running |
| `p` | Pause: hand the arms back to the native controller (they move to its pose); `r` resumes with the same calibration |
| `q` | Exit and clean up owned processes |

Head yaw is frozen at calibration so looking around does not steer the arms. A run lasts until `p` or `q` (or `--duration SECONDS`).

## Motion behaviour

- 14 arm joints through `rt/arm_sdk`, plus DFX fingers. No locomotion, waist or balance commands.
- 1:1 wrist translation within 1 m of the calibration pose; targets are clipped only to URDF joint limits.
- The robot bridge ramps the overlay weight in over 1 s, then raises the joint velocity limit from 1 to 10 rad/s over 3 s (upstream: 20–30 rad/s). Gains match upstream: kp 80 / kd 3, wrists kp 40 / kd 1.5. IK model gravity torque is sent as feedforward (`--zero-feedforward` disables it).
- Tracking dropouts (hand packets older than 0.5 s), IK failures, SSH backpressure and malformed packets hold the last target. Mode, FSM, torque, drift and clock checks were removed.
- If the DFX service or hand bridge fails, arms keep running and the hand bridge restarts. A detected competing controller produces a warning, not a refusal.

See the [operating guide](docs/OPERATING.md) for tuning flags and troubleshooting.

## Development

```bash
./run_python.sh -m unittest discover -s tests -v
```

Tests use synthetic data and mocked transports; camera protocol tests bind only local sockets. A passing unit test does not prove physical behaviour. Generated certificates, local configuration, recordings and reports are ignored by Git.

GitHub Actions runs the lightweight transport, lifecycle, configuration, bootstrap and serial checks; the full numerical smoke check runs locally with the installed environment.

[Architecture](docs/ARCHITECTURE.md) · [Commissioning record](docs/COMMISSIONING.md) · [Third-party notices](THIRD_PARTY_NOTICES.md) · [Apache-2.0](LICENSE)
