# Install and deploy without starting motion

The workstation runs Quest tracking, retargeting and IK. The G1 runs the arm/hand
bridges and the compiled DFX serial driver. Do not copy the workstation's x86-64
Conda environment to the robot's ARM64 computer. Installation and deployment
commands below do not start the driver, open a serial device, initialize DDS,
or command the robot.

## Workstation dependencies

Use Linux x86-64, Python 3 for the bootstrap, Git, and Conda/Miniforge on `PATH`.
From this repository:

```bash
python3 scripts/fetch_upstream.py
./rebuild_environment.sh
./run_python.sh scripts/smoke_test.py
```

The fetcher reads the seven complete commit IDs in `upstream.lock.json`. It
creates detached checkouts under ignored `repos/`, then applies
`dex-conda-pinocchio.patch` and `dfx-serial-transactions.patch`. The DFX patch
already contains the build fix. Do not apply an additional build-only patch.
Running the fetcher again checks the same revisions and exact patched file
contents. Unexpected tracked changes or a different HEAD cause an error; the
fetcher does not reset them. `--check` verifies existing checkouts without
fetching or patching. Reports are written under ignored `.cache/`.

The default environment build solves generic x86-64 Conda packages with Python
3.10.22, Pinocchio 3.1.0, NumPy 1.26.4 and CasADi 3.6.7. It installs CPU PyTorch
2.3.0 and the pip versions constrained by `pip-versions.lock`. The native
transitive Conda package builds can differ from the historical environment.
`--exact-lock` replays `conda-linux-64.explicit.txt`, which includes a Zen4
microarchitecture package and is only appropriate for a compatible CPU.

The rebuild command refuses an existing destination. To choose another new
prefix, use `./rebuild_environment.sh --prefix /absolute/path`, then export
`TELEOP_ENV=/absolute/path` when using `run_python.sh`. Existing Conda and pip
caches can be reused through their ordinary environment variables.

## Local configuration and Ethernet

```bash
cp config/example.env config/local.env
```

Edit that file as plain `KEY=VALUE` data. It is ignored by Git and is never
executed as a shell script. Set `HOST_LAN_IP` to the Ethernet workstation address
reachable from Quest. Set `G1_SSH_HOST` to an SSH alias or `user@address` that
reaches the robot. SSH reachability, not workstation-to-robot DDS discovery, is
required. An Ethernet workstation and a Wi-Fi Quest must still have a routable
connection without client isolation.

Set `G1_REMOTE_ROOT` to a dedicated absolute robot directory and
`G1_ROBOT_PYTHON` to the robot's Python executable. Use the robot's actual
internal native DDS interface for `G1_ARM_INTERFACE`; the example `enP8p1s0`
is not the workstation's Ethernet adapter. `G1_HAND_INTERFACE=lo` keeps hand
DDS on robot loopback. Choose a unique `G1_DDS_NAMESPACE` consisting of letters,
digits and underscores. Runtime launchers forward these values explicitly to
the remote helpers. No shell `source` step is required.

Generate local HTTPS certificates for the workstation address using
`scripts/make_certs.py` as described in the main README. Private keys and
machine settings are never included in deployment archives.

## Robot prerequisites

The selected robot Python must be version 3.10 or newer and have NumPy plus the
SDK2 Python package. An existing compatible robot environment can be used.
For a new environment, create it separately with a robot-native Python 3.10+
installation and set `G1_ROBOT_PYTHON` to its executable. The optional
`--install-python-sdk` deployment flag installs this repository's pinned SDK2
Python source and its declared dependencies into that explicitly selected
interpreter. It does not install the workstation's full IK/XR environment.

Building the DFX driver also requires the robot-compatible **Unitree SDK2 C++**
installation and its matching CycloneDDS C/C++ libraries. SDK2 Python alone is
insufficient. The expected default prefix is `/usr/local`, with:

- `include/unitree/robot/channel/channel_publisher.hpp`
- `include/ddscxx/dds/dds.hpp` and the matching C/iceoryx headers
- linkable `unitree_sdk2`, `ddsc`, and `ddscxx` libraries

Install the ordinary build dependencies on the robot if absent:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git python3 \
  libboost-program-options-dev libspdlog-dev libeigen3-dev
```

The pinned DFX CMake file links Boost's static `libboost_program_options.a` and
uses the SDK2/CycloneDDS headers and libraries above. Keep their versions
compatible. The original robot's SDK2 C++ installation was pre-existing; its
source revision is not captured by this project's upstream lock. This tooling
therefore does **not** replace or silently install a different C++ SDK. If the
headers/libraries are missing, provision the robot-approved
[Unitree SDK2 C++ distribution](https://github.com/unitreerobotics/unitree_sdk2)
and record that revision separately before continuing.

If pip reports that it cannot locate CycloneDDS when installing SDK2 Python,
follow the pinned SDK2 Python README's `CYCLONEDDS_HOME` instructions for a
matching local CycloneDDS installation. The deployment tool does not build or
replace the robot's system DDS libraries.

## Review and deploy

First prepare an archive and plan locally:

```bash
python3 scripts/deploy_robot.py
```

This makes no SSH connection. Inspect `.cache/deploy/deploy-plan.json` and the
archive beside it. It contains the robot helper scripts, `teleop_config.py`,
licenses/notices, pinned SDK2 Python source and patched DFX source. It excludes
local configuration, certificates, reports, `.git`, environments and existing
build outputs.

To upload and compile the driver:

```bash
python3 scripts/deploy_robot.py --apply --build-driver
```

If SDK2 Python must also be installed in the selected robot environment:

```bash
python3 scripts/deploy_robot.py --apply --install-python-sdk --build-driver
```

`--host`, `--remote-root` and `--remote-python` override the local configuration
for this deployment only. `--sdk-prefix` selects an existing SDK2 C++ prefix;
`--jobs` controls compilation parallelism. All flags can be included in the
default local plan before adding `--apply`. Uploads replace the named project
files in the configured destination; they do not remove unrelated files. The
tool checks robot Python imports without constructing DDS participants.

For manual compilation on the robot after upload:

```bash
bash /home/unitree/quest-g1-teleop/build_robot_driver.sh \
  --source /home/unitree/quest-g1-teleop/DFX_inspire_service
```

Adjust the paths to your configuration. This builds **only** `inspire_g1`, and
does not invoke it or `hand_example`. The driver retains upstream hard-coded
G1 serial assignments: right hand `/dev/ttyUSB1`, left hand `/dev/ttyUSB2`,
115200 baud, ID 1. Verify those device identities and serial permissions for
your robot before commissioning. A different hardware mapping requires an
explicit source change and a reviewed build; this installer does not change
IDs, force/speed settings, USB latency timers, or device permissions.

Deployment success establishes installed files and compilation, not hardware
readiness or motion safety. Continue with [commissioning](COMMISSIONING.md):
clock/ownership checks, read-only feedback, a supported arms-only run, and then
the separately observed combined run. Starting services remains a separate,
explicit launcher action.
