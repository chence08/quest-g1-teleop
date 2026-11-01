# Third-party components

Dependencies are fetched at commits recorded in [upstream.lock.json](upstream.lock.json); their source, assets, binaries, and environments are not vendored. Each dependency retains its own license. The Apache-2.0 license at this repository root covers the integration helpers; it does not relicense dependencies.

| Component | Source | License / notice |
| --- | --- | --- |
| xr_teleoperate | [Unitree Robotics](https://github.com/unitreerobotics/xr_teleoperate) | Apache-2.0; [upstream notice](third_party/xr_teleoperate.NOTICE) |
| Unitree SDK2 Python | [Unitree Robotics](https://github.com/unitreerobotics/unitree_sdk2_python) | BSD-3-Clause; [license](third_party/unitree_sdk2_python.LICENSE) |
| DFX Inspire service | [Unitree Robotics](https://github.com/unitreerobotics/DFX_inspire_service) | BSD-3-Clause; [license](third_party/DFX_inspire_service.LICENSE.txt) |
| dex-retargeting fork | [silencht](https://github.com/silencht/dex-retargeting) | MIT; [license](third_party/dex-retargeting.LICENSE); additional legacy notices remain in its checkout |
| TeleVuer | [Unitree Robotics](https://github.com/unitreerobotics/televuer) | See pinned checkout LICENSE |
| TeleImager | [Unitree Robotics](https://github.com/unitreerobotics/teleimager) | See pinned checkout LICENSE |
| Inspire SDK dependency | [NaCl-1374](https://github.com/NaCl-1374/inspire_hand_ws) | See upstream distribution; not vendored here |

The DFX patch changes serial transaction handling, response validation, startup state and build linkage. The dex-retargeting patch changes dependency metadata to use Conda's Pinocchio. Patch context retains upstream attribution through the notices above. Applying a patch does not change the upstream license.

Python and native dependencies, including Vuer, Pinocchio, CasADi, NumPy, PyTorch and CycloneDDS, retain the licenses supplied with their distributions.
