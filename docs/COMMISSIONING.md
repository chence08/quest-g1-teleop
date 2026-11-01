# Commissioning record

**Status at 1 November 2026:** a bounded fingers-only test is the only accepted physical teleoperation result. Arm following, handover and combined operation remain unvalidated; the full system is not commissioned.

This record summarizes the original installation. See [source pins](../upstream.lock.json) and the new Ethernet-workstation [architecture](ARCHITECTURE.md).

## 29 October: build and fingers

The workstation build passed dependency/import, synthetic IK, retargeting and HTTPS checks. A DFX driver patch fixed delayed/fragmented USB reply handling, passed 100 simulated write/read cycles with latency emulation, and compiled on the robot.

A supported 30-second fingers-only test recorded **583 robot DDS publications** and no sampled serial-loss-counter growth. The operator confirmed correct following by both hands. Target excursion was ±0.15 normalized units; measured displacement reached about 0.164, so the target bound is not guaranteed physical travel. Full finger travel, force, arms and balance were not validated. An earlier unsuccessful trial was not accepted.

RGB cameras were occupied by existing services, which were preserved. The operator selected Quest Pass-through; robot video remained unvalidated.

## 30 October: read-only upper-body preparation

Stationary native feedback checks passed. Independent movement during an unarmed preview revealed another controller. Execution checks were extended to detect known competing controllers and reject incomplete checks. They cannot prove exclusive control or prevent a later publisher. No arm result was accepted.

## 1 November: interrupted arm attempts

The operator changed native FSM 0 to 500 using the physical remote; read-only feedback confirmed the expected mode-6 configuration. Software did not change modes.

After certificate/page-expiration troubleshooting, valid tracking and calibration worked. The actual button is **Pass-through**; retain the URL's `?ws=` query. A longer unarmed wait retained the **20-second** execution bound.

| Attempt | Recorded result |
| --- | --- |
| Two initial execute starts | Stopped with `tracking_invalid_or_stale`; zero SSH arm targets and zero robot DDS publications |
| Final patched attempt | Four calibrations, one execute session, **four SSH arm-target submissions**, **zero recorded robot DDS publications**, and overlay weight zero throughout the recorded feedback |

The final attempt stopped at **0.267759-second** hand-event ages, beyond the unchanged **0.25-second** limit. Head/wrist SE(3) transforms were valid: events were stale, not malformed. Four SSH submissions do not establish robot actuation. Combined operation was not started that day.

The sender patch added 0.5-second pre-write qualification, latched stopping after any write attempt—including backpressure/partial failure—and detailed diagnostics. **24 sender tests** and independent review passed; numerical gates were unchanged.

Temporarily disabling Wi-Fi power saving reduced median tracking age from about 120 to 32 ms, but ping still lost 1/30 packets and peaked near 238 ms. Wi-Fi was not proven the sole cause; Quest/browser pauses remained possible. Moving closer to the access point or wiring that workstation was unavailable, so attempts ended. Original power saving was restored; task-owned processes and the preview listener were verified stopped.

## Remaining acceptance steps

1. Demonstrate stable Quest tracking delivery on the new workstation's Ethernet/LAN arrangement without relaxing freshness checks.
2. Reconfirm physical support, stop access, native state and exclusive controller ownership; calibrate from current robot feedback.
3. Complete and accept a bounded arms-only test, including direction, tracking loss and native handover behavior.
4. Only then test combined arms and DFX fingers. Separately validate larger travel, payload/force behavior, collision clearance and any camera view.

Freestanding/full-range operation remains unvalidated. Hidden Pass-through overlays alone do not prove disconnection; inspect tracking validity and event ages.
