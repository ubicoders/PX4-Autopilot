# ips_phase1 — PX4 config for IPS → PX4 offboard control (Phase 1)

This folder holds the **PX4-side configuration artifacts** for Phase 1 of the
IPS → PX4 offboard integration. It is **params-only**: there are **no firmware
source changes** and no docker build is needed — a human applies these to a real
flashed FMUv5 board.

Contents:
- [`ips_phase1.params`](ips_phase1.params) — QGroundControl-importable parameter
  file (EKF2 external-vision aiding + GNSS disabled for indoor flight).
- [`apply_params.md`](apply_params.md) — how to apply (QGC and nsh) and the §5.4
  fusion verification checklist (Check B).

Order of operations:
1. Flash **stock** `px4_fmu-v5_default` firmware (no source changes).
2. Load `ips_phase1.params` (QGC or nsh — see `apply_params.md`).
3. **Reboot** the vehicle (some params are reboot-required).
4. Run the companion dummy sender: `cargo run --bin control_test -- --dummy`.
5. Run **Check B** in the nsh shell to confirm EV fusion (see `apply_params.md`).

Full plan and architecture: [../../tasks/phase1.md](../../tasks/phase1.md).
