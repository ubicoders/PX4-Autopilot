# PX4-Autopilot — Build & Flash (Docker)

Fork: **ubicoders/PX4-Autopilot**, branch **`ubicoders_v1.17.0`** (based on the `v1.17.0` tag).
Target board: **FMUv5** → make target **`px4_fmu-v5_default`** (note the underscores, not spaces).

The source tree is **bind-mounted live** into the container by [docker-compose.yml](docker-compose.yml) — it is *not* copied into the image. Edit on the host, build in the container. The image (`px4-autopilot-dev:v1.17.0`) is built from [Dockerfile](Dockerfile) on top of `px4io/px4-dev:v1.17.0`.

## Project context

Autopilot firmware for the **ips_onboard** integration (companion: [../ips_onboard_px4](../ips_onboard_px4), plan: [../tasks/phase1.md](../tasks/phase1.md)). Phase 1 uses this **params-only** — EKF2 external-vision fusion + offboard, driven by **stock MAVLink messages**, with **no source changes**. Don't edit firmware for Phase 1.

Only touch source for the deferred **A2** option (a custom receiver shim that republishes a bespoke message into `vehicle_visual_odometry` + `trajectory_setpoint`); the relevant subsystems then are `mavlink_receiver` (external-vision + offboard ingestion), `ekf2` EV fusion, and `mc_pos_control`.

## Quick reference (docker compose — preferred)

```bash
# One-time: build the dev image
docker compose build

# Compile firmware → build/px4_fmu-v5_default/px4_fmu-v5_default.px4
docker compose run --rm px4 make px4_fmu-v5_default

# Compile AND flash an attached board over USB
docker compose run --rm px4 make px4_fmu-v5_default upload

# Open an interactive shell in the container
docker compose run --rm px4 bash

# Clean (must run in-container — see gotcha below)
docker compose run --rm px4 make clean        # clean one target
docker compose run --rm px4 make distclean     # nuke build/ + reset submodules
```

`upload` works because compose runs the container **`privileged`** with the host `/dev/` mounted, so the board's USB/serial port is reachable for flashing.

## Equivalent raw `docker run`

```bash
docker run --rm -it \
  --privileged \
  -v /dev:/dev \
  -v "$PWD":/workspace/PX4-Autopilot \
  -w /workspace/PX4-Autopilot \
  px4-autopilot-dev:v1.17.0 \
  make px4_fmu-v5_default upload
```

Without the `-v "$PWD":...` mount the image has no source to build, and without `--privileged -v /dev:/dev` the `upload` step can't reach the board.

## Other build targets

```bash
make px4_fmu-v5_default        # FMUv5 (this board)
make px4_sitl                  # software-in-the-loop (no hardware)
make list_config_targets       # show all board/config targets
```

## Gotcha: root-owned build artifacts

The container runs as **root**, so files it writes (`build/`, parts of `.git/modules/`) become root-owned on the host. A host-side `rm -rf build` or `git submodule update` then fails with permission errors.

**Always run clean / submodule / build steps inside the container** (i.e. via `docker compose run`), or `sudo` them on the host. The current build output `build/px4_fmu-v5_default/px4_fmu-v5_default.px4` (~1.8 MB) is root-owned for this reason.

## Submodules / version pinning

This branch must stay on the `v1.17.0` tag's submodules (incl. NuttX). After any version/branch switch, re-pin inside the container:

```bash
docker compose run --rm px4 git submodule update --force --recursive
```
