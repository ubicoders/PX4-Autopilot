# PX4-Autopilot (ubicoders fork) — build, IntelliSense, and branch contents

Fork: **ubicoders/PX4-Autopilot**, branch **`ubicoders_v1.17.0`** (based on the `v1.17.0` tag).
Boards in use: **FMUv5** (`px4_fmu-v5_default`) and **FMUv6X** (`px4_fmu-v6x_default`).
Target names follow `px4_<board>_<config>`; `make list_config_targets` lists them all.

## Project context

Autopilot firmware for the **ips_onboard** integration (companion: [../ips_onboard_px4](../ips_onboard_px4),
plan: [../tasks/phase1.md](../tasks/phase1.md)). Phase 1 is **params-only**: EKF2 external-vision
fusion plus offboard control, driven by stock MAVLink messages, with **no firmware source changes**.
Do not edit firmware for Phase 1.

Only touch source for the deferred **A2** option (a custom receiver shim that republishes a bespoke
message into `vehicle_visual_odometry` and `trajectory_setpoint`). The relevant subsystems are
`mavlink_receiver` (external-vision and offboard ingestion), `ekf2` EV fusion, and `mc_pos_control`.

## Build (docker compose)

The image `px4-autopilot-dev:v1.17.0` is built from [Dockerfile](Dockerfile) on top of
`px4io/px4-dev:v1.17.0`. The source tree is **bind-mounted live** by
[docker-compose.yml](docker-compose.yml); it is not copied into the image. Edit on the host,
build in the container.

```bash
docker compose build                                        # one-time image build
docker compose run --rm px4 make px4_fmu-v6x_default        # compile
docker compose run --rm px4 make px4_fmu-v6x_default upload # compile and flash over USB
docker compose run --rm px4 make px4_fmu-v5_default         # the other board
docker compose run --rm px4 bash                            # interactive shell
docker compose run --rm px4 make clean                      # clean one target
docker compose run --rm px4 make distclean                  # remove build/ and reset submodules
```

Output: `build/<target>/<target>.px4`. `upload` works because compose runs the container
**privileged** with the host `/dev/` mounted.

### The mount mirrors the host path (important)

The compose file mounts the tree at **`${PWD}`**, the same absolute path it has on the host, and
sets `working_dir` to match. Docker creates that path inside the container at start; nothing has to
exist in the image. Consequences:

- Every absolute path the build writes, such as `build/<target>/compile_commands.json` and the
  NuttX symlinks (`platforms/nuttx/NuttX/nuttx/include/arch`, `arch/arm/include/board`, and so on),
  is valid on the host. Host-side IntelliSense needs no path rewriting.
- **Always run `docker compose` from the repo root.** `${PWD}` is the shell's current directory.
- On another machine, clone anywhere and it works unchanged. A fresh clone builds cleanly.
- **After moving an existing checkout** (or after changing the mount path), the old absolute path
  survives in three places: the CMake cache in `build/<target>`, the NuttX symlinks, and the
  generated files NuttX writes *into its own source tree* (`apps/Kconfig`, `.config`, object files).
  Symptoms: "CMakeCache.txt directory is different" or `Kconfig: '/old/path/...' not found`.
  Fix, from the repo root:

  ```bash
  docker compose run --rm px4 bash -c \
    'git -C platforms/nuttx/NuttX/nuttx clean -fdX -q && git -C platforms/nuttx/NuttX/apps clean -fdX -q'
  rm -rf build/<target>            # sudo on a Linux-native filesystem
  docker compose run --rm px4 make <target>
  ```

  (`make distclean` does the same but also resets every submodule; `-X` removes only ignored
  files, so anything untracked but not ignored is kept.)
- Windows hosts present paths differently; use the dev container there (see below).

## IntelliSense (VS Code)

Two prerequisites on the host, once per machine:

```bash
code --install-extension ms-vscode.cpptools
sudo apt install gcc-arm-none-eabi     # provides /usr/bin/arm-none-eabi-g++ for built-in headers
```

Then build the target once and reload the window. Header-only files take their flags from a
source file that includes them, which is normal for the C/C++ extension.

How the configuration is produced (do not hand-edit `.vscode/c_cpp_properties.json`; CMake
overwrites it on every configure, which is why it is git-ignored):

1. [platforms/common/c_cpp_properties.json.nuttx.in](platforms/common/c_cpp_properties.json.nuttx.in)
   (and the `.in` for POSIX/SITL) are the templates. This fork changed them to emit
   `compileCommands` pointing at the configured target's `compile_commands.json` and
   `compilerPath` from CMake, instead of the upstream `configurationProvider: ms-vscode.cmake-tools`,
   which is never configured for Docker builds and would shadow the compile database. The
   generated file therefore always tracks the **last configured target**.
2. [.vscode/settings.json](.vscode/settings.json) (tracked) carries the same two values as
   `C_Cpp.default.*`. The extension uses these for any property the generated file omits, so
   IntelliSense still works if the templates are ever reverted by an upstream merge. Update the
   target name there if the default board changes. `cmake.configureOnOpen` is off so CMake Tools
   does not try to configure on the host.

Errors such as `identifier "MODULE_NAME" is undefined` or `_param_... is not a nonstatic data
member` mean the compile database is not being applied to that file: the ARM compiler named in
the database is missing on the host, the window was not reloaded after the build, or the
generated file was regenerated from an old template.

"cannot open source file arch/types.h" means the NuttX `include/arch` symlink is stale or missing:
rebuild the target, which recreates it.

Alternative: [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) opens the folder
inside the same compose service ("Dev Containers: Reopen in Container"). It installs the C/C++
extension in the container and uses the container's toolchain, so no host toolchain is needed.

## Gotcha: root-owned build artifacts

The container runs as **root**. On a Linux-native filesystem, files it writes (`build/`, parts of
`.git/modules/`) become root-owned and host-side `rm -rf build` or `git submodule update` fail.
Run clean, submodule, and build steps inside the container, or `sudo` them on the host. (On an
NTFS/exFAT mount, as on the original development machine, ownership is ignored and this does not
apply.)

## Submodules / version pinning

This branch must stay on the `v1.17.0` tag's submodules (including NuttX). After any version or
branch switch, re-pin inside the container:

```bash
docker compose run --rm px4 git submodule update --force --recursive
```

## What this branch adds over upstream v1.17.0

No firmware source is changed. The branch adds tooling and documentation only:

| Path | Purpose |
|---|---|
| [Dockerfile](Dockerfile), [docker-compose.yml](docker-compose.yml), `.dockerignore` | Containerised build described above. |
| [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) | Optional VS Code dev container on the compose service. |
| [README.md](README.md) | Build quick reference, then the **flight-mode switch** (RC ch7 toggles Stabilized and Offboard) and the **offboard parameters** (`COM_*`) for the IPS integration, with a copy-paste nsh block. |
| [ips_phase1/ips_phase1.params](ips_phase1/ips_phase1.params) | QGC-loadable EKF2 params: `EKF2_EV_CTRL=7` (EV pos+vel), `EKF2_HGT_REF=3` (vision height), `EKF2_EV_NOISE_MD=1`, EV noise and delay to tune, `EKF2_GPS_CTRL=0` (indoor). |
| [ips_phase1/apply_params.md](ips_phase1/apply_params.md) | How to apply the params (QGC or nsh), which need a reboot, and how to verify EV fusion. |
| [hflow/hflow.params](hflow/hflow.params), [hflow/README.md](hflow/README.md) | Holybro H-Flow (DroneCAN flow + rangefinder): `UAVCAN_ENABLE=2`, `UAVCAN_SUB_FLOW/RNG=1`, EKF2 flow+range fusion with `EKF2_HGT_REF=2`, plus apply and verify steps. |
| [notes/hflow_dataflow.md](notes/hflow_dataflow.md) | Mermaid diagram of the H-Flow build: uavcan → sensors → ekf2 → flight_mode_manager / mc_pos_control → mc_att_control → mc_rate_control → control_allocator, plus how `mc_pos_control` closes P-position / PID-velocity on `vehicle_local_position` and the uORB names. |
| [commander.md](commander.md) | Architecture notes on the `commander` module: its collaborators, uORB I/O, `UserModeIntention`, and the `run()` loop. |
| [offboard.md](offboard.md) | How `commander` enters and holds OFFBOARD: the RC request path and the `offboard_control_mode` heartbeat gate, with a line-number index. |
| [odom.md](odom.md) | The ODOMETRY data path from the companion into EKF2 and back out: inbound state, setpoint, control cascade, readback stream. |
| `gitpush.bash` | Convenience script that commits everything with message "d" and pushes. |
| `src/drivers/uavcan/libuavcan` | Submodule pointer present on this branch (no local changes). |

Reading order for the IPS work: `ips_phase1/README.md`, `README.md` sections 1 to 5, then `odom.md`
for the data path and `offboard.md` and `commander.md` when debugging mode entry.
