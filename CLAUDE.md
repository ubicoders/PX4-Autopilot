# PX4-Autopilot (ubicoders fork) — build, IntelliSense, and branch contents

Fork: **ubicoders/PX4-Autopilot**, branch **`ubicoders_v1.17.0`** (based on the `v1.17.0` tag).
Boards in use: **FMUv5** (`px4_fmu-v5_default`) and **FMUv6X** (`px4_fmu-v6x_default`).
Target names follow `px4_<board>_<config>`; `make list_config_targets` lists them all.

## Project context

Autopilot firmware for the **ips_onboard** integration (companion: [../ips_onboard_px4](../ips_onboard_px4),
design: [../ips_onboard_px4/diagram.md](../ips_onboard_px4/diagram.md)). The companion sends the IPS
cube pose as `px4_msgs/VehicleOdometry` over **uXRCE-DDS on USB** (`/fmu/in/vehicle_visual_odometry`),
and EKF2 fuses it as external vision. The firmware side is small and deliberately so: **one module**
(`ubicoders_ips_debug`), **one message** (`UbicodersIpsDebug.msg`), **one DDS publication** (`/fmu/out/ubicoders_ips_debug`) and
**one boot profile** (`rc.ubicoders_ips`, sourced by one line in `rcS`) that sets every parameter per
fusion case from four switches (`IPS`, `HFLOW`, `FUSE_IPS`, `FUSE_HFLOW`). **EKF2 is untouched**:
fusion is chosen by parameters only. Keep it that way; change behaviour in the boot profile, not in `ekf2`,
`uxrce_dds_client` or the board folders. Reference: [notes/ubicoders_ips/README.md](notes/ubicoders_ips/README.md);
bench procedure: [notes/ubicoders_ips/BENCH.md](notes/ubicoders_ips/BENCH.md). The FMU on the drone still
runs an older firmware until `./flash.sh` (in the companion repo) has been run.

The earlier MAVLink, params-only path (`ips_phase1/`, the `README.md` offboard sections) is kept for
reference; the boot profile now sets those parameters itself. The relevant subsystems are
`uxrce_dds_client` (ingestion), `ekf2` EV, optical-flow and range fusion, and `mc_pos_control`.

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

Tooling and documentation, plus (for the IPS work) one small module, one message, one DDS
publication and a boot profile sourced by one line in `rcS`; in the board folders only the
`.px4board` configs change, to compile the module in. EKF2 is untouched:

| Path | Purpose |
|---|---|
| [Dockerfile](Dockerfile), [docker-compose.yml](docker-compose.yml), `.dockerignore` | Containerised build described above. |
| [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) | Optional VS Code dev container on the compose service. |
| [README.md](README.md) | Build quick reference, then the **flight-mode switch** (RC ch7 toggles Stabilized and Offboard) and the **offboard parameters** (`COM_*`) for the IPS integration, with a copy-paste nsh block. |
| [ips_phase1/ips_phase1.params](ips_phase1/ips_phase1.params) | Earlier, params-only path, superseded by `rc.ubicoders_ips` (kept for reference). QGC-loadable EKF2 params: `EKF2_EV_CTRL=7` (EV pos+vel), `EKF2_HGT_REF=3` (vision height), `EKF2_EV_NOISE_MD=1`, EV noise and delay to tune, `EKF2_GPS_CTRL=0` (indoor). |
| [ips_phase1/apply_params.md](ips_phase1/apply_params.md) | How to apply the params (QGC or nsh), which need a reboot, and how to verify EV fusion. |
| [notes/hflow/hflow.params](notes/hflow/hflow.params), [notes/hflow/README.md](notes/hflow/README.md) | Holybro H-Flow (DroneCAN flow + rangefinder): `UAVCAN_ENABLE=2`, `UAVCAN_SUB_FLOW/RNG=1`, EKF2 flow+range fusion with `EKF2_HGT_REF=2`, plus apply and verify steps. |
| [ROMFS/px4fmu_common/init.d/rc.ubicoders_ips](ROMFS/px4fmu_common/init.d/rc.ubicoders_ips) | Boot profile for the IPS work, four switches: `IPS` (0 USB stays MAVLink, nothing starts; 1 `SYS_USB_AUTO=0`, starts `uxrce_dds_client` on USB and `ubicoders_ips_debug`), `HFLOW` (0 CAN off; 1 H-Flow driver on CAN1, raw flow + range), `FUSE_IPS` (1 EKF2 fuses the IPS) and `FUSE_HFLOW` (1 EKF2 fuses flow + range). A sensor that is off cannot be fused: the script zeroes `FUSE_IPS` when `IPS` is 0 and `FUSE_HFLOW` when `HFLOW` is 0; the effective values are echoed at boot. IPS fused: EV pos + yaw, vision height, mag off, lever arm, Offboard on RC ch7; H-Flow fused: flow + range, range height unless the IPS is fused. All forced with `param set`. Sourced by one line in `rcS`, right after the board defaults block. |
| [src/modules/ubicoders_ips_debug](src/modules/ubicoders_ips_debug), [msg/UbicodersIpsDebug.msg](msg/UbicodersIpsDebug.msg) | On each `vehicle_visual_odometry` sample, and at 10 Hz while none arrives, publishes one five-block `ubicoders_ips_debug`: (1) the IPS pose as received, (2) raw H-Flow (`vehicle_optical_flow`, `distance_sensor`), `ekf2_case` + `cs_*` flags from `estimator_status_flags`, and `vehicle_local_position` in the block of the live case: (3) H-Flow only, (4) IPS only, (5) IPS + H-Flow (others NaN / false); `ekf2_case` 0 none, 1 IPS, 2 H-Flow, 3 both. Inputs older than 1 s read as missing. Streamed out as `/fmu/out/ubicoders_ips_debug` (entry in `src/modules/uxrce_dds_client/dds_topics.yaml`). Enabled in the fmu-v5 and fmu-v6x board files. |
| [notes/ubicoders_ips/README.md](notes/ubicoders_ips/README.md) | The profile's switches and truth table, the four fusion cases, parameters per case, frames, lever arm and delay, how to read `ubicoders_ips_debug`, flashing with the companion's `flash.sh` (no QGC, no SD card) and ROS 2 verification. |
| [notes/ubicoders_ips/BENCH.md](notes/ubicoders_ips/BENCH.md) | Bench procedure, one section per case (0 to 3) with switch values, the `ubicoders_ips_debug` fields to check, the hand moves and the expected numbers; lever-arm and delay measurement; go/no-go list before the first hover. |
| [notes/hflow/hflow_dataflow.md](notes/hflow/hflow_dataflow.md) | Mermaid diagram of the H-Flow build: uavcan → sensors → ekf2 → flight_mode_manager / mc_pos_control → mc_att_control → mc_rate_control → control_allocator, plus how `mc_pos_control` closes P-position / PID-velocity on `vehicle_local_position` and the uORB names. |
| [notes/commander.md](notes/commander.md) | Architecture notes on the `commander` module: its collaborators, uORB I/O, `UserModeIntention`, and the `run()` loop. |
| [notes/offboard.md](notes/offboard.md) | How `commander` enters and holds OFFBOARD: the RC request path and the `offboard_control_mode` heartbeat gate, with a line-number index. |
| [notes/odom.md](notes/odom.md) | The ODOMETRY data path from the companion into EKF2 and back out: inbound state, setpoint, control cascade, readback stream. |
| `gitpush.bash` | Convenience script that commits everything with message "d" and pushes. |
| `src/drivers/uavcan/libuavcan` | Submodule pointer present on this branch (no local changes). |

Reading order for the IPS work: [../ips_onboard_px4/diagram.md](../ips_onboard_px4/diagram.md) (the
whole design), [notes/ubicoders_ips/README.md](notes/ubicoders_ips/README.md), then
[notes/ubicoders_ips/BENCH.md](notes/ubicoders_ips/BENCH.md); `README.md` sections 1 to 5 for the
flight-mode switch and offboard parameters, `notes/offboard.md` and `notes/commander.md` when debugging
mode entry. `notes/odom.md` describes the older MAVLink ODOMETRY path.
