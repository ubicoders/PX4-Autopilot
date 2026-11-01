# ubicoders_ips: the IPS + H-Flow fusion-cases boot profile

Everything the FMU needs for the indoor-positioning work is in the firmware
image. There is no parameter to set by hand, no SD card file, no console step:
flash and go.

> **Status.** The FMU on the drone still runs an **older firmware**. Until
> `./flash.sh` has been run once from `ips_onboard_px4` (see [Flash](#flash)),
> the board does not behave as described here.

Related documents:

- [BENCH.md](BENCH.md): the step-by-step bench procedure per case, the
  lever-arm and delay measurements, and the go/no-go list before the first
  hover.
- The companion side, in the repo next to this one:
  [`ips_onboard_px4/README.md`](../../../ips_onboard_px4/README.md) (the bridge,
  `ips_to_px4`, the agent, `flash.sh`) and
  [`ips_onboard_px4/diagram.md`](../../../ips_onboard_px4/diagram.md) (the whole
  design for a first-time reader: hardware, data path, frames, cases).

| File | Role |
|---|---|
| [`ROMFS/px4fmu_common/init.d/rc.ubicoders_ips`](../../ROMFS/px4fmu_common/init.d/rc.ubicoders_ips) | The profile: four switches at the top, the fallback, the parameters per case, then (IPS 1) the start of the uXRCE-DDS client on USB and of `ubicoders_ips_debug`. |
| [`ROMFS/px4fmu_common/init.d/rcS`](../../ROMFS/px4fmu_common/init.d/rcS), [`ROMFS/px4fmu_common/init.d/CMakeLists.txt`](../../ROMFS/px4fmu_common/init.d/CMakeLists.txt) | Stock boot script plus one sourcing line, right after the board defaults block (search for `rc.ubicoders_ips`); the script listed in the ROMFS build. |
| [`src/modules/ubicoders_ips_debug`](../../src/modules/ubicoders_ips_debug), [`msg/UbicodersIpsDebug.msg`](../../msg/UbicodersIpsDebug.msg) | The debug module and its five-block message. |
| [`src/modules/uxrce_dds_client/dds_topics.yaml`](../../src/modules/uxrce_dds_client/dds_topics.yaml) | `/fmu/out/ubicoders_ips_debug` publication. |
| [`boards/px4/fmu-v5/default.px4board`](../../boards/px4/fmu-v5/default.px4board), [`boards/px4/fmu-v6x/default.px4board`](../../boards/px4/fmu-v6x/default.px4board) | `CONFIG_MODULES_UBICODERS_IPS_DEBUG=y`. |
| [`BENCH.md`](BENCH.md) | The bench procedure. |

EKF2 is stock: no estimator source is changed, only its parameters.

The boards' boot scripts are untouched; in the board folders only the
`.px4board` build configs change, to compile `ubicoders_ips_debug` in. `rcS` sources the profile right after the
board defaults and before sensors, EKF2 and the CAN driver start, so they read
the values when they start. uORB is already up at that point, which is why the two module starts
can live in the same script (the fmu-v5 board defaults start `rgbled_pwm` and
`safety_button` from the same stage).

## The four switches

Edit the top of `rc.ubicoders_ips`, rebuild, flash. Two switches say what
exists on the vehicle (`IPS`, `HFLOW`), two say what EKF2 may fuse
(`FUSE_IPS`, `FUSE_HFLOW`).

| Switch | Value | Meaning |
|---|---|---|
| `IPS` | `0` | No companion link. `SYS_USB_AUTO` forced back to 2, so USB is MAVLink and QGC works over USB; `uxrce_dds_client` and `ubicoders_ips_debug` do not start. |
| | `1` | `uxrce_dds_client` on USB (`/dev/ttyACM0`) and `ubicoders_ips_debug` start; `SYS_USB_AUTO` forced to 0. |
| `HFLOW` | `0` | CAN driver off (`UAVCAN_ENABLE 0`; set it yourself if other CAN devices are on the bus). |
| | `1` | H-Flow on CAN1: driver on, raw flow and range published (`vehicle_optical_flow`, `distance_sensor`). Fusion is decided by `FUSE_HFLOW`. |
| `FUSE_IPS` | `0` | EKF2 ignores the IPS pose (it is still echoed by `ubicoders_ips_debug`). |
| | `1` | EKF2 fuses the IPS pose as external vision: position, height, yaw; vision height reference. |
| `FUSE_HFLOW` | `0` | EKF2 ignores flow and range. |
| | `1` | EKF2 fuses optical flow and the rangefinder; range height reference unless the IPS is fused too. |

The image as built has all four at `1`.

**Fallback.** A sensor that is off cannot be fused; the script zeroes the
matching FUSE switch:

- `FUSE_IPS` is forced to 0 when `IPS = 0`
- `FUSE_HFLOW` is forced to 0 when `HFLOW = 0`

So `FUSE_HFLOW 1` with `HFLOW 0` quietly becomes "H-Flow not fused". The
effective values are printed on the console at boot, for example:

```
ubicoders_ips: IPS=1 HFLOW=1 -> fuse ips=1 hflow=1
```

Full truth table ("set" = the value written at the top of the script, "fuse" =
the effective value after the fallback, as echoed at boot; "-" = not touched
by the script, keeps the saved value):

| IPS | HFLOW | FUSE_IPS set | FUSE_HFLOW set | fuse ips | fuse hflow | EKF2_HGT_REF | EKF2_MAG_TYPE |
|---|---|---|---|---|---|---|---|
| 1 | 1 | 0 | 0 | 0 | 0 | - | 0 |
| 1 | 1 | 1 | 0 | 1 | 0 | 3 | 5 |
| 1 | 1 | 0 | 1 | 0 | 1 | 2 | 0 |
| 1 | 1 | 1 | 1 | 1 | 1 | 3 | 5 |
| 1 | 0 | 0 | any | 0 | 0 | - | 0 |
| 1 | 0 | 1 | any | 1 | 0 | 3 | 5 |
| 0 | 1 | any | 0 | 0 | 0 | - | 0 |
| 0 | 1 | any | 1 | 0 | 1 | 2 | 0 |
| 0 | 0 | any | any | 0 | 0 | - | 0 |

## The four fusion cases

What `vehicle_local_position` (and so `mc_pos_control`) means in each case:

| Case (`ekf2_case`) | `vehicle_local_position` |
|---|---|
| 0 none | No horizontal aiding: `xy_valid` and `v_xy_valid` false. Height from the barometer in practice: `EKF2_HGT_REF` keeps its saved value (stock default 1, GNSS; 3 or 2 after an earlier case), that source is not fused, and EKF2 falls back to the barometer. Stabilized / Altitude only. |
| 1 IPS only | Position, height and yaw are the IPS frame: x toward IPS forward, y toward IPS right, z down (vision height reference, yaw from the IPS attitude, no magnetometer). The position is the FMU's, the cube position moved by the lever arm. Velocity is EKF2's estimate from IMU + IPS position (EV velocity is not fused yet). Offboard from the companion is possible. |
| 2 H-Flow only | Horizontal velocity from optical flow; horizontal position is dead-reckoned from it (drifts, origin where EKF2 started). Height is the range to the floor (range height reference). Yaw from the magnetometer (automatic), so x is magnetic north, not IPS forward. |
| 3 IPS + H-Flow | As IPS only, with optical flow as an additional velocity aid and the range fused as a secondary height source; the IPS stays the height reference. |

## Parameters per case

All forced with `param set` on every boot.

| Condition | Parameters |
|---|---|
| `IPS 1` | `SYS_USB_AUTO 0` |
| `IPS 0` | `SYS_USB_AUTO 2` (stock) |
| `HFLOW 1` | `UAVCAN_ENABLE 2`, `UAVCAN_SUB_FLOW 1`, `UAVCAN_SUB_RNG 1`, `UAVCAN_RNG_MIN 0.08`, `UAVCAN_RNG_MAX 30`, `SENS_FLOW_ROT 0`, `SENS_FLOW_MINHGT 0.08`, `SENS_FLOW_MAXHGT 25`, `SENS_FLOW_MAXR 7.4` |
| `HFLOW 0` | `UAVCAN_ENABLE 0` |
| `FUSE_IPS 1` | `EKF2_EV_CTRL 11` (bit0 hor. pos, bit1 vert. pos, bit3 yaw; velocity bit2 deliberately off), `EKF2_HGT_REF 3`, `EKF2_EV_NOISE_MD 1`, `EKF2_EVP_NOISE 0.05`, `EKF2_EVA_NOISE 0.05`, `EKF2_EV_DELAY 5` (ms, placeholder: see **Delay** below), `EKF2_GPS_CTRL 0`, `EKF2_MAG_TYPE 5` (None), `EKF2_EV_POS_X 0.0`, `EKF2_EV_POS_Y 0.0`, `EKF2_EV_POS_Z -0.20`; Offboard: `RC_MAP_FLTMODE 7`, `COM_FLTMODE1 8`, `COM_FLTMODE6 7`, `COM_FLTMODE2..5 -1`, `COM_RC_IN_MODE 0`, `COM_OF_LOSS_T 1.0`, `COM_OBL_RC_ACT 2`, `COM_RCL_EXCEPT 4`, `COM_ARM_WO_GPS 1` |
| `FUSE_IPS 0` | `EKF2_EV_CTRL 0`, `EKF2_MAG_TYPE 0` (stock default, automatic) |
| `FUSE_HFLOW 1` | `EKF2_OF_CTRL 1`, `EKF2_RNG_CTRL 1`, `EKF2_RNG_A_HMAX 10`, `EKF2_RNG_QLTY_T 0.2`, `EKF2_GPS_CTRL 0`; `EKF2_HGT_REF 2` only when `FUSE_IPS 0` (the IPS wins the height reference) |
| `FUSE_HFLOW 0` | `EKF2_OF_CTRL 0`, `EKF2_RNG_CTRL 0` |
| neither fused | `EKF2_HGT_REF` not touched |

**Yaw.** With the IPS fused, yaw comes from the IPS attitude and the
magnetometer is off: indoors the field is distorted by motors and steel, and two
yaw sources would fight. The companion must therefore send a valid attitude
quaternion (`ips_q` in `ubicoders_ips_debug` must not be NaN).

**Frames.** The companion sends `pose_frame` NED. Its world is the IPS frame
relabelled to FRD by the bridge (x = IPS board forward, y = board right, z
down), which is NED by convention: "north" is the IPS forward axis. Because
EKF2 takes its yaw from the same IPS, no rotation is needed anywhere. The cube
must be mounted front face to the nose and bottom face down; details in the
companion's
[`PX4_FRAMES.md`](../../../ips_onboard_px4/ros2_ws/src/ips_ros2_bridge/PX4_FRAMES.md).

**Lever arm.** The IPS tracks the cube, not the FMU. `EKF2_EV_POS_X/Y/Z` is the
cube centre relative to the FMU in body FRD (x forward, y right, z down). The
default `-0.20` m on z means the cube sits 20 cm above the FMU: measure it and
adjust ([BENCH.md](BENCH.md#lever-arm-ekf2_ev_pos_xyz)).

**Delay.** `EKF2_EV_DELAY 5` ms is a placeholder. `ips_to_px4` stamps each
sample when it reaches the Jetson, so EKF2_EV_DELAY should be the IPS's latency
before that point; `ips_age_us` is the hop after it (Jetson to PX4), useful to
see the link keeps up but not the same number
([BENCH.md](BENCH.md#delay-ips_age_us-and-ekf2_ev_delay)).

**RC switch.** With the IPS fused, RC channel 7 selects Stabilized (low,
`COM_FLTMODE1 8`) or Offboard (high, `COM_FLTMODE6 7`); the other slots are
unassigned, so Position mode is selected from QGC (BENCH.md, go/no-go item 7).

**Persistence.** `param set` also saves the value. A parameter the current case
does not touch keeps whatever an earlier image or QGC saved: the Offboard
`COM_*` values when the IPS is not fused, and `EKF2_HGT_REF` when nothing is
fused. `SYS_USB_AUTO` is forced in both directions (0 with `IPS 1`, 2 with
`IPS 0`), so switching `IPS` off really gives USB back to QGC.

## Reading ubicoders_ips_debug

One message shows every case side by side:

| Block | Fields | Source |
|---|---|---|
| 1. IPS as received | `ips_timestamp`, `ips_timestamp_sample`, `ips_seq`, `ips_age_us`, `ips_pose_frame`, `ips_position`, `ips_q`, `ips_velocity` | `vehicle_visual_odometry` |
| 2. H-Flow raw | `flow_rate`, `flow_quality`, `range_m`, `range_signal_quality` | `vehicle_optical_flow`, `distance_sensor` |
| case | `ekf2_case`, `cs_opt_flow`, `cs_rng_hgt`, `cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw` | `estimator_status_flags` |
| 3. EKF2, H-Flow only | `ekf2_hflow_position`, `ekf2_hflow_velocity`, `ekf2_hflow_xy_valid`, `ekf2_hflow_v_xy_valid` | `vehicle_local_position` while `ekf2_case == 2` |
| 4. EKF2, IPS only | `ekf2_ips_*` | `vehicle_local_position` while `ekf2_case == 1` |
| 5. EKF2, IPS + H-Flow | `ekf2_ips_hflow_*` | `vehicle_local_position` while `ekf2_case == 3` |

`ekf2_case = (cs_ev_pos ? 1 : 0) + (cs_opt_flow ? 2 : 0)`: bit0 = IPS fused,
bit1 = H-Flow fused, so 0 nothing, 1 IPS only, 2 H-Flow only, 3 both, the same
as `fuse ips` + 2 × `fuse hflow` in the boot echo (constants
`EKF2_CASE_NONE/IPS/HFLOW/IPS_HFLOW`). It is what EKF2 actually fuses, taken
from the live estimator flags, not what the switches asked for, so a mismatch with the boot echo
means EKF2 rejected or has not yet started that source. PX4 runs one estimator,
so only one of blocks 3-5 is live; the other two are NaN / false (all three while
`ekf2_case` is 0).

**When it is published:** on every `vehicle_visual_odometry` sample, and in
addition every 100 ms when no sample has arrived for 100 ms (a 10 Hz fallback),
so with the IPS server off or the cube hidden the H-Flow and EKF2 blocks stay
visible at 10 Hz.

**Staleness (1 s):** block 2 is NaN / 0 when its topic has no sample or the last
one is older than 1 s; `ekf2_case` is 0 with all flags false when
`estimator_status_flags` is missing or older than 1 s; blocks 3-5 are NaN / false
when `vehicle_local_position` is. Block 1 is NaN / 0 only before the first IPS
sample; after that it keeps the last sample and `ips_age_us` keeps growing, which
is how a stopped IPS shows. (The header comment of `UbicodersIpsDebug.msg` puts block 1
under the 1 s rule too; the module code, described here, does not.)

## Flash

On the Jetson, from the companion repo. The companion's container holds
`/dev/ttyACM0` (MicroXRCEAgent), and [`flash.sh`](../../../ips_onboard_px4/flash.sh)
takes care of that:

```bash
cd /home/el/Github/ips_onboard_px4
./flash.sh                # reboot the FMU into its bootloader over ROS 2, stop, flash, start
./flash.sh --no-reboot    # the same without the ROS 2 reboot (FMU without a uXRCE-DDS session)
```

What it does:

1. If the `onboard` container runs and `/fmu/out/*` topics are visible, it
   publishes `VehicleCommand` 246 (`VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN`) with
   `param1` 3 on `/fmu/in/vehicle_command`: reboot into the bootloader. PX4
   refuses it while armed. Otherwise this step is skipped.
2. `docker compose stop`: the agent releases `/dev/ttyACM0`.
3. `docker compose run --rm px4 make px4_fmu-v5_default upload` in this repo.
4. `docker compose up -d` in the companion repo.
5. It prints the command that echoes `/fmu/out/ubicoders_ips_debug`.

If the uploader keeps printing `attempting reboot`, it cannot reach the
bootloader (firmware with `IPS 1` has no MAVLink on USB to answer the
uploader's own reboot request): unplug the FMU's USB cable and plug it back in,
and it catches the bootloader during power-up. A board with stock firmware
(MAVLink on USB) is rebooted by the uploader itself.

## Verify, from ROS 2 on the companion

The companion's `px4_msgs` must carry the same `UbicodersIpsDebug.msg` as this firmware
(copy it from `msg/UbicodersIpsDebug.msg`) and be rebuilt, or the topic will not
deserialize. The companion's copy matches this build; an older firmware's
`ubicoders_ips_debug` may not.

```bash
cd /home/el/Github/ips_onboard_px4
docker compose logs --tail=30             # [agent-3] lines show a session once the FMU boots
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic echo --qos-reliability best_effort /fmu/out/ubicoders_ips_debug
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic hz --qos-reliability best_effort /fmu/out/ubicoders_ips_debug
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic echo --qos-reliability best_effort /fmu/out/estimator_status_flags
docker compose exec onboard /app/entrypoint.sh ros2 topic echo --once \
  --qos-reliability best_effort /fmu/in/vehicle_visual_odometry px4_msgs/msg/VehicleOdometry
```

Checks:

- `ips_seq` rises, `ips_position` and `ips_q` match
  `/fmu/in/vehicle_visual_odometry`, `ips_age_us` stays small and steady (what
  it means for `EKF2_EV_DELAY`: see **Delay** above).
- With `HFLOW 1`, `flow_quality` and `range_m` show live values over a textured
  floor more than 8 cm away; with `HFLOW 0` they stay NaN / 0.
- `ekf2_case` matches the boot echo (`fuse ips=1 hflow=1` -> 3), and only the
  matching block 3-5 holds numbers. In cases 1 and 3, `cs_ev_pos`, `cs_ev_hgt`
  and `cs_ev_yaw` are true and the live block follows the cube.

With a console (QGC's MAVLink Console over the telemetry radio, or over USB
with `IPS 0`): `uxrce_dds_client status`, `ubicoders_ips_debug status` (counter, last
position and its age, current `ekf2_case`), `listener ubicoders_ips_debug 5`, and `dmesg`
for the boot line. With `IPS 0`, `ubicoders_ips_debug` is not started.

The full per-case procedure with expected numbers is [BENCH.md](BENCH.md).

## Undo (stock USB behaviour)

Set `IPS 0`, rebuild and flash (`./flash.sh`; unplug and re-plug the USB cable
if the uploader keeps printing `attempting reboot`). The profile then forces
`SYS_USB_AUTO` back to 2, so USB is MAVLink again and QGC works over USB, with no
console step.

Removing the profile altogether (the sourcing line in `rcS` and the script's
entry in `ROMFS/px4fmu_common/init.d/CMakeLists.txt`) is different: nothing
forces the value back then, and the saved `SYS_USB_AUTO 0` stays. Set it back
once from QGC's MAVLink Console over the telemetry radio:
`param set SYS_USB_AUTO 2` (or `param reset SYS_USB_AUTO`), then reboot. The
other saved values (`EKF2_*`, `COM_*`, `UAVCAN_*`) also stay until reset.
