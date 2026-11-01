# ubicoders_ips: bench procedure

Step by step, one section per fusion case, then the measurements and the
go/no-go list before the first hover. "Bench" means **propellers off**, the
drone carried by hand inside the IPS camera volume. Nothing here arms the
motors.

Background: the switches, the cases and the parameters are in
[README.md](README.md); the whole design, frames included, is in the
companion repo's [diagram.md](../../../ips_onboard_px4/diagram.md) (the two
repos sit side by side in `/home/el/Github`).

> The FMU still runs an **older firmware** until `./flash.sh` has been run
> once from `ips_onboard_px4`. Do step 0 first.

## 0. Before every case

### Set the switches and flash

1. Edit the four `set` lines at the top of
   [`rc.ubicoders_ips`](../../ROMFS/px4fmu_common/init.d/rc.ubicoders_ips)
   to the values in the case's section.
2. On the Jetson:

   ```bash
   cd /home/el/Github/ips_onboard_px4
   ./flash.sh                # reboot to bootloader over ROS 2, stop, flash, start
   ./flash.sh --no-reboot    # if no uXRCE-DDS session is up (e.g. the older firmware)
   ```

   If the uploader keeps printing `attempting reboot`, unplug the FMU's USB
   cable and plug it back in.
3. Wait for the agent's session: `docker compose logs -f` shows `[agent-3]`
   lines about a session once the FMU has booted.
4. Optional, in QGC's MAVLink Console over the telemetry radio: `dmesg` shows
   the boot line `ubicoders_ips: IPS=... HFLOW=... -> fuse ips=... hflow=...`.
   `param show EKF2_EV_CTRL` and friends show what the profile set.

### The commands used below

All from `/home/el/Github/ips_onboard_px4` on the Jetson. PX4's topics are
best effort, so every echo names that QoS; naming the type avoids
`Could not determine the type` before discovery has finished.

```bash
# the live stream
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic echo --qos-reliability best_effort /fmu/out/ubicoders_ips_debug
# one sample
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic echo --once --qos-reliability best_effort /fmu/out/ubicoders_ips_debug px4_msgs/msg/UbicodersIpsDebug
# the rate: the IPS rate while poses arrive, 10 Hz while none do
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic hz --qos-reliability best_effort /fmu/out/ubicoders_ips_debug
# PX4's attitude, for the yaw check
docker compose exec onboard /app/entrypoint.sh \
  ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_attitude px4_msgs/msg/VehicleAttitude
```

Yaw in degrees from a quaternion (w, x, y, z), on any machine with Python:

```bash
python3 -c 'import math,sys; w,x,y,z=map(float,sys.argv[1:]); print(round(math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))),1))' 0.7071 0 0 0.7071
# prints 90.0
```

### Where the axes point

The IPS world (`ips_frd`) is PX4's NED by convention: **x** points to the IPS
**forward** side of the room (the calibration board's top row, board −y),
**y** to the IPS **right** side, **z** down, with z = 0 at the board's plane.
Mark the forward and right directions on the floor once; every walk test uses
them. Mount the cube front face to the nose, bottom face down.

### Reading the case

| `ekf2_case` | Meaning | Live EKF2 block | `cs_*` flags expected true |
|---|---|---|---|
| 0 | nothing fused | none (all NaN) | none |
| 1 | IPS only | 4, `ekf2_ips_*` | `cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw` |
| 2 | H-Flow only | 3, `ekf2_hflow_*` | `cs_opt_flow` (and usually `cs_rng_hgt`) |
| 3 | IPS + H-Flow | 5, `ekf2_ips_hflow_*` | `cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw`, `cs_opt_flow` |

`ekf2_case` is what EKF2 actually fuses, so it can lag the boot echo by a few
seconds after boot, or stay lower if EKF2 refuses a source.

---

## Case 0: nothing fused (link and raw sensors only)

**Switches:** `IPS 1`, `HFLOW 1`, `FUSE_IPS 0`, `FUSE_HFLOW 0`. Boot line:
`fuse ips=0 hflow=0`.

**Confirm the case:** `ekf2_case` 0; `cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw`,
`cs_opt_flow`, `cs_rng_hgt` all false; blocks 3, 4 and 5 all NaN / false.

**By hand:**

1. Set the drone level on the floor in view of the cameras. Read one sample.
2. Carry it around slowly for 10 s, then put it back.
3. Lift it to about 0.5 m over a textured floor (carpet, tape marks; a plain
   shiny floor gives the flow camera nothing to track) and hold it still.
4. Cover the cube with a cloth for 3 s, then uncover it.

**Expected numbers:**

- `ips_seq` rises steadily; `ros2 topic hz` shows the IPS rate.
- `ips_pose_frame` 1; `ips_position` follows the cube; at rest on the floor z
  is negative (the cube centre is above the board's plane).
- `ips_q` is four finite numbers (orientation on by default);
  `ips_velocity` is NaN (velocity off by default).
- `ips_age_us` a few thousand (a few milliseconds) and steady.
- Held at 0.5 m: `range_m` about 0.5 (the H-Flow's height, a little less than
  the cube's), `range_signal_quality` above 0, `flow_quality` well above 0.
- Cube covered: `ips_seq` stops, `ips_age_us` grows past 1 000 000, and the
  message keeps coming at 10 Hz; uncovered, `ips_seq` rises again.

**Pass:** `ips_seq` rises, `ips_position` follows the cube, `ips_q` finite,
`ips_age_us` below 10 000 and steady, `range_m` and `flow_quality` live when
held up, nothing fused. **Fail:** anything else; see the troubleshooting
table in [diagram.md](../../../ips_onboard_px4/diagram.md#10-if-something-does-not-work).

---

## Case 1: IPS only

**Switches:** `IPS 1`, `HFLOW 0`, `FUSE_IPS 1`, `FUSE_HFLOW 0` (or `HFLOW 1`
with `FUSE_HFLOW 0`, to keep the raw flow and range visible in block 2). Boot
line: `fuse ips=1 hflow=0`.

**Confirm the case:** `ekf2_case` 1; `cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw`
true; `cs_opt_flow`, `cs_rng_hgt` false; block 4 (`ekf2_ips_*`) holds
numbers, blocks 3 and 5 NaN; `ekf2_ips_xy_valid` and `ekf2_ips_v_xy_valid`
true. With `HFLOW 0`, `flow_rate` and `range_m` are NaN.

**By hand,** starting level on the floor, nose toward IPS forward. Take one
sample before and after each move and compare.

| Test | Do | Expected numbers |
|---|---|---|
| Rest | nothing | `ekf2_ips_position` within a few cm of `ips_position` in x and y; `ekf2_ips_position[2]` about 0.20 larger than `ips_position[2]` (the lever arm: the FMU is 20 cm below the cube) |
| Walk forward | carry the drone 1 m toward IPS forward, keep it level | `ips_position[0]` and `ekf2_ips_position[0]` both rise by about 1.0 (±0.1); y and z change by less than 0.1 |
| Walk right | carry it 1 m toward IPS right | `[1]` rises by about 1.0 (±0.1); x and z change by less than 0.1 |
| Lift | lift it 20 cm straight up | `[2]` falls by about 0.2 (±0.03): z points down |
| Turn | turn it 90° clockwise seen from above, in place | yaw from `ips_q` goes from about 0° to about +90°; the yaw from `/fmu/out/vehicle_attitude` `q` follows to within 5° |
| Velocity | walk forward at a steady pace | `ekf2_ips_velocity[0]` positive while moving, back near 0 at rest |

**Pass:** the case is confirmed, and all six rows hold. **Fail** on any row.
The usual causes: x and y swapped or negated (frame or cube mounting), z rising
on the lift (sign), the two yaws differing by a fixed angle (cube rotated on
its mount), `ekf2_case` stuck at 0 (no heading: check `ips_q` is not NaN).

---

## Case 2: H-Flow only

**Switches:** `IPS 1`, `HFLOW 1`, `FUSE_IPS 0`, `FUSE_HFLOW 1`. Boot line:
`fuse ips=0 hflow=1`.
`IPS 1` keeps the DDS link up so `ubicoders_ips_debug` reaches the Jetson; the IPS pose
is still echoed in block 1 but not fused.

In this case the magnetometer is back on (stock automatic mode), so PX4's x is
**magnetic north**, not IPS forward. Calibrate the magnetometer in QGC first
if it never was, and expect a poorer heading indoors.

**Confirm the case:** with the drone held 0.3 m or more over a textured
floor: `ekf2_case` 2; `cs_opt_flow` true; `cs_ev_pos`, `cs_ev_hgt`,
`cs_ev_yaw` false; block 3 (`ekf2_hflow_*`) holds numbers, blocks 4 and 5 NaN.
`cs_rng_hgt` is normally true while held still and low (conditional range
mode); record it. On the floor, below the H-Flow's 8 cm minimum, flow is
not fused and `ekf2_case` may read 0.

**By hand:**

| Test | Do | Expected numbers |
|---|---|---|
| Hold | hold it level and still at a tape-measured 0.5 m (H-Flow lens to floor) | `range_m` 0.5 ±0.05; `flow_quality` well above 0; `flow_rate` near 0; `ekf2_hflow_v_xy_valid` true |
| Lift | raise it by 20 cm | `range_m` rises by about 0.2; `ekf2_hflow_position[2]` falls by about 0.2 |
| Carry | carry it 1 m in the direction its nose points, level, at constant height | `ekf2_hflow_position` moves about 1 m along the nose direction (x if the nose points at magnetic north); some drift is normal |
| Still | hold it still for 10 s | `ekf2_hflow_velocity` stays within ±0.1 m/s |

**Pass:** case confirmed, range matches the tape, the lift sign is right,
the carry moves the position the right way within about 30 % of the
distance. **Fail:** the carry moves the position sideways or backwards
(`SENS_FLOW_ROT` does not match the mounting), `range_m` NaN (CAN wiring,
`UAVCAN_ENABLE`), `ekf2_case` never 2 (floor texture, height, `flow_quality`).

---

## Case 3: IPS + H-Flow

**Switches:** `IPS 1`, `HFLOW 1`, `FUSE_IPS 1`, `FUSE_HFLOW 1` (all four at 1).
This is the image as built. Boot line: `fuse ips=1 hflow=1`.

**Confirm the case:** held 0.3 m or more over a textured floor: `ekf2_case` 3;
`cs_ev_pos`, `cs_ev_hgt`, `cs_ev_yaw`, `cs_opt_flow` true (`cs_rng_hgt` may be
either: the IPS is the height reference and the range only a conditional
helper); block 5 (`ekf2_ips_hflow_*`) holds numbers, blocks 3 and 4 NaN. On
the floor `ekf2_case` may read 1 until flow starts.

**By hand:** repeat the six case 1 tests on block 5, holding the drone over a
textured floor, then one more:

| Test | Do | Expected numbers |
|---|---|---|
| Case 1 tests | as in case 1, on `ekf2_ips_hflow_*` | same numbers as case 1 |
| Range vs IPS | hold it still at a tape-measured 0.5 m, then lift 20 cm | `range_m` reads the tape within 5 cm; on the lift `range_m` rises by the same 0.2 that `ips_position[2]` falls |
| IPS dropout (observe only) | cover the cube for 5 s, then uncover | `ips_age_us` grows; record whether and when `ekf2_case` drops to 2 and comes back to 3; not a pass criterion yet |

**Pass:** case confirmed and the case 1 tests pass on block 5. **Fail** as
in case 1 or case 2.

---

## Measurements

### Lever arm (`EKF2_EV_POS_X/Y/Z`)

The IPS tracks the cube centre; EKF2 reports the FMU's position (its IMU, while
`EKF2_IMU_POS_*` keep their stock 0). The lever arm is the cube centre
relative to the FMU in **body FRD**: x toward the nose, y to the right, z
down, so a cube above the FMU has a **negative** z.

1. Set the drone level. Find the FMU's centre (the IMU is close to the middle
   of the board) and the cube's centre.
2. Measure with a ruler: forward offset (cube ahead of the FMU = positive x),
   right offset (cube to the right = positive y), vertical offset (cube above
   = negative z). Record to the nearest 5 mm.
3. Put the three values into the `EKF2_EV_POS_X/Y/Z` lines of
   `rc.ubicoders_ips` (now `0.0`, `0.0`, `-0.20`), rebuild and flash.
4. Check in case 1, drone level, nose toward IPS forward:
   `ekf2_ips_position - ips_position` is about (−X, −Y, −Z), for example
   (0, 0, +0.20) for the placeholder.
5. Optional tilt check: hold the drone by its FMU and tilt it 30° nose up.
   `ips_position[0]` moves back by about 0.5 × |Z| (0.10 m for 0.20); the EKF2
   x should move much less, because the FMU stayed where it was.

**Pass:** step 4 matches within 2 cm.

### Delay (`ips_age_us` and `EKF2_EV_DELAY`)

EKF2 treats each IPS sample as `EKF2_EV_DELAY` older than its
`timestamp_sample`. `ips_to_px4` sets `timestamp_sample` when the pose reaches
the Jetson, so two latencies are involved:

| Latency | What it covers | How to read it |
|---|---|---|
| Jetson to PX4 | after the stamp: `ips_to_px4` publish, agent, USB, client, `ubicoders_ips_debug` wake-up | `ips_age_us` in `ubicoders_ips_debug` (the client converts the stamp to PX4 time, so it is a true difference) |
| IPS to Jetson | before the stamp: camera exposure, detection, tracking, Zenoh, bridge | `ros2 topic delay /ips/cube_1/pose` on the Jetson: the pose header carries the camera capture time, so this is meaningful only if the IPS server's and the Jetson's clocks are synchronised (both on the same NTP server, for example) |

1. With poses flowing, read `ips_age_us` over 30 s (stream, or ten `--once`
   samples). Record the typical value and the spread.
2. If the clocks are synchronised:

   ```bash
   docker compose exec onboard /app/entrypoint.sh ros2 topic delay /ips/cube_1/pose
   ```

   Record the average.
3. `EKF2_EV_DELAY` (ms) belongs to the second number: it is the age of the
   measurement at the moment it was stamped. The first number is already in
   the time stamp and shows only that the link keeps up. Put the value into
   the `EKF2_EV_DELAY` line of `rc.ubicoders_ips` (now 5 ms), rebuild, flash.

**Pass:** `ips_age_us` below 10 000 (10 ms) and steady within a few ms;
the IPS-to-Jetson latency measured (or recorded as unmeasurable, with the
reason) and `EKF2_EV_DELAY` set from it.

---

## Go / no-go before the first hover

All must be **go**. Any **no-go** stops the flight.

| # | Item | Go when |
|---|---|---|
| 1 | Firmware | the board runs this build (flashed with `./flash.sh`); the boot line shows the intended case |
| 2 | Case 1 bench | every case 1 test passed on the drone as it will fly (cube mounted, props off) |
| 3 | Case 3 bench | every case 3 test passed |
| 4 | Lever arm | measured and in the image; step 4 check passed |
| 5 | Delay | `ips_age_us` steady below 10 ms; `EKF2_EV_DELAY` set from a measurement or a recorded decision |
| 6 | Yaw | IPS yaw and PX4 yaw agree within 5° at 0°, 90°, 180°, 270° |
| 7 | Mode change | with props off and the drone disarmed: Position selected from QGC over the telemetry radio, then the pilot gets back to Stabilized with the RC channel 7 switch, and QGC shows Stabilized. Position mode is not on the RC switch (low = Stabilized, high = Offboard), so this gesture is rehearsed until it is automatic |
| 8 | RC | transmitter bound, failsafe tested, channel 7 low = Stabilized |
| 9 | Space | net or clear area, people behind the pilot, IPS coverage at 1 m height across the hover area |
| 10 | Battery | full, and the low-battery failsafe set |

**The first hovers,** in this order, each a separate flash:

1. **Case 1** (`IPS 1`, `HFLOW 0`, `FUSE_IPS 1`, `FUSE_HFLOW 0`). Take off in
   Stabilized, climb to about 1 m, switch to Position from QGC, hands near the
   sticks. Hold 30 s.
   Pilot's thumb on the channel 7 switch; any drift, oscillation or toilet-bowl
   circling (a growing spiral, the sign of a heading error): back to Stabilized
   and land.
2. **Case 3** (`IPS 1`, `HFLOW 1`, `FUSE_IPS 1`, `FUSE_HFLOW 1`), the same.

Offboard (the companion flying) comes after both hovers are clean.
