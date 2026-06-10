# Apply IPS Phase 1 params + verify fusion

Params-only configuration for PX4 v1.17.0 (ubicoders fork, FMUv5). No firmware
source changes. This applies [`ips_phase1.params`](ips_phase1.params) and runs the
Phase 1 §5.4 "Check B" fusion verification. See [../../tasks/phase1.md](../../tasks/phase1.md).

The board must already be flashed with stock firmware before applying these params.

---

## 1. Apply the params

### Option A — QGroundControl (recommended)

1. Connect to the vehicle.
2. **Vehicle Setup → Parameters → Tools (top-right) → Load from file**.
3. Select `ips_phase1.params`.
4. Review the diff QGC shows, then **write/save** the params to the vehicle.
5. **Reboot the vehicle** (Tools → Reboot Vehicle, or power-cycle). A reboot is
   mandatory because some of these params are `reboot_required` (see below).

### Option B — nsh / `mavlink_shell.py`

Reach the nsh shell over a MAVLink link (`Tools/mavlink_shell.py /dev/ttyUSB0`,
your TELEM radio / USB-UART) or QGC **Analyze → MAVLink Console**, then:

```sh
param set EKF2_EV_CTRL 7
param set EKF2_HGT_REF 3
param set EKF2_EV_NOISE_MD 1
param set EKF2_EVP_NOISE 0.1
param set EKF2_EVV_NOISE 0.1
param set EKF2_EV_DELAY 0.0
param set EKF2_GPS_CTRL 0
param save
reboot
```

After reboot, confirm with `param show EKF2_EV_CTRL` etc.

### Reboot-required params

Both apply paths MUST be followed by a reboot. The params that explicitly take
effect only after reboot are:

| Param | Source | reboot_required |
|---|---|---|
| `EKF2_HGT_REF` | `src/modules/ekf2/module.yaml:104` | **true** |
| `EKF2_EV_DELAY` | `src/modules/ekf2/params_external_vision.yaml:27` | **true** |

(`EKF2_EV_CTRL`, `EKF2_EV_NOISE_MD`, `EKF2_EVP_NOISE`, `EKF2_EVV_NOISE`,
`EKF2_GPS_CTRL` are not flagged reboot_required, but reboot anyway so the whole
EV/height configuration is re-initialised cleanly.)

---

## 2. Tune / measure before flight

Two values are placeholders shipped at their firmware defaults and MUST be set on
the bench:

- **`EKF2_EV_DELAY`** — measure the IPS→autopilot latency and set it (ms).
  Default shipped = 0.0 ms.
- **`EKF2_EVP_NOISE` / `EKF2_EVV_NOISE`** — tune against the observed IPS jitter.
  Defaults shipped = 0.1 m / 0.1 m/s.

---

## 3. Verify fusion — Check B (Phase 1 §5.4)

Run the companion dummy sender first so a distinctive moving signal is flowing:

```sh
# on the companion host (other repo):
cargo run --bin control_test -- --dummy   # streams ODOMETRY + setpoints on serial:/dev/ttyACM0
```

Then, in the nsh shell (reached over a SECOND MAVLink link — TELEM radio /
USB-UART — so it doesn't share the companion's USB port):

| Command | Confirms | PASS looks like |
|---|---|---|
| `uorb top` | which topics publish + their rates | `vehicle_visual_odometry`, `vehicle_local_position`, `estimator_aid_src_ev_*` all updating |
| `listener vehicle_visual_odometry` | the receiver **decoded** the companion's ODOMETRY | non-zero pos/vel that track the injected circle |
| `listener vehicle_local_position` | EKF2 **fused** it | `x/y/z`, `vx/vy/vz` track the injected circle (not static, not NaN) |
| `listener estimator_aid_src_ev_pos` | EV **position** accepted, not rejected | `fused: True` **and** `test_ratio < 1` |
| `listener estimator_aid_src_ev_vel` | EV **velocity** accepted, not rejected | `fused: True` **and** `test_ratio < 1` |
| `listener trajectory_setpoint` | the setpoint reached `mc_pos_control` | non-NaN target matching what the companion sent |
| `listener offboard_control_mode` | offboard flags set | `position`/`velocity` flags true, recent timestamp |
| `mavlink status` | per-link message rates | shows `ODOMETRY` / `SET_POSITION_TARGET_LOCAL_NED` inbound (received ≠ used) |

### Interpretation

- **PASS (used correctly):** `estimator_aid_src_ev_pos` and `_ev_vel` show
  `fused: True` with `test_ratio < 1`, **and** `vehicle_local_position` follows the
  injected circle.
- **FAIL (arriving but rejected):** `mavlink status` shows `ODOMETRY` inbound but
  `estimator_aid_src_ev_*` reads `fused: False` / `test_ratio > 1` → a frame or
  timestamp bug. Re-check the ODOMETRY **`child_frame_id`** (the velocity frame;
  must be `MAV_FRAME_LOCAL_NED`) and look at `listener` / `STATUSTEXT` for the
  rejection reason.
- **Frame sanity:** inject pure **+forward** → `vehicle_local_position.x` (and
  `LOCAL_POSITION_NED.x`) increases — not `y`, not `−x`.
- **Negative test (proves dependence):** stop the companion sender → EV aiding
  times out (and if armed in Offboard, it failsafes). That proves PX4 was actually
  using the stream.

---

## 4. Offboard / arming note (§2.3, §4.2 step 7)

- `offboard_control_mode` must arrive **faster than `COM_OF_LOSS_T`** — default
  **1.0 s** (`PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 1.0f)`,
  `src/modules/commander/commander_params.c:340`).
- The companion streams `SET_POSITION_TARGET_LOCAL_NED` (+ `OFFBOARD_CONTROL_MODE`
  heartbeat) at ~20 Hz (50 ms period) — comfortably inside the 1.0 s window.
- **Order of operations:** the stream must be flowing **before** you switch to
  Offboard, and the vehicle must be **armed**. Switch to Offboard while the
  heartbeat is live, then arm (or arm then switch, but never enter Offboard with no
  stream — that trips the loss-of-offboard failsafe immediately).

## 5. Read-back stream note (no PX4-side change needed)

On this PX4 USB MAVLink port the default stream set does **not** include message
IDs **85** (`POSITION_TARGET_LOCAL_NED`) or **230** (`ESTIMATOR_STATUS`). The
companion requests them at runtime via `MAV_CMD_SET_MESSAGE_INTERVAL` (#511), so no
PX4 parameter or firmware change is required to get the read-back streams.
