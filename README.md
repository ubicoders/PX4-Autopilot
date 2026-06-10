# PX4 parameter setup — IPS → PX4 offboard control

This fork (`ubicoders/PX4-Autopilot`, `ubicoders_v1.17.0`, FMUv5) is used **params-only**
for the IPS → PX4 integration — no firmware source changes. This README documents the
**flight-mode switch** (RC ch7 toggles a manual/default mode ⇄ Jetson-driven offboard) and the
**offboard parameters**.

- EKF2 external-vision params (so PX4 *consumes* the IPS position) live in
  [`ips_phase1/`](ips_phase1/) — apply those too; they are a prerequisite (see bottom).
- Companion app & plan: [`../ips_onboard_px4`](../ips_onboard_px4), [`../tasks/phase1.md`](../tasks/phase1.md).

---

## 1. Flight-mode switch — RC ch7: default (manual) ⇄ offboard

PX4 maps **one RC channel to six flight-mode "slots"**. A 2-position switch lands on
**slot 1 (low)** and **slot 6 (high)**; a 3-position switch adds a middle slot. Put your
default/manual mode on slot 1 and Offboard on slot 6, on **channel 7**:

| Param | Value | Meaning |
|---|---|---|
| `RC_MAP_FLTMODE` | `7` | ch7 is the flight-mode select channel (the 6-slot mechanism) |
| `COM_FLTMODE1` | `8` | **slot 1 = switch LOW / resting = your default mode** → Stabilized (pilot flies) |
| `COM_FLTMODE6` | `7` | **slot 6 = switch HIGH** → Offboard (the Jetson flies) |
| `COM_FLTMODE2..5` | `-1` | unused slots (Unassigned) |

**"Default mode"** = whatever slot the switch rests in when you power up / aren't commanding
offboard, i.e. `COM_FLTMODE1`. Pick it from the mode-number table:

| # | Mode | Notes |
|---|---|---|
| 0 | Manual | direct (multicopter ≈ rate/acro) |
| **8** | **Stabilized** | self-levelling, manual throttle — **safe manual default, needs no position estimate** |
| 1 | Altitude | self-level + altitude hold |
| 2 | Position | holds position — needs a valid estimate (your EV) |
| **7** | **Offboard** | external setpoints from the Jetson |

Use **8 (Stabilized)** for the manual slot if you want a fallback that works even before EV is
fusing; use **2 (Position)** only once the IPS estimate is good.

> Calibrate ch7 as a switch first: **QGC → Radio**, toggle the switch so PX4 registers the
> channel. An unmapped channel does nothing. QGC → **Flight Modes** shows the slot assignment.

## 2. Offboard parameters

| Param | Value | Default | Meaning |
|---|---|---|---|
| `COM_RC_IN_MODE` | `0` | 3 | `0` = RC only (you have a transmitter). Keep RC available so the ch7 switch works. |
| `COM_OF_LOSS_T` | `1.0` | 1.0 | Max gap (s) with **no offboard setpoint** before failsafe. The companion streams at ~20 Hz, well under this — just never let the stream stall >1 s. |
| `COM_OBL_RC_ACT` | `2` | 0 | Action if offboard is lost **and RC is available**: `2` = fall back to Stabilized (pilot takes over). |
| `COM_RCL_EXCEPT` | `4` | 0 | Failsafe **exceptions** bitmask; **bit 2 = Offboard**. `4` = don't RC-loss-failsafe while in Offboard (let the Jetson keep flying if the RC link drops). *Safety choice — omit if you want RC loss to always failsafe.* |
| `COM_ARM_WO_GPS` | `1` | 1 | Allow arming without GPS — required indoors. |

## 3. Copy-paste (nsh / QGC MAVLink Console)

```sh
# --- flight-mode switch: ch7 toggles default(manual) <-> offboard ---
param set RC_MAP_FLTMODE 7
param set COM_FLTMODE1 8        # LOW  / default -> Stabilized (pilot)
param set COM_FLTMODE6 7        # HIGH           -> Offboard  (Jetson)
param set COM_FLTMODE2 -1
param set COM_FLTMODE3 -1
param set COM_FLTMODE4 -1
param set COM_FLTMODE5 -1

# --- offboard behaviour ---
param set COM_RC_IN_MODE 0      # RC available (drives the ch7 switch)
param set COM_OF_LOSS_T 1.0     # offboard setpoint-loss timeout (s)
param set COM_OBL_RC_ACT 2      # on offboard loss -> Stabilized
param set COM_RCL_EXCEPT 4      # bit2=Offboard: keep flying if RC drops in offboard (optional)
param set COM_ARM_WO_GPS 1      # arm without GPS (indoor)

param save
reboot
```

(In QGC: **Parameters → Tools → Load from file** also works; mode params take effect without a
reboot, but reboot anyway since the airframe/EV params below are reboot-required.)

## 4. Engaging offboard — order of operations & gotchas

1. **Companion streaming first.** The Jetson must already be sending
   `SET_POSITION_TARGET_LOCAL_NED` at >2 Hz (the app does 20 Hz) **before** you flip ch7 to the
   Offboard slot — otherwise PX4 **rejects** the switch and stays in the manual slot.
2. Flip ch7 **HIGH** → Offboard engages (no companion set-mode command needed — the RC switch
   does it). Flip **LOW** → back to Stabilized, pilot has control.
3. **Arm** via RC stick gesture, an arm switch, or QGC.

## 5. Prerequisites (without these, Offboard won't engage / stays NaN)

- **EV fusion** — load [`ips_phase1/ips_phase1.params`](ips_phase1/ips_phase1.params)
  (`EKF2_EV_CTRL=7`, …) so EKF2 actually consumes the IPS position into
  `vehicle_local_position`. Without it EKF2 sits in `CONST_POS_MODE` and there is no position to
  control. See [`ips_phase1/apply_params.md`](ips_phase1/apply_params.md).
- **Controllers running** — select a multicopter airframe so `mc_pos_control` starts
  (`rc.mc_apps`): e.g. `param set SYS_AUTOSTART 4001` (generic quad) + reboot. No airframe →
  `mc_pos_control` never starts → `POSITION_TARGET_LOCAL_NED` (#85) stays NaN.

Verify on the FC: flip ch7 and watch the mode change in QGC (or the HEARTBEAT `custom_mode`);
`listener vehicle_local_position` should track the IPS once EV is fused.
