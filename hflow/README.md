# Holybro H-Flow on PX4 v1.17.0 — params only

The [H-Flow](https://holybro.com/products/h-flow) is a DroneCAN module (PAA3905
optical flow + AFBR-S50LV85D rangefinder + IMU). The stock `px4_fmu-v5_default`
and `px4_fmu-v6x_default` builds already contain the CAN flow and rangefinder
bridges, but the CAN driver and both subscriptions are **off by default**, so it
is not plug and play. [`hflow.params`](hflow.params) turns them on and makes
EKF2 fuse flow + range for indoor (no-GNSS) velocity and position.

Wiring: H-Flow to the flight controller's **CAN1** port, sensor facing down.
Make sure the bus is terminated (the H-Flow is usually the only node).

## 1. Apply

### Option A — QGroundControl (recommended)

1. **Vehicle Setup → Parameters → Tools → Load from file**, pick `hflow.params`.
2. Review the diff, write it to the vehicle.
3. **Reboot** (Tools → Reboot Vehicle). `UAVCAN_*` and `EKF2_HGT_REF` are
   `reboot_required`.

### Option B — nsh (QGC Analyze Tools → MAVLink Console)

Paste as one block:

```
param set UAVCAN_ENABLE 2
param set UAVCAN_SUB_FLOW 1
param set UAVCAN_SUB_RNG 1
param set UAVCAN_RNG_MIN 0.08
param set UAVCAN_RNG_MAX 30
param set EKF2_OF_CTRL 1
param set EKF2_RNG_CTRL 1
param set EKF2_HGT_REF 2
param set EKF2_GPS_CTRL 0
param set EKF2_RNG_A_HMAX 10
param set EKF2_RNG_QLTY_T 0.2
param set SENS_FLOW_ROT 0
param set SENS_FLOW_MINHGT 0.08
param set SENS_FLOW_MAXHGT 25
param set SENS_FLOW_MAXR 7.4
param save
reboot
```

## 2. Verify the sensor is seen

After the reboot, in the MAVLink Console:

```
uavcan status                 # H-Flow listed as a node
listener sensor_optical_flow  # ~100 Hz: pixel_flow_x/y, quality
listener distance_sensor      # ~40 Hz: current_distance
```

Point the sensor at a textured floor more than 8 cm away or `quality` stays 0.

## 3. See velocity and position

- **QGC:** Analyze Tools → MAVLink Inspector → `LOCAL_POSITION_NED`, tick
  `x y vx vy` to plot. `OPTICAL_FLOW_RAD` and `DISTANCE_SENSOR` show the raw data.
- **Console:** `listener vehicle_local_position` (`xy_valid`, `v_xy_valid`,
  `x y vx vy`), `ekf2 status` for active aiding sources.

`x/y` stay 0 and invalid until EKF2 starts flow fusion, which needs a valid
range above `SENS_FLOW_MINHGT` and flow quality above `EKF2_OF_QMIN_GND`. On the
bench, hold the vehicle 20–50 cm over a patterned surface and slide it sideways.

## Notes

- **Conflicts with `ips_phase1.params`:** that file sets `EKF2_HGT_REF=3`
  (Vision). With no IPS stream the EKF has no valid height and will not fuse
  flow. This file sets `2` (Range) for the flow-only test; reload
  `ips_phase1.params` (or set `EKF2_HGT_REF 3`) before going back to EV fusion.
- `SENS_FLOW_ROT` depends on how the module is mounted. Verify with the
  movement table in [docs/en/sensor/optical_flow.md](../docs/en/sensor/optical_flow.md)
  (forward → +Y, right → −X).
- Values follow the ARK Flow / ARK Flow MR setup pages in `docs/en/dronecan/`,
  which use the same DroneCAN messages and the same flow/range chips.
