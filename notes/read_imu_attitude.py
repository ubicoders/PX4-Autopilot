#!/usr/bin/env python3
"""Read raw IMU data and attitude (quaternion and roll/pitch/yaw) from PX4 over MAVLink.

Requires: pip install pymavlink

Usage examples:
    python3 read_imu_attitude.py                          # USB, /dev/ttyACM0
    python3 read_imu_attitude.py --port /dev/ttyUSB0 --baud 57600   # telemetry radio
    python3 read_imu_attitude.py --port udp:0.0.0.0:14550           # SITL, or QGC forwarding
    python3 read_imu_attitude.py --rate 50 --quiet-imu               # attitude only, 50 Hz

The script asks PX4 to stream three messages at a fixed rate:
    ATTITUDE            roll, pitch, yaw (rad) and body rates (rad/s), from vehicle_attitude
    ATTITUDE_QUATERNION q1..q4 (w, x, y, z), the same attitude as a quaternion
    HIGHRES_IMU         calibrated gyro (rad/s), accelerometer (m/s^2), magnetometer (gauss),
                        from sensor_combined and friends
Angles are in the FRD body frame relative to NED: roll right, pitch up, yaw clockwise from north.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

MESSAGES = {
    "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    "ATTITUDE_QUATERNION": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
    "HIGHRES_IMU": mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
}


def request_stream(master, msg_id, rate_hz):
    """Ask the autopilot to send one message type at rate_hz (MAV_CMD_SET_MESSAGE_INTERVAL)."""
    interval_us = int(1e6 / rate_hz) if rate_hz > 0 else -1  # -1 disables the stream
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,              # confirmation
        msg_id,         # param1: message id
        interval_us,    # param2: interval in microseconds
        0, 0, 0, 0, 0,  # unused
    )


def quat_to_euler(w, x, y, z):
    """Quaternion (w, x, y, z), body FRD to NED, to roll, pitch, yaw in radians (ZYX convention)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="serial device, or udp:host:port / tcp:host:port (default /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=57600, help="serial baud rate (ignored for USB and UDP)")
    ap.add_argument("--rate", type=float, default=10.0, help="stream rate in Hz for each message (default 10)")
    ap.add_argument("--quiet-imu", action="store_true", help="do not print HIGHRES_IMU lines")
    ap.add_argument("--quiet-quat", action="store_true", help="do not print ATTITUDE_QUATERNION lines")
    args = ap.parse_args()

    master = mavutil.mavlink_connection(args.port, baud=args.baud)
    print(f"connecting to {args.port} ...", file=sys.stderr)
    master.wait_heartbeat()
    print(f"heartbeat from system {master.target_system} component {master.target_component}", file=sys.stderr)

    for name, msg_id in MESSAGES.items():
        request_stream(master, msg_id, args.rate)
        print(f"requested {name} at {args.rate:g} Hz", file=sys.stderr)

    wanted = list(MESSAGES)
    last_print = {name: 0.0 for name in wanted}
    try:
        while True:
            msg = master.recv_match(type=wanted, blocking=True, timeout=5.0)
            if msg is None:
                print("no data for 5 s (is the vehicle powered, and is another program holding the port?)",
                      file=sys.stderr)
                continue

            kind = msg.get_type()
            now = time.time()
            last_print[kind] = now

            if kind == "ATTITUDE":
                print(f"[ATT ] roll {math.degrees(msg.roll):7.2f}  pitch {math.degrees(msg.pitch):7.2f}  "
                      f"yaw {math.degrees(msg.yaw):7.2f} deg   "
                      f"rates p {msg.rollspeed:6.3f} q {msg.pitchspeed:6.3f} r {msg.yawspeed:6.3f} rad/s")

            elif kind == "ATTITUDE_QUATERNION" and not args.quiet_quat:
                r, p, y = quat_to_euler(msg.q1, msg.q2, msg.q3, msg.q4)
                print(f"[QUAT] w {msg.q1:7.4f} x {msg.q2:7.4f} y {msg.q3:7.4f} z {msg.q4:7.4f}   "
                      f"(-> rpy {math.degrees(r):6.1f} {math.degrees(p):6.1f} {math.degrees(y):6.1f} deg)")

            elif kind == "HIGHRES_IMU" and not args.quiet_imu:
                print(f"[IMU ] gyro {msg.xgyro:7.3f} {msg.ygyro:7.3f} {msg.zgyro:7.3f} rad/s   "
                      f"accel {msg.xacc:7.2f} {msg.yacc:7.2f} {msg.zacc:7.2f} m/s^2   "
                      f"mag {msg.xmag:6.3f} {msg.ymag:6.3f} {msg.zmag:6.3f} gauss")

    except KeyboardInterrupt:
        pass
    finally:
        # Put the streams back to the link's default so the next program is not surprised.
        for msg_id in MESSAGES.values():
            request_stream(master, msg_id, 0)
        master.close()


if __name__ == "__main__":
    main()
