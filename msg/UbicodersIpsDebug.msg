# IPS debug: one message that shows every fusion case side by side.
#
# PX4 runs one estimator, so only one of blocks 3-5 is live at a time:
# ekf2_case says which one, the other two are NaN / false. ekf2_case is
# bit0 = IPS fused, bit1 = H-Flow fused (0 none, 1 IPS, 2 H-Flow, 3 both), taken
# from what EKF2 actually fuses (estimator_status_flags), not from the FUSE_*
# switches in rc.ubicoders_ips. Block 1 keeps the last IPS sample and ips_age_us grows while no
# new one arrives (NaN before the first). Blocks 2 to 5 are NaN / 0 / false when
# their topic has no sample or the last one is older than 1 s. Published on
# every vehicle_visual_odometry sample and at 10 Hz when none arrives.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                    # [us] time since system start, when PX4 published this
# 1. IPS as received (vehicle_visual_odometry)
uint64 ips_timestamp                # [us] vehicle_visual_odometry.timestamp as received
uint64 ips_timestamp_sample         # [us] vehicle_visual_odometry.timestamp_sample as received
uint32 ips_seq                      # count of vehicle_visual_odometry samples seen since start
uint32 ips_age_us                   # [us] timestamp minus ips_timestamp (how stale the input is)
uint8 ips_pose_frame                # echoed pose_frame (1 = NED, 2 = FRD)
float32[3] ips_position             # [m] echoed position, NED
float32[4] ips_q                    # [-] echoed attitude quaternion (w, x, y, z), NaN if not sent
float32[3] ips_velocity             # [m/s] echoed velocity, NaN if not sent

# 2. H-Flow raw (vehicle_optical_flow, distance_sensor)
float32[2] flow_rate                # [rad/s] optical flow rate about body x, y
uint8 flow_quality                  # [0..255] vehicle_optical_flow.quality
float32 range_m                     # [m] distance_sensor.current_distance
uint8 range_signal_quality          # [%] distance_sensor.signal_quality

# which EKF2 case is live (from estimator_status_flags; NONE if missing or older than 1 s)
uint8 ekf2_case                     # (cs_ev_pos ? 1 : 0) + (cs_opt_flow ? 2 : 0)
uint8 EKF2_CASE_NONE = 0            # neither IPS nor H-Flow fused
uint8 EKF2_CASE_IPS = 1             # IPS only
uint8 EKF2_CASE_HFLOW = 2           # H-Flow only
uint8 EKF2_CASE_IPS_HFLOW = 3       # IPS + H-Flow
bool cs_opt_flow                    # estimator_status_flags.cs_opt_flow: optical flow fusion intended
bool cs_rng_hgt                     # estimator_status_flags.cs_rng_hgt: range fused for height
bool cs_ev_pos                      # estimator_status_flags.cs_ev_pos: EV position fusion intended
bool cs_ev_hgt                      # estimator_status_flags.cs_ev_hgt: EV height fused
bool cs_ev_yaw                      # estimator_status_flags.cs_ev_yaw: EV yaw fusion intended

# 3. EKF2 output, H-Flow only (vehicle_local_position while ekf2_case == 2)
float32[3] ekf2_hflow_position      # [m] NED
float32[3] ekf2_hflow_velocity      # [m/s] NED
bool ekf2_hflow_xy_valid            # vehicle_local_position.xy_valid
bool ekf2_hflow_v_xy_valid          # vehicle_local_position.v_xy_valid

# 4. EKF2 output, IPS only (vehicle_local_position while ekf2_case == 1)
float32[3] ekf2_ips_position        # [m] NED
float32[3] ekf2_ips_velocity        # [m/s] NED
bool ekf2_ips_xy_valid              # vehicle_local_position.xy_valid
bool ekf2_ips_v_xy_valid            # vehicle_local_position.v_xy_valid

# 5. EKF2 output, IPS + H-Flow (vehicle_local_position while ekf2_case == 3)
float32[3] ekf2_ips_hflow_position  # [m] NED
float32[3] ekf2_ips_hflow_velocity  # [m/s] NED
bool ekf2_ips_hflow_xy_valid        # vehicle_local_position.xy_valid
bool ekf2_ips_hflow_v_xy_valid      # vehicle_local_position.v_xy_valid
