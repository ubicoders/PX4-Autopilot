#include "ubicoders_auto_pos.hpp"

UbicodersAutoPosModule::UbicodersAutoPosModule() : ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::ubicoders_wq)
{
}


UbicodersAutoPosModule::~UbicodersAutoPosModule()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool UbicodersAutoPosModule::init()
{
	ScheduleOnInterval(10000_us);
	return true;
}

void UbicodersAutoPosModule::error_integrator()
{
    for (int i = 0; i < 3; i++){
        _pos_err_int[i] += _pos_err[i];
        clamp(_pos_err_int[i], _pos_err_limit[i]);
    }
}

void UbicodersAutoPosModule::clamp(float &val, float limit)
{
    if (val > limit){
        val = limit;
    }
    if (val < -limit){
        val = -limit;
    }
}

void UbicodersAutoPosModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// if (!_ips_sub.updated()){
	// 	perf_end(_loop_perf);
	// 	return;
	// }

    // subs attitude
	
	if (_vehicle_attitude_sub.update(&_vehicle_attitude)){
		// _vehicle_attitude_sub.copy(&_vehicle_attitude);
		//Convert the quaternion to a rotation matrix
		matrix::Dcmf rotm(_vehicle_attitude.q);
		matrix::Eulerf euler_angles(rotm);
		
		_ubi_att.roll = euler_angles(0);
		_ubi_att.pitch = euler_angles(1);
		_ubi_att.yaw = euler_angles(2);

		_ubi_att.qx = _vehicle_attitude.q[0];
		_ubi_att.qy = _vehicle_attitude.q[1];
		_ubi_att.qz = _vehicle_attitude.q[2];
		_ubi_att.qw = _vehicle_attitude.q[3];
		_ubi_att_pub.publish(_ubi_att);
		// PX4_INFO("pub	attitude");
	
	}
		// // subs ips
		// _ips_sub.copy(&_ips);

		// // calculate position error
		// _pos_err[0] = _pos[0] - _ips.ips_x;
		// _pos_err[1] = _pos[1] - _ips.ips_y;
		// _pos_err[2] = _pos[2] - _ips.ips_z;
		// error_integrator();

		// // calculate velo setpoint in global frame
		// _kp[0] = 0.1;
		// _vel_sp[0] = _kp[0] * _pos_err[0] + _ki[0] * _pos_err_int[0] ;
		// clamp(_vel_sp[0], 1);

		// _kp[1] = 0.1;
		// _vel_sp[1] = _kp[1] * _pos_err[1] + _ki[1] * _pos_err_int[1] ;
		// clamp(_vel_sp[1], 1);

		// matrix::Vector3f vel_xy_sp_global(_vel_sp[0], _vel_sp[1], 0);
		// matrix::Vector3f vel_xy_sp_body = rotm.transpose() * vel_xy_sp_global;


		// // publish msg
		// _auto_ctrl_sp.roll = vel_xy_sp_body(0);
		// _auto_ctrl_sp.pitch = vel_xy_sp_body(1);
		// _auto_ctrl_sp.yaw = 0;
		// _auto_ctrl_sp.thrust = 0;
		// _auto_control_sp_pub.publish(_auto_ctrl_sp);


		_debug_msg.pos_x = 0.12;
		_debug_msg.pos_y = 0.21;
		_debug_msg.pos_z = 0.33;
		_debug_msg.vdist = 0.44;
		_ubi_debug_pub.publish(_debug_msg);

			
	

	// //  publish some data
	// orb_test_s data{};
	// data.val = 123456789;
	// data.timestamp = hrt_absolute_time();
	// _orb_test_pub.publish(data);

	perf_end(_loop_perf);
}

int UbicodersAutoPosModule::task_spawn(int argc, char *argv[])
{
	UbicodersAutoPosModule *instance = new UbicodersAutoPosModule();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int UbicodersAutoPosModule::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int UbicodersAutoPosModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UbicodersAutoPosModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ubicoders_module", "ubicoders_module_category");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ubicoders_auto_pos_main(int argc, char *argv[])
{
	return UbicodersAutoPosModule::main(argc, argv);
}
