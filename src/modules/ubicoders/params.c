/**
 * Ubicoders example integer parameter
 *
 * Read by the ubicoders example module and exposed as UBI_SYS_PARAM1.
 *
 * @group Ubicoders
 */
PARAM_DEFINE_INT32(UBI_SYS_PARAM1, 1234);

/**
 * Ubicoders example float parameter
 *
 * Read by the ubicoders example module (the module stores its reciprocal).
 *
 * @group Ubicoders
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UBI_SYS_PARAM2, 10.1f);

/**
 * Ubicoders actuator override
 *
 * When enabled, the first four outputs of every output driver are driven directly
 * from the ubicoders_msg_act_out topic (pin0..pin3, PWM microseconds) instead of
 * the control allocator. Disarmed, kill switch and lockdown still force the
 * driver's disarmed values. Intended for simulation experiments only.
 *
 * @boolean
 * @group Ubicoders
 */
PARAM_DEFINE_INT32(UBI_ACT_OVERRIDE, 0);
