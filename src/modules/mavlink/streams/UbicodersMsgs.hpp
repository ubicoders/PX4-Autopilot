#ifndef MAVLINK_UBICODERS_MSGS_HPP
#define MAVLINK_UBICODERS_MSGS_HPP

#include <uORB/topics/ubicoders_msg_pub.h>
// #include <mavlink/ubicoders_msgs/mavlink_msg_ubicoders_custom.h>
#include <mavlink/ubicoders_msgs/mavlink_msg_ubicoders_custom.h>

class MavlinkStreamUbiMsgs : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamUbiMsgs(mavlink); }

    static constexpr const char *get_name_static() { return "UBICODERS_CUSTOM"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_UBICODERS_CUSTOM; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _ubicoders_msgs_outgoing_sub.advertised() ? MAVLINK_MSG_ID_UBICODERS_CUSTOM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

private:
    explicit MavlinkStreamUbiMsgs(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _ubicoders_msgs_outgoing_sub{ORB_ID(ubicoders_msg_pub)};

    bool send() override
    {

        ubicoders_msg_pub_s _ubicoders_msgs_outgoing_data{};

        if (_ubicoders_msgs_outgoing_sub.update(&_ubicoders_msgs_outgoing_data))
        {
            // PX4_INFO("in ubicoders");
            __mavlink_ubicoders_custom_t msg{};

            msg.even_number = _ubicoders_msgs_outgoing_data.even_number_output;
            // PX4_INFO("even number: %d", (int)msg.even_number);
            mavlink_msg_ubicoders_custom_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // UBICODERS_MSGS_HPP
