/**
 * \file
 * \brief   Plugin for MAVROS to demonstrate error in distance_sensor plugin
 * \author  Andrey Stepanov
 * \version 0.0.1
 * \copyright
 * MIT License \n
 * Copyright (c) 2020 Andrey Stepanov \n
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: \n
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. \n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <mavros/mavros_plugin.h>

namespace dist_err {

namespace plugins {

using Subscriptions = mavros::plugin::PluginBase::Subscriptions;

class DistErrPlugin : public mavros::plugin::PluginBase {
    public:
        DistErrPlugin();

        void initialize(mavros::UAS& uas_);
        Subscriptions get_subscriptions();

        ros::Timer send_heartbeat_timer;
        void send_data(const ros::TimerEvent& te);

    protected:
        ros::NodeHandle plugin_nh;
        const ros::Time start_time;
        mavlink::common::msg::HEARTBEAT heartbeat_msg;
        mavlink::common::msg::DISTANCE_SENSOR dist_msg;
};

DistErrPlugin::DistErrPlugin():
        PluginBase(),
        plugin_nh("~dist_err"),
        start_time(ros::Time::now()),
        heartbeat_msg{},
        dist_msg{}
{}

void DistErrPlugin::initialize(mavros::UAS& uas_) {
    PluginBase::initialize(uas_);

    // fill up HEARTBEAT message
    heartbeat_msg.type = mavros::utils::enum_value(mavlink::common::MAV_TYPE::GENERIC);
    heartbeat_msg.autopilot = mavros::utils::enum_value(mavlink::common::MAV_AUTOPILOT::INVALID);
    heartbeat_msg.base_mode = 0;
    heartbeat_msg.custom_mode = 0;
    heartbeat_msg.system_status = mavros::utils::enum_value(mavlink::common::MAV_STATE::ACTIVE);

    // fill up DISTANCE_SENSOR message
    dist_msg.min_distance = 5;
    dist_msg.max_distance = 12000;
    dist_msg.current_distance = 50;
    dist_msg.type = mavros::utils::enum_value(mavlink::common::MAV_DISTANCE_SENSOR::LASER);
    dist_msg.id = plugin_nh.param("sensor_id", 0);
    dist_msg.orientation = 25; // see https://github.com/ArduPilot/ardupilot/blob/49693540bd555a44d30fc33366b1b8a24977e429/libraries/AP_RangeFinder/AP_RangeFinder_Params.cpp#L122
    dist_msg.covariance = 255;
    dist_msg.horizontal_fov = 0.02;
    dist_msg.vertical_fov = 0.02;
    dist_msg.quaternion = { 0.f, 0.f, 0.f, 0.f };

    send_heartbeat_timer = plugin_nh.createTimer(ros::Rate(1.0), &DistErrPlugin::send_data, this, false, true);
}

Subscriptions DistErrPlugin::get_subscriptions() {
    return {};
}

void DistErrPlugin::send_data(const ros::TimerEvent& te) {
    // send HEARTBEAT message
    UAS_FCU(m_uas)->send_message_ignore_drop(heartbeat_msg);

    // send DISTANCE_SENSOR message
    dist_msg.time_boot_ms = (te.current_expected - start_time).toSec() * 1e3;
    UAS_FCU(m_uas)->send_message_ignore_drop(dist_msg);
}

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dist_err::plugins::DistErrPlugin, mavros::plugin::PluginBase)
