/*
 * Copyright (c) 2015-2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "joint_state_publisher.h"
#include "utils.h"
#include "logger.h"
#include "cia_402.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <string>

namespace kaco {

JointStatePublisher::JointStatePublisher(Device& device, int32_t position_0_degree,
	int32_t position_360_degree, std::string topic_name)
    : m_device(device), m_position_0_degree(position_0_degree),
    	m_position_360_degree(position_360_degree), m_topic_name(topic_name)
{

	const uint16_t profile = device.get_device_profile_number();
	
	// TODO: Error handling in constructor is bad.

	if (profile != 402) {
		ERROR("JointStatePublisher can only be used with a CiA 401 device. You passed a device with profile number "<<profile);
		return;
	}

	const int8_t operation_mode = device.get_entry("Modes of operation display");

	// TODO: look into INTERPOLATED_POSITION_MODE
	if (operation_mode != cia_402::ModeOfOperation::PROFILE_POSITION_MODE
		&& operation_mode != cia_402::ModeOfOperation::INTERPOLATED_POSITION_MODE) {
		ERROR("[JointStatePublisher] Only position mode supported yet.");
		PRINT("Try set_entry(\"Modes of operation\", cia_402::ModeOfOperation::PROFILE_POSITION_MODE).");
		return;
	}

    if (m_topic_name.empty()) {
		uint8_t node_id = device.get_node_id();
		m_topic_name = "device" + std::to_string(node_id) + "/get_joint_state";
    }

}

void JointStatePublisher::advertise() {
	DEBUG_LOG("Advertising "<<m_topic_name);
	ros::NodeHandle nh;
	m_publisher = nh.advertise<sensor_msgs::JointState>(m_topic_name, queue_size);
}

void JointStatePublisher::publish() {
	
	sensor_msgs::JointState js;
    
    js.name.resize(1);
    js.position.resize(1);

    // only position supported yet.
    js.velocity.resize(0);
    js.effort.resize(0);
    
    js.name[0] = m_topic_name;

    const int32_t pos = m_device.get_entry("Position actual value");
    js.position[0] = pos_to_rad(pos);

    DEBUG_LOG("Sending JointState message");
    DEBUG_DUMP(pos);
    DEBUG_DUMP(js.position[0]);

    m_publisher.publish(js);

}

double JointStatePublisher::pos_to_rad(int32_t pos) const {

	const double p = pos;
	const double min = m_position_0_degree;
	const double max = m_position_360_degree;
	const double dist = max - min;
	const double result = ((p-min)/dist)*2*pi();
	return result;
}

} // end namespace kaco