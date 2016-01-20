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
 
#include "joint_state_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "cia_402.h"

#include "ros/ros.h"

#include <string>

namespace kaco {

JointStateSubscriber::JointStateSubscriber(Device& device, int32_t position_0_degree,
	int32_t position_360_degree, std::string topic_name)
    : m_device(device), m_position_0_degree(position_0_degree),
    	m_position_360_degree(position_360_degree), m_topic_name(topic_name)
{

	const uint16_t profile = device.get_device_profile_number();
	
	// TODO: Error handling in constructor is bad.

	if (profile != 402) {
		ERROR("JointStateSubscriber can only be used with a CiA 402 device. You passed a device with profile number "<<profile);
		return;
	}

	const int8_t operation_mode = device.get_entry("Modes of operation display");

	// TODO: look into INTERPOLATED_POSITION_MODE
	if (operation_mode != cia_402::ModeOfOperation::PROFILE_POSITION_MODE
		&& operation_mode != cia_402::ModeOfOperation::INTERPOLATED_POSITION_MODE) {
		ERROR("[JointStateSubscriber] Only position mode supported yet.");
		PRINT("Try set_entry(\"Modes of operation\", cia_402::ModeOfOperation::PROFILE_POSITION_MODE).");
		return;
	}

    if (m_topic_name.empty()) {
		uint8_t node_id = device.get_node_id();
		m_topic_name = "device" + std::to_string(node_id) + "/set_joint_state";
    }

}

void JointStateSubscriber::advertise() {
	DEBUG_LOG("Advertising "<<m_topic_name);
	ros::NodeHandle nh;
	m_subscriber = nh.subscribe(m_topic_name, queue_size, &JointStateSubscriber::receive, this);

}

void JointStateSubscriber::receive(const sensor_msgs::JointState& msg) {

	assert(msg.position.size()>0);
    const int32_t pos = rad_to_pos(msg.position[0]);

    DEBUG_LOG("Received JointState message");
    DEBUG_DUMP(pos);
    DEBUG_DUMP(msg.position[0]);

	m_device.set_entry("Target position", pos); // auto cast to Value!
	m_device.set_entry("Controlword", (uint16_t) 0x001F); // notify update

}

int32_t JointStateSubscriber::rad_to_pos(double rad) const {

	const double p = rad;
	const double min = m_position_0_degree;
	const double max = m_position_360_degree;
	const double dist = max - min;
	const double result = ((p/(2*pi()))*dist)+min;
	return (int32_t) result;

}

} // end namespace kaco