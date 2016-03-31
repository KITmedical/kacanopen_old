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
 
#include "bridge.h"
#include "logger.h"
#include "ros/ros.h"

namespace kaco {

Bridge::Bridge() {
	//int argc = 0;
	//char *argv[] {};
	//ros::init(argc, argv, "canopen_bridge");
}

Bridge::~Bridge() 
	{ }

void Bridge::add_publisher(std::shared_ptr<Publisher> publisher) {
	m_publishers.push_back(publisher);
	publisher->advertise();
}

void Bridge::add_subscriber(std::shared_ptr<Subscriber> subscriber) {
	m_subscribers.push_back(subscriber);
	subscriber->advertise();
}

void Bridge::run(double loop_rate) {

	ros::Rate rate(loop_rate);

	// TODO after making core fully thread-safe
	//ros::AsyncSpinner spinner(1);
	//spinner.start();

	while(ros::ok()) {

		for (std::shared_ptr<Publisher> publisher : m_publishers) {
			publisher->publish();
		}

		ros::spinOnce();  // TODO see above
		rate.sleep();
	}

}

} // end namespace kaco
