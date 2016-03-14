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
#include "joint_state_publisher.h"
#include "joint_state_subscriber.h"
#include "cia_402.h"
 
#include <thread>
#include <chrono>
#include <memory>

// #define BUSNAME ... // set by CMake
// #define BAUDRATE ... // set by CMake

int main(int argc, char* argv[]) {

	kaco::Master master;
	bool success = master.start(BUSNAME, BAUDRATE);

	if (!success) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));
	while (master.get_devices().size()<1) {
		ERROR("No CiA 402 devices found. Waiting.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// should be a 401 device
	kaco::Device& device = master.get_devices()[0];
	device.start();
	
	if (!device.load_dictionary_from_library()) {
		ERROR("No suitable EDS file found for this device.");
		return EXIT_FAILURE;
	}

	const uint16_t profile = device.get_device_profile_number();
	
	if (profile != 402) {
		ERROR("This example is intended for use with a CiA 402 device. You plugged a device with profile number "<<std::dec<<profile);
		return EXIT_FAILURE;
	}

	device.print_dictionary();

	DUMP(device.get_entry("manufacturer_device_name"));
	
	PRINT("Enable operation");
	device.set_entry("controlword", (uint16_t) 0x0006); // shutdown
	device.set_entry("controlword", (uint16_t) 0x0007); // switch on
	device.set_entry("controlword", (uint16_t) 0x000F); // enable operation

	PRINT("Set position mode");
	device.set_entry("modes_of_operation", (int8_t) kaco::cia_402::ModeOfOperation::PROFILE_POSITION_MODE);
	
	ros::init(argc, argv, "canopen_bridge");
	
	// Create bridge
	kaco::Bridge bridge;
	
	auto jspub = std::make_shared<kaco::JointStatePublisher>(device, 0, 350000);
	bridge.add_publisher(jspub);
	
	auto jssub = std::make_shared<kaco::JointStateSubscriber>(device, 0, 350000);
	bridge.add_subscriber(jssub);

	// run ROS loop and publish everything repeatedly with 1 Hz
	bridge.run(1);

	master.stop();

}
