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
 
#include <thread>
#include <chrono>
#include <vector>

#include "core.h"
#include "logger.h"

// #define BUSNAME ... // set by CMake
// #define BAUDRATE ... // set by CMake

int main() {

	kaco::Core core;
	const uint8_t node_id = 10;

	auto callback = [] (const uint8_t node_id) {
		PRINT("New device! ID = "<<(unsigned)node_id);
	};
	core.nmt.register_new_device_callback(callback);

	bool success = core.start(BUSNAME, BAUDRATE);
	if (!success) {
		ERROR("Starting core failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	core.nmt.reset_all_nodes();

	std::this_thread::sleep_for(std::chrono::seconds(1));

	core.nmt.send_nmt_message(node_id,kaco::NMT::Command::start_node);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	// set digital output
	core.sdo.download(node_id, 0x6200, 0x1, 1, {0x7F});

	std::this_thread::sleep_for(std::chrono::seconds(1));

	// get device type (usually expedited transfer)
	std::vector<uint8_t> device_type = core.sdo.upload(node_id,0x1000,0x0);
	for (uint8_t device_type_byte : device_type) {
		DUMP_HEX(device_type_byte);
	}
	
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// get device name (usually segmented transfer)
	std::vector<uint8_t> device_name = core.sdo.upload(node_id,0x1008,0x0);
	std::string result(reinterpret_cast<char const*>(device_name.data()), device_name.size());
	PRINT("Device name: "<<result);

	std::this_thread::sleep_for(std::chrono::seconds(3));
	core.stop();

}
