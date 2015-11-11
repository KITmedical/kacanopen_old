/*
 * Copyright (c) 2015, Thomas Keh
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
#include <cstdint>

#include "master.h"
#include "logger.h"

int main(int argc, char** argv) {

	PRINT("This example runs a counter completely without SDO transfers.");
	PRINT("The CiA 401 device must be configured to receive 'write_output' via RPDO1 and to send 'read_digital_input' via TPDO1.");

	kaco::Master master;
	bool success = master.start();
	if (!success) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	if (master.get_devices().size()<1) {
		ERROR("No devices found.");
		return EXIT_FAILURE;
	}

	kaco::Device& device = master.get_devices()[0];
	device.start();

	DUMP(device.get_entry("manufacturer_device_name"));

	device.add_receive_pdo_mapping(0x188, "read_digital_input", 0, 0, 0); // index 0
	device.add_receive_pdo_mapping(0x188, "read_digital_input", 1, 1, 1); // index 1
	
	// transmit PDO on change
	device.add_transmit_pdo_mapping(0x208, {{"write_output", 0, 0, 0}});

	// transmit PDO every 500ms
	//device.add_transmit_pdo_mapping(0x208, {{"write_output", 0, 0, 0}}, kaco::TransmissionType::PERIODIC, std::chrono::milliseconds(500));
	
	for (uint8_t i=0; i<10; ++i) {
		
		PRINT("Set output to 0x"<<std::hex<<i<<" (via cache!) and wait 1 second");
		device.set_entry("write_output", i, 0,kaco::WriteAccessMethod::cache);
		std::this_thread::sleep_for(std::chrono::seconds(1));

		DUMP_HEX(device.get_entry("write_output",0,kaco::ReadAccessMethod::cache));
		DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
		DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));

	}

	master.stop();

}
