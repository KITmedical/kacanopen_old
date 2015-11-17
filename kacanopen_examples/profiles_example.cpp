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

#include "master.h"
#include "logger.h"

int main(int argc, char** argv) {

	kaco::Master master;
	bool success = master.start();
	if (!success)
		return false;

	std::this_thread::sleep_for(std::chrono::seconds(1));

	PRINT(master.get_devices().size());

	if (master.get_devices().size()<1)
		return EXIT_FAILURE;

	kaco::Device& device = master.get_devices()[0];

	device.start();

	DUMP(device.get_entry("manufacturer_device_name"));

	// TODO: The following is only for testing. write_output etc. is CiA401 and should not be available here.

	device.add_receive_pdo_mapping(0x188, "read_digital_input", 0, 0); // offest 0, array index 0
	device.add_receive_pdo_mapping(0x188, "read_digital_input", 1, 1); // offset 1, array index 1

	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::sdo));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::sdo));

	std::this_thread::sleep_for(std::chrono::seconds(1));
	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));
	
	device.set_entry("write_output", (uint8_t)0xF0, 0);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));

	device.set_entry("write_output", (uint8_t)0xF1, 0);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));
	
	device.set_entry("write_output", (uint8_t)0xF2, 0);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));
	
	device.set_entry("write_output", (uint8_t)0xF3, 0);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));
	
	device.set_entry("write_output", (uint8_t)0xF4, 0);

	std::this_thread::sleep_for(std::chrono::seconds(1));
	DUMP_HEX(device.get_entry("read_digital_input",0,kaco::ReadAccessMethod::cache));
	DUMP_HEX(device.get_entry("read_digital_input",1,kaco::ReadAccessMethod::cache));
	
	device.set_entry("write_output", (uint8_t)0xF5, 0);

	std::this_thread::sleep_for(std::chrono::seconds(4));

	master.stop();

}
