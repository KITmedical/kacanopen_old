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
#include <cstdint>
#include <chrono>

#include "master.h"
#include "logger.h"

int main() {

	kaco::Master master;
	bool success = master.start(BUSNAME, BAUDRATE);
	if (!success) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	if (master.num_devices()<1) {
		ERROR("No devices found.");
		return EXIT_FAILURE;
	}

	size_t index;
	bool found = false;
	for (size_t i=0; i<master.num_devices(); ++i) {
		kaco::Device& device = master.get_device(i);
		device.start();
		if (device.get_device_profile_number()==401) {
			index = i;
			found = true;
		}
	}

	if (!found) {
		ERROR("This example is intended for use with a CiA 401 device but I can't find one.");
		return EXIT_FAILURE;
	}

	kaco::Device& device = master.get_device(index);

	success = device.load_dictionary_from_library();
	if (!success) {
		ERROR("Specializing device failed.");
		return EXIT_FAILURE;
	}

	const auto start = std::chrono::steady_clock::now();
	uint8_t out = 0;

	while ((std::chrono::steady_clock::now()-start)<std::chrono::seconds(10)) {
		device.set_entry("Write output 8-bit/Digital Outputs 1-8", out++, 0, kaco::WriteAccessMethod::sdo);
		device.get_entry("Read input 8-bit/Digital Inputs 1-8",0,kaco::ReadAccessMethod::sdo);
	}

}
