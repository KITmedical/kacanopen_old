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
 
#include "pdo.h"

 #include <iostream>

namespace kaco {

PDO::PDO(Core& core) 
	: m_core(core)
	{ }

PDO::~PDO() 
	{ }

void PDO::process_incoming_message(const message_type& message) {

	uint8_t node_id = message.get_node_id();
	std::vector<uint8_t> data;

	DEBUG("Received transmit PDO from node "<<(unsigned)node_id);

	for (unsigned i=0; i<message.len; ++i) {
		data.push_back(message.data[i]);
	}

	// call registered callbacks
	bool found_callback = false;
	for (const pdo_callback_type& callback : m_receive_callbacks) {
		if (callback.node_id == node_id) {
			found_callback = true;
			// This is not async because callbacks are only registered internally.
			callback.callback(data);
		}
	}

	if (!found_callback && debug) {
		LOG("PDO is unassigned. Here is the data (LSB):");
		for (unsigned i=0; i<data.size(); ++i) {
			std::cout << std::hex << (unsigned)data[i] << " ";
		}
		std::cout << std::endl;
	}

}

} // end namespace kaco
