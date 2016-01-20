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
 
#include "nmt.h"
#include "core.h"
#include "logger.h"

#include <iostream>
#include <cstdint>
#include <future>

namespace kaco {

NMT::NMT(Core& core) 
	: m_core(core)
	{ }

NMT::~NMT() 
	{ }

void NMT::send_nmt_message(uint8_t node_id, Command cmd) {
	DEBUG_LOG("Set NMT state of "<<(unsigned)node_id<<" to "<<static_cast<uint32_t>(cmd));
	Message message = { 0x0000, false, 2, {static_cast<uint8_t>(cmd),node_id,0,0,0,0,0,0} };
	m_core.send(message);
}

void NMT::broadcast_nmt_message(Command cmd) {
	send_nmt_message(0, cmd);
}

void NMT::reset_all_nodes() {
	broadcast_nmt_message(Command::reset_node);
}

void NMT::process_incoming_message(const Message& message) {

	DEBUG_LOG("NMT Error Control message from node "
		<<(unsigned)message.get_node_id()<<".");
	
	uint8_t data = message.data[0];
	bool toggle_bit = data>>7;
	uint8_t state = data&0x3F;

	//DEBUG_DUMP(toggle_bit);

	switch (state) {
		
		case 0: {
			DEBUG_LOG("New state is Initialising");

			for (const auto& callback : m_new_device_callbacks) {
				DEBUG_LOG("Calling new device callback (async)");
				std::async(std::launch::async, callback, message.get_node_id());
			}

			break;
		}
		
		case 1: {
			DEBUG_LOG("New state is Disconnected");
			break;
		}
		
		case 2: {
			DEBUG_LOG("New state is Connecting");
			break;
		}
		
		case 3: {
			DEBUG_LOG("New state is Preparing");
			break;
		}
		
		case 4: {
			DEBUG_LOG("New state is Stopped");
			break;
		}
		
		case 5: {
			DEBUG_LOG("New state is Operational");
			break;
		}
		
		case 127: {
			DEBUG_LOG("New state is Pre-operational");
			break;
		}
		
		default: {
			DEBUG_LOG("New state is unknown: "<<(unsigned)state);
			break;
		}

	}

}

void NMT::register_new_device_callback(const NewDeviceCallback& callback) {
	m_new_device_callbacks.push_back(callback);
}


} // end namespace kaco