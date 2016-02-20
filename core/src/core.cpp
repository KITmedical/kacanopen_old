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
 
#include <iostream>
#include <thread>
#include <chrono>
#include <future>
#include <string>

#include "logger.h"
#include "core.h"

namespace kaco {

// linkage to driver

extern "C" uint8_t canReceive_driver(CANHandle, Message *);
extern "C" uint8_t canSend_driver(CANHandle, Message const *);
extern "C" CANHandle canOpen_driver(CANBoard *);
extern "C" int32_t canClose_driver(CANHandle);
extern "C" uint8_t canChangeBaudRate_driver(CANHandle, char *);

Core::Core()
	: nmt(*this),
		sdo(*this),
		pdo(*this)
	{ }
	
Core::~Core() {
	if (m_running) {
		stop();
	}
}

bool Core::start(const std::string busname, unsigned baudrate) {

    CANBoard board = {busname.c_str(), std::to_string(baudrate).c_str()} ;
    m_handle = canOpen_driver(&board);

    if(!m_handle) {
        ERROR("Cannot open the CANOpen device.");
        return false;
    }

	m_running = true;
	m_loop_thread = std::thread(&Core::receive_loop, this, std::ref(m_running));
	return true;

}

void Core::stop() {
	
	m_running = false;
	m_loop_thread.detach();

	DEBUG_LOG("Calling canClose.");
	canClose_driver(m_handle);

}

void Core::receive_loop(std::atomic<bool>& running) {

	Message message;

	while (running) {

		canReceive_driver(m_handle, &message);
		received_message(message);

	}

}

void Core::register_receive_callback(const MessageReceivedCallback& callback) {
	m_receive_callbacks.push_back(callback);
}

void Core::received_message(const Message& message) {

	DEBUG_LOG(" ");
	DEBUG_LOG("Received message:");

	// first call registered callbacks
	for (const MessageReceivedCallback& callback : m_receive_callbacks) {
		std::async(std::launch::async, callback, message);
	}

	// sencondly process known message types
	switch (message.get_function_code()) {
		
		case 0: {
			DEBUG_LOG("NMT Module Control");
			DEBUG(message.print();)
			break;
		}

		case 1: {
			DEBUG_LOG("Sync or Emergency");
			DEBUG(message.print();)
			break;
		}

		case 2: {
			DEBUG_LOG("Time stamp");
			DEBUG(message.print();)
			break;
		}

		case 3:
		case 5:
		case 7:
		case 9: {
			pdo.process_incoming_message(message);
			break;
		}
		
		case 4:
		case 6:
		case 8:
		case 10: {
			DEBUG_LOG("PDO receive");
			DEBUG(message.print();)
			break;
		}
		
		case 11: {
			sdo.process_incoming_message(message);
			break;
		}
		
		case 12: {
			DEBUG_LOG("SDO (receive/client)");
			break;
		}
		
		case 14: {
			// NMT Error Control
			nmt.process_incoming_message(message);
			break;
		}

		default: {
			DEBUG_LOG("Unknown message:");
			DEBUG(message.print();)
			break;
		}

	}

	DEBUG_LOG(" ");

}

void Core::send(const Message& message) {
	
	DEBUG_LOG_EXHAUSTIVE("Sending message:");
	DEBUG_EXHAUSTIVE(message.print();)
	canSend_driver(m_handle, &message);

}

} // namespace co