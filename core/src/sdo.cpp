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
 
#include "sdo.h"
#include "core.h"
#include "logger.h"
#include "sdo_error.h"

#include <iostream>
#include <chrono>
#include <cassert>
#include <future>

// Set by CMake:
// #define SDO_RESPONSE_TIMEOUT_MS ...

namespace kaco {

SDO::SDO(Core& core) 
	: m_core(core)
	{ }

SDO::~SDO() 
	{ }

void SDO::download(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t size, const std::vector<uint8_t>& data) {

	assert(size>0);
	assert(data.size()>=size);

	if (size<=4) {
		
		// expedited transfer

		uint8_t command = Flag::initiate_download_request
							| size_flag(size)
							| Flag::size_indicated
							| Flag::expedited_transfer;

		SDOResponse response;
		send_sdo_and_wait(command, node_id, index, subindex,
			data[0], data[1], data[2], data[3], response);

		if (response.failed()) {
			throw sdo_error(response.get_data());
		}

	} else {

		// segmented transfer
		throw sdo_error(sdo_error::type::segmented_download);

	}

}

std::vector<uint8_t> SDO::upload(uint8_t node_id, uint16_t index, uint8_t subindex) {
	
	std::vector<uint8_t> result;

	uint8_t command = Flag::initiate_upload_request;
	SDOResponse response;
	send_sdo_and_wait(command, node_id, index, subindex,
		0, 0, 0, 0, response);

	if (response.failed()) {
		throw sdo_error(response.get_data());
	}

	if (response.command & Flag::expedited_transfer) {

		for (unsigned i=0; i<response.get_length(); ++i) {
			result.push_back(response.data[3+i]);
		}

	} else {

		if ((response.command & Flag::size_indicated)==0) {
			throw sdo_error(sdo_error::type::response_command,"Command "+std::to_string(response.command)+" is reserved for further use.");
		}

		// TODO: the &0xFF is necessary (tested with sysWOORXX IO-X1) but I haven't found this restriction in CiA301...
		uint32_t original_size = response.get_data() & 0xFFFF;
		uint32_t size = original_size;
		bool more_segments = true;
		uint8_t toggle_bit = 0;

		// request data
		while (more_segments) {

			if (size==0) {
				WARN("[SDO::upload] [Restrictive] Uploaded already all "<<original_size<<" bytes but there are still more segments. Ignore...");
				return result;
			}

			uint8_t command = Flag::upload_segment_request
								| toggle_bit;
			SDOResponse response;
			send_sdo_and_wait(command, node_id, 0, 0, 0, 0, 0, 0, response);

			if (response.failed()) {
				throw sdo_error(response.get_data());
			}

			if (toggle_bit != (response.command & Flag::toggle_bit)) {
				throw sdo_error(sdo_error::type::response_toggle_bit);
			}

			unsigned i=0;
			while(i<7 && size>0) {
				result.push_back(response.data[i]);
				++i;
				--size;
			}

			toggle_bit = (toggle_bit>0) ? 0 : Flag::toggle_bit;
			more_segments = (response.command & Flag::no_more_segments)==0;

		}

		if (size>0) {
			WARN("[SDO::upload] [Restrictive] Uploaded just "<<(original_size-size)<<" of "<<original_size<<" bytes but there are no more segments. Ignore...");
			return result;
		}

	}

	return result;

}

void SDO::process_incoming_message(const Message& message) {

	SDOResponse response;
	response.node_id = message.get_node_id();
	response.command = message.data[0];
	
	for (unsigned i=0; i<7; ++i) {
		response.data[i] = message.data[1+i];
	}

	DEBUG_LOG("Received SDO (transmit/server) from node "<<(unsigned)response.node_id);

	// call registered callbacks
	bool found_callback = false;

	{
		std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
		for (const SDOReceivedCallback& callback : m_receive_callbacks) {
			if (callback.node_id == response.node_id) {
				found_callback = true;
				// This is not async because callbacks are only registered internally.
				// and it cannot be async because response is taken by reference
				callback.callback(response);
			}
		}
	}

	if (!found_callback) {
		DEBUG_LOG("Received unassigned SDO (transmit/server)");
		DEBUG(response.print();)
	}

}

void SDO::send_sdo_and_wait(uint8_t command, uint8_t node_id, uint16_t index, uint8_t subindex,
	uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3,
	SDOResponse& response) {
	
	std::lock_guard<std::mutex> scoped_lock(m_send_and_wait_mutex);

	std::promise<void> received_promise;
	std::future<void> received_future = received_promise.get_future();

	SDOReceivedCallback receiver = { node_id, [&] (const SDOResponse& _response) {
		//f (_response.node_id == node_id) { //&& _response.get_index() == index && _response.get_subindex() == subindex) {
			// We should not check for index/subindex because this fails for segmented transfer.
			// TODO: check for correct server command specifier?
			response = _response;
			received_promise.set_value();
		//}
	} };
	
	std::list<SDOReceivedCallback>::const_iterator receiver_handle;
	{
		std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
		m_receive_callbacks.push_front(std::move(receiver));
		receiver_handle = m_receive_callbacks.cbegin();
	}

	// send message
	Message message;
	message.cob_id = 0x600+node_id;
	message.rtr = false;
	message.len = 8;
	message.data[0] = command;
	message.data[1] = index & 0xFF;
	message.data[2] = (index >> 8) & 0xFF;
	message.data[3] = subindex;
	message.data[4] = byte0;
	message.data[5] = byte1;
	message.data[6] = byte2;
	message.data[7] = byte3;
	m_core.send(message);

	const auto timeout = std::chrono::milliseconds(SDO_RESPONSE_TIMEOUT_MS);
	const auto status = received_future.wait_for(timeout);
	
	if (status == std::future_status::timeout) {
		throw sdo_error(sdo_error::type::response_timeout, "Timeout was "+std::to_string(timeout.count())+"ms.");
	}

	{
		std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
		// std::list iterators to existing elements are never invalidated
		m_receive_callbacks.erase(receiver_handle);
	}

}

uint8_t SDO::size_flag(uint8_t size) {
	assert(size>0);
	assert(size<=4);
	switch(size) {
		case 1: return 0x0C;
		case 2: return 0x08;
		case 3: return 0x04;
		case 4: return 0x00;
	}
	assert(false && "Dead code!");
	return 0;
}

}
