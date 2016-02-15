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
 
#pragma once

#include <functional>
#include <vector>

#include "message.h"
#include "sdo_response.h"

namespace kaco {
	
	// forward declaration
	class Core;

	class SDO {

	public:

		//! type of a sdo message receiver function
		struct SDOReceivedCallback {
			typedef std::function< void(const SDOResponse&) > sdo_callback_function_type;
			uint8_t node_id;
			sdo_callback_function_type callback;
		};

		SDO(Core& core);
		~SDO();
		
		void download(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t size, const std::vector<uint8_t>& bytes);
		
		std::vector<uint8_t> upload(uint8_t node_id, uint16_t index, uint8_t subindex);

		void process_incoming_message(const Message& message);

		bool send_sdo_and_wait(uint8_t command, uint8_t node_id, uint16_t index, uint8_t subindex,
			uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3,
			SDOResponse& response);

	private:

		enum Flag : uint8_t {

			// client command specifiers
			initiate_download_request = 0x20,
			download_segment_request = 0x00,
			initiate_upload_request = 0x40,
			upload_segment_request = 0x60,

			// server command specifiers
			initiate_download_response = 0x60,
			download_segment_response = 0x20,
			initiate_upload_response = 0x40,
			upload_segment_response = 0x00,

			toggle_bit = 0x10,
			no_more_segments = 0x01,
			size_indicated = 0x01,
			expedited_transfer = 0x02,

			error = 0x80

		};

		static const bool debug = true;
		static const uint64_t response_timeout_ms = SDO_RESPONSE_TIMEOUT_MS;
		
		Core& m_core;
		std::vector<SDOReceivedCallback> m_receive_callbacks;
		
		uint8_t size_flag(uint8_t size);

	};

} // end namespace kaco