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

	/// \class SDO
	///
	/// This class implements the CanOpen SDO protocol
	///
	/// \todo Add add_client_sdo_callback(SDOReceivedCallback) (callback signature void(const SDOResponse&)) and/or
	///		add_request_callback(node_id, SDORequestCallback) (listening only for SDOs with slave's own node_id, callback signature void(index, subindex)).
	/// \todo Add send_response(node_id, index, subindex, vector<uint8_t> data) (chooses segmented/expedited transfer on it's own).
	/// \todo Add abort_transfer(node_id, index, subindex, errorcode).
	class SDO {

	public:

		/// Type of a sdo message receiver function with it's node id
		struct SDOReceivedCallback {

			/// Type of the callback
			typedef std::function< void(const SDOResponse&) > sdo_callback_function_type;
			
			/// Node id
			uint8_t node_id;

			/// The callback
			sdo_callback_function_type callback;
			
		};

		/// Constructor
		/// \param core Reference to the Core
		SDO(Core& core);

		/// Destructor
		~SDO();
		
		/// SDO download: Write value into remote device's object dictionary.
		/// \param node_id Node id of remote device
		/// \param index Dictionary index
		/// \param subindex Subindex
		/// \param size Size of the entry/value in bytes
		/// \param bytes Vector containing the data bytes of the value in little-endian order. Vector size must be equal to size argument.
		void download(uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t size, const std::vector<uint8_t>& bytes);
		
		/// SDO download: Get value from remote device's object dictionary.
		/// \param node_id Node id of remote device
		/// \param index Dictionary index
		/// \param subindex Subindex
		/// \returns Vector containing the data bytes of the value in little-endian order.
		std::vector<uint8_t> upload(uint8_t node_id, uint16_t index, uint8_t subindex);

		/// Process incoming SDO message.
		/// \param message The received CanOpen message.
		/// \todo Rename this to process_incoming_server_sdo() and add process_incoming_client_sdo()
		void process_incoming_message(const Message& message);

		/// Sends an SDO message and waits for the response.
		/// \param command SDO command specifier
		/// \param node_id Node id of remote device
		/// \param index Dictionary index
		/// \param subindex Subindex
		/// \param byte0 first data byte (little endian!)
		/// \param byte1 second data byte
		/// \param byte2 third data byte
		/// \param byte3 fourth data byte
		/// \param response Will contain the response.
		/// \todo Make response a return value.
		/// \todo Make byte0 - byte3 arguments an array
		void send_sdo_and_wait(uint8_t command, uint8_t node_id, uint16_t index, uint8_t subindex,
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
		
		Core& m_core;

		/// \todo Rename to m_server_sdo_callbacks and add m_client_sdo_callbacks.
		std::vector<SDOReceivedCallback> m_receive_callbacks;

		uint8_t size_flag(uint8_t size);

	};

} // end namespace kaco