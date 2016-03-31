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

#include "message.h"

#include <vector>
#include <functional>

namespace kaco {
	
	// forward declaration
	class Core;

	/// \class NMT
	///
	/// This class implements the CanOpen NMT protocol
	class NMT {

	public:
		
		/// Type of a new device callback function
		typedef std::function< void(const uint8_t node_id) > NewDeviceCallback;

		/// NMT commands
		enum class Command : uint8_t {
			start_node = 0x01,
			stop_node = 0x02,
			enter_preoperational = 0x80,
			reset_node = 0x81,
			reset_communication = 0x82
		};

		/// Constructor.
		/// \param core Reference to the Core
		NMT(Core& core);

		/// Destructor
		~NMT();

		/// Process incoming NMT message.
		/// \param message The received CanOpen message.
		void process_incoming_message(const Message& message);

		/// Sends a NMT message to a given device
		/// \param node_id Node id of the device.
		/// \param cmd The NMT command.
		void send_nmt_message(uint8_t node_id, Command cmd);

		/// Sends a broadcast NMT message
		/// \param cmd The NMT command.
		void broadcast_nmt_message(Command cmd);

		/// Resets all nodes in the network.
		void reset_all_nodes();

		/// Discovers nodes in the network via node guard protocol.
		void discover_nodes();

		/// Registers a callback which will be called when a new slave device is discovered.
		/// \todo rename to device_alive_callback
		void register_new_device_callback(const NewDeviceCallback& callback);	

	private:

		static const bool debug = false;
		Core& m_core;

		/// \todo rename to device_alive_callback
		std::vector<NewDeviceCallback> m_new_device_callbacks;

	};

} // end namespace kaco