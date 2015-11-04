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

#include "defines.h"
#include "message_type.h"

#include <vector>
#include <functional>

namespace kaco {
	
	// forward declaration
	class Core;

	class NMT {

	public:
		
		//! type of a message receiver function
		typedef std::function< void(const uint8_t node_id) > new_device_callback_type;

		enum class command : uint8_t {
			start_node = 0x01,
			stop_node = 0x02,
			enter_preoperational = 0x80,
			reset_node = 0x81,
			reset_communication = 0x82
		};

		NMT(Core& core);
		~NMT();

		void process_incoming_message(const message_type& message);
		void send_nmt_message(uint8_t node_id, command cmd);
		void broadcast_nmt_message(command cmd);
		void reset_all_nodes();

		void register_new_device_callback(const new_device_callback_type& callback);	

	private:

		static const bool debug = true;
		Core& m_core;
		std::vector<new_device_callback_type> m_new_device_callbacks;

	};

} // end namespace kaco