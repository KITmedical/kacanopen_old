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

#include <string>
#include <vector>
#include <chrono>
#include <map>
#include <thread>
#include <memory>

#include "mapping.h"
#include "types.h"

namespace kaco {

	// forward declarations
	class Core;
	class Entry;

	struct TransmitPDOMapping {

		TransmitPDOMapping(Core& core, const std::map<std::string, Entry>& dictionary, uint16_t cob_id_,
			TransmissionType transmission_type_, std::chrono::milliseconds repeat_time_, const std::vector<Mapping>& mappings_);

		/// Stops the transmitter thread if there is one.
		~TransmitPDOMapping();

		Core& m_core;
		const std::map<std::string, Entry>& m_dictionary;

		uint16_t cob_id;
		TransmissionType transmission_type;
		std::chrono::milliseconds repeat_time;
		std::vector<Mapping> mappings;

		// this is a shared pointer because threads cannot be copied,
		// but TransmitPDOMapping is default-copy-constructed by std::vector.
		std::shared_ptr<std::thread> transmitter;

		void send() const;
		bool check_correctness() const;

		static const bool debug = false;

	};

} // end namespace kaco