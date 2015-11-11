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

#include <cassert>

#include "transmit_pdo_mapping.h"
#include "core.h"
#include "entry.h"
#include "logger.h"

namespace kaco {

TransmitPDOMapping::TransmitPDOMapping(Core& core, const std::map<std::string, Entry>& dictionary, uint16_t cob_id_,
			TransmissionType transmission_type_, std::chrono::milliseconds repeat_time_, const std::vector<Mapping>& mappings_)
	: m_core(core),
		m_dictionary(dictionary),
		cob_id(cob_id_),
		transmission_type(transmission_type_),
		repeat_time(repeat_time_),
		mappings(mappings_)
	{ }

TransmitPDOMapping::~TransmitPDOMapping() {
	if (transmitter) {
		transmitter->detach();
		if (!transmitter.unique()) {
			WARN("Transmitter thread has been copied and cannot be deleted now!");
		}
	}
}


void TransmitPDOMapping::send() const {

	std::vector<uint8_t> data(8,0);
	uint8_t max_byte=0;
	
	DEBUG_LOG("[TransmitPDOMapping::send] Sending transmit PDO with cob_id 0x"<<std::hex<<cob_id);

	for (const Mapping& mapping : mappings) {

		const Entry& entry = m_dictionary.at(mapping.entry_name);
		const Value& value = entry.get_value(mapping.array_index);
		const std::vector<uint8_t> bytes = value.get_bytes();
		assert(mapping.last_byte+1 - mapping.first_byte == bytes.size());

		DEBUG_LOG("[TransmitPDOMapping::send] Mapping:");
		DEBUG_DUMP(mapping.first_byte);
		DEBUG_DUMP(mapping.last_byte);
		DEBUG_DUMP(mapping.entry_name);
		DEBUG_DUMP_HEX(value);
		
		uint8_t count = 0;
		for (uint8_t i=mapping.first_byte; i<mapping.last_byte+1; ++i) {
			data[i] = bytes[count++];
		}

		max_byte = std::max(max_byte, mapping.last_byte);

	}

	data.resize(max_byte+1);
	m_core.pdo.send(cob_id, data);

}

bool TransmitPDOMapping::check_correctness() const {

	unsigned size = 0;
	std::vector<bool> byte_mapped(8,false);

	for (const Mapping& mapping : mappings) {

		if (mapping.first_byte > mapping.last_byte) {
			ERROR("[TransmitPDOMapping::check_correctness] first_byte > last_byte");
			return false;
		}

		if (mapping.first_byte > 7 || mapping.last_byte > 7) {
			ERROR("[TransmitPDOMapping::check_correctness] first_byte > 7 or last_byte > 7");
			return false;
		}

		if (m_dictionary.find(mapping.entry_name) == m_dictionary.end()) {
			ERROR("[TransmitPDOMapping::check_correctness] Dictionary entry \""<<mapping.entry_name<<"\" not available.");
			return false;
		}

		const Entry& entry = m_dictionary.at(mapping.entry_name);
		const uint8_t type_size = Utils::get_type_size(entry.type);

		if (mapping.last_byte+1-mapping.first_byte != type_size) {
			ERROR("[TransmitPDOMapping::check_correctness] PDO mapping has wrong size!");
			DUMP(type_size);
			DUMP(mapping.first_byte);
			DUMP(mapping.last_byte);
			return false;
		}

		if (mapping.array_index > 0 && !entry.is_array) {
			ERROR("[TransmitPDOMapping::check_correctness] array_index specified but entry \""<<mapping.entry_name<<"\" is no array.");
			return false;
		}

		for (uint8_t i=mapping.first_byte; i<mapping.last_byte; ++i) {
			if (byte_mapped[i]) {
				ERROR("[TransmitPDOMapping::check_correctness] Overlapping mappings at byte index "<<i);
				return false;
			}
			byte_mapped[i] = true;
		}

		size += mapping.last_byte - mapping.first_byte;

	}

	if (size > 8) {
		ERROR("[TransmitPDOMapping::check_correctness] size > 8");
		return false;
	}

	return true;

}

} // end namespace kaco