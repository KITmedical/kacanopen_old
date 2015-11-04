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
 
#include "device.h"
#include "core.h"
#include "utils.h"

#include <cassert>

namespace kaco {

Device::Device(Core& core, uint8_t node_id)
	: m_core(core), m_node_id(node_id) {

		for (const auto& entry : profile301) {
			m_dictionary[entry.name] = entry;
		}

	}

Device::~Device() 
	{ }

void Device::start() {
	m_core.nmt.send_nmt_message(m_node_id,NMT::command::start_node);
}

value_type Device::get_entry_via_sdo(uint32_t index, uint8_t subindex, data_type type) {
	
	std::vector<uint8_t> data = m_core.sdo.upload(m_node_id, index, subindex);

	switch(type) {
			
		case data_type::uint8: {
			uint8_t val = data[0];
			return value_type(val);
		}
			
		case data_type::uint16: {
			uint16_t val = data[0] + (data[1]<<8);
			return value_type(val);
		}
			
		case data_type::uint32: {
			uint32_t val = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24);
			return value_type(val);
		}
			
		case data_type::int8: {
			int8_t val = data[0];
			return value_type(val);
		}
			
		case data_type::int16: {
			int16_t val = data[0] + (data[1]<<8);
			return value_type(val);
		}
			
		case data_type::int32: {
			int32_t val = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24);
			return value_type(val);
		}

		case data_type::string: {
			// TODO: check correct encoding
		    std::string val(reinterpret_cast<char const*>(data.data()), data.size());
		    return value_type(val);
		}

		default: {
			LOG("Unknown data type.");
			return value_type();
		}

	}

}

value_type Device::get_entry(std::string name, uint8_t array_index) {
	
	if (m_dictionary.find(name) == m_dictionary.end()) {
		LOG("Dictionary entry \""<<name<<"\" not available.");
		return value_type();
	}

	entry_type& entry = m_dictionary[name];

	if (entry.sdo_on_read) {
		
		LOG("update_on_read.");

		uint8_t subindex = (entry.is_array) ? 0x1+array_index : entry.subindex;
		entry.value = get_entry_via_sdo(entry.index, subindex, entry.type);
		entry.valid = true;

	}

	return entry.value;

}

void Device::set_entry_via_sdo(uint32_t index, uint8_t subindex, const value_type& value) {

	switch(value.type) {
			
		case data_type::uint8:
		case data_type::int8: {
			uint8_t byte0 = value.uint8;
			m_core.sdo.download(m_node_id,index,subindex,1,{byte0});
			break;
		}

		case data_type::uint16:
		case data_type::int16: {
			uint8_t byte0 = value.uint16 & 0xFF;
			uint8_t byte1 = (value.uint16 >> 8) & 0xFF;
			m_core.sdo.download(m_node_id,index,subindex,2,{byte0,byte1});
			break;
		}

		case data_type::uint32:
		case data_type::int32: {
			uint8_t byte0 = value.uint32 & 0xFF;
			uint8_t byte1 = (value.uint32 >> 8) & 0xFF;
			uint8_t byte2 = (value.uint32 >> 16) & 0xFF;
			uint8_t byte3 = (value.uint32 >> 24) & 0xFF;
			m_core.sdo.download(m_node_id,index,subindex,4,{byte0,byte1,byte2,byte3});
			break;
		}

		case data_type::string: {
			LOG("Strings not yet supported.");
			break;
		}

		default: {
			LOG("Unknown data type.");
			break;
		}

	}

}

void Device::set_entry(std::string name, const value_type& value, uint8_t array_index) {
	
	if (m_dictionary.find(name) == m_dictionary.end()) {
		LOG("Dictionary entry \""<<name<<"\" not available.");
		return;
	}

	entry_type& entry = m_dictionary[name];

	if (value.type != entry.type) {
		LOG("You passed a value of wrong type: "<<Utils::type_to_string(value.type));
		LOG("Dictionary entry \""<<name<<"\" must be of type "<<Utils::type_to_string(entry.type)<<".");
		return;
	}

	entry.value = value;
	entry.valid = true;

	if (entry.sdo_on_write) {
		
		LOG("update_on_write.");

		uint8_t subindex = (entry.is_array) ? 0x1+array_index : entry.subindex;
		set_entry_via_sdo(entry.index, subindex, value);

	}

}


} // end namespace kaco