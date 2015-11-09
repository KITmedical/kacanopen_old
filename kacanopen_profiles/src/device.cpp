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
#include "logger.h"

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
	m_core.nmt.send_nmt_message(m_node_id,NMT::Command::start_node);
}

Value Device::get_entry_via_sdo(uint32_t index, uint8_t subindex, Type type) {
	
	std::vector<uint8_t> data = m_core.sdo.upload(m_node_id, index, subindex);

	switch(type) {
			
		case Type::uint8: {
			uint8_t val = data[0];
			return Value(val);
		}
			
		case Type::uint16: {
			uint16_t val = data[0] + (data[1]<<8);
			return Value(val);
		}
			
		case Type::uint32: {
			uint32_t val = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24);
			return Value(val);
		}
			
		case Type::int8: {
			int8_t val = data[0];
			return Value(val);
		}
			
		case Type::int16: {
			int16_t val = data[0] + (data[1]<<8);
			return Value(val);
		}
			
		case Type::int32: {
			int32_t val = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24);
			return Value(val);
		}

		case Type::string: {
			// TODO: check correct encoding
		    std::string val(reinterpret_cast<char const*>(data.data()), data.size());
		    return Value(val);
		}

		default: {
			ERROR("[Device::get_entry_via_sdo] Unknown data type.");
			return Value();
		}

	}

}

const Value& Device::get_entry(std::string name, uint8_t array_index, AccessMethod access_method) {
	
	if (m_dictionary.find(name) == m_dictionary.end()) {
		ERROR("[Device::get_entry] Dictionary entry \""<<name<<"\" not available.");
		return m_dummy_value;
	}

	Entry& entry = m_dictionary[name];

	if (access_method==AccessMethod::sdo || (access_method==AccessMethod::use_default && entry.sdo_on_read)) {
		
		DEBUG_LOG("[Device::get_entry] update_on_read.");

		uint8_t subindex = (entry.is_array) ? 0x1+array_index : entry.subindex;
		entry.set_value(get_entry_via_sdo(entry.index, subindex, entry.type), array_index);

	}

	return entry.get_value(array_index);

}

void Device::set_entry_via_sdo(uint32_t index, uint8_t subindex, const Value& value) {

	switch(value.type) {
			
		case Type::uint8:
		case Type::int8: {
			uint8_t byte0 = value.uint8;
			m_core.sdo.download(m_node_id,index,subindex,1,{byte0});
			break;
		}

		case Type::uint16:
		case Type::int16: {
			uint8_t byte0 = value.uint16 & 0xFF;
			uint8_t byte1 = (value.uint16 >> 8) & 0xFF;
			m_core.sdo.download(m_node_id,index,subindex,2,{byte0,byte1});
			break;
		}

		case Type::uint32:
		case Type::int32: {
			uint8_t byte0 = value.uint32 & 0xFF;
			uint8_t byte1 = (value.uint32 >> 8) & 0xFF;
			uint8_t byte2 = (value.uint32 >> 16) & 0xFF;
			uint8_t byte3 = (value.uint32 >> 24) & 0xFF;
			m_core.sdo.download(m_node_id,index,subindex,4,{byte0,byte1,byte2,byte3});
			break;
		}

		case Type::string: {
			ERROR("[Device::set_entry_via_sdo] Strings not yet supported.");
			break;
		}

		default: {
			ERROR("[Device::set_entry_via_sdo] Unknown data type.");
			break;
		}

	}

}

void Device::set_entry(std::string name, const Value& value, uint8_t array_index, AccessMethod access_method) {
	
	if (m_dictionary.find(name) == m_dictionary.end()) {
		ERROR("[Device::set_entry] Dictionary entry \""<<name<<"\" not available.");
		return;
	}

	Entry& entry = m_dictionary[name];

	if (value.type != entry.type) {
		ERROR("[Device::set_entry] You passed a value of wrong type: "<<Utils::type_to_string(value.type));
		ERROR("[Device::set_entry] Dictionary entry \""<<name<<"\" must be of type "<<Utils::type_to_string(entry.type)<<".");
		return;
	}

	entry.set_value(value, array_index);

	if (access_method==AccessMethod::sdo || (access_method==AccessMethod::use_default && entry.sdo_on_write)) {
		
		DEBUG_LOG("[Device::set_entry] update_on_write.");

		const uint8_t subindex = (entry.is_array) ? 0x1+array_index : entry.subindex;
		set_entry_via_sdo(entry.index, subindex, value);

	}

}

void Device::add_pdo_mapping(uint16_t cob_id, std::string entry_name, uint8_t first_byte, uint8_t last_byte, uint8_t array_index) {
	
	Entry& entry = m_dictionary[entry_name];
	const uint8_t type_size = Utils::get_type_size(entry.type);

	if (last_byte+1-first_byte != type_size) {
		ERROR("[Device::add_pdo_mapping] PDO mapping has wrong size!");
		DUMP(type_size);
		DUMP(first_byte);
		DUMP(last_byte);
	}

	m_pdo_mappings.push_back({cob_id,entry_name,first_byte,last_byte,array_index});

	// TODO: this only works while add_pdo_received_callback takes callback by value.
	auto binding = std::bind(&Device::pdo_received_callback, this, m_pdo_mappings.back(), std::placeholders::_1);
	m_core.pdo.add_pdo_received_callback(cob_id, binding);

}

void Device::pdo_received_callback(const PDOMapping& mapping, std::vector<uint8_t> data) {
	Entry& entry = m_dictionary[mapping.entry_name];
	const uint8_t array_index = mapping.array_index;
	const uint8_t first_byte = mapping.first_byte;
	const uint8_t last_byte = mapping.last_byte;

	if (data.size() <= first_byte || data.size() <= last_byte) {
		ERROR("[Device::pdo_received_callback] PDO has wrong size!");
		DUMP(data.size());
		DUMP(first_byte);
		DUMP(last_byte);
	}

	DEBUG_LOG("Updating entry "<<entry.name<<"...");
	DEBUG_DUMP(array_index);

	switch(entry.type) {
			
		case Type::uint8: {
			entry.set_value(Value((uint8_t)data[first_byte+0]), array_index);
			break;
		}
			
		case Type::int8: {
			entry.set_value(Value((int8_t)data[first_byte+0]), array_index);
			break;
		}

		case Type::uint16: {
			entry.set_value(Value((uint16_t)data[first_byte+0] + ((uint16_t)data[first_byte+1] << 8)), array_index);
			break;
		}

		case Type::int16: {
			entry.set_value(Value((int16_t)data[first_byte+0] + ((int16_t)data[first_byte+1] << 8)), array_index);
			break;
		}

		case Type::uint32: {
			entry.set_value(Value((uint32_t)data[first_byte+0] + ((uint32_t)data[first_byte+1] << 8) + ((uint32_t)data[first_byte+2] << 16) + ((uint32_t)data[first_byte+3] << 24)), array_index);
		}

		case Type::int32: {
			entry.set_value(Value((int32_t)data[first_byte+0] + ((int32_t)data[first_byte+1] << 8) + ((int32_t)data[first_byte+2] << 16) + ((int32_t)data[first_byte+3] << 24)), array_index);
			break;
		}

		case Type::string: {
			ERROR("[Device::pdo_received_callback] Mapping PDOs to string typed entries is not supported!");
			break;
		}

		default: {
			ERROR("[Device::pdo_received_callback] Unknown data type.");
			break;
		}

	}

}


} // end namespace kaco