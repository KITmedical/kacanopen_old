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

#include "cia_301.h"
#include "cia_401.h"
#include "cia_402.h"

#include <cassert>
#include <algorithm>

namespace kaco {

Device::Device(Core& core, uint8_t node_id)
	: m_core(core), m_node_id(node_id) {

		for (const auto& entry : cia_301::dictionary) {
			m_dictionary[entry.name] = entry;
		}

	}

Device::~Device() 
	{ }

void Device::start() {
	m_core.nmt.send_nmt_message(m_node_id,NMT::Command::start_node);
}

uint8_t Device::get_node_id() const {
	return m_node_id;
}

Value Device::get_entry_via_sdo(uint32_t index, uint8_t subindex, Type type) {
	
	std::vector<uint8_t> data = m_core.sdo.upload(m_node_id, index, subindex);
	return std::move(Value(type, data));

}

const Value& Device::get_entry(std::string name, uint8_t array_index, ReadAccessMethod access_method) {
	
	if (m_dictionary.find(name) == m_dictionary.end()) {
		ERROR("[Device::get_entry] Dictionary entry \""<<name<<"\" not available.");
		return m_dummy_value;
	}

	Entry& entry = m_dictionary[name];

	if (access_method==ReadAccessMethod::sdo || (access_method==ReadAccessMethod::use_default && entry.read_access_method==ReadAccessMethod::sdo)) {
		
		DEBUG_LOG("[Device::get_entry] update_on_read.");

		uint8_t subindex = (entry.is_array) ? 0x1+array_index : entry.subindex;
		entry.set_value(get_entry_via_sdo(entry.index, subindex, entry.type), array_index);

	}

	return entry.get_value(array_index);

}

Type Device::get_entry_type(std::string name) {

	if (m_dictionary.find(name) == m_dictionary.end()) {
		ERROR("[Device::get_entry_type] Dictionary entry \""<<name<<"\" not available.");
		return Type::invalid;
	}

	return m_dictionary[name].get_type();

}

void Device::set_entry_via_sdo(uint32_t index, uint8_t subindex, const Value& value) {

	const auto& bytes = value.get_bytes();
	m_core.sdo.download(m_node_id,index,subindex,bytes.size(),bytes);

}

void Device::set_entry(std::string name, const Value& value, uint8_t array_index, WriteAccessMethod access_method) {
	
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

	if (access_method==WriteAccessMethod::sdo || (access_method==WriteAccessMethod::use_default && entry.write_access_method==WriteAccessMethod::sdo)) {
		
		DEBUG_LOG("[Device::set_entry] update_on_write.");

		const uint8_t subindex = (entry.is_array) ? 0x1+array_index : entry.subindex;
		set_entry_via_sdo(entry.index, subindex, value);

	}

}

void Device::add_receive_pdo_mapping(uint16_t cob_id, const std::string& entry_name, uint8_t offset, uint8_t array_index) {
	
	if (m_dictionary.find(entry_name) == m_dictionary.end()) {
		ERROR("[Device::add_receive_pdo_mapping] Dictionary entry \""<<entry_name<<"\" not available.");
		return;
	}
	
	Entry& entry = m_dictionary[entry_name];
	const uint8_t type_size = Utils::get_type_size(entry.type);

	if (offset+type_size > 8) {
		ERROR("[Device::add_receive_pdo_mapping] offset+type_size > 8.");
		DUMP(type_size);
		DUMP(offset);
	}

	m_receive_pdo_mappings.push_back({cob_id,entry_name,offset,array_index});

	// TODO: this only works while add_pdo_received_callback takes callback by value.
	auto binding = std::bind(&Device::pdo_received_callback, this, m_receive_pdo_mappings.back(), std::placeholders::_1);
	m_core.pdo.add_pdo_received_callback(cob_id, binding);

}


void Device::add_transmit_pdo_mapping(uint16_t cob_id, const std::vector<Mapping>& mappings, TransmissionType transmission_type, std::chrono::milliseconds repeat_time) {

	m_transmit_pdo_mappings.emplace_back(m_core, m_dictionary, cob_id, transmission_type, repeat_time, mappings);
	TransmitPDOMapping& pdo = m_transmit_pdo_mappings.back();

	if (!pdo.check_correctness()) {
		ERROR("The given PDO mapping is not correct.");
		m_transmit_pdo_mappings.pop_back();
		return;
	}

	if (transmission_type==TransmissionType::ON_CHANGE) {

		for (const Mapping& mapping : pdo.mappings) {
			
			// entry exists because check_correctness() == true.
			Entry& entry = m_dictionary.at(mapping.entry_name);

			entry.add_value_changed_callback([&](const Value& value){
				DEBUG_LOG("[Callback] Value of "<<mapping.entry_name<<" changed to "<<value);
				pdo.send();
			});
		}

	} else {

		// transmission_type==TransmissionType::PERIODIC
		
		if (repeat_time == std::chrono::milliseconds(0)) {
			WARN("[Device::add_transmit_pdo_mapping] Repeat time is 0. This could overload the bus.");
		}

		pdo.transmitter = std::move(std::shared_ptr<std::thread>(new std::thread([&pdo, repeat_time](){
			
			while (true) {
				DEBUG_LOG("[Timer thread] Sending periodic PDO.");
				pdo.send();
				std::this_thread::sleep_for(repeat_time);
			}

		})));

	}

}

void Device::pdo_received_callback(const ReceivePDOMapping& mapping, std::vector<uint8_t> data) {
	Entry& entry = m_dictionary[mapping.entry_name];
	const uint8_t array_index = mapping.array_index;
	const uint8_t offset = mapping.offset;
	const uint8_t type_size = Utils::get_type_size(entry.type);

	if (data.size() < offset+type_size) {
		ERROR("[Device::pdo_received_callback] PDO has wrong size!");
		DUMP(data.size());
		DUMP(offset);
		DUMP(type_size);
	}

	DEBUG_LOG("Updating entry "<<entry.name<<" (in case it's an array, index="<<array_index<<")");
	std::vector<uint8_t> bytes(data.begin()+offset, data.begin()+offset+type_size);
	entry.set_value(Value(entry.type,bytes), array_index);

}

uint16_t Device::get_device_profile_number() {
	uint32_t device_type = get_entry("Device type");
	return (device_type & 0xFFFF);
}

bool Device::specialize() {
	uint16_t profile = get_device_profile_number();
	switch (profile) {
		case 401: {
			for (const auto& entry : cia_401::dictionary) {
				m_dictionary[entry.name] = entry;
			}
			return true;
		}
		case 402: {
			for (const auto& entry : cia_402::dictionary) {
				m_dictionary[entry.name] = entry;
			}
			return true;
		}
		default: {
			ERROR("Unknown CiA profile: "<<profile<<". Cannot specialize device.");
			return false;
		}
	}
}

void Device::print_dictionary() const {

	std::vector< kaco::Entry > entries;

	for (const auto& pair : m_dictionary) {
		entries.push_back(pair.second);
	}

	// sort by index and subindex
	std::sort(entries.begin(), entries.end());

	for (const auto& entry : entries) {
		entry.print();
	}

}


} // end namespace kaco