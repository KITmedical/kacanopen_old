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

#include "core.h"
#include "entry.h"
#include "types.h"
#include "receive_pdo_mapping.h"
#include "transmit_pdo_mapping.h"
#include "access_method.h"

#include <vector>
#include <map>
#include <string>
#include <chrono>

namespace kaco {

	class Device {

	public:

		Device(Core& core, uint8_t node_id);
		~Device();

		void start();
		uint8_t get_node_id() const;

		/// Gets the value of a dictionary entry by index via SDO
		/// It does not change the corresponding internal value and therefore the new value
		/// cannot be used by Transmit PDOs.
		Value get_entry_via_sdo(uint32_t index, uint8_t subindex, Type type);

		/// Gets the value of a dictionary entry by name internally.
		/// If there is no cached value or the entry is configured to send an SDO on request, the new value is fetched from the device via SDO.
		/// Otherwise it returns the cached value. This makes sense, if a Reveive PDO is configured on the corresponding entry.
		const Value& get_entry(const std::string& entry_name, uint8_t array_index=0, ReadAccessMethod access_method = ReadAccessMethod::use_default);

		/// Returns the type of a dictionary entry identified by name as it is defined in the local dictionary.
		Type get_entry_type(const std::string& entry_name);

		/// Sets the value of a dictionary entry by index via SDO
		/// It does not change the corresponding internal value and therefore the new value
		/// cannot be used by Transmit PDOs.
		void set_entry_via_sdo(uint32_t index, uint8_t subindex, const Value& value);

		/// Sets the value of a dictionary entry by name internally.
		/// If the entry is configured to send an SDO on update, the new value is also sent to the device via SDO.
		/// If a PDO is configured on the corresponding entry, it will from now on use the new value stored internally.
		void set_entry(const std::string& entry_name, const Value& value, uint8_t array_index=0, WriteAccessMethod access_method = WriteAccessMethod::use_default);

		/// Adds a receive PDO mapping. This means values sent by the device via PDO are saved into the dictionary cache.
		/// \param offset index of the first mapped byte in the PDO message
		void add_receive_pdo_mapping(uint16_t cob_id, const std::string& entry_name, uint8_t offset, uint8_t array_index=0);

		/// Adds a transmit PDO mapping. This means values from the dictionary cache are sent to the device.
		///
		/// \param cob_id The cob_id of the PDO to transmit
		/// \param mappings A vector of mappings. A mapping maps a dictionary entry (by name) to a part of a PDO (by first and last byte index)
		/// \param transmission_type Send PDO "ON_CHANGE" or "PERIODIC"
		/// \param repeat_time If transmission_type==TransmissionType::PERIODIC, PDO is sent periodically according to repeat_time.
		///
		/// \example 
		/// 	The following command maps the first value of the "write_output" entry (it's an arraym see CiA 401)
		///		to the first byte of the PDO channel with cob_id 0x208 (RPDO1 of CANOpen device 8). The PDO is
		///		sent whenever the value is changed via set_entry("write_output", value, 0).
		///
		/// 	device.add_transmit_pdo_mapping(0x208, {{"write_output", 0, 0, 0}});
		///
		void add_transmit_pdo_mapping(uint16_t cob_id, const std::vector<Mapping>& mappings, TransmissionType transmission_type=TransmissionType::ON_CHANGE, std::chrono::milliseconds repeat_time=std::chrono::milliseconds(0));

		/// Returns the CiA profile number (determined via SDO)
		uint16_t get_device_profile_number();

		/// Adds entries to the dictionary according to the CiA profile number. Returns true if successful.
		bool specialize();

		void print_dictionary() const;

	private:

		void pdo_received_callback(const ReceivePDOMapping& mapping, std::vector<uint8_t> data);

		static const bool debug = true;

		Core& m_core;
		uint8_t m_node_id;

		std::map<std::string, Entry> m_dictionary;
		std::vector<ReceivePDOMapping> m_receive_pdo_mappings;
		std::vector<TransmitPDOMapping> m_transmit_pdo_mappings;
		const Value m_dummy_value;

	};

} // end namespace kaco