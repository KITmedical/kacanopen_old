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
#include "type.h"
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
		const Value& get_entry(std::string name, uint8_t array_index=0, ReadAccessMethod access_method = ReadAccessMethod::use_default);

		/// Sets the value of a dictionary entry by index via SDO
		/// It does not change the corresponding internal value and therefore the new value
		/// cannot be used by Transmit PDOs.
		void set_entry_via_sdo(uint32_t index, uint8_t subindex, const Value& value);

		/// Sets the value of a dictionary entry by name internally.
		/// If the entry is configured to send an SDO on update, the new value is also sent to the device via SDO.
		/// If a PDO is configured on the corresponding entry, it will from now on use the new value stored internally.
		void set_entry(std::string name, const Value& value, uint8_t array_index=0, WriteAccessMethod access_method = WriteAccessMethod::use_default);

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

	private:

		void pdo_received_callback(const ReceivePDOMapping& mapping, std::vector<uint8_t> data);

		static const bool debug = true;

		Core& m_core;
		uint8_t m_node_id;

		std::map<std::string, Entry> m_dictionary;
		std::vector<ReceivePDOMapping> m_receive_pdo_mappings;
		std::vector<TransmitPDOMapping> m_transmit_pdo_mappings;
		const Value m_dummy_value;

		const std::vector<Entry> profile301 {

			/*Entry(Entry::variable_tag, 0x1000, 0, "device_type", Type::uint32, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1001, 0, "error_register", Type::uint8, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1008, 0, "manufacturer_device_name", Type::string, AccessType::constant),*/
			
			// TODO remove. This is CiA401!
			/*Entry(Entry::array_tag, 0x6000, "read_digital_input", Type::uint8, AccessType::read_only),
			Entry(Entry::array_tag, 0x6200, "write_output", Type::uint8, AccessType::write_only),
			Entry(Entry::variable_tag, 0x1400, 1, "rpdo1_cob_id", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1401, 1, "rpdo2_cob_id", Type::uint32, AccessType::read_write),*/
			

			Entry(Entry::variable_tag, 0x1000, 0, "Device type", Type::uint32, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1001, 0, "Error register", Type::uint8, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1002, 0, "Manufacturer status register", Type::uint32, AccessType::read_only),
			Entry(Entry::array_tag, 0x1003, "Pre-defined error field", Type::uint32, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1005, 0, "COB-ID SYNC", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1006, 0, "Communication cycle period", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1007, 0, "Synchronous window length", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1008, 0, "Manufacturer device name", Type::string, AccessType::constant),
			Entry(Entry::variable_tag, 0x1009, 0, "Manufacturer hardware version", Type::string, AccessType::constant),
			Entry(Entry::variable_tag, 0x100A, 0, "Manufacturer software version", Type::string, AccessType::constant),
			Entry(Entry::variable_tag, 0x100C, 0, "Guard time", Type::uint16, AccessType::read_write),
			Entry(Entry::variable_tag, 0x100D, 0, "Life time factor", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x1010, "Store parameters", Type::uint32, AccessType::read_write), // TODO: this is actually a record/array hybrid
			Entry(Entry::array_tag, 0x1011, "Restore default parameters", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1012, 0, "COB-ID time stamp object", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1013, 0, "High resolution time stamp", Type::uint32, AccessType::read_write),

			Entry(Entry::variable_tag, 0x1400, 1, "Receive PDO Communication Parameter 1/COB-ID", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1400, 2, "Receive PDO Communication Parameter 1/Transmission Type", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1401, 1, "Receive PDO Communication Parameter 2/COB-ID", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1401, 2, "Receive PDO Communication Parameter 2/Transmission Type", Type::uint32, AccessType::read_write)
			
		};

		const std::vector<Entry> profile401 {

			Entry(Entry::array_tag, 0x6000, "Read input 8-bit", Type::uint8, AccessType::read_only),
			Entry(Entry::array_tag, 0x6002, "Polarity input 8-bit", Type::uint8, AccessType::read_only),
			Entry(Entry::array_tag, 0x6003, "Filter constant input 8-bit", Type::uint8, AccessType::read_only),
			Entry(Entry::variable_tag, 0x6005, 0, "Global interrupt enable digital 8-bit", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6006, "Interrupt mask any change 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6007, "Interrupt mask low-to-high 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6008, "Interrupt mask high-to-low 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6020, "Read input bit 1 to 128", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6021, "Read input bit 129 to 256", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6022, "Read input bit 257 to 384", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6023, "Read input bit 385 to 512", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6024, "Read input bit 513 to 640", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6025, "Read input bit 641 to 768", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6026, "Read input bit 769 to 896", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6027, "Read input bit 897 to 1024", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6030, "Polarity input bit 1 to 128", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6031, "Polarity input bit 129 to 256", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6032, "Polarity input bit 257 to 384", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6033, "Polarity input bit 385 to 512", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6034, "Polarity input bit 513 to 640", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6035, "Polarity input bit 641 to 768", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6036, "Polarity input bit 769 to 896", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6037, "Polarity input bit 897 to 1024", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6038, "Filter constant input bit 1 to 128", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6039, "Filter constant input bit 129 to 256", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603A, "Filter constant input bit 257 to 384", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603B, "Filter constant input bit 385 to 512", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603C, "Filter constant input bit 513 to 640", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603D, "Filter constant input bit 641 to 768", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603E, "Filter constant input bit 769 to 896", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603F, "Filter constant input bit 897 to 1024", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6050, "Interrupt mask input bit 1 to 128", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6051, "Interrupt mask input bit 129 to 256", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6052, "Interrupt mask input bit 257 to 384", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6053, "Interrupt mask input bit 385 to 512", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6054, "Interrupt mask input bit 513 to 640", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6055, "Interrupt mask input bit 641 to 768", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6056, "Interrupt mask input bit 769 to 896", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6057, "Interrupt mask input bit 897 to 1024", Type::boolean, AccessType::read_write),

			// TODO ...

			Entry(Entry::array_tag, 0x6100, "Read input 16-bit", Type::uint16, AccessType::read_only),
			Entry(Entry::array_tag, 0x6102, "Polarity input 16-bit", Type::uint16, AccessType::read_write),
			Entry(Entry::array_tag, 0x6103, "Filter constant input 16-bit", Type::uint16, AccessType::read_write),
			
			// TODO ...

			Entry(Entry::array_tag, 0x6200, "Write output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6202, "Change polarity output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6205, "Error mode output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6207, "Error value output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6208, "Filter mask output 8-bit", Type::uint8, AccessType::read_write),
			
			// TODO ...
			
		};


	};

} // end namespace kaco