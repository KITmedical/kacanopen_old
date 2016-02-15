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

#include <cstdint>
#include <string>
#include <stdexcept>

#include "canopen_error.h"

namespace kaco {

	/// This type of exception is thrown when there are problems
	/// while accessing devices via SDO. Most of the errors are
	/// reported by the device itself via SDO abort transfer
	/// protocol. There are also some internal errors like
	/// timeout while waiting for response.
	/// You can get the type of error via get_type()
	class sdo_error : public canopen_error {

	public:

		/// Exact type of the SDO error
		enum class type : uint32_t {

			// Standard Cia 301 SDO error codes

			toggle_bit = 0x05030000,
			timeout = 0x05040000,
			command_specifier = 0x05040001,
			block_size = 0x05040002,
			sequence_number = 0x05040003,
			crc = 0x05040004,
			memory = 0x05040005,
			access = 0x06010000,
			write_only = 0x06010001,
			read_only = 0x06010002,
			not_in_dictionary = 0x06020000,
			no_mapping = 0x06040041,
			pdo_length_exceeded = 0x06040042,
			parameter_incompatibility = 0x06040043,
			internal_incompatibility = 0x06040047,
			hardware_error = 0x06060000,
			service_parameter = 0x06070010,
			service_parameter_too_high = 0x06070012,
			service_parameter_too_low = 0x06070013,
			subindex = 0x06090011,
			value = 0x06090030,
			value_too_high = 0x06090031,
			value_too_low = 0x06090032,
			max_less_than_min = 0x06090036,
			sdo_connection = 0x060A0023,
			general = 0x08000000,
			transfer_or_storage = 0x08000020,
			transfer_or_storage_local_control = 0x08000021,
			transfer_or_storage_device_state = 0x08000022,
			no_dictionary = 0x08000023,
			no_data = 0x08000024,

			// Custom KaCanOpen error codes

			response_timeout = 0x10000000,
			segmented_download = 0x10000001,
			response_command = 0x10000002,
			response_toggle_bit = 0x10000003,

			// Unknown error

			unknown = 0x20000000

		};

		/// Constructor when type is known
		/// \param error_type Type of the error
		/// \param additional_information Additional information, appended to the error type string in what()
		explicit sdo_error(type error_type, const std::string& additional_information = "");

		/// Constructor when type shall be deduced from it's CiA 301 SDO abort code representation
		/// \param sdo_data SDO data which contains the SDO abort code
		/// \param additional_information Additional information, appended to the error type string in what()
		explicit sdo_error(uint32_t sdo_data, const std::string& additional_information = "");

		/// Destructor
		virtual ~sdo_error() { }

		/// Returns error description
		virtual const char* what() const noexcept override;

		/// Returns type of the error
		type get_type() const noexcept;

	private:

		std::string m_message;
		type m_type;

	};

} // end namespace kaco