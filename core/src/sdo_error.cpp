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
 
#include "sdo_error.h"

namespace kaco {

	sdo_error::sdo_error(type error_type, const std::string& additional_information)
		: sdo_error(static_cast<uint32_t>(error_type),additional_information)
		{ }

	sdo_error::sdo_error(uint32_t sdo_data, const std::string& additional_information)
		: canopen_error("") {

		#define SDO_ERROR_CASE(t,msg) case static_cast<uint32_t>(type:: t ): m_message = msg ; m_type = type:: t ; break;

		switch (sdo_data) {

			// Standard Cia 301 SDO error codes

			SDO_ERROR_CASE(toggle_bit,"Toggle bit not alternated.")
			SDO_ERROR_CASE(timeout,"SDO protocol timed out.")
			SDO_ERROR_CASE(command_specifier,"Client/server command specifier not valid or unknown.")
			SDO_ERROR_CASE(block_size,"Invalid block size (block mode only).")
			SDO_ERROR_CASE(sequence_number,"Invalid sequence number (block mode only).")
			SDO_ERROR_CASE(crc,"CRC error (block mode only).")
			SDO_ERROR_CASE(memory,"Out of memory.")
			SDO_ERROR_CASE(access,"Unsupported access to an object.")
			SDO_ERROR_CASE(write_only,"Attempt to read a write only object.")
			SDO_ERROR_CASE(read_only,"Attempt to write a read only object.")
			SDO_ERROR_CASE(not_in_dictionary,"Object does not exist in the object dictionary.")
			SDO_ERROR_CASE(no_mapping,"Object cannot be mapped to the PDO.")
			SDO_ERROR_CASE(pdo_length_exceeded,"The number and length of the objects to be mapped whould exeed PDO length.")
			SDO_ERROR_CASE(parameter_incompatibility,"General parameter incompatibility reason.")
			SDO_ERROR_CASE(internal_incompatibility,"General internal incompatibility in the device.")
			SDO_ERROR_CASE(hardware_error,"Access failed due to a hardware error.")
			SDO_ERROR_CASE(service_parameter,"Data type does not match, length of service parameter does not match.")
			SDO_ERROR_CASE(service_parameter_too_high,"Data type does not match, length of service parameter too high.")
			SDO_ERROR_CASE(service_parameter_too_low,"Data type does not match, length of service parameter too low.")
			SDO_ERROR_CASE(subindex,"Sub-index does not exist.")
			SDO_ERROR_CASE(value,"Invalid value for parameter.")
			SDO_ERROR_CASE(value_too_high,"Value of parameter written too high.")
			SDO_ERROR_CASE(value_too_low,"Value of parameter written too low.")
			SDO_ERROR_CASE(max_less_than_min,"Maximum value is less than minimum value")
			SDO_ERROR_CASE(sdo_connection,"Resource not available: SDO connection")
			SDO_ERROR_CASE(general,"General error")
			SDO_ERROR_CASE(transfer_or_storage,"Data cannot be transferred or stored to the application.")
			SDO_ERROR_CASE(transfer_or_storage_local_control,"Data cannot be transferred or stored to the application because of local control.")
			SDO_ERROR_CASE(transfer_or_storage_device_state,"Data cannot be transferred or stored to the application because of the present device state.")
			SDO_ERROR_CASE(no_dictionary,"Object dictionary dynamic generation fails or no object dictionary is present (e.g. object dictionary is generated from file and generation fails because of a file error).")
			SDO_ERROR_CASE(no_data,"No data available.")

			// Custom KaCanOpen error codes

			SDO_ERROR_CASE(response_timeout,"Timeout while waiting for response.")
			SDO_ERROR_CASE(segmented_download,"Segmented download not yet supported.")
			SDO_ERROR_CASE(response_command,"Invalid response command.")
			SDO_ERROR_CASE(response_toggle_bit,"Toggle bit in response is not equal to toggle bit in request.")

			// just for completeness
			SDO_ERROR_CASE(unknown,"Unknown SDO error")

			// Unknown error

			default:
				m_message = "Unknown SDO error.";
				m_type = type::unknown;

		}

		#undef SDO_ERROR_CASE

		m_message = "SDO Error: " + m_message + (additional_information.empty()?"":" "+additional_information);

	}

	const char* sdo_error::what() const noexcept {
		return m_message.c_str();
	}

	sdo_error::type sdo_error::get_type() const noexcept {
		return m_type;
	}

}