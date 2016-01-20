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
 
#include "sdo_response.h"
#include "logger.h"

#include <cstdint>
#include <string>

namespace kaco {

uint16_t SDOResponse::get_index() const {
	return (((uint16_t)data[2])<<8) + (uint16_t)data[1];
}

uint16_t SDOResponse::get_subindex() const {
	return data[3];
}

uint8_t SDOResponse::get_length() const {
	return 4-((command&0x0C)>>2);
}

uint8_t SDOResponse::failed() const {
	return command == 0x80;
}

uint32_t SDOResponse::get_data() const {
	return (((uint32_t) data[3+3]) << 24)
		+ (((uint32_t) data[3+2]) << 16)
		+ (((uint32_t) data[3+1]) << 8)
		+ (uint32_t) data[3+0];
}

void SDOResponse::print() const {

	DUMP_HEX(node_id);
	DUMP_HEX(command);
	DUMP_HEX(get_index());
	DUMP_HEX(get_subindex());

	for (unsigned i=0;i<7;++i) {
		if (data[i]>0) {
			PRINT("data["<<i<<"] = 0x"<<std::hex<<(unsigned)data[i]);
		}
	}

	DUMP_HEX(failed());
	DUMP_HEX(get_length());
	DUMP_HEX(get_data());
	PRINT(get_error());

}

std::string SDOResponse::get_error() const {
	
	if (!failed()) {
		return "no error";
	}

	switch(get_data()) {
		case 0x05030000: return "Toggle bit not alternated";
		case 0x05040000: return "SDO protocol timed out";
		case 0x05040001: return "Client/server command specifier not valid or unknown";
		case 0x05040002: return "Invalid block size (block mode only)";
		case 0x05040003: return "Invalid sequence number (block mode only)";
		case 0x05040004: return "CRC error (block mode only)";
		case 0x05040005: return "Out of memory";
		case 0x06010000: return "Unsupported access to an object";
		case 0x06010001: return "Attempt to read a write only object";
		case 0x06010002: return "Attempt to write a read only object";
		case 0x06020000: return "Object does not exist in the object dictionary";
		case 0x06040041: return "Object cannot be mapped to the PDO";
		case 0x06040042: return "The number and length of the objects to be mapped whould exeed PDO length";
		case 0x06040043: return "General parameter incompatibility reason";
		case 0x06040047: return "General internal incompatibility in the device";
		case 0x06060000: return "Access failed due to a hardware error";
		case 0x06070010: return "Data type does not match, length of service parameter does not match";
		case 0x06070012: return "Data type does not match, length of service parameter too high";
		case 0x06070013: return "Data type does not match, length of service parameter too low";
		case 0x06090011: return "Sub-index does not exist.";
		case 0x06090030: return "Value range of parameter exceeded (only for write access)";
		case 0x06090031: return "Value of parameter written too high";
		case 0x06090032: return "Value of parameter written too low";
		case 0x06090036: return "Maximum value is less than minimum value";
		case 0x08000000: return "General error";
		case 0x08000020: return "Data cannot be transferred or stored to the application";
		case 0x08000021: return "Data cannot be transferred or stored to the application because of local control";
		case 0x08000022: return "Data cannot be transferred or stored to the application because ofthe present device";
		case 0x08000023: return "Object dictionary dynamic generation fails or no object dictionary is present.";
		default: return "unknown error code";
	}

}

}