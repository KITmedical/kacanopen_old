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

#include "utils.h"
#include "logger.h"

#include <string>
#include <cstdint>
#include <algorithm>

namespace kaco {

std::string Utils::type_to_string(Type type) {
	switch(type) {
		case Type::uint8:
			return "uint8";
		case Type::uint16:
			return "uint16";
		case Type::uint32:
			return "uint32";
		case Type::int8:
			return "int8";
		case Type::int16:
			return "int16";
		case Type::int32:
			return "int32";
		case Type::boolean:
			return "boolean";
		case Type::string:
			return "string";
		default:
			return "unknown type";
	}
}

uint8_t Utils::get_type_size(Type type) {
	switch(type) {
		case Type::uint8:
		case Type::int8:
		case Type::boolean:
			return 1;
		case Type::uint16:
		case Type::int16:
			return 2;
		case Type::uint32:
		case Type::int32:
			return 4;
		case Type::string:
		default:
			ERROR("[Utils::get_type_size] Unknown type or type with variable size.");
			return 0;
	}
}

Type Utils::type_code_to_type(uint16_t code) {
	switch (code) {
		case (uint16_t) DataType::BOOLEAN: return Type::boolean;
		case (uint16_t) DataType::INTEGER8: return Type::int8;
		case (uint16_t) DataType::INTEGER16: return Type::int16;
		case (uint16_t) DataType::INTEGER32: return Type::int32;
		case (uint16_t) DataType::UNSIGNED8: return Type::uint8;
		case (uint16_t) DataType::UNSIGNED16: return Type::uint16;
		case (uint16_t) DataType::UNSIGNED32: return Type::uint32;
		//case REAL32: return Type::;
		case (uint16_t) DataType::VISIBLE_STRING: return Type::string;
		//case OCTET_STRING: return Type::;
		//case UNICODE_STRING: return Type::;
		//case TIME_OF_DAY: return Type::;
		//case TIME_DIFFERENCE: return Type::;
		//case LARGEDATA: return Type::;
		//case INTEGER24: return Type::;
		//case REAL64: return Type::;
		//case INTEGER40: return Type::;
		default: return Type::invalid;
	}
}

std::string Utils::escape(std::string str) {
	std::string out = str;
	std::transform(out.begin(), out.end(), out.begin(), ::tolower);
	std::replace(out.begin(), out.end(), ' ', '_');
	std::replace(out.begin(), out.end(), '-', '_');
	return std::move(out);
}

unsigned long long Utils::hexstr_to_uint(std::string str) {
	try {
		return std::stoull(str, nullptr, 16);
	} catch ( const std::exception& e ) {
		ERROR(e.what());
		return 0;
	}
}

unsigned long long Utils::decstr_to_uint(std::string str) {
	try {
		return std::stoull(str, nullptr, 10);
	} catch ( const std::exception& e ) {
		ERROR(e.what());
		return 0;
	}
}

AccessType Utils::string_to_access_type(std::string str) {
	if (str == "ro") {
		return AccessType::read_only;
	} else if (str == "wo") {
		return AccessType::write_only;
	} else if (str == "const") {
		return AccessType::constant;
	} else if (str == "rw" || str == "rwr" || str == "rww") {
		return AccessType::read_write;
	} else {
		ERROR("[Utils::str_to_access_type] Invalid access type string. Returning AccessType::read_write.")
		return AccessType::read_write;
	}
}

std::string Utils::access_type_to_string(AccessType type) {
	switch(type) {
		case AccessType::read_only: return "ro";
		case AccessType::write_only: return "wo";
		case AccessType::constant: return "const";
		case AccessType::read_write: return "rw";
		default:
			ERROR("[Utils::access_type_to_string] Unknown access type!");
			return "unknown access type";
	}
}

} // end namespace kaco