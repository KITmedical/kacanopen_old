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

namespace kaco {

	/// Data types which are supported by this library
	enum class Type : uint16_t {

		uint8,
		uint16,
		uint32,
		int8,
		int16,
		int32,
		real32,
		real64,
		string,
		boolean,
		invalid

	};

	/// Data types like they are defined in Cia 301 together with the according key / index
	enum class DataType : uint16_t {
		BOOLEAN = 0x1,
		INTEGER8 = 0x2,
		INTEGER16 = 0x3,
		INTEGER32 = 0x4,
		UNSIGNED8 = 0x5,
		UNSIGNED16 = 0x6,
		UNSIGNED32 = 0x7,
		REAL32 = 0x8,
		VISIBLE_STRING = 0x9,
		OCTET_STRING = 0xA,
		UNICODE_STRING = 0xB,
		TIME_OF_DAY = 0xC,
		TIME_DIFFERENCE = 0xD,
		// 0xE reserved
		LARGEDATA = 0xF, // DOMAIN
		INTEGER24 = 0x10,
		REAL64 = 0x11,
		INTEGER40 = 0x12,
		INTEGER48 = 0x13,
		INTEGER56 = 0x14,
		INTEGER64 = 0x15,
		// 0x17 reserved
		UNSIGNED40 = 0x18,
		UNSIGNED48 = 0x19,
		UNSIGNED56 = 0x1A,
		UNSIGNED64 = 0x1B,
		// 0x1C - 0x1F reserved
		PDO_COMMUNICATION_PARAMETER = 0x20,
		PDO_MAPPING = 0x21,
		SDO_PARAMETER = 0x22,
		IDENTITY = 0x23,
		// 0x24 - 0x3F reserved
		// 0x40 - 0x5F Manufacturer specific
		// 0x60 - 0x25F Device profile specific
	};

	/// Type of an object in the dictionary together with the according key / index
	enum class ObjectType : uint8_t {
		EMPTY = 0x0, // NULL
		LARGEDATA = 0x2, // DOMAIN
		DEFTYPE = 0x5,
		DEFSTRUCT = 0x6,
		VAR = 0x7,
		ARRAY = 0x8,
		RECORD = 0x9
	};
	
	/// Access type of a dictionary entry
	enum AccessType {
		read_only,
		write_only,
		read_write,
		constant
	};
	
	/// Transmission type of a PDO mapping
	enum class TransmissionType {
		PERIODIC,
		ON_CHANGE
	};

} // end namespace kaco