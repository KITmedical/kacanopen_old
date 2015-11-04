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

#include "data_type.h"
#include "value_type.h"

namespace kaco {

	enum access_type {
		read_only,
		write_only,
		read_write,
		constant
	};

	struct entry_type {

		entry_type() {}

		// standard constructor
		entry_type(uint32_t _index, uint8_t _subindex, std::string _name, data_type _type, access_type _access)
			: name(_name),
				index(_index),
				subindex(_subindex),
				is_array(false),
				access(_access),
				type(_type),
				value(),
				valid(false),
				sdo_on_read(true),
				sdo_on_write(true),
				is_slice(false),
				slice_first_bit(0),
				slice_last_bit(0),
				description("")
			{ }

		// array constructor
		entry_type(uint32_t _index, std::string _name, data_type _type, access_type _access)
			: name(_name),
				index(_index),
				subindex(0),
				is_array(true),
				access(_access),
				type(_type),
				value(),
				valid(false),
				sdo_on_read(true),
				sdo_on_write(true),
				is_slice(false),
				slice_first_bit(0),
				slice_last_bit(0),
				description("")
			{ }

		std::string name;
		uint16_t index;
		
		uint8_t subindex; // only used if is_array==false
		bool is_array;

		access_type access;

		data_type type;
		value_type value;
		bool valid;

		bool sdo_on_read;
		bool sdo_on_write;

		bool is_slice;
		uint8_t slice_first_bit;
		uint8_t slice_last_bit;

		std::string description;

	};

} // end namespace kaco