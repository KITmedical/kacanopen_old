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

#include "utils.h"

namespace kaco {

	struct value_type {

		static const bool debug = true;
		
		value_type() {
			DEBUG("Creating empty value");
			type = data_type::invalid;
		}
		
		value_type(uint8_t value) {
			DEBUG("Creating uint8 value");
			type = data_type::uint8;
			uint8 = value;
		}
		
		value_type(uint16_t value) {
			DEBUG("Creating uint16 value");
			type = data_type::uint16;
			uint16 = value;
		}
		
		value_type(uint32_t value) {
			DEBUG("Creating uint32 value");
			type = data_type::uint32;
			uint32 = value;
		}
		
		value_type(int8_t value) {
			DEBUG("Creating int8 value");
			type = data_type::int8;
			int8 = value;
		}
		
		value_type(int16_t value) {
			DEBUG("Creating int16 value");
			type = data_type::int16;
			int16 = value;
		}
		
		value_type(int32_t value) {
			DEBUG("Creating int32 value");
			type = data_type::int32;
			int32 = value;
		}
		
		value_type(const std::string& value) {
			DEBUG("Creating string value");
			type = data_type::string;
			string = value;
		}

		data_type type;

		// std::string is non-trivial and should not be part of a union.
		std::string string;

		union {
			uint8_t uint8;
			uint16_t uint16;
			uint32_t uint32;
			int8_t int8;
			int16_t int16;
			int32_t int32;
		};

		////////////////////
		// Cast operators //
		////////////////////

		#define CO_VALUE_TYPE_CAST_OP(mtypeout, mtypein) \
			operator mtypeout() const { \
				if (type != data_type::mtypein ) { \
					LOG("Illegal conversion from "<<Utils::type_to_string(type)<<" to " #mtypein " !") \
				} \
				return mtypein; \
			}

		#define CO_VALUE_TYPE_CAST_OP_INT(mtype) CO_VALUE_TYPE_CAST_OP(mtype##_t, mtype)

		CO_VALUE_TYPE_CAST_OP_INT(uint8);
		CO_VALUE_TYPE_CAST_OP_INT(uint16);
		CO_VALUE_TYPE_CAST_OP_INT(uint32);
		CO_VALUE_TYPE_CAST_OP_INT(int8);
		CO_VALUE_TYPE_CAST_OP_INT(int16);
		CO_VALUE_TYPE_CAST_OP_INT(int32);
		CO_VALUE_TYPE_CAST_OP(std::string, string);

	};

} // end namespace kaco