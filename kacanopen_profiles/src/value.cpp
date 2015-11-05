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
 
#include "value.h"
#include "logger.h"

namespace kaco {

Value::Value() {
	DEBUG("Creating empty value");
	type = Type::invalid;
}

Value::Value(uint8_t value) {
	DEBUG("Creating uint8 value");
	type = Type::uint8;
	uint8 = value;
}

Value::Value(uint16_t value) {
	DEBUG("Creating uint16 value");
	type = Type::uint16;
	uint16 = value;
}

Value::Value(uint32_t value) {
	DEBUG("Creating uint32 value");
	type = Type::uint32;
	uint32 = value;
}

Value::Value(int8_t value) {
	DEBUG("Creating int8 value");
	type = Type::int8;
	int8 = value;
}

Value::Value(int16_t value) {
	DEBUG("Creating int16 value");
	type = Type::int16;
	int16 = value;
}

Value::Value(int32_t value) {
	DEBUG("Creating int32 value");
	type = Type::int32;
	int32 = value;
}

Value::Value(const std::string& value) {
	DEBUG("Creating string value");
	type = Type::string;
	string = value;
}

//----------------//
// Cast operators //
//----------------//

/// converts a CanOpen type <mtypein> and a C++ type <mtypeout>
/// to a cast operator
#define CO_VALUE_TYPE_CAST_OP(mtypeout, mtypein) \
	Value::operator mtypeout() const { \
		if (type != Type::mtypein ) { \
			LOG("Illegal conversion from "<<Utils::type_to_string(type)<<" to " #mtypein " !") \
		} \
		return mtypein; \
	}

/// converts a CanOpen type <mtypein> to a cast method to
/// the C++ output type <mtypein>_t
#define CO_VALUE_TYPE_CAST_OP_INT(mtype) CO_VALUE_TYPE_CAST_OP(mtype##_t, mtype)

CO_VALUE_TYPE_CAST_OP_INT(uint8);
CO_VALUE_TYPE_CAST_OP_INT(uint16);
CO_VALUE_TYPE_CAST_OP_INT(uint32);
CO_VALUE_TYPE_CAST_OP_INT(int8);
CO_VALUE_TYPE_CAST_OP_INT(int16);
CO_VALUE_TYPE_CAST_OP_INT(int32);
CO_VALUE_TYPE_CAST_OP(std::string, string);

} // end namespace kaco