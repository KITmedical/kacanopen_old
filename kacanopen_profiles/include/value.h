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
#include <vector>

#include "utils.h"

namespace kaco {

	struct Value {

		static const bool debug = true;

		Type type;

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
		
		Value();
		Value(uint8_t value);
		Value(uint16_t value);
		Value(uint32_t value);
		Value(int8_t value);
		Value(int16_t value);
		Value(int32_t value);
		Value(const std::string& value);

		std::vector<uint8_t> get_bytes() const;

		operator uint8_t() const;
		operator uint16_t() const;
		operator uint32_t() const;
		operator int8_t() const;
		operator int16_t() const;
		operator int32_t() const;
		operator std::string() const;

		bool operator==(const Value& other) const;
		bool operator!=(const Value& other) const;

	};

	namespace value_printer {
		std::ostream &operator<<(std::ostream &os, Value val);
	}

} // end namespace kaco
// For correct usage of logger.h one should use the int8_printers, so
// the int8_printers namespace is automatically included. Because of
// this, logger.h should not be included in library headers.
using namespace kaco::value_printer;