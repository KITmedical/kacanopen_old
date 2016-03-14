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
#include <limits>

#include "utils.h"

namespace kaco {

	/// This class contains a value to be stored in the object dictionary.
	/// The value can have one of the types which are supported by the
	/// KaCanOpen library (see enum Type). It's very similar to Boost's
	/// variant type, but a bit simpler.
	///
	/// There are implicit cast constructors and implicit cast operators.
	/// All conversions are checked at runtime (at least in debug mode).
	///
	/// \todo Maybe we should introduce a specific exception if types don't match.
	/// \todo Maybe we could be less restrictive for integer types of different size (e.g casting uint16_t to uint32_t should be safe).
	struct Value {
		
		// Check real32 / real64 support
		static_assert(std::numeric_limits<float>::is_iec559, "Your machine doesn't use IEEE 754 compliant single-precision floating point numbers.");
		static_assert(sizeof(float)==4, "sizeof(float)!=4 on your machine.");
		static_assert(sizeof(double)==8, "sizeof(double)!=8 on your machine.");

		/// Tyoe of the value
		Type type;

		/// The value if type==Type::string
		/// It's seperate because std::string is non-trivial and should not be part of a union.
		std::string string;

		/// Anonymous union containing the value if type!=Tyoe::string
		union {
			uint8_t uint8;
			uint16_t uint16;
			uint32_t uint32;
			int8_t int8;
			int16_t int16;
			int32_t int32;
			float real32;
			double real64;
			bool boolean;
		};
		
		/// Constructs an invalid value.
		Value();

		/// Constructs a uint8 value.
		Value(uint8_t value);

		/// Constructs a uin16 value.
		Value(uint16_t value);

		/// Constructs a uin32 value.
		Value(uint32_t value);

		/// Constructs an int8 value.
		Value(int8_t value);

		/// Constructs an int16 value.
		Value(int16_t value);

		/// Constructs an int32 value.
		Value(int32_t value);

		/// Constructs a real32 value.
		Value(float value);

		/// Constructs a real64 value.
		Value(double value);

		/// Constructs a boolean value.
		Value(bool value);

		/// Constructs a string value.
		Value(const std::string& value);

		/// Creates a value given a type and the byte representation (little-endian) in a vector.
		Value(Type type_, const std::vector<uint8_t>& data);

		/// Returns the byte representation (little-endian) as a vector.
		std::vector<uint8_t> get_bytes() const;

		/// Casts uint8 to C++ type.
		operator uint8_t() const;

		/// Casts uint16 to C++ type.
		operator uint16_t() const;

		/// Casts uint32 to C++ type.
		operator uint32_t() const;

		/// Casts int8 to C++ type.
		operator int8_t() const;

		/// Casts int16 to C++ type.
		operator int16_t() const;

		/// Casts int32 to C++ type.
		operator int32_t() const;

		/// Casts real32 to C++ type.
		operator float() const;

		/// Casts real64 to C++ type.
		operator double() const;

		/// Casts boolean to C++ type.
		operator bool() const;

		/// Casts string to C++ type.
		operator std::string() const;

		/// Compares equal
		bool operator==(const Value& other) const;

		/// Compares not equal
		bool operator!=(const Value& other) const;

	private:

		static const bool debug = false;

	};

	namespace value_printer {
		/// Prints a Value
		std::ostream &operator<<(std::ostream &os, Value val);
	}

} // end namespace kaco

// For correct usage of logger.h one should use the int8_printers, so
// the int8_printers namespace is automatically included. Because of
// this, logger.h should not be included in library headers.
using namespace kaco::value_printer;