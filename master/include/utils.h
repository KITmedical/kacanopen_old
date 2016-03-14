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

#include "types.h"

#include <string>
#include <cstdint>

namespace kaco {

/// This class provides various static utility functions.
struct Utils {

	/// Converts data types to a string.
	static std::string type_to_string(Type type);

	/// Converts CanOpen data types to a string.
	static std::string data_type_to_string(DataType type);

	/// Returns the size of a data type in bytes.
	static uint8_t get_type_size(Type type);

	/// Maps type codes from an EDS file to a data type
	/// \see enum DataType in types.h
	static Type type_code_to_type(uint16_t code);

	/// Converts entry names to lower case and replaces all spaces and '-' by underscores.
	static std::string escape(std::string str);

	/// Converts a string containing a hexadecimal numer to unsigned.
	static unsigned long long  hexstr_to_uint(std::string str);

	/// Converts a string containing a decimal numer to unsigned.
	static unsigned long long  decstr_to_uint(std::string str);

	/// Converts a string representation of AccessType from an EDS file to AccessType.
	static AccessType string_to_access_type(std::string str);

	/// Converts access types to a string.
	static std::string access_type_to_string(AccessType type);

private:

	static const bool debug = false;

};

} // end namespace kaco