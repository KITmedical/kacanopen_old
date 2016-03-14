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

#include <iostream>

// DO NOT INCLUDE THIS IN A LIBRARY HEADER!

// defined/not defined by CMake
// #define EXHAUSTIVE_DEBUGGING

#define PRINT(x) std::cout << x << std::endl;
#define WARN(x) PRINT("WARNING: " << x);
#define ERROR(x) std::cerr << "ERROR: " << x << std::endl;
#define DUMP(x) PRINT(#x << " = " << std::dec << x);
#define DUMP_HEX(x) PRINT(#x << " = 0x" << std::hex << x);

#ifdef NDEBUG
	#define DEBUG(x)
	#define DEBUG_LOG(x)
	#define DEBUG_DUMP(x)
	#define DEBUG_DUMP_HEX(x)

	#define DEBUG_EXHAUSTIVE(x)
	#define DEBUG_LOG_EXHAUSTIVE(x)
#else
	#define DEBUG(x) if (debug) { x }
	#define DEBUG_LOG(x) if (debug) { PRINT("DEBUG: " << x); }
	#define DEBUG_DUMP(x) if (debug) { PRINT("DEBUG: " << #x << " = " << std::dec << x); }
	#define DEBUG_DUMP_HEX(x) if (debug) { PRINT("DEBUG: " << #x << " = 0x" << std::hex << x); }

	#ifdef EXHAUSTIVE_DEBUGGING
		#define DEBUG_EXHAUSTIVE(x) if (debug) { x }
		#define DEBUG_LOG_EXHAUSTIVE(x) if (debug) { PRINT("DEBUG_EXHAUSTIVE: " << x); }
	#else
		#define DEBUG_EXHAUSTIVE(x)
		#define DEBUG_LOG_EXHAUSTIVE(x)
	#endif

#endif

// Fix printing uint8_t and int8_t types using std::cout
namespace kaco {
	namespace int8_printers {

		inline std::ostream &operator<<(std::ostream &os, char c) {
			return os << (std::is_signed<char>::value ? static_cast<int>(c)
				: static_cast<unsigned int>(c));
		}

		inline std::ostream &operator<<(std::ostream &os, signed char c) {
			return os << static_cast<int>(c);
		}

		inline std::ostream &operator<<(std::ostream &os, unsigned char c) {
			return os << static_cast<unsigned int>(c);
		}
	}
}

// For correct usage of logger.h one should use the int8_printers, so
// the int8_printers namespace is automatically included. Because of
// this, logger.h should not be included in library headers.
using namespace kaco::int8_printers;
