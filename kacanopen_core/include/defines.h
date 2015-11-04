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
#include <iostream>

typedef struct {
	const char * busname;
	const char * baudrate;
} board_type;

typedef void* handle_type;

#define UINTDUMP(x) std::cout <<  #x << " = 0x" << std::hex << (uint64_t)x << std::endl;
#define LOG(x) std::cout << x << std::endl;
#define DEBUG(x) if (debug) {std::cout << x << std::endl;}

/*#ifdef CANOPEN_BIG_ENDIAN

	#define UNS16_LE(v)  ((((UNS16)(v) & 0xff00) >> 8) | \
			      (((UNS16)(v) & 0x00ff) << 8))

	#define UNS32_LE(v)  ((((UNS32)(v) & 0xff000000) >> 24) |	\
			      (((UNS32)(v) & 0x00ff0000) >> 8)  |	\
			      (((UNS32)(v) & 0x0000ff00) << 8)  |	\
			      (((UNS32)(v) & 0x000000ff) << 24))

#else

	#define UNS16_LE(v)  (v)

	#define UNS32_LE(v)  (v)

#endif*/