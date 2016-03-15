/*
 * Copyright (c) 2016, Thomas Keh
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

#include "profiles.h"
#include "logger.h"

namespace kaco {

	const std::map<uint16_t,std::map<std::string, const Profiles::Operation>> Profiles::operations {
		{
			(uint16_t) 402,
			{
				{
					"enable_operation",
					[](Device& device) -> Value {
						device.set_entry("controlword", (uint16_t) 0x0006); // shutdown
						device.set_entry("controlword", (uint16_t) 0x0007); // switch on
						device.set_entry("controlword", (uint16_t) 0x000F); // enable operation
						return Value(); // invalid value (return value not needed)
					}
				}
			}
		}
	};

	const std::map<uint16_t,std::map<std::string, const Value>> Profiles::constants {
		{
			(uint16_t) 402,
			{
				{ "profile_position_mode",		(int8_t) 1 },
				{ "velocity_mode",				(int8_t) 2 },
				{ "profile_velocity_mode",		(int8_t) 3 },
				{ "torque_profile_mode",		(int8_t) 4 },
				{ "homing_mode",				(int8_t) 6 },
				{ "interpolated_position_mode",	(int8_t) 7 }
			}
		}
	};

} // end namespace kaco