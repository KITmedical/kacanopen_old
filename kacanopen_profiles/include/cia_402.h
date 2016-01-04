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

namespace kaco {

	namespace cia_402 {

		const std::vector<Entry> dictionary {

			// DEVICE CONTROL

			Entry(Entry::variable_tag, 0x6060, 0, "Modes of operation", Type::int8, AccessType::read_write),
			Entry(Entry::variable_tag, 0x6061, 0, "Modes of operation display", Type::int8, AccessType::read_only),

			// TARGET POSITION MODE

			Entry(Entry::variable_tag, 0x6064, 0, "Position actual value", Type::int32, AccessType::read_only),
			Entry(Entry::variable_tag, 0x607A, 0, "Target position", Type::int32, AccessType::read_write),

		};

		enum ModeOfOperation : int8_t {
			PROFILE_POSITION_MODE = 1,
			VELOCITY_MODE = 2,
			PROFILE_VELOCITY_MODE = 3,
			TORQUE_PROFILE_MODE = 4,
			HOMING_MODE = 6,
			INTERPOLATED_POSITION_MODE = 7
		};

	};

} // end namespace kaco