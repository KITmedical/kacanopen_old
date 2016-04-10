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

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <functional>

#include "value.h"
#include "device.h"

namespace kaco {

	/// This class profides static data related to CiA profiles.
	/// This includes profile-specific convenience operations and constants.
	/// Names must be escaped using Utils::escape().
	struct Profiles {

		/// Type of a convenience operation.
		/// Takes reference to device and a Value typed argument. Ignore, if you don't need it.
		/// Returns Value. Just return Value() if you don't need it.
		using Operation = std::function<Value(Device&,const Value&)>;

		/// Convenience operations for CiA profiles.
		/// Type: map < profile number , map < operation name , operation function object > >
		static const std::map<uint16_t,std::map<std::string,const Operation>> operations;

		/// Constants for CiA profiles.
		/// Type: map < profile number , map < operation name , constant value > >
		static const std::map<uint16_t,std::map<std::string,const Value>> constants;

	private:

		static const bool debug = false;

	};

} // end namespace kaco