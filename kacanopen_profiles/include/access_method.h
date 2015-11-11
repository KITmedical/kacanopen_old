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

namespace kaco {

	/// ReadAccessMethod lists methods on how to read from the dictionary of a device.
	enum class ReadAccessMethod {

		/// Use default access method from dictionary.
		use_default,

		/// Return cached value.
		cache,

		/// Send an SDO message (and wait for confirmation).
		sdo,

		/// Return cached value and send a PDO request for a PDO
		/// which is associated with this entry (if there is such
		/// a mapping). 
		cache_and_pdo_request,

		/// Send a PDO request for a PDO which is associated with
		/// this entry (if there is such a mapping), wait for the
		// result and return it.
		pdo_request_and_wait

	};

	/// WriteAccessMethod lists methods on how to write to the dictionary of a device.
	enum class WriteAccessMethod {

		/// Use default access method from dictionary.
		use_default,

		/// Just write value into cache.
		cache,

		/// Send an SDO message (and wait for confirmation).
		sdo,

		/// Send a PDO which is associated with this entry, if there is such
		/// a mapping. 
		pdo

	};

} // end namespace kaco