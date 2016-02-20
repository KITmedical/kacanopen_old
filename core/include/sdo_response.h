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

namespace kaco {

/// This struct contains the response of a SDO request.
/// Note that there are two types of response - segmented
/// and expedited. Index, subindex and get_data() are not valid
/// in segmented responses.
struct SDOResponse {

	/// Node id
	uint8_t node_id;
	
	/// SDO command specifier
	uint8_t command;

	/// Dictionary index
	/// \deprecated
	/// \todo Remove this.
	uint16_t index;

	/// Subindex
	/// \deprecated
	/// \todo Remove this.
	uint8_t subindex;

	/// Data bytes
	uint8_t data[7];

	/// Returns the dictionary index (only for expedited transfer).
	uint16_t get_index() const;

	/// Returns the subindex (only for expedited transfer).
	uint16_t get_subindex() const;

	/// Returns the number of data bytes.
	uint8_t get_length() const;

	/// Check if the transfer failed.
	uint8_t failed() const;

	/// Returns the data as a single 4-byte value (only for expedited transfer).
	uint32_t get_data() const;

	/// Prints the response to command line.
	void print() const;

	/// Returns a human-readable representation of the error (only if failed()==true)
	/// \todo reimplement this using sdo_error class -> remove code duplication
	std::string get_error() const;

};

} // end namespace kaco