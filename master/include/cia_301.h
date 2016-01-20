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

	namespace cia_301 {

		const std::vector<Entry> dictionary {

			Entry(Entry::variable_tag, 0x1000, 0, "Device type", Type::uint32, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1001, 0, "Error register", Type::uint8, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1002, 0, "Manufacturer status register", Type::uint32, AccessType::read_only),
			Entry(Entry::array_tag, 0x1003, "Pre-defined error field", Type::uint32, AccessType::read_only),
			Entry(Entry::variable_tag, 0x1005, 0, "COB-ID SYNC", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1006, 0, "Communication cycle period", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1007, 0, "Synchronous window length", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1008, 0, "Manufacturer device name", Type::string, AccessType::constant),
			Entry(Entry::variable_tag, 0x1009, 0, "Manufacturer hardware version", Type::string, AccessType::constant),
			Entry(Entry::variable_tag, 0x100A, 0, "Manufacturer software version", Type::string, AccessType::constant),
			Entry(Entry::variable_tag, 0x100C, 0, "Guard time", Type::uint16, AccessType::read_write),
			Entry(Entry::variable_tag, 0x100D, 0, "Life time factor", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x1010, "Store parameters", Type::uint32, AccessType::read_write), // TODO: this is actually a record/array hybrid
			Entry(Entry::array_tag, 0x1011, "Restore default parameters", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1012, 0, "COB-ID time stamp object", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1013, 0, "High resolution time stamp", Type::uint32, AccessType::read_write),

			Entry(Entry::variable_tag, 0x1400, 1, "Receive PDO Communication Parameter 1/COB-ID", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1400, 2, "Receive PDO Communication Parameter 1/Transmission Type", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1401, 1, "Receive PDO Communication Parameter 2/COB-ID", Type::uint32, AccessType::read_write),
			Entry(Entry::variable_tag, 0x1401, 2, "Receive PDO Communication Parameter 2/Transmission Type", Type::uint32, AccessType::read_write)
			
		};

	} // end namespace cia_profiles

} // end namespace kaco