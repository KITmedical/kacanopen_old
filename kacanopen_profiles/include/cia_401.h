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

	namespace cia_401 {

		const std::vector<Entry> dictionary {

			Entry(Entry::array_tag, 0x6000, "Read input 8-bit", Type::uint8, AccessType::read_only),
			Entry(Entry::array_tag, 0x6002, "Polarity input 8-bit", Type::uint8, AccessType::read_only),
			Entry(Entry::array_tag, 0x6003, "Filter constant input 8-bit", Type::uint8, AccessType::read_only),
			Entry(Entry::variable_tag, 0x6005, 0, "Global interrupt enable digital 8-bit", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6006, "Interrupt mask any change 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6007, "Interrupt mask low-to-high 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6008, "Interrupt mask high-to-low 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6020, "Read input bit 1 to 128", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6021, "Read input bit 129 to 256", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6022, "Read input bit 257 to 384", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6023, "Read input bit 385 to 512", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6024, "Read input bit 513 to 640", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6025, "Read input bit 641 to 768", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6026, "Read input bit 769 to 896", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6027, "Read input bit 897 to 1024", Type::boolean, AccessType::read_only),
			Entry(Entry::array_tag, 0x6030, "Polarity input bit 1 to 128", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6031, "Polarity input bit 129 to 256", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6032, "Polarity input bit 257 to 384", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6033, "Polarity input bit 385 to 512", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6034, "Polarity input bit 513 to 640", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6035, "Polarity input bit 641 to 768", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6036, "Polarity input bit 769 to 896", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6037, "Polarity input bit 897 to 1024", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6038, "Filter constant input bit 1 to 128", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6039, "Filter constant input bit 129 to 256", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603A, "Filter constant input bit 257 to 384", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603B, "Filter constant input bit 385 to 512", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603C, "Filter constant input bit 513 to 640", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603D, "Filter constant input bit 641 to 768", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603E, "Filter constant input bit 769 to 896", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x603F, "Filter constant input bit 897 to 1024", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6050, "Interrupt mask input bit 1 to 128", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6051, "Interrupt mask input bit 129 to 256", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6052, "Interrupt mask input bit 257 to 384", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6053, "Interrupt mask input bit 385 to 512", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6054, "Interrupt mask input bit 513 to 640", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6055, "Interrupt mask input bit 641 to 768", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6056, "Interrupt mask input bit 769 to 896", Type::boolean, AccessType::read_write),
			Entry(Entry::array_tag, 0x6057, "Interrupt mask input bit 897 to 1024", Type::boolean, AccessType::read_write),

			// TODO ...

			Entry(Entry::array_tag, 0x6100, "Read input 16-bit", Type::uint16, AccessType::read_only),
			Entry(Entry::array_tag, 0x6102, "Polarity input 16-bit", Type::uint16, AccessType::read_write),
			Entry(Entry::array_tag, 0x6103, "Filter constant input 16-bit", Type::uint16, AccessType::read_write),
			
			// TODO ...

			Entry(Entry::array_tag, 0x6200, "Write output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6202, "Change polarity output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6205, "Error mode output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6207, "Error value output 8-bit", Type::uint8, AccessType::read_write),
			Entry(Entry::array_tag, 0x6208, "Filter mask output 8-bit", Type::uint8, AccessType::read_write),
			
			// TODO ...
			
		};

	};

} // end namespace kaco