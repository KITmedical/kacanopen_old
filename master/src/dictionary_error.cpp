/*
 * Copyright (c) 2015-2016, Thomas Keh
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
 
#include "dictionary_error.h"

namespace kaco {

	dictionary_error::dictionary_error(type error_type, const std::string& entry_name, const std::string& additional_information)
		: canopen_error("")
		{

		switch (error_type) {

			case type::unknown_entry:
				m_message = "Dictionary entry \""+entry_name+"\" not available.";
				break;
			case type::read_only:
				m_message = "Attempt to write the read-only dictionary entry \""+entry_name+"\".";
				break;
			case type::write_only:
				m_message = "Attempt to read the write-only dictionary entry \""+entry_name+"\".";
				break;
			case type::wrong_type:
				m_message = "Data type does not match to the dictionary entry \""+entry_name+"\".";
				break;
			case type::no_array:
				m_message = "Dictionary entry \""+entry_name+"\" is no array, but you specified an array index.";
				break;
			case type::mapping_size:
				m_message = "Invalid mapping size.";
				break;
			case type::mapping_overlap:
				m_message = "Mappings overlap for this PDO.";
				break;
			case type::unknown_operation:
				m_message = "Operation \""+entry_name+"\" not available.";
				break;
			case type::unknown_constant:
				m_message = "Constant \""+entry_name+"\" not available.";
				break;

			// no default -> compiler should warn if a type is missing.

		}

		m_type = error_type;
		m_entry_name = entry_name;
		m_message = "Dictionary error: " + m_message + (additional_information.empty()?"":" "+additional_information);

	}

	const char* dictionary_error::what() const noexcept {
		return m_message.c_str();
	}

	dictionary_error::type dictionary_error::get_type() const noexcept {
		return m_type;
	}

	std::string dictionary_error::get_entry_name() const noexcept {
		return m_entry_name;
	}

}