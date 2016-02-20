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
#include <stdexcept>

#include "canopen_error.h"

namespace kaco {

	/// This type of exception is thrown if there are
	/// problems accessing the object dictionary
	/// or arguments don't match the type of a
	/// dictionary entry.
	/// You can get the type of the error via get_type()
	/// and the name of the causing entry via
	/// get_entry_name(). 
	class dictionary_error : public canopen_error {

	public:

		/// Exact type of the error
		enum class type {
			unknown_entry,
			read_only,
			write_only,
			wrong_type,
			no_array,
			mapping_size,
			mapping_overlap
		};

		/// Constructor
		/// \param error_type Type of the error
		/// \param entry_name Name of the dictionary entry
		/// \param additional_information Additional information, appended to the error type string in what()
		explicit dictionary_error(type error_type, const std::string& entry_name, const std::string& additional_information = "");

		/// Destructor
		virtual ~dictionary_error() { }

		/// Returns error description
		virtual const char* what() const noexcept override;

		/// Returns type of the error
		type get_type() const noexcept;

		/// Returns the name of the dictionary entry.
		std::string get_entry_name() const noexcept;

	private:

		std::string m_message;
		std::string m_entry_name;
		type m_type;

	};

} // end namespace kaco