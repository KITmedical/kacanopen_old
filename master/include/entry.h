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
#include <vector>
#include <functional>
#include <mutex>
#include <memory>

#include "type.h"
#include "value.h"
#include "access_method.h"

namespace kaco {

	enum AccessType {
		read_only,
		write_only,
		read_write,
		constant
	};

	class Entry {

	public:

		struct VariableTag {};
		struct ArrayTag {};

		static const VariableTag variable_tag;
		static const ArrayTag array_tag;

		/// type of a callback for a value changed event
		typedef std::function< void(const Value& value) > ValueChangedCallback;

		/// constructs an empty entry
		Entry();

		/// standard constructor
		Entry(VariableTag tag, uint16_t _index, uint8_t _subindex, std::string _name, Type _type, AccessType _access);

		/// array constructor
		Entry(ArrayTag tag, uint16_t _index, std::string _name, Type _type, AccessType _access);

		/// Sets the value. If the entry is an array, array_index can be specified
		void set_value(const Value& value, uint8_t array_index=0);

		/// Returns the value. If the entry is an array, array_index can be specified
		const Value& get_value(uint8_t array_index=0) const;

		/// Returns the data type.
		Type get_type() const;

		/// Registers a given function to be called when the value is changed.
		void add_value_changed_callback(ValueChangedCallback callback);


		uint16_t index;
		uint8_t subindex; // only used if is_array==false
		std::string name;
		Type type;
		AccessType access_type;

		bool is_array = false;
		ReadAccessMethod read_access_method = ReadAccessMethod::sdo;
		WriteAccessMethod write_access_method = WriteAccessMethod::sdo;
		std::string description = "";
		
		// maybe supported in future:
		//bool is_slice;
		//uint8_t slice_first_bit;
		//uint8_t slice_last_bit;

	private:

		std::vector<Value> m_value;
		std::vector<bool> m_valid;
		std::vector<ValueChangedCallback> m_value_changed_callbacks;
		Value m_dummy_value;

		/// read_write_mutex locks get_value() and set_value() because a PDO
		/// transmitter thread could read a value reference while it is
		/// set by the main thread.
		std::shared_ptr<std::mutex> read_write_mutex;


	};

} // end namespace kaco