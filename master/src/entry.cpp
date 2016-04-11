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
 
#include "entry.h"
#include "logger.h"

#include <cassert>
#include <future>
#include <iomanip>
#include <memory>

namespace kaco {

Entry::Entry()
	: type(Type::invalid), read_write_mutex(new std::mutex)
	{ }

// standard constructor
Entry::Entry(Entry::VariableTag, uint16_t _index, uint8_t _subindex, std::string _name, Type _type, AccessType _access_type)
	: index(_index),
		subindex(_subindex),
		name(_name),
		type(_type),
		access_type(_access_type),
		read_write_mutex(new std::mutex)
	{ }

// array constructor
Entry::Entry(Entry::ArrayTag, uint16_t _index, std::string _name, Type _type, AccessType _access_type)
	: index(_index),
		subindex(0),
		name(_name),
		type(_type),
		access_type(_access_type),
		is_array(true),
		read_write_mutex(new std::mutex)
	{ }

void Entry::set_value(const Value& value, uint8_t array_index) {

	if (value.type != type) {
		ERROR("[Entry::set_value] You passed a value of wrong type.");
		return;
	}

	if (array_index>0 && !is_array) {
		ERROR("[Entry::set_value] This is no array but you specified an array_index.");
		return;
	}

	bool value_changed = false;

	{
		std::lock_guard<std::mutex> lock(*read_write_mutex);

		if (m_value.size()<=array_index) { 
			m_value.resize(array_index+1);
			value_changed = true;
		}

		if (m_valid.size()<=array_index) {
			m_valid.resize(array_index+1, false);
			value_changed = true;
		}

		if (!value_changed && (m_value[array_index].type != type || m_value[array_index] != value) ) {
			value_changed = true;
		}

		m_value[array_index] = value;
		m_valid[array_index] = true;
	}

	if (value_changed) {
		for (auto& callback : m_value_changed_callbacks) {
			// TODO: currently callbacks are only internal and it's ok to call them synchonously.
			//std::async(std::launch::async, callback, value);
			callback(value);
		}
	}

}

const Value& Entry::get_value(uint8_t array_index) const {

	std::lock_guard<std::mutex> lock(*read_write_mutex);

	if (valid(array_index)) {
		return m_value[array_index];
	} else {
		ERROR("[Entry::get_value] Value not valid.");
		return m_dummy_value;
	}

}

bool Entry::valid(uint8_t array_index) const {

	if (array_index>0 && !is_array) {
		ERROR("[Entry::valid] This is no array but you specified an array_index.");
		return false;
	}

	if ( array_index>=m_value.size()
		|| array_index>=m_valid.size()
		|| !m_valid[array_index] ) {
		return false;
	}

	return true;

}

Type Entry::get_type() const {
	return type;
}

void Entry::add_value_changed_callback(ValueChangedCallback callback) {
	m_value_changed_callbacks.push_back(std::move(callback));
}

void Entry::print() const {

	std::cout << "0x"<<std::hex<<index;
	std::cout << "/";
	std::cout << std::dec<<(unsigned)subindex;
	std::cout << "\t";
	std::cout << Utils::access_type_to_string(access_type);
	std::cout << "\t";
	std::cout << std::setw(75) << std::left << name;
	std::cout << " ";

	if (valid()) {
		std::cout << std::setw(10) << get_value();
	} else {
		std::cout << std::setw(10) << "empty";
	}

	std::cout << std::endl;

}

bool Entry::operator<(const Entry& other) const {
	return (index<other.index) || (index==other.index && subindex<other.subindex);
}

} // end namespace kaco