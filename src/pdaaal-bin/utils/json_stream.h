/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Copyright Morten K. Schou
 */

/*
 * File:   json_stream.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on October 9, 2020
 */

// Taken from AalWiNes

#ifndef PDAAAL_JSON_STREAM_H
#define PDAAAL_JSON_STREAM_H

#include <nlohmann/json.hpp>
#include <string>
#include <iostream>
#include <iomanip>

using json = nlohmann::json;

class json_stream {
public:
    explicit json_stream(size_t indent_size = 4, std::ostream &out = std::cout) : _indent_size(indent_size), _out(out) { };
    virtual ~json_stream() {
        close();
    }

    void begin_object(const std::string& key) {
        start_entry(key);
        _started = false;
    }
    void end_object(){
        if (!_started) {
            _out << "{}";
            _started = true;
        } else {
            _out << std::endl;
            _indent--;
            indent() << "}";
        }
    }

    void entry(const std::string& key, const json& value) {
        start_entry(key) << value.dump();
    }

    void entry_object(const std::string& key, const json& value) {
        if (!value.is_object()) {
            entry(key,value);
        } else {
            begin_object(key);
            for (const auto& [k, v] : value.items()) {
                entry(k,v);
            }
            end_object();
        }
    }

    void close() {
        while (_indent > 0) {
            end_object();
        }
        _started = false;
    }

private:
    std::ostream& indent() {
        return _out << std::setw(_indent * _indent_size) << "";
    }
    std::ostream& start_entry(const std::string& key){
        if (!_started) {
            _out << "{" << std::endl;
            _started = true;
            _indent++;
        } else {
            _out << ',' << std::endl;
        }
        return indent() << "\"" << key << "\" : ";
    }

    bool _started = false;
    size_t _indent = 0;
    size_t _indent_size;
    std::ostream & _out;
};


#endif //PDAAAL_JSON_STREAM_H
