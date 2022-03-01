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
 * File:   SaxHandlerDispatch.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 01-03-2022.
 */

#ifndef PDAAAL_SAXHANDLERDISPATCH_H
#define PDAAAL_SAXHANDLERDISPATCH_H

#include <pdaaal/utils/SaxHandlerHelpers.h>

namespace pdaaal::parsing {

    // This class allows combining multiple SAX handlers by providing SAXHandlerDispatch to nlohmann::json::sax_parse,
    // and using the _callback function to change the SAX handler dispatch functions during parsing.
    // The _callback function is called, when the internal SAX handler function returns false, but has not used the error stream.
    // This indirect approach is not the most efficient, but nlohmann/json does not support combining SAX handlers
    // (at least not in the public interface), so this is good enough for now.
    // An alternative could be to use a different JSON library, but the maturity of nlohmann/json is currently preferred.
    class SAXHandlerDispatch {
    public:
        using number_integer_t = typename nlohmann::json::number_integer_t;
        using number_unsigned_t = typename nlohmann::json::number_unsigned_t;
        using number_float_t = typename nlohmann::json::number_float_t;
        using string_t = typename nlohmann::json::string_t;
        using binary_t = typename nlohmann::json::binary_t;

        template<typename other_handler_t, typename CallbackFn>
        SAXHandlerDispatch(other_handler_t& other_handler, CallbackFn&& callback) : _callback(std::forward<CallbackFn>(callback)) {
            static_assert(std::is_base_of_v<SAXHandlerBase,other_handler_t>,
                    "The SAX handler needs to derive from SAXHandlerBase to allow checking if an error occurred when returning false.");
            set_dispatch(other_handler);
        }
        template<typename other_handler_t>
        constexpr void set_dispatch(other_handler_t& other_handler) {
            static_assert(std::is_base_of_v<SAXHandlerBase,other_handler_t>);
            _dispatch_null = [&](){ return other_handler.null(); };
            _dispatch_boolean = [&](bool value){ return other_handler.boolean(value); };
            _dispatch_number_integer = [&](number_integer_t value){ return other_handler.number_integer(value); };
            _dispatch_number_unsigned = [&](number_unsigned_t value){ return other_handler.number_unsigned(value); };
            _dispatch_number_float = [&](number_float_t value, const string_t& unused){ return other_handler.number_float(value,unused); };
            _dispatch_string = [&](string_t& value){ return other_handler.string(value); };
            _dispatch_binary = [&](binary_t& value){ return other_handler.binary(value); };
            _dispatch_key = [&](string_t& key){ return other_handler.key(key); };
            _dispatch_start_object = [&](std::size_t size){ return other_handler.start_object(size); };
            _dispatch_end_object = [&](){ return other_handler.end_object(); };
            _dispatch_start_array = [&](std::size_t size){ return other_handler.start_array(size); };
            _dispatch_end_array = [&](){ return other_handler.end_array(); };
            _dispatch_parse_error = [&](std::size_t location, const std::string& last_token, const nlohmann::detail::exception& e){ return other_handler.parse_error(location, last_token, e); };
            _dispatch_is_errored = [&](){ return other_handler.is_errored(); };
        }

        bool null() {
            return _dispatch_null() || (!_dispatch_is_errored() && _callback(*this));
        }
        bool boolean(bool value) {
            return _dispatch_boolean(value) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool number_integer(number_integer_t value) {
            return _dispatch_number_integer(value) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool number_unsigned(number_unsigned_t value) {
            return _dispatch_number_unsigned(value) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool number_float(number_float_t value, const string_t& unused) {
            return _dispatch_number_float(value,unused) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool string(string_t& value) {
            return _dispatch_string(value) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool binary(binary_t& value) {
            return _dispatch_binary(value) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool key(string_t& key) {
            return _dispatch_key(key) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool start_object(std::size_t size = std::size_t(-1)) {
            return _dispatch_start_object(size) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool end_object() {
            return _dispatch_end_object() || (!_dispatch_is_errored() && _callback(*this));
        }
        bool start_array(std::size_t size = std::size_t(-1)) {
            return _dispatch_start_array(size) || (!_dispatch_is_errored() && _callback(*this));
        }
        bool end_array() {
            return _dispatch_end_array() || (!_dispatch_is_errored() && _callback(*this));
        }
        bool parse_error(std::size_t location, const std::string& last_token, const nlohmann::detail::exception& e) {
            return _dispatch_parse_error(location,last_token,e) || (!_dispatch_is_errored() && _callback(*this));
        }

    private:
        std::function<bool(SAXHandlerDispatch&)> _callback;

        std::function<bool()> _dispatch_null;
        std::function<bool(bool)> _dispatch_boolean;
        std::function<bool(number_integer_t)> _dispatch_number_integer;
        std::function<bool(number_unsigned_t)> _dispatch_number_unsigned;
        std::function<bool(number_float_t value, const string_t&)> _dispatch_number_float;
        std::function<bool(string_t&)> _dispatch_string;
        std::function<bool(binary_t&)> _dispatch_binary;
        std::function<bool(string_t&)> _dispatch_key;
        std::function<bool(std::size_t)> _dispatch_start_object;
        std::function<bool()> _dispatch_end_object;
        std::function<bool(std::size_t)> _dispatch_start_array;
        std::function<bool()> _dispatch_end_array;
        std::function<bool()> _dispatch_is_errored;
        std::function<bool(std::size_t, const std::string&, const nlohmann::detail::exception&)> _dispatch_parse_error;
    };

}

#endif //PDAAAL_SAXHANDLERDISPATCH_H
