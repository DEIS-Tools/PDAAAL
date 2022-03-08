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
 * File:   SaxHandlerHelpers.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-02-2022.
 */

#ifndef PDAAAL_SAXHANDLERHELPERS_H
#define PDAAAL_SAXHANDLERHELPERS_H

#include "flags.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <variant>
#include <stack>
#include <cassert>

namespace pdaaal::parsing {
    // We wish to be able to stop parsing before the end of input, but still handle errors correctly.
    // Keep track of whether the error stream was used, and this way distinguish between error and early return.
    class SAXHandlerBase {
    public:
        explicit SAXHandlerBase(std::ostream& errors) : _errors(errors) {};

        bool parse_error(std::size_t location, const std::string& last_token, const nlohmann::detail::exception& e) {
            errors() << "error at line " << location << " with last token " << last_token << ". " << std::endl;
            errors() << "\terror message: " << e.what() << std::endl;
            return false;
        }
        [[nodiscard]] bool is_errored() const { return _errored; }

    protected:
        std::ostream& errors() {
            _errored = true;
            return _errors;
        }
    private:
        std::ostream& _errors;
        bool _errored = false;
    };

    // Object used when parsing JSON files with some required format.
    // A context can either be an object (where we keep track of the required keys)
    // or an array (where we keep track of the current index). Used by SAXHandlerContextStack.
    template<typename context_type, std::size_t N>
    struct JsonParserContext {
        using type_t = context_type; // Expose template parameter.
        using mask_t = utils::flag_mask<N>;
        using flag_t = typename mask_t::flag_t;
        using array_size_t = std::size_t;

        constexpr JsonParserContext(context_type type, flag_t flags, std::in_place_index_t<0>) noexcept
        : type(type), _v(std::in_place_index<0>, flags) {};
        constexpr JsonParserContext(context_type type, array_size_t flags, std::in_place_index_t<1>) noexcept
        : type(type), _v(std::in_place_index<1>, flags) {};

        [[nodiscard]] constexpr bool is_object() const { return _v.index() == 0; }
        [[nodiscard]] constexpr bool is_array() const { return _v.index() == 1; }

        constexpr void got_flag(flag_t value) { std::get<0>(_v).got_flag(value); }
        [[nodiscard]] constexpr bool needs_flag(flag_t value) const { return std::get<0>(_v).needs_flag(value); }
        [[nodiscard]] constexpr bool has_missing_flags() const { return std::get<0>(_v).has_missing_flags(); }
        std::vector<flag_t> get_missing_flags() const { return std::get<0>(_v).get_missing_flags(); }

        [[nodiscard]] constexpr array_size_t get_index() const { return std::get<1>(_v); }
        constexpr void increment_index() { ++std::get<1>(_v); }

        const context_type type;
    private:
        std::variant<mask_t, array_size_t> _v;
    };
    template<typename context, typename context::type_t type, size_t n_flags> inline constexpr context make_context_object() {
        return context(type, context::mask_t::template fill<n_flags>(), std::in_place_index<0>);
    }
    template<typename context, typename context::type_t type> inline constexpr context make_context_array() {
        return context(type, 0, std::in_place_index<1>);
    }

    // This class provides functionality for SAXHandlers (JSON parsers) that need to keep track of a context stack,
    // where contexts can have some required keys. Supports descriptive error messages.
    // The template <helper> class should provide the following:
    //   - 'enum class keys' with printing 'ostream& operator<<(ostream&,keys)'.
    //   - 'enum class context_type' with printing 'ostream& operator<<(ostream&,context_type)'.
    //   - 'static constexpr size_t N', which is the largest number of keys needed.
    //   - 'static constexpr keys get_key(context_type,flag)' that for object contexts map the type and flag to the corresponding key (only needed if handle_key or error_missing_keys are used).
    template<typename Helper>
    struct SAXHandlerContextStack : public SAXHandlerBase, public Helper {
        using keys = typename Helper::keys;
        using context_type = typename Helper::context_type;
        static constexpr std::size_t N = Helper::N;
        using context_t = JsonParserContext<context_type,N>;

        using number_integer_t = typename nlohmann::json::number_integer_t;
        using number_unsigned_t = typename nlohmann::json::number_unsigned_t;
        using number_float_t = typename nlohmann::json::number_float_t;
        using string_t = typename nlohmann::json::string_t;
        using binary_t = typename nlohmann::json::binary_t;

        explicit SAXHandlerContextStack(std::ostream& errors) : SAXHandlerBase(errors) {};
        explicit SAXHandlerContextStack(const SAXHandlerBase& base) : SAXHandlerBase(base) {};

        template<context_type type, size_t n_flags> static constexpr context_t context_object() {
            return make_context_object<context_t,type,n_flags>();
        }
        template<context_type type> static constexpr context_t context_array() {
            return make_context_array<context_t,type>();
        }

        void pop_context() { _context_stack.pop(); }
        void push_context(const context_t& c) { _context_stack.push(c); }
        void push_context(context_t&& c) { _context_stack.push(std::move(c)); }
        [[nodiscard]] bool no_context() const { return _context_stack.empty(); }
        [[nodiscard]] const context_t& current_context() const { return _context_stack.top(); }
        [[nodiscard]] context_t& current_context() { return _context_stack.top(); }
        [[nodiscard]] context_type current_context_type() const { return _context_stack.top().type; }

        template <context_type type, typename context_t::flag_t flag, keys current_key, keys... alternatives>
        bool handle_key() {
            static_assert(context_t::mask_t::is_single_flag(flag), "Template parameter flag must be a single key, not a union or empty.");
            static_assert(((Helper::get_key(type, flag) == current_key) || ... || (Helper::get_key(type, flag) == alternatives)),
                          "The result of get_key(type, flag) must match 'key' or one of the alternatives");
            assert(current_context().is_object());
            if (!current_context().needs_flag(flag)) {
                auto& s = (errors() << "Duplicate definition of key: ");
                print_keys<current_key,alternatives...>(s) << " in " << type << " object." << std::endl;
                return false;
            }
            current_context().got_flag(flag);
            last_key = current_key;
            return true;
        }

        bool element_done() {
            if (!no_context() && current_context().is_array()) {
                current_context().increment_index();
            } else {
                last_key = keys::none;
            }
            return true; // Allows for nice code at the use site.
        }

        bool error_unexpected(const std::string& what) {
            auto& s = (errors() << "error: Unexpected " << what);
            describe_context(s) << "." << std::endl;
            return false; // Allows for nice code at the use site.
        }
        template<typename value_t> bool error_unexpected(const std::string& what, const value_t& value, const std::string& explanation = "") {
            auto& s = (errors() << "error: Unexpected " << what << " value ");
            print_value(s, value);
            describe_context(s) << ".";
            if (!explanation.empty()) {
                s << " " << explanation;
            }
            s << std::endl;
            return false; // Allows for nice code at the use site.
        }
        bool error_unexpected_key(const string_t& key) {
            auto& s = (errors() << "error: Unexpected key ");
            print_value(s, key);
            if (no_context()) {
                s << " before the start of an object." << std::endl;
            } else {
                s << " in " << current_context_type() << " object." << std::endl;
            }
            return false; // Allows for nice code at the use site.
        }
        template<keys... alternatives> // If one out of multiple alternative keys are required, we can specify it here. (only supports one such set)
        bool error_missing_keys() {
            auto& s = (errors() << "error: Missing key(s): ");
            bool first = true;
            for (const auto& flag : current_context().get_missing_flags()) {
                if (!first) s << ", ";
                first = false;
                if constexpr (sizeof...(alternatives) == 0) {
                    s << "\"" << Helper::get_key(current_context_type(), flag) << "\"";
                } else {
                    auto key = Helper::get_key(current_context_type(), flag);
                    if (((key == alternatives) || ...)) {
                        print_keys<alternatives...>(s);
                    } else {
                        s << "\"" << key << "\"";
                    }
                }
            }
            s << " in " << current_context_type() << " object." << std::endl;
            return false;
        }
        template<typename value_t> static std::ostream& print_value(std::ostream& s, const value_t& value) {
            if constexpr(std::is_same_v<value_t,string_t>) {
                s << "\"" << value << "\"";
            } else if constexpr(std::is_same_v<value_t,bool>) {
                s << std::boolalpha << value;
            } else {
                s << value;
            }
            return s;
        }
        std::ostream& describe_context(std::ostream& s) const {
            if (no_context()) {
                s << " outside of object";
            } else {
                if (current_context().is_object()) {
                    s << " after key \"" << last_key << "\" in " << current_context_type();
                } else {
                    assert(current_context().is_array());
                    s << " in " << current_context_type() << " at index " << current_context().get_index();
                }
            }
            return s;
        }
        keys last_key = keys::none;
    private:
        template<keys key, keys... alternatives>
        static constexpr std::ostream& print_keys(std::ostream& s) {
            s << "\"" << key;
            ((s << "\"/\"" << alternatives), ...);
            return s << "\"";
        }

        std::stack<context_t> _context_stack;
    };

}

#endif //PDAAAL_SAXHANDLERHELPERS_H
