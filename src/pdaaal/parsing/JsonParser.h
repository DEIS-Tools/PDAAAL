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
 * File:   JsonParser.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-06-2021.
 */

#ifndef PDAAAL_JSONPARSER_H
#define PDAAAL_JSONPARSER_H

#include <json.hpp>

#include "pdaaal/TypedPDA.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <stack>
#include <memory>

using json = nlohmann::json;

namespace pdaaal {

    class PdaaalSAXHandler {
    private:
        enum class keys : uint32_t { none, unknown, pda, labels, states, state_name, from_label, to, pop, swap, push };
        friend constexpr std::ostream& operator<<( std::ostream&, keys key );

    public:
        struct context {
            enum class context_type : uint32_t { unknown, initial, pda, label_array, state_array, states_object, state, rule_array, rule };
            friend constexpr std::ostream& operator<<(std::ostream&, context_type type );
            enum key_flag : uint32_t {
                NO_FLAGS = 0,
                FLAG_1 = 1,
                FLAG_2 = 2,
                // Required values for each object type.
                REQUIRES_0 = NO_FLAGS,
                REQUIRES_1 = FLAG_1,
                REQUIRES_2 = FLAG_1 | FLAG_2,
            };
            static constexpr std::array<key_flag,2> all_flags{key_flag::FLAG_1, key_flag::FLAG_2};

            context_type type;
            key_flag values_left;

            constexpr void got_value(key_flag value) {
                values_left = static_cast<context::key_flag>(static_cast<uint32_t>(values_left) & ~static_cast<uint32_t>(value));
            }
            [[nodiscard]] constexpr bool needs_value(key_flag value) const {
                return static_cast<uint32_t>(value) == (static_cast<uint32_t>(values_left) & static_cast<uint32_t>(value));
            }
            [[nodiscard]] constexpr bool missing_keys() const {
                return values_left != NO_FLAGS;
            }
            static constexpr keys get_key(context_type context_type, key_flag flag);
        };
    private:
        constexpr static context unknown_context = {context::context_type::unknown, context::NO_FLAGS };
        constexpr static context initial_context = {context::context_type::initial, context::REQUIRES_1 };
        constexpr static context pda_context = {context::context_type::pda, context::REQUIRES_2 };
        constexpr static context label_array = {context::context_type::label_array, context::NO_FLAGS };
        constexpr static context state_array = {context::context_type::state_array, context::NO_FLAGS };
        constexpr static context states_object = {context::context_type::states_object, context::NO_FLAGS };
        constexpr static context state_context = {context::context_type::state, context::NO_FLAGS };
        constexpr static context rule_array = {context::context_type::rule_array, context::NO_FLAGS };
        constexpr static context rule_context = {context::context_type::rule, context::REQUIRES_2 };

        std::stack<context> context_stack;
        keys last_key = keys::none;
        std::ostream& errors;

        std::optional<TypedPDA<std::string>> build_pda;

        std::unordered_set<std::string> labels;
        bool numeric_states = false;
        std::string current_state_name;
        size_t current_from_state = 0;
        size_t current_to_state = 0;
        op_t current_op = op_t::POP;
        std::string current_from_label, current_op_label;

        bool add_label(const std::string& label);

        template <context::context_type type, context::key_flag flag, keys key, keys... alternatives> bool handle_key();
    public:
        using number_integer_t = typename json::number_integer_t;
        using number_unsigned_t = typename json::number_unsigned_t;
        using number_float_t = typename json::number_float_t;
        using string_t = typename json::string_t;
        using binary_t = typename json::binary_t;

        explicit PdaaalSAXHandler(std::ostream& errors = std::cerr) : errors(errors) {};

        size_t get_stuff() {
            return build_pda->number_of_labels();
        }

        bool null();
        bool boolean(bool value);
        bool number_integer(number_integer_t value);
        bool number_unsigned(number_unsigned_t value);
        bool number_float(number_float_t value, const string_t& /*unused*/);
        bool string(string_t& value);
        bool binary(binary_t& val);
        bool start_object(std::size_t /*unused*/ = std::size_t(-1));
        bool key(string_t& key);
        bool end_object();
        bool start_array(std::size_t /*unused*/ = std::size_t(-1));
        bool end_array();
        bool parse_error(std::size_t location, const std::string& last_token, const nlohmann::detail::exception& e);
    };

    class JsonParser {
    public:
        static auto parse(std::istream& stream, std::ostream& warnings, json::input_format_t format = json::input_format_t::json) {
            std::stringstream es; // For errors;
            PdaaalSAXHandler my_sax(es);
            if (!json::sax_parse(stream, &my_sax, format)) {
                throw std::runtime_error(es.str());
            }
            return my_sax.get_stuff();
        }
    };
}

#endif //PDAAAL_JSONPARSER_H
