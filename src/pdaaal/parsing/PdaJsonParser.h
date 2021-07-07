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

#ifndef PDAAAL_PDAJSONPARSER_H
#define PDAAAL_PDAJSONPARSER_H

#include <json.hpp>

#include <pdaaal/TypedPDA.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <stack>
#include <memory>

using json = nlohmann::json;

namespace pdaaal {

    template <typename W = weight<void>, bool use_state_names = true>
    class PdaaalSAXHandler {
    public:
        using pda_t = std::conditional_t<use_state_names,
                TypedPDA<std::string, W, fut::type::vector, std::string>,
                TypedPDA<std::string, W, fut::type::vector, size_t>>;
    private:
        using build_pda_t = std::conditional_t<use_state_names,
                TypedPDA<std::string, W, fut::type::hash, std::string>,
                TypedPDA<std::string, W, fut::type::hash, size_t>>;

        static constexpr bool expect_weight = is_weighted<W>;

        enum class keys : uint32_t { none, unknown, pda, states, state_name, from_label, to, pop, swap, push, weight };
        friend constexpr std::ostream& operator<<(std::ostream& s, keys key) {
            switch (key) {
                case keys::none:
                    s << "<initial>";
                    break;
                case keys::unknown:
                    s << "<unknown>";
                    break;
                case keys::pda:
                    s << "pda";
                    break;
                case keys::states:
                    s << "states";
                    break;
                case keys::state_name:
                    s << "<name of a state>";
                    break;
                case keys::from_label:
                    s << "<from-label in a state>";
                    break;
                case keys::to:
                    s << "to";
                    break;
                case keys::pop:
                    s << "pop";
                    break;
                case keys::swap:
                    s << "swap";
                    break;
                case keys::push:
                    s << "push";
                    break;
                case keys::weight:
                    s << "weight";
                    break;
            }
            return s;
        }
        struct context {
            enum class context_type : uint32_t { unknown, initial, pda, state_array, states_object, state, rule_array, rule };
            friend constexpr std::ostream& operator<<(std::ostream& s, context_type t) {
                switch (t) {
                    case context_type::unknown:
                        s << "<unknown>";
                        break;
                    case context_type::initial:
                        s << "initial";
                        break;
                    case context_type::pda:
                        s << "pda";
                        break;
                    case context_type::state_array:
                        s << "states array";
                        break;
                    case context_type::states_object:
                        s << "states object";
                        break;
                    case context_type::state:
                        s << "state";
                        break;
                    case context_type::rule_array:
                        s << "rule array";
                        break;
                    case context_type::rule:
                        s << "rule";
                        break;
                }
                return s;
            }
            enum key_flag : uint32_t {
                NO_FLAGS = 0,
                FLAG_1 = 1,
                FLAG_2 = 2,
                FLAG_3 = 4,
                // Required values for each object type.
                REQUIRES_0 = NO_FLAGS,
                REQUIRES_1 = FLAG_1,
                REQUIRES_2 = FLAG_1 | FLAG_2,
                REQUIRES_3 = FLAG_1 | FLAG_2 | FLAG_3
            };
            static constexpr std::array<key_flag,3> all_flags{key_flag::FLAG_1, key_flag::FLAG_2, key_flag::FLAG_3};

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
            static constexpr keys get_key(context_type context_type, key_flag flag) {
                switch (context_type) {
                    case context_type::initial:
                        if (flag == context::key_flag::FLAG_1) {
                            return keys::pda;
                        }
                        break;
                    case context_type::pda:
                        if (flag == context::key_flag::FLAG_1) {
                            return keys::states;
                        }
                        break;
                    case context_type::rule:
                        if constexpr (expect_weight) {
                            switch (flag) {
                                case key_flag::FLAG_1:
                                    return keys::to;
                                case key_flag::FLAG_2:
                                    return keys::pop; // NOTE: Also keys::swap and keys::push
                                case key_flag::FLAG_3:
                                    return keys::weight;
                                default:
                                    break;
                            }
                        } else {
                            switch (flag) {
                                case key_flag::FLAG_1:
                                    return keys::to;
                                case key_flag::FLAG_2:
                                    return keys::pop; // NOTE: Also keys::swap and keys::push
                                default:
                                    break;
                            }
                        }
                        break;
                    default:
                        break;
                }
                assert(false);
                return keys::unknown;
            }
        };
        constexpr static context unknown_context = {context::context_type::unknown, context::NO_FLAGS };
        constexpr static context initial_context = {context::context_type::initial, context::REQUIRES_1 };
        constexpr static context pda_context = {context::context_type::pda, context::REQUIRES_1 };
        constexpr static context state_array = {context::context_type::state_array, context::NO_FLAGS };
        constexpr static context states_object = {context::context_type::states_object, context::NO_FLAGS };
        constexpr static context state_context = {context::context_type::state, context::NO_FLAGS };
        constexpr static context rule_array = {context::context_type::rule_array, context::NO_FLAGS };
        constexpr static context rule_context = {context::context_type::rule, is_weighted<W> ? context::REQUIRES_3 : context::REQUIRES_2 };

        template <typename context::context_type type, typename context::key_flag flag, keys current_key, keys... alternatives>
        // 'current_key' is the key to use. 'alternatives' are any other keys using the same flag in the same context (i.e. a one_of(current_key, alternatives...) requirement).
        bool handle_key() {
            static_assert(flag == context::FLAG_1 || flag == context::FLAG_2 || flag == PdaaalSAXHandler::context::FLAG_3,
                    "Template parameter flag must be a single key, not a union or empty.");
            static_assert(((context::get_key(type, flag) == current_key) || ... || (context::get_key(type, flag) == alternatives)),
                    "The result of get_key(type, flag) must match 'key' or one of the alternatives");
            if (!context_stack.top().needs_value(flag)) {
                errors << "Duplicate definition of key: \"" << current_key;
                ((errors << "\"/\"" << alternatives), ...);
                errors << "\" in " << type << " object. " << std::endl;
                return false;
            }
            context_stack.top().got_value(flag);
            last_key = current_key;
            return true;
        }

        std::stack<context> context_stack;
        keys last_key = keys::none;
        std::ostream& errors;

        build_pda_t build_pda;

        size_t current_from_state = 0;
        std::vector<uint32_t> current_pre;
        bool current_negated = false;
        //size_t current_to_state = 0;
        //op_t current_op = op_t::POP;
        //typename W::type current_weight
        //uint32_t current_op_label = std::numeric_limits<uint32_t>::max();
        typename PDA<W>::rule_t current_rule;

    public:
        using number_integer_t = typename json::number_integer_t;
        using number_unsigned_t = typename json::number_unsigned_t;
        using number_float_t = typename json::number_float_t;
        using string_t = typename json::string_t;
        using binary_t = typename json::binary_t;

        explicit PdaaalSAXHandler(std::ostream& errors = std::cerr) : errors(errors) {
            build_pda.insert_label(""); // Now id 0 can represent unknown labels.
        };

        pda_t get_pda() {
            return pda_t(std::move(build_pda));
        }

        bool null() {
            switch (this->last_key) {
                case keys::unknown:
                    break;
                default:
                    errors << "error: Unexpected null value after key: " << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool boolean(bool value) {
            switch (last_key) {
                case keys::unknown:
                    break;
                default:
                    errors << "error: Unexpected boolean value: " << value << " after key: " << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool number_integer(number_integer_t value) {
            switch (last_key) {
                case keys::unknown:
                    break;
                default:
                    errors << "error: Integer value: " << value << " found after key:" << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool number_unsigned(number_unsigned_t value) {
            switch (last_key) {
                case keys::to:
                    if constexpr(use_state_names) {
                        errors << "error: Rule destination was numeric: " << value << ", but string state names are used." << std::endl;
                        return false;
                    } else {
                        current_rule._to = value;
                        break;
                    }
                case keys::weight:
                    if constexpr (expect_weight) { // TODO: Parameterize on weight type...
                        current_rule._weight = value;
                    }
                    break;
                case keys::unknown:
                    break;
                default:
                    errors << "error: Unsigned value: " << value << " found after key: " << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool number_float(number_float_t value, const string_t& /*unused*/) {
            switch (last_key) {
                case keys::unknown:
                    break;
                default:
                    errors << "error: Float value: " << value << " comes after key: " << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool string(string_t& value) {
            if (context_stack.empty()) {
                errors << "error: Unexpected string value: \"" << value << "\" outside of object." << std::endl;
                return false;
            }
            switch (last_key) {
                case keys::to:
                    if constexpr (use_state_names) {
                        current_rule._to = build_pda.insert_state(value);
                        break;
                    } else {
                        errors << "error: Rule \"to\" state was string: " << value << ", but state names are disabled in this setting. Try with --state-names" << std::endl;
                        return false;
                    }
                case keys::pop:
                    assert(value.empty()); // TODO: Should this be error?
                    current_rule._operation = op_t::POP;
                    current_rule._op_label = std::numeric_limits<uint32_t>::max();
                    break;
                case keys::swap:
                    current_rule._operation = op_t::SWAP;
                    current_rule._op_label = build_pda.insert_label(value);
                    break;
                case keys::push:
                    current_rule._operation = op_t::PUSH;
                    current_rule._op_label = build_pda.insert_label(value);
                    break;
                case keys::unknown:
                    break;
                default:
                case keys::none:
                    errors << "error: String value: " << value << " found after key:" << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool binary(binary_t& val) {
            if (last_key == keys::unknown) {
                return true;
            }
            errors << "error: Unexpected binary value found after key:" << last_key << std::endl;
            return false;
        }
        bool start_object(std::size_t /*unused*/ = std::size_t(-1)) {
            if (context_stack.empty()) {
                context_stack.push(initial_context);
                return true;
            }
            switch (context_stack.top().type) {
                case context::context_type::state_array:
                    context_stack.push(state_context);
                    return true;
                case context::context_type::rule_array:
                    context_stack.push(rule_context);
                    return true;
                default:
                    break;
            }
            switch (last_key) {
                case keys::pda:
                    context_stack.push(pda_context);
                    break;
                case keys::states:
                    if constexpr(use_state_names) {
                        context_stack.push(states_object);
                        break;
                    } else {
                        errors << "error: Found object after key: " << last_key << ", but state names are disabled in this setting. Try with --state-names." << std::endl;
                        return false;
                    }
                case keys::state_name:
                    context_stack.push(state_context);
                    break;
                case keys::from_label:
                    context_stack.push(rule_context);
                    break;
                case keys::unknown:
                    context_stack.push(unknown_context);
                    break;
                default:
                    errors << "error: Found object after key: " << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool key(string_t& key) {
            if (context_stack.empty()) {
                errors << "Expected the start of an object before key: " << key << std::endl;
                return false;
            }
            switch (context_stack.top().type) {
                case context::context_type::initial:
                    if (key == "pda") {
                        if (!handle_key<context::context_type::initial,context::FLAG_1,keys::pda>()) return false;
                    } else {
                        last_key = keys::unknown;
                    }
                    break;
                case context::context_type::pda:
                    if (key == "states") {
                        if (!handle_key<context::context_type::pda,context::FLAG_1,keys::states>()) return false;
                    } else { // "additionalProperties": true
                        last_key = keys::unknown;
                    }
                    break;
                case context::context_type::states_object:
                    if constexpr(use_state_names) {
                        last_key = keys::state_name;
                        current_from_state = build_pda.insert_state(key);
                        break;
                    } else {
                        errors << "error: Encountered state name: \"" << key << "\" in context: " << context_stack.top().type << ", but state names are disabled in this setting." << std::endl;
                        return false;
                    }
                case context::context_type::state:
                    last_key = keys::from_label;
                    if (key == "*") {
                        current_pre = std::vector<uint32_t>{};
                        current_negated = true;
                    } else {
                        current_pre = std::vector<uint32_t>{build_pda.insert_label(key)};
                        current_negated = false;
                    }
                    break;
                case context::context_type::rule:
                    if (key == "to") {
                        if (!handle_key<context::context_type::rule,context::FLAG_1,keys::to>()) return false;
                    } else if (key == "pop") {
                        if (!handle_key<context::context_type::rule,context::FLAG_2,keys::pop, keys::swap, keys::push>()) return false;
                    } else if (key == "swap") {
                        if (!handle_key<context::context_type::rule,context::FLAG_2,keys::swap, keys::pop, keys::push>()) return false;
                    } else if (key == "push") {
                        if (!handle_key<context::context_type::rule,context::FLAG_2,keys::push, keys::pop, keys::swap>()) return false;
                    } else {
                        if constexpr (expect_weight) {
                            if (key == "weight") {
                                if (!handle_key<context::context_type::rule,context::FLAG_3,keys::weight>()) return false;
                                break;
                            }
                        }
                        errors << "Unexpected key in operation object: " << key << std::endl;
                        return false;
                    }
                    break;
                case context::context_type::unknown:
                    break;
                default:
                    errors << "error: Encountered unexpected key: \"" << key << "\" in context: " << context_stack.top().type << std::endl;
                    return false;
            }
            return true;
        }
        bool end_object() {
            if (context_stack.empty()) {
                errors << "error: Unexpected end of object." << std::endl;
                return false;
            }
            if (context_stack.top().missing_keys()) {
                errors << "error: Missing key(s): ";
                bool first = true;
                for (const auto& flag : context::all_flags) {
                    if (context_stack.top().needs_value(flag)) {
                        if (!first) errors << ", ";
                        first = false;
                        auto key = context::get_key(context_stack.top().type, flag);
                        errors << key;
                        if (key == keys::pop) {
                            errors << "/" << keys::swap << "/" << keys::push;
                        }
                    }
                }
                errors << " in object: " << context_stack.top().type << std::endl;
                return false;
            }
            switch (context_stack.top().type) {
                case context::context_type::state:
                    if constexpr (!use_state_names) {
                        ++current_from_state;
                    }
                    break;
                case context::context_type::rule:
                    build_pda.add_rule_detail(current_from_state, current_rule, current_negated, current_pre);
                    break;
                default:
                    break;
            }
            context_stack.pop();
            return true;
        }
        bool start_array(std::size_t /*unused*/ = std::size_t(-1)) {
            if (context_stack.empty()) {
                errors << "error: Encountered start of array, but must start with an object." << std::endl;
                return false;
            }
            switch (last_key) {
                case keys::states:
                    if constexpr (use_state_names) {
                        errors << "Unexpected start of array after key " << last_key << ". Note that state names are used in this setting." << std::endl;
                        return false;
                    } else {
                        context_stack.push(state_array);
                        current_from_state = 0;
                        break;
                    }
                case keys::from_label:
                    context_stack.push(rule_array);
                    break;
                case keys::unknown:
                    context_stack.push(unknown_context);
                    break;
                default:
                    errors << "Unexpected start of array after key " << last_key << std::endl;
                    return false;
            }
            return true;
        }
        bool end_array() {
            if (context_stack.empty()) {
                errors << "error: Unexpected end of array." << std::endl;
                return false;
            }
            context_stack.pop();
            return true;
        }
        bool parse_error(std::size_t location, const std::string& last_token, const nlohmann::detail::exception& e) {
            errors << "error at line " << location << " with last token " << last_token << ". " << std::endl;
            errors << "\terror message: " << e.what() << std::endl;
            return false;
        }
    };

    class PdaJSONParser {
    public:
        template <typename W = weight<void>, bool use_state_names = true>
        static auto parse(std::istream& stream, std::ostream& warnings, json::input_format_t format = json::input_format_t::json) {
            std::stringstream es; // For errors;
            PdaaalSAXHandler<W,use_state_names> my_sax(es);
            if (!json::sax_parse(stream, &my_sax, format)) {
                throw std::runtime_error(es.str());
            }
            return my_sax.get_pda();
        }
    };
}

#endif //PDAAAL_PDAJSONPARSER_H
