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

#include "SaxHandlerHelpers.h"
#include <pdaaal/PDA.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <stack>
#include <memory>

using json = nlohmann::json;

namespace pdaaal::parsing {

    struct PdaSaxHelper {
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
        enum class context_type : uint8_t { unknown, initial, pda, state_array, states_object, state, rule_array, rule };
        friend constexpr std::ostream& operator<<(std::ostream& s, context_type type) {
            switch (type) {
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
        static constexpr std::size_t N = 3;
        static constexpr auto FLAG_1 = utils::flag_mask<N>::template flag<1>();
        static constexpr auto FLAG_2 = utils::flag_mask<N>::template flag<2>();
        static constexpr auto FLAG_3 = utils::flag_mask<N>::template flag<3>();
        static constexpr keys get_key(context_type type, typename utils::flag_mask<N>::flag_t flag) {
            switch (type) {
                case context_type::initial:
                    if (flag == FLAG_1) {
                        return keys::pda;
                    }
                    break;
                case context_type::pda:
                    if (flag == FLAG_1) {
                        return keys::states;
                    }
                    break;
                case context_type::rule:
                    switch (flag) {
                        case FLAG_1:
                            return keys::to;
                        case FLAG_2:
                            return keys::pop; // NOTE: Also keys::swap and keys::push
                        case FLAG_3:
                            return keys::weight;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            assert(false);
            return keys::unknown;
        }
    };

    template <typename W = weight<void>, bool use_state_names = true, bool embedded_parser = false>
    class PdaSaxHandler : public SAXHandlerContextStack<PdaSaxHelper> {
        using parent_t = SAXHandlerContextStack<PdaSaxHelper>;
    public:
        using pda_t = std::conditional_t<use_state_names,
                PDA<std::string, W, fut::type::vector, std::string>,
                PDA<std::string, W, fut::type::vector, size_t>>;
    private:
        using build_pda_t = std::conditional_t<use_state_names,
                PDA<std::string, W, fut::type::hash, std::string>,
                PDA<std::string, W, fut::type::hash, size_t>>;

        static constexpr bool expect_weight = W::is_weight;

        static constexpr context_t unknown_context = context_object<context_type::unknown, 0>();
        static constexpr context_t initial_context = context_object<context_type::initial, 1>();
        static constexpr context_t pda_context = context_object<context_type::pda, 1>();
        static constexpr context_t state_array = context_array<context_type::state_array>();
        static constexpr context_t states_object = context_object<context_type::states_object, 0>();
        static constexpr context_t state_context = context_object<context_type::state, 0>();
        static constexpr context_t rule_array = context_array<context_type::rule_array>();
        static constexpr context_t rule_context = context_object<context_type::rule, W::is_weight?3:2>();

        build_pda_t build_pda;

        size_t current_from_state = 0;
        std::vector<uint32_t> current_pre;
        bool current_wildcard = false;
        typename internal::PDA<W>::rule_t current_rule;

        void init() {
            if constexpr(embedded_parser) {
                push_context(initial_context);
                last_key = keys::pda;
            }
        }
    public:

        explicit PdaSaxHandler(std::ostream& errors = std::cerr) : parent_t(errors) { init(); };
        explicit PdaSaxHandler(const SAXHandlerBase& base) : parent_t(base) { init(); };

        pda_t get_pda() {
            return pda_t(std::move(build_pda));
        }

        bool null() {
            switch (last_key) {
                case keys::unknown:
                    break;
                default:
                    error_unexpected("null value");
                    return false;
            }
            element_done();
            return true;
        }
        bool boolean(bool value) {
            switch (last_key) {
                case keys::unknown:
                    break;
                default:
                    error_unexpected("boolean", value);
                    return false;
            }
            element_done();
            return true;
        }
        bool number_integer(number_integer_t value) {
            switch (last_key) {
                case keys::unknown:
                    break;
                case keys::weight:
                    if constexpr (expect_weight && std::numeric_limits<typename W::type>::is_signed) {
                        if (value >= std::numeric_limits<typename W::type>::max()) {
                            errors() << "error: Integer value " << value << " is too large. Maximum value is: " << std::numeric_limits<typename W::type>::max()-1 << std::endl;
                            return false;
                        }
                        if (value <= std::numeric_limits<typename W::type>::min()) {
                            errors() << "error: Integer value " << value << " is too low. Minimum value is: " << std::numeric_limits<typename W::type>::min()+1 << std::endl;
                            return false;
                        }
                        current_rule._weight = value;
                        break;
                    }
                default:
                    error_unexpected("integer", value);
                    return false;
            }
            element_done();
            return true;
        }
        bool number_unsigned(number_unsigned_t value) {
            switch (last_key) {
                case keys::to:
                    if constexpr(use_state_names) {
                        errors() << "error: Rule destination was numeric: " << value << ", but string state names are used." << std::endl;
                        return false;
                    } else {
                        current_rule._to = value;
                        break;
                    }
                case keys::weight:
                    if constexpr (expect_weight && !W::is_vector) { // TODO: Parameterize on weight type...
                        if (value >= std::numeric_limits<typename W::type>::max()) {
                            errors() << "error: Unsigned value " << value << " is too large. Maximum value is: " << std::numeric_limits<typename W::type>::max()-1 << std::endl;
                            return false;
                        }
                        current_rule._weight = value;
                    }
                    break;
                case keys::unknown:
                    break;
                default:
                    error_unexpected("unsigned", value);
                    return false;
            }
            element_done();
            return true;
        }
        bool number_float(number_float_t value, const string_t& /*unused*/) {
            switch (last_key) {
                case keys::unknown:
                    break;
                default:
                    error_unexpected("float", value);
                    return false;
            }
            element_done();
            return true;
        }
        bool string(string_t& value) {
            if (no_context()) {
                error_unexpected("string", value);
                return false;
            }
            switch (last_key) {
                case keys::to:
                    if constexpr (use_state_names) {
                        current_rule._to = build_pda.insert_state(value);
                        break;
                    } else {
                        errors() << "error: Rule \"to\" state was string: " << value << ", but state names are disabled in this setting. Try with --state-names" << std::endl;
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
                    error_unexpected("string", value);
                    return false;
            }
            return true;
        }
        bool binary(binary_t& /*val*/) {
            if (last_key == keys::unknown) {
                element_done();
                return true;
            }
            error_unexpected("binary value");
            return false;
        }
        bool start_object(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                push_context(initial_context);
                return true;
            }
            switch (current_context_type()) {
                case context_type::state_array:
                    push_context(state_context);
                    return true;
                case context_type::rule_array:
                    push_context(rule_context);
                    return true;
                default:
                    break;
            }
            switch (last_key) {
                case keys::pda:
                    push_context(pda_context);
                    break;
                case keys::states:
                    if constexpr(use_state_names) {
                        push_context(states_object);
                        break;
                    } else {
                        errors() << "error: Found object after key: " << last_key << ", but state names are disabled in this setting. Try with --state-names." << std::endl;
                        return false;
                    }
                case keys::state_name:
                    push_context(state_context);
                    break;
                case keys::from_label:
                    push_context(rule_context);
                    break;
                case keys::unknown:
                    push_context(unknown_context);
                    break;
                default:
                    error_unexpected("start of object");
                    return false;
            }
            return true;
        }
        bool key(string_t& key) {
            if (no_context()) {
                errors() << "Expected the start of an object before key: " << key << std::endl;
                return false;
            }
            switch (current_context_type()) {
                case context_type::initial:
                    if (key == "pda") {
                        if (!handle_key<context_type::initial,FLAG_1,keys::pda>()) return false;
                    } else {
                        last_key = keys::unknown;
                    }
                    break;
                case context_type::pda:
                    if (key == "states") {
                        if (!handle_key<context_type::pda,FLAG_1,keys::states>()) return false;
                    } else { // "additionalProperties": true
                        last_key = keys::unknown;
                    }
                    break;
                case context_type::states_object:
                    if constexpr(use_state_names) {
                        last_key = keys::state_name;
                        current_from_state = build_pda.insert_state(key);
                        break;
                    } else {
                        errors() << "error: Encountered state name: \"" << key << "\" in context: " << current_context_type() << ", but state names are disabled in this setting." << std::endl;
                        return false;
                    }
                case context_type::state:
                    last_key = keys::from_label;
                    if (key == "*") {
                        current_pre = std::vector<uint32_t>{};
                        current_wildcard = true;
                    } else {
                        current_pre = std::vector<uint32_t>{build_pda.insert_label(key)};
                        current_wildcard = false;
                    }
                    break;
                case context_type::rule:
                    if (key == "to") {
                        if (!handle_key<context_type::rule,FLAG_1,keys::to>()) return false;
                    } else if (key == "pop") {
                        if (!handle_key<context_type::rule,FLAG_2,keys::pop, keys::swap, keys::push>()) return false;
                    } else if (key == "swap") {
                        if (!handle_key<context_type::rule,FLAG_2,keys::swap, keys::pop, keys::push>()) return false;
                    } else if (key == "push") {
                        if (!handle_key<context_type::rule,FLAG_2,keys::push, keys::pop, keys::swap>()) return false;
                    } else {
                        if constexpr (expect_weight) {
                            if (key == "weight") {
                                if (!handle_key<context_type::rule,FLAG_3,keys::weight>()) return false;
                                break;
                            }
                        }
                        errors() << "Unexpected key in operation object: " << key << std::endl;
                        return false;
                    }
                    break;
                case context_type::unknown:
                    break;
                default:
                    errors() << "error: Encountered unexpected key: \"" << key << "\" in context: " << current_context_type() << std::endl;
                    return false;
            }
            return true;
        }
        bool end_object() {
            if (no_context()) {
                errors() << "error: Unexpected end of object." << std::endl;
                return false;
            }
            if (current_context().has_missing_flags()) {
                auto& s = (errors() << "error: Missing key(s): ");
                bool first = true;
                for (const auto& flag : current_context().get_missing_flags()) {
                    if (!first) s << ", ";
                    first = false;
                    auto key = get_key(current_context_type(), flag);
                    s << key;
                    if (key == keys::pop) {
                        s << "/" << keys::swap << "/" << keys::push;
                    }
                }
                s << " in object: " << current_context_type() << std::endl;
                return false;
            }
            switch (current_context_type()) {
                case context_type::state:
                    if constexpr (!use_state_names) {
                        ++current_from_state;
                    }
                    break;
                case context_type::rule:
                    build_pda.add_rule_detail(current_from_state, current_rule, current_wildcard, current_pre);
                    break;
                default:
                    break;
            }
            pop_context();
            if constexpr(embedded_parser) {
                if (current_context_type() == context_type::initial) {
                    return false; // Stop using this SAXHandler.
                }
            }
            element_done();
            return true;
        }
        bool start_array(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                errors() << "error: Encountered start of array, but must start with an object." << std::endl;
                return false;
            }
            switch (last_key) {
                case keys::states:
                    if constexpr (use_state_names) {
                        errors() << "Unexpected start of array after key " << last_key << ". Note that state names are used in this setting." << std::endl;
                        return false;
                    } else {
                        push_context(state_array);
                        current_from_state = 0;
                        break;
                    }
                case keys::from_label:
                    push_context(rule_array);
                    break;
                case keys::unknown:
                    push_context(unknown_context);
                    break;
                default:
                    error_unexpected("start of array");
                    return false;
            }
            return true;
        }
        bool end_array() {
            if (no_context()) {
                errors() << "error: Unexpected end of array." << std::endl;
                return false;
            }
            pop_context();
            element_done();
            return true;
        }
    };

    class PdaJsonParser {
    public:
        template <typename W = weight<void>, bool use_state_names = true>
        static auto parse(std::istream& stream, std::ostream& /*warnings*/, json::input_format_t format = json::input_format_t::json) {
            std::stringstream es; // For errors;
            PdaSaxHandler<W,use_state_names> my_sax(es);
            if (!json::sax_parse(stream, &my_sax, format)) {
                throw std::runtime_error(es.str());
            }
            return my_sax.get_pda();
        }
    };
}

#endif //PDAAAL_PDAJSONPARSER_H
