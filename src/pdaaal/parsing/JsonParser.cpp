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
 * File:   JsonParser.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-06-2021.
 */

#include <pdaaal/parsing/JsonParser.h>

namespace pdaaal {
    constexpr std::ostream& operator<<(std::ostream& s, PdaaalSAXHandler::keys key) {
        switch (key) {
            case PdaaalSAXHandler::keys::none:
                s << "<initial>";
                break;
            case PdaaalSAXHandler::keys::unknown:
                s << "<unknown>";
                break;
            case PdaaalSAXHandler::keys::pda:
                s << "pda";
                break;
            case PdaaalSAXHandler::keys::labels:
                s << "labels";
                break;
            case PdaaalSAXHandler::keys::states:
                s << "states";
                break;
            case PdaaalSAXHandler::keys::state_name:
                s << "<name of a state>";
                break;
            case PdaaalSAXHandler::keys::from_label:
                s << "<from-label in a state>";
                break;
            case PdaaalSAXHandler::keys::to:
                s << "to";
                break;
            case PdaaalSAXHandler::keys::pop:
                s << "pop";
                break;
            case PdaaalSAXHandler::keys::swap:
                s << "swap";
                break;
            case PdaaalSAXHandler::keys::push:
                s << "push";
                break;
        }
        return s;
    }

    constexpr std::ostream &operator<<(std::ostream& s, PdaaalSAXHandler::context::context_type type) {
        switch (type) {
            case PdaaalSAXHandler::context::context_type::unknown:
                s << "<unknown>";
                break;
            case PdaaalSAXHandler::context::context_type::initial:
                s << "initial";
                break;
            case PdaaalSAXHandler::context::context_type::pda:
                s << "pda";
                break;
            case PdaaalSAXHandler::context::context_type::label_array:
                s << "labels";
                break;
            case PdaaalSAXHandler::context::context_type::state_array:
                s << "states array";
                break;
            case PdaaalSAXHandler::context::context_type::states_object:
                s << "states object";
                break;
            case PdaaalSAXHandler::context::context_type::state:
                s << "state";
                break;
            case PdaaalSAXHandler::context::context_type::rule_array:
                s << "rule array";
                break;
            case PdaaalSAXHandler::context::context_type::rule:
                s << "rule";
                break;
        }
        return s;
    }
    constexpr PdaaalSAXHandler::keys PdaaalSAXHandler::context::get_key(PdaaalSAXHandler::context::context_type context_type, PdaaalSAXHandler::context::key_flag flag) {
        switch (context_type) {
            case PdaaalSAXHandler::context::context_type::initial:
                if (flag == PdaaalSAXHandler::context::key_flag::FLAG_1) {
                    return PdaaalSAXHandler::keys::pda;
                }
                break;
            case PdaaalSAXHandler::context::context_type::pda:
                switch (flag) {
                    case PdaaalSAXHandler::context::key_flag::FLAG_1:
                        return PdaaalSAXHandler::keys::labels;
                    case PdaaalSAXHandler::context::key_flag::FLAG_2:
                        return PdaaalSAXHandler::keys::states;
                    default:
                        break;
                }
                break;
            case PdaaalSAXHandler::context::context_type::rule:
                switch (flag) {
                    case PdaaalSAXHandler::context::key_flag::FLAG_1:
                        return PdaaalSAXHandler::keys::to;
                    case PdaaalSAXHandler::context::key_flag::FLAG_2:
                        return PdaaalSAXHandler::keys::pop; // NOTE: Also PdaaalSAXHandler::keys::swap and PdaaalSAXHandler::keys::push
                    default:
                        break;
                }
                break;
            default:
                break;
        }
        assert(false);
        return PdaaalSAXHandler::keys::unknown;
    }

    bool PdaaalSAXHandler::null() {
        switch (last_key) {
            case keys::unknown:
                break;
            default:
                errors << "error: Unexpected null value after key: " << last_key << std::endl;
                return false;
        }
        return true;
    }
    bool PdaaalSAXHandler::boolean(bool value) {
        switch (last_key) {
            case keys::unknown:
                break;
            default:
                errors << "error: Unexpected boolean value: " << value << " after key: " << last_key << std::endl;
                return false;
        }
        return true;
    }

    bool PdaaalSAXHandler::number_integer(PdaaalSAXHandler::number_integer_t value) {
        switch (last_key) {
            case keys::unknown:
                break;
            default:
                errors << "error: Integer value: " << value << " found after key:" << last_key << std::endl;
                return false;
        }
        return true;
    }

    bool PdaaalSAXHandler::number_unsigned(PdaaalSAXHandler::number_unsigned_t value) {
        switch (last_key) {
            case keys::to:
                if (!numeric_states) {
                    errors << "error: Rule destination was numeric: " << value << ", but string state names are used." << std::endl;
                    return false;
                }
                current_to_state = value;
                break;
            case keys::unknown:
                break;
            default:
                errors << "error: Unsigned value: " << value << " found after key: " << last_key << std::endl;
                return false;
        }
        return true;
    }

    bool PdaaalSAXHandler::number_float(PdaaalSAXHandler::number_float_t value, const PdaaalSAXHandler::string_t &) {
        switch (last_key) {
            case keys::unknown:
                break;
            default:
                errors << "error: Float value: " << value << " comes after key: " << last_key << std::endl;
                return false;
        }
        return true;
    }

    bool PdaaalSAXHandler::add_label(const std::string& label) {
        if (!labels.emplace(label).second) {
            errors << "error: Duplicate label \"" << label << "\" in labels array." << std::endl;
            return false;
        }
        return true;
    }

    bool PdaaalSAXHandler::string(PdaaalSAXHandler::string_t &value) {
        if (context_stack.empty()) {
            errors << "error: Unexpected string value: \"" << value << "\" outside of object." << std::endl;
            return false;
        }
        switch (context_stack.top().type) {
            case context::context_type::label_array:
                return add_label(value);
            default:
                break;
        }
        switch (last_key) {
            case keys::to:
                if (numeric_states) {
                    errors << "error: Rule \"to\" state was string: " << value << ", but numeric state indices are used." << std::endl;
                    return false;
                }
                // TODO: use state_name_mapping ... value
                //current_to_state = value;
                break;
            case keys::pop:
                assert(value.empty()); // TODO: Should this be error?
                current_op = op_t::POP;
                current_op_label = value;
                break;
            case keys::swap:
                current_op = op_t::SWAP;
                current_op_label = value;
                break;
            case keys::push:
                current_op = op_t::PUSH;
                current_op_label = value;
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

    bool PdaaalSAXHandler::binary(PdaaalSAXHandler::binary_t& val) {
        if (last_key == keys::unknown) {
            return true;
        }
        errors << "error: Unexpected binary value found after key:" << last_key << std::endl;
        return false;
    }

    bool PdaaalSAXHandler::start_object(std::size_t) {
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
                numeric_states = false;
                context_stack.push(states_object);
                break;
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

    template <PdaaalSAXHandler::context::context_type type, PdaaalSAXHandler::context::key_flag flag,
            PdaaalSAXHandler::keys current_key, PdaaalSAXHandler::keys... alternatives>
    // 'key' is the key to use. 'alternatives' are any other keys using the same flag in the same context (i.e. a one_of(key, alternatives...) requirement).
    bool PdaaalSAXHandler::handle_key() {
        static_assert(flag == PdaaalSAXHandler::context::FLAG_1 || flag == PdaaalSAXHandler::context::FLAG_2
                      //|| flag == PdaaalSAXHandler::context::FLAG_3 || flag == PdaaalSAXHandler::context::FLAG_4
                      ,"Template parameter flag must be a single key, not a union or empty.");
        static_assert(((PdaaalSAXHandler::context::get_key(type, flag) == current_key) || ... || (PdaaalSAXHandler::context::get_key(type, flag) == alternatives)),
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

    bool PdaaalSAXHandler::key(PdaaalSAXHandler::string_t &key) {
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
                if (key == "labels") {
                    if (!handle_key<context::context_type::pda,context::FLAG_1,keys::labels>()) return false;
                } else if (key == "states") {
                    if (!handle_key<context::context_type::pda,context::FLAG_2,keys::states>()) return false;
                } else { // "additionalProperties": true
                    last_key = keys::unknown;
                }
                break;
            case context::context_type::states_object:
                last_key = keys::state_name;
                current_state_name = key;
                break;
            case context::context_type::state:
                last_key = keys::from_label;
                current_from_label = key;
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
                    errors << "Unexpected key in operation object: " << key << std::endl;
                    return false;
                }
            case context::context_type::unknown:
                break;
            default:
                errors << "error: Encountered unexpected key: \"" << key << "\" in context: " << context_stack.top().type << std::endl;
                return false;
        }
        return true;
    }

    bool PdaaalSAXHandler::end_object() {
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
                    auto key = PdaaalSAXHandler::context::get_key(context_stack.top().type, flag);
                    if (key == PdaaalSAXHandler::keys::pop) {
                        errors << "/" << PdaaalSAXHandler::keys::swap << "/" << PdaaalSAXHandler::keys::push;
                    }
                    errors << key;
                }
            }
            errors << " in object: " << context_stack.top().type << std::endl;
            return false;
        }
        switch (context_stack.top().type) {
            case context::context_type::state:
                if (numeric_states) {
                    ++current_from_state;
                }
                break;
            case context::context_type::rule:
                if (build_pda) {
                    build_pda->add_rule(current_from_state, current_to_state, current_op, current_op_label, current_from_label);
                } else {
                    // TODO: Store rule for later.
                }
                break;
            default:
                break;
        }
        context_stack.pop();
        return true;
    }

    bool PdaaalSAXHandler::start_array(std::size_t) {
        if (context_stack.empty()) {
            errors << "error: Encountered start of array, but must start with an object." << std::endl;
            return false;
        }
        switch (last_key) {
            case keys::labels:
                context_stack.push(label_array);
                break;
            case keys::states:
                numeric_states = true;
                context_stack.push(state_array);
                break;
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

    bool PdaaalSAXHandler::end_array() {
        if (context_stack.empty()) {
            errors << "error: Unexpected end of array." << std::endl;
            return false;
        }
        switch (context_stack.top().type) {
            case context::context_type::label_array:
                build_pda.emplace(labels);
                break;
            default:
                break;
        }
        context_stack.pop();
        return true;
    }

    bool PdaaalSAXHandler::parse_error(std::size_t location, const std::string &last_token, const nlohmann::detail::exception &e) {
        errors << "error at line " << location << " with last token " << last_token << ". " << std::endl;
        errors << "\terror message: " << e.what() << std::endl;
        return false;
    }

}