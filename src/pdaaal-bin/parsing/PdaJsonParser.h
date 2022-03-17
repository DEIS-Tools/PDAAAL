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

#include <pdaaal/utils/SaxHandlerHelpers.h>
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
            return last_key == keys::unknown ? element_done() : error_unexpected("null value");
        }
        bool boolean(bool value) {
            return last_key == keys::unknown ? element_done() : error_unexpected("boolean", value);
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
                    return error_unexpected("integer", value);
            }
            return element_done();
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
                    return error_unexpected("unsigned", value);
            }
            return element_done();
        }
        bool number_float(number_float_t value, const string_t& /*unused*/) {
            return last_key == keys::unknown ? element_done() : error_unexpected("float", value);
        }
        bool string(string_t& value) {
            if (no_context()) {
                return error_unexpected("string", value);
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
                    if (current_wildcard && value == "*") { // Allow wildcard-noop as "*":{"swap":"*",...}. This gives for all labels "l" a rule "l":{"swap":"l",...}.
                        current_rule._operation = op_t::NOOP;
                        current_rule._op_label = std::numeric_limits<uint32_t>::max();
                    } else {
                        current_rule._operation = op_t::SWAP;
                        current_rule._op_label = build_pda.insert_label(value);
                    }
                    break;
                case keys::push:
                    current_rule._operation = op_t::PUSH;
                    current_rule._op_label = build_pda.insert_label(value);
                    break;
                case keys::unknown:
                    break;
                default:
                case keys::none:
                    return error_unexpected("string", value);
            }
            return element_done();
        }
        bool binary(binary_t& /*val*/) {
            return last_key == keys::unknown ? element_done() : error_unexpected("binary value");
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
                return error_unexpected_key(key);
            }
            switch (current_context_type()) {
                case context_type::initial:
                    if (key == "pda") {
                        return handle_key<context_type::initial,FLAG_1,keys::pda>();
                    } else { // "additionalProperties": true
                        last_key = keys::unknown;
                    }
                    break;
                case context_type::pda:
                    if (key == "states") {
                        return handle_key<context_type::pda,FLAG_1,keys::states>();
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
                        return handle_key<context_type::rule,FLAG_1,keys::to>();
                    } else if (key == "pop") {
                        return handle_key<context_type::rule,FLAG_2,keys::pop, keys::swap, keys::push>();
                    } else if (key == "swap") {
                        return handle_key<context_type::rule,FLAG_2,keys::swap, keys::pop, keys::push>();
                    } else if (key == "push") {
                        return handle_key<context_type::rule,FLAG_2,keys::push, keys::pop, keys::swap>();
                    } else {
                        if constexpr (expect_weight) {
                            if (key == "weight") {
                                return handle_key<context_type::rule,FLAG_3,keys::weight>();
                            }
                        }
                        return error_unexpected_key(key);
                    }
                    break;
                case context_type::unknown:
                    break;
                default:
                    return error_unexpected_key(key);
            }
            return true;
        }
        bool end_object() {
            if (no_context()) {
                errors() << "error: Unexpected end of object." << std::endl;
                return false;
            }
            if (current_context().has_missing_flags()) {
                return error_missing_keys<keys::pop,keys::swap,keys::push>();
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
            return element_done();
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
                    return error_unexpected("start of array");
            }
            return true;
        }
        bool end_array() {
            if (no_context()) {
                errors() << "error: Unexpected end of array." << std::endl;
                return false;
            }
            pop_context();
            return element_done();
        }
    };

    class PdaJsonParser {
    public:
        template <typename W = weight<void>, bool use_state_names = true>
        static auto parse(std::istream& stream, std::ostream& /*warnings*/, json::input_format_t format = json::input_format_t::json) {
            std::stringstream error_stream;
            PdaSaxHandler<W,use_state_names> pda_sax(error_stream);
            if (!json::sax_parse(stream, &pda_sax, format)) {
                throw std::runtime_error(error_stream.str());
            }
            return pda_sax.get_pda();
        }
    };

    // The PDA-type can be prepended to the PDA JSON structure to indicate the parameters
    // (weight type and whether states are named) for the PDA parser. 
    struct PdaTypeSaxHelper {
        enum class keys : uint32_t { none, state_names, weight_type };
        friend constexpr std::ostream& operator<<(std::ostream& s, keys key) {
            switch (key) {
                case keys::none:
                    s << "<initial>";
                    break;
                case keys::state_names:
                    s << "state-names";
                    break;
                case keys::weight_type:
                    s << "weight-type";
                    break;
            }
            return s;
        }
        enum class context_type : uint8_t { initial, weight_array };
        friend constexpr std::ostream& operator<<(std::ostream& s, context_type type) {
            switch (type) {
                case context_type::initial:
                    s << "<pda-type>";
                    break;
                case context_type::weight_array:
                    s << "weight-type array";
                    break;
            }
            return s;
        }
        static constexpr std::size_t N = 2;
        static constexpr auto FLAG_1 = utils::flag_mask<N>::template flag<1>();
        static constexpr auto FLAG_2 = utils::flag_mask<N>::template flag<2>();
        static constexpr keys get_key(context_type type, typename utils::flag_mask<N>::flag_t flag) {
            switch (type) {
                case context_type::initial:
                    if (flag == FLAG_1) {
                        return keys::state_names;
                    }
                    if (flag == FLAG_2) {
                        return keys::weight_type;
                    }
                    break;
                default:
                    break;
            }
            assert(false);
            return keys::none;
        }
    };

    class PdaTypeSaxHandler : public SAXHandlerContextStack<PdaTypeSaxHelper> {
        using parent_t = SAXHandlerContextStack<PdaTypeSaxHelper>;

        constexpr static context_t initial_context = context_object<context_type::initial,2>();
        constexpr static context_t weight_array_context = context_array<context_type::weight_array>();

        template <context_type type, size_t index, typename value_t>
        bool handle_index(const value_t& value) {
            if (current_context().get_index() != index) {
                auto& s = (errors() << "error: value ");
                print_value(s, value) << "at index " << current_context().get_index() << " in " << type << " was expected to be at index " << index << "." << std::endl;
                return false;
            }
            current_context().increment_index();
            return true;
        }

        template<typename T> struct weight_to_pda_sax_handler;
        template<typename... Args> struct weight_to_pda_sax_handler<std::tuple<Args...>> {
            using type = std::variant<PdaSaxHandler<weight<Args>, true, true>...,
                    PdaSaxHandler<weight<Args>, false, true>...>;
        };
        template<typename T> using weight_to_pda_sax_handler_t = typename weight_to_pda_sax_handler<T>::type;
        template<typename T> struct get_pda;
        template<typename... Args> struct get_pda<std::variant<Args...>> {
            using type = std::variant<decltype(std::declval<Args>().get_pda())...>;
        };
        template<typename T> using get_pda_t = typename get_pda<T>::type;

        using weight_types = std::tuple<void,uint32_t,int32_t,std::vector<uint32_t>,std::vector<int32_t>>;
    public:
        using pda_sax_variant_t = weight_to_pda_sax_handler_t<weight_types>;
        using pda_variant_t = get_pda_t<pda_sax_variant_t>;
    private:
        template <typename W>
        pda_sax_variant_t get_pda_sax_handler_w() {
            if (use_state_names) {
                return PdaSaxHandler<W, true, true>(static_cast<const SAXHandlerBase&>(*this));
            } else {
                return PdaSaxHandler<W, false, true>(static_cast<const SAXHandlerBase&>(*this));
            }
        }
        template <typename T>
        pda_sax_variant_t get_pda_sax_handler_a() {
            if (use_weight_array) {
                if (weight_array_size) {
                    switch (weight_array_size.value()) { // TODO: Should we just ignore fixed size std::array to limit the amount of different types the compiler needs to deal with.?
//                        case 2:
//                            return get_pda_sax_handler_w<weight<std::array<T,2>>>();
//                        case 3:
//                            return get_pda_sax_handler_w<weight<std::array<T,3>>>();
//                        case 4:
//                            return get_pda_sax_handler_w<weight<std::array<T,4>>>();
                        default:
                            return get_pda_sax_handler_w<weight<std::vector<T>>>();
                    }
                } else {
                    return get_pda_sax_handler_w<weight<std::vector<T>>>();
                }
            } else {
                return get_pda_sax_handler_w<weight<T>>();
            }
        }

        std::string weight_str;
        bool use_state_names = false;
        bool use_weight_array = false;
        std::optional<size_t> weight_array_size;

    public:
        explicit PdaTypeSaxHandler(std::ostream& errors) : parent_t(errors) {};
        explicit PdaTypeSaxHandler(const SAXHandlerBase& base) : parent_t(base) {};

        pda_sax_variant_t get_pda_sax_handler() {
            if (weight_str == "uint") {
                return get_pda_sax_handler_a<uint32_t>();
            } else if (weight_str == "int") {
                return get_pda_sax_handler_a<int32_t>();
            } else {
                return get_pda_sax_handler_w<weight<void>>();
            }
        }

        bool null() {
            return error_unexpected("null value");
        }
        bool boolean(bool value) {
            if (last_key == keys::state_names) {
                use_state_names = value;
                return element_done();
            } else {
                return error_unexpected("boolean", value);
            }
        }
        bool number_integer(number_integer_t value) {
            return error_unexpected("integer", value);
        }
        bool number_unsigned(number_unsigned_t value) {
            if (current_context_type() == context_type::weight_array) {
                if (!handle_index<context_type::weight_array,1>(value)) return false;
                weight_array_size.emplace(value);
                return element_done();
            }
            return error_unexpected("unsigned", value);
        }
        bool number_float(number_float_t value, const string_t& /*unused*/) {
            return error_unexpected("float", value);
        }
        bool string(string_t& value) {
            if (no_context()) {
                return error_unexpected("string", value);
            }
            if (current_context_type() == context_type::weight_array) {
                if (!handle_index<context_type::weight_array,0>(value)) return false;
                weight_str = value;
                return element_done();
            }
            if (last_key == keys::weight_type) {
                weight_str = value;
                return element_done();
            }
            return error_unexpected("string", value);
        }
        bool binary(binary_t& /*val*/) {
            return error_unexpected("binary value");
        }
        bool start_object(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                push_context(initial_context);
                return true;
            }
            return error_unexpected("start of object");
        }
        bool key(string_t& key) {
            if (no_context()) {
                return error_unexpected_key(key);
            }
            if (current_context_type() == context_type::initial) {
                if (key == "state-names") {
                    return handle_key<context_type::initial,FLAG_1,keys::state_names>();
                } else if (key == "weight-type") {
                    return handle_key<context_type::initial,FLAG_2,keys::weight_type>();
                }
            }
            return error_unexpected_key(key);
        }
        bool end_object() {
            if (no_context()) {
                errors() << "error: Unexpected end of object." << std::endl;
                return false;
            }
            if (current_context().has_missing_flags()) {
                return error_missing_keys();
            }
            pop_context();
            if (no_context()) {
                return false; // Stop using this SAXHandler.
            }
            return element_done();
        }
        bool start_array(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                errors() << "error: Encountered start of array, but must start with an object." << std::endl;
                return false;
            }
            if (last_key == keys::weight_type) {
                push_context(weight_array_context);
                use_weight_array = true;
                return true;
            }
            return error_unexpected("start of array");
        }
        bool end_array() {
            if (no_context()) {
                errors() << "error: Unexpected end of array." << std::endl;
                return false;
            }
            pop_context();
            return element_done();
        }
    };

}

#endif //PDAAAL_PDAJSONPARSER_H
