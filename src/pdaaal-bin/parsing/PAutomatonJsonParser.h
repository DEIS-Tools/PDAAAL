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
 * File:   PAutomatonJsonParser.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-02-2022.
 */

#ifndef PDAAAL_PAUTOMATONJSONPARSER_H
#define PDAAAL_PAUTOMATONJSONPARSER_H

#include "SaxHandlerHelpers.h"
#include <pdaaal/PAutomaton.h>

namespace pdaaal::parsing {

    struct PAutomatonSaxHelper {
        enum class keys : uint32_t { none, p_automaton, initial, accepting, edges };
        friend constexpr std::ostream& operator<<(std::ostream& s, keys key) {
            switch (key) {
                case keys::none:
                    s << "<none>";
                    break;
                case keys::p_automaton:
                    s << "P-automaton";
                    break;
                case keys::initial:
                    s << "initial";
                    break;
                case keys::accepting:
                    s << "accepting";
                    break;
                case keys::edges:
                    s << "edges";
                    break;
            }
            return s;
        }
        enum class context_type : uint8_t { initial, p_automaton, initial_array, accepting_array, edges_array, edge_context };
        friend constexpr std::ostream& operator<<(std::ostream& s, context_type t) {
            switch (t) {
                case context_type::initial:
                    s << "<initial>";
                    break;
                case context_type::p_automaton:
                    s << "P-automaton";
                    break;
                case context_type::initial_array:
                    s << "initial array";
                    break;
                case context_type::accepting_array:
                    s << "accepting array";
                    break;
                case context_type::edges_array:
                    s << "edges array";
                    break;
                case context_type::edge_context:
                    s << "edge";
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
                        return keys::p_automaton;
                    }
                    break;
                case context_type::p_automaton:
                    switch (flag) {
                        case FLAG_1:
                            return keys::accepting;
                        case FLAG_2:
                            return keys::edges;
                        case FLAG_3:
                            return keys::initial;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            assert(false);
            return keys::none;
        }
    };

    template <typename pda_t, TraceInfoType trace_info_type = TraceInfoType::Single, bool embedded_parser = false, bool require_initial = !embedded_parser>
    class PAutomatonSaxHandler : public SAXHandlerContextStack<PAutomatonSaxHelper> {
        using parent_t = SAXHandlerContextStack<PAutomatonSaxHelper>;

        using automaton_t = pda_to_pautomaton_t<pda_t,trace_info_type>;

        using pda_state_t = typename pda_t::expose_state_t;
        static constexpr bool skip_state_mapping = pda_t::expose_skip_state_mapping;

        constexpr static context_t initial_context = context_object<context_type::initial, 1>();
        constexpr static context_t p_automaton = context_object<context_type::p_automaton, require_initial ? 3 : 2>();
        constexpr static context_t initial_array = context_array<context_type::initial_array>();
        constexpr static context_t accepting_array = context_array<context_type::accepting_array>();
        constexpr static context_t edges_array = context_array<context_type::edges_array>();
        constexpr static context_t edge_context = context_array<context_type::edge_context>();

        pda_t& pda;
        automaton_t automaton;

        std::unordered_map<size_t,size_t> extra_state_map;

        std::string current_label;
        size_t current_from_state = 0;
        size_t current_to_state = 0;

        void init() {
            if constexpr(embedded_parser) {
                push_context(initial_context);
                last_key = keys::p_automaton;
            }
        }

        size_t number_state(number_unsigned_t state) {
            if constexpr(std::is_same_v<pda_state_t,size_t>) {
                auto [exists, id] = automaton.exists_state(state);
                if (exists) return id;
                if constexpr(!skip_state_mapping) {
                    id = automaton.add_state(false, false); // accepting is set later.
#ifndef NDEBUG
                    auto id2 =
#endif
                            automaton.insert_state(state);
                    assert(id == id2);
                    return id;
                }
            }
            if constexpr(skip_state_mapping || !std::is_same_v<pda_state_t,std::size_t>){
                // This is a non-initial state, without a name that is recorded, so we use an auxiliary mapping.
                auto it = extra_state_map.find(state);
                if (it != extra_state_map.end()) {
                    return it->second;
                }
                auto id = automaton.add_state(false, false); // accepting is set later.
                extra_state_map.emplace(state,id);
                return id;
            }
        }
        std::optional<size_t> string_state(const string_t& state) {
            if constexpr(std::is_same_v<pda_state_t,std::string>) {
                auto [exists, id] = automaton.exists_state(state);
                if (!exists) {
                    id = automaton.add_state(false, false); // accepting is set later.
#ifndef NDEBUG
                    auto id2 =
#endif
                            automaton.insert_state(state);
                    assert(id == id2);
                }
                return id;
            } else {
                error_unexpected("string", state, "PDA does not use string states.");
                return std::nullopt;
            }
        }

    public:

        explicit PAutomatonSaxHandler(pda_t& pda, std::ostream& errors = std::cerr) : parent_t(errors), pda(pda), automaton(pda,std::vector<size_t>()) { init(); };
        PAutomatonSaxHandler(pda_t& pda, const SAXHandlerBase& base) : parent_t(base), pda(pda), automaton(pda,std::vector<size_t>()) { init(); };

        automaton_t get_automaton() {
            return std::move(automaton);
        }

        bool null() { return error_unexpected("null value"); }
        bool boolean(bool value) { return error_unexpected("boolean", value); }
        bool number_integer(number_integer_t value) { return error_unexpected("integer", value); }
        bool number_unsigned(number_unsigned_t value) {
            switch (current_context_type()) {
                case context_type::initial_array:
                    if constexpr(std::is_same_v<pda_state_t,size_t>) {
                        if (!pda.exists_state(value).first) {
                            errors() << "Initial state " << value << " is not in PDA." << std::endl;
                            return false;
                        }
                    } else {
                        return error_unexpected("unsigned", value, "PDA does not use indexed states.");
                    }
                    break;
                case context_type::accepting_array:
                    automaton.set_state_accepting(number_state(value));
                    break;
                case context_type::edge_context:
                    switch (current_context().get_index()) {
                        case 0:
                            current_from_state = number_state(value);
                            break;
                        case 1:
                            return error_unexpected("unsigned", value);
                        case 2:
                            current_to_state = number_state(value);
                            break;
                        default:
                            return error_unexpected("unsigned", value, "An edge should contain exactly three elements.");
                    }
                    break;
                default:
                    return error_unexpected("unsigned", value);
            }
            return element_done();
        }
        bool number_float(number_float_t value, const string_t& /*unused*/) {
            return error_unexpected("float", value);
        }
        bool string(string_t& value) {
            if (no_context()) {
                return error_unexpected("string", value);
            }
            switch (current_context_type()) {
                case context_type::initial_array:
                    if constexpr(std::is_same_v<pda_state_t,std::string>) {
                        if (!pda.exists_state(value).first) {
                            errors() << "Initial state \"" << value << "\" is not in PDA." << std::endl;
                            return false;
                        }
                    } else {
                        return error_unexpected("string", value, "PDA does not use string states.");
                    }
                    break;
                case context_type::accepting_array: {
                    if (auto s = string_state(value); !s) return false; else {
                        automaton.set_state_accepting(s.value());
                    }
                    break;
                }
                case context_type::edge_context:
                    switch (current_context().get_index()) {
                        case 0:
                            if (auto s = string_state(value); !s) return false; else {
                                current_from_state = s.value();
                            }
                            break;
                        case 1:
                            current_label = value;
                            break;
                        case 2:
                            if (auto s = string_state(value); !s) return false; else {
                                current_to_state = s.value();
                            }
                            break;
                        default:
                            return error_unexpected("string", value, "An edge should contain exactly three elements.");
                    }
                    break;
                default:
                    return error_unexpected("string", value);
            }
            return element_done();
        }
        bool binary(binary_t& /*val*/) {
            return error_unexpected("binary value");
        }
        bool start_object(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                push_context(initial_context);
                return true;
            }
            if (last_key == keys::p_automaton) {
                push_context(p_automaton);
                return true;
            }
            return error_unexpected("start of object");
        }
        bool key(string_t& key) {
            if (no_context()) {
                return error_unexpected_key(key);
            }
            switch (current_context_type()) {
                case context_type::initial:
                    if (key == "P-automaton") {
                        return handle_key<context_type::initial,FLAG_1,keys::p_automaton>();
                    } else {
                        return error_unexpected_key(key);
                    }
                    break;
                case context_type::p_automaton:
                    if (key == "accepting") {
                        return handle_key<context_type::p_automaton,FLAG_1,keys::accepting>();
                    } else if (key == "edges"){
                        return handle_key<context_type::p_automaton,FLAG_2,keys::edges>();
                    } else if (key == "initial") {
                        if constexpr(require_initial) {
                            return handle_key<context_type::p_automaton,FLAG_3,keys::initial>();
                        } else {
                            last_key = keys::initial;
                        }
                    } else {
                        return error_unexpected_key(key);
                    }
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
                return error_missing_keys();
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
            if (current_context_type() == context_type::edges_array) {
                push_context(edge_context);
                current_from_state = std::numeric_limits<size_t>::max();
                current_to_state = std::numeric_limits<size_t>::max();
                current_label.clear();
                return true;
            }
            switch (last_key) {
                case keys::initial:
                    push_context(initial_array);
                    break;
                case keys::accepting:
                    push_context(accepting_array);
                    break;
                case keys::edges:
                    push_context(edges_array);
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
            switch (current_context_type()) {
                case context_type::edge_context:
                    assert(current_from_state != std::numeric_limits<size_t>::max());
                    assert(current_to_state != std::numeric_limits<size_t>::max());
                    if (current_label.empty()) {
                        automaton.add_epsilon_edge(current_from_state, current_to_state);
                    } else {
                        automaton.add_edge(current_from_state, current_to_state, pda.insert_label(current_label));
                    }
                    break;
                default:
                    break;
            }
            pop_context();
            return element_done();
        }
    };

    class PAutomatonJsonParser_New {
    public:
        template <TraceInfoType trace_info_type = TraceInfoType::Single, typename pda_t>
        static auto parse(std::istream& stream, pda_t& pda, json::input_format_t format = json::input_format_t::json) {
            std::stringstream error_stream;
            PAutomatonSaxHandler<pda_t,trace_info_type> automaton_sax(pda, error_stream);
            if (!json::sax_parse(stream, &automaton_sax, format)) {
                throw std::runtime_error(error_stream.str());
            }
            return automaton_sax.get_automaton();
        }
    };

    class PAutomatonJsonParser {
        public:
        template <TraceInfoType trace_info_type = TraceInfoType::Single, typename pda_t>
        static auto parse(const std::string& file, pda_t& pda, const std::string& name = "P-automaton") {
            std::ifstream file_stream(file);
            if (!file_stream.is_open()) {
                std::stringstream error;
                error << "Could not open " << name << " file: " << file << std::endl;
                throw std::runtime_error(error.str());
            }
            return parse<trace_info_type>(file_stream, pda, name);
        }
        template <TraceInfoType trace_info_type = TraceInfoType::Single, typename pda_t>
        static auto parse(std::istream& istream, pda_t& pda, const std::string& name = "P-automaton") {
            json j;
            istream >> j;
            return from_json<trace_info_type>(j[name], pda);
        }
        template <TraceInfoType trace_info_type, typename W>
        static auto from_json(const json& j, PDA<std::string,W,fut::type::vector, std::string>& pda) {
            return from_json<trace_info_type, std::string>(j, pda, [](const std::string& s) { return s; });
        }
        template <TraceInfoType trace_info_type, typename W, bool ssm>
        static auto from_json(const json& j, PDA<std::string,W,fut::type::vector, size_t, ssm>& pda) {
            return from_json<trace_info_type, size_t>(j, pda, [](const std::string& s) -> size_t { return std::stoul(s); });
        }
        private:
        template <TraceInfoType trace_info_type, typename state_t, typename W, bool skip_state_mapping>
        static auto from_json(const json& j,
                              PDA<std::string,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                              const std::function<state_t(const std::string&)>& state_mapping) {
            // TODO: Proper error checking and handling.!
            auto iterate_states = [&j,&state_mapping](const std::function<void(const state_t&,const json&)>& fn) {
                if constexpr (skip_state_mapping) {
                    assert(j["states"].is_array());
                    size_t i = 0;
                    for (const auto& j_state : j["states"]) {
                        fn(i, j_state);
                        ++i;
                    }
                } else {
                    assert(j["states"].is_object());
                    for (const auto& [name,j_state] : j["states"].items()) {
                        fn(state_mapping(name), j_state);
                    }
                }
            };
            std::vector<size_t> accepting_initial_states;
            std20::unordered_set<state_t> accepting_extra_states;
            iterate_states([&pda,&accepting_initial_states,&accepting_extra_states](const state_t& state, const json& j_state){
                auto [exists, id] = pda.exists_state(state);
                if (exists) {
                    if (j_state.contains("accepting") && j_state["accepting"].get<bool>()) {
                        accepting_initial_states.emplace_back(id);
                    }
                    assert(j_state.contains("initial") && j_state["initial"].get<bool>());
                } else {
                    if (j_state.contains("accepting") && j_state["accepting"].get<bool>()) {
                        accepting_extra_states.emplace(state);
                    }
                    assert(!(j_state.contains("initial") && j_state["initial"].get<bool>()));
                }
            });
            std::sort(accepting_initial_states.begin(), accepting_initial_states.end());
            accepting_initial_states.erase(std::unique(accepting_initial_states.begin(), accepting_initial_states.end()), accepting_initial_states.end());
            PAutomaton<std::string, W, state_t, skip_state_mapping, trace_info_type> automaton(pda, accepting_initial_states, true);

            auto state_to_id = [&automaton,&accepting_extra_states](const state_t& state){
                auto [exists, id] = automaton.exists_state(state);
                if (!exists) {
                    id = automaton.add_state(false, accepting_extra_states.contains(state));
                    auto id2 = automaton.insert_state(state);
                    assert(id == id2);
                }
                return id;
            };

            iterate_states([&state_to_id,&automaton,&state_mapping,&pda](const state_t& state, const json& j_state){
                size_t from = state_to_id(state);
                for (const auto& edge : j_state["edges"]) {
                    size_t to;
                    if constexpr (skip_state_mapping) {
                        assert(edge["to"].is_number_unsigned());
                        to = state_to_id(edge["to"].get<size_t>());
                    } else {
                        assert(edge["to"].is_string());
                        to = state_to_id(state_mapping(edge["to"].get<std::string>()));
                    }
                    auto label_string = edge["label"].get<std::string>();
                    if (label_string.empty()) {
                        automaton.add_epsilon_edge(from,to);
                    } else {
                        automaton.add_edge(from,to,pda.insert_label(label_string));
                    }
                }
            });
            return automaton;
        }
    };
};

#endif //PDAAAL_PAUTOMATONJSONPARSER_H
