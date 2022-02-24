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

#include <pdaaal/PAutomaton.h>

namespace pdaaal {

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
