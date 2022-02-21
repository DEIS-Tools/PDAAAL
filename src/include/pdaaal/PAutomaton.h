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
 * File:   PAutomaton.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 16-11-2021.
 */

#ifndef PDAAAL_PAUTOMATON_H
#define PDAAAL_PAUTOMATON_H

#include "PDA.h"
#include "pdaaal/internal/PAutomaton.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

namespace pdaaal {

    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type = TraceInfoType::Single>
    class PAutomaton : public internal::PAutomaton<W,trace_info_type>, private std::conditional_t<skip_state_mapping, no_state_mapping, state_mapping<state_t>> {
        static_assert(!skip_state_mapping || std::is_same_v<state_t,size_t>, "When skip_state_mapping==true, you must use state_t=size_t");
        using parent_t = internal::PAutomaton<W,trace_info_type>;
        using pda_t = PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>;
        using internal_pda_t = typename pda_t::parent_t;
    public:
        static constexpr auto epsilon = parent_t::epsilon;

        // Accept one control state with given stack.
        PAutomaton(const pda_t& pda, size_t initial_state, const std::vector<label_t>& initial_stack)
        : parent_t(static_cast<const internal_pda_t&>(pda), initial_state, pda.encode_pre(initial_stack)), _pda(pda) { };

        // Construct a PAutomaton that accepts a configuration <p,w> iff states contains p and nfa accepts w.
        PAutomaton(const pda_t& pda, const NFA<label_t>& nfa, const std::vector<size_t>& states)
        : parent_t(static_cast<const internal_pda_t&>(pda), states, nfa.empty_accept()), _pda(pda) {
            this->template construct<label_t>(nfa, states, [&pda](const auto& e){ return pda.encode_pre(e._symbols); });
        }
        // Same, but where the NFA contains the symbols mapped to ids already.
        PAutomaton(const pda_t& pda, const NFA<uint32_t>& nfa, const std::vector<size_t>& states)
        : parent_t(static_cast<const internal_pda_t&>(pda), nfa, states), _pda(pda) { };

        PAutomaton(const pda_t& pda, const std::vector<size_t>& special_initial_states, bool special_accepting = true)
        : parent_t(static_cast<const internal_pda_t&>(pda), special_initial_states, special_accepting), _pda(pda) { };

        [[nodiscard]] nlohmann::json to_json(const std::string& name = "P-automaton") const {
            nlohmann::json j;
            j[name] = *this;
            return j;
        }

        [[nodiscard]] std::pair<bool,size_t> exists_state(const state_t& state) const {
            if constexpr (skip_state_mapping) {
                return std::make_pair(state < this->states().size(), state);
            } else {
                auto res = _pda.exists_state(state);
                if (!res.first) {
                    auto [exists, temp_id] = this->_state_map.exists(state);
                    if (exists) {
                        return std::make_pair(true, temp_id + _pda.states().size());
                    } else {
                        return std::make_pair(false, temp_id);
                    }
                }
                return res;
            }
        }
        size_t insert_state(const state_t& state) {
            if constexpr (skip_state_mapping) {
                return state; // + this->pda().states().size();
            } else {
                return this->_state_map.insert(state).second + _pda.states().size();
            }
        }

        [[nodiscard]] state_t get_state(size_t id) const {
            if constexpr (skip_state_mapping) {
                return id;
            } else {
                if (id < _pda.states().size()) {
                    return _pda.get_state(id);
                } else {
                    return static_cast<const state_mapping<state_t>*>(this)->get_state(id - _pda.states().size());
                }
            }
        }
        [[nodiscard]] label_t get_symbol(size_t id) const { return _pda.get_symbol(id); }

    private:
        const pda_t& _pda;
    };
    // CTAD guides
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type = TraceInfoType::Single>
    PAutomaton(const PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>&, const NFA<label_t>&, const std::vector<size_t>&) -> PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>;
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type = TraceInfoType::Single>
    PAutomaton(const PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>&, const NFA<uint32_t>&, const std::vector<size_t>&) -> PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>;
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type = TraceInfoType::Single>
    PAutomaton(const PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>&, const std::vector<size_t>&, bool special_accepting = true) -> PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>;

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

    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type>
    void to_json(json& j, const PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>& automaton) {
        j = json::object();
        size_t num_pda_states = automaton.pda().states().size();
        auto state_to_string = [&automaton](size_t state){
            return details::label_to_string(automaton.get_state(state));
        };
        json j_states;
        for (const auto& state : automaton.states()) {
            auto j_state = json::object();
            if (state->_id < num_pda_states) {
                j_state["initial"] = true;
            }
            if (state->_accepting) {
                j_state["accepting"] = true;
            }
            j_state["edges"] = json::array();
            for (const auto& [to, labels] : state->_edges) {
                for (const auto& [label,tw] : labels) {
                    json edge;
                    if constexpr (skip_state_mapping) {
                        edge["to"] = to;
                    } else {
                        edge["to"] = state_to_string(to);
                    }
                    edge["label"] = label == automaton.epsilon ? "" : details::label_to_string(automaton.get_symbol(label));
                    j_state["edges"].emplace_back(edge);
                }
            }
            if constexpr (skip_state_mapping) {
                j_states.emplace_back(j_state);
            } else {
                j_states[state_to_string(state->_id)] = j_state;
            }
        }
        j["states"] = j_states;
    }

}

#endif //PDAAAL_PAUTOMATON_H
