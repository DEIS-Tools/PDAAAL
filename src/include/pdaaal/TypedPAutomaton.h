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
 * File:   TypedPAutomaton.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 16-11-2021.
 */

#ifndef PDAAAL_TYPEDPAUTOMATON_H
#define PDAAAL_TYPEDPAUTOMATON_H

#include <pdaaal/PAutomaton.h>
#include <pdaaal/TypedPDA.h>

#include <nlohmann/json.hpp>

namespace pdaaal {

    template<typename label_t, typename W, fut::type Container, typename state_t, bool skip_state_mapping, bool indirect = true>
    class TypedPAutomaton : public PAutomaton<W,indirect> {
        using parent_t = PAutomaton<W,indirect>;
        using typed_pda_t = TypedPDA<label_t,W,Container,state_t,skip_state_mapping>;
    public:
        TypedPAutomaton(const TypedPDA<label_t,W,Container,state_t,skip_state_mapping>& pda, const NFA<label_t>& nfa, const std::vector<size_t>& states)
        : parent_t(pda, nfa, states), _typed_pda(pda) { }
        // Same, but where the NFA contains the symbols mapped to ids already.
        TypedPAutomaton(const TypedPDA<label_t,W,Container,state_t,skip_state_mapping>& pda, const NFA<uint32_t>& nfa, const std::vector<size_t>& states)
        : parent_t(static_cast<const PDA<W,Container>&>(pda), nfa, states), _typed_pda(pda) { }

        [[nodiscard]] nlohmann::json to_json(const std::string& name = "P-automaton") const {
            nlohmann::json j;
            j[name] = *this;
            return j;
        }
        const typed_pda_t& typed_pda() const { return _typed_pda; }

    private:
        const typed_pda_t& _typed_pda;
    };

    template<typename label_t, typename W, fut::type C, typename state_t, bool skip_state_mapping, bool indirect>
    void to_json(json& j, const TypedPAutomaton<label_t,W,C,state_t,skip_state_mapping,indirect>& automaton) {
        j = json::object();
        size_t num_pda_states = automaton.pda().states().size();
        auto state_to_string = [num_pda_states,&automaton](size_t state){
            return state < num_pda_states ? details::label_to_string(automaton.typed_pda().get_state(state)) : details::label_to_string(state);
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
                    edge["label"] = label == PAutomaton<W,indirect>::epsilon ? "" : details::label_to_string(automaton.typed_pda().get_symbol(label));
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

#endif //PDAAAL_TYPEDPAUTOMATON_H
