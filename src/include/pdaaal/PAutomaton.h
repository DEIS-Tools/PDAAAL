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
        using parent2_t = std::conditional_t<skip_state_mapping, no_state_mapping, state_mapping<state_t>>;
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

        PAutomaton(PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>&& other, const pda_t& pda) noexcept // Move constructor, but update reference to PDA.
        : parent_t(std::move(other), static_cast<const internal_pda_t&>(pda)), parent2_t(std::move(other)), _pda(pda) {};
        virtual ~PAutomaton() = default;

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
                return state;
            } else {
                return this->_state_map.insert(state).second + _pda.states().size();
            }
        }

        [[nodiscard]] std::optional<state_t> get_state_optional(size_t id) const {
            if constexpr (skip_state_mapping) {
                return id;
            } else {
                if (id < _pda.states().size()) {
                    return _pda.get_state(id);
                } else {
                    auto offset = id - _pda.states().size();
                    if (offset < this->state_map_size()) {
                        return static_cast<const state_mapping<state_t>*>(this)->get_state(offset);
                    } else {
                        return std::nullopt;
                    }
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

    // Simplify type deduction elsewhere.
    namespace details {
        template<typename pda_t, TraceInfoType trace_info_type> struct pda_to_pautomaton;
        template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type>
        struct pda_to_pautomaton<PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>, trace_info_type> {
            using type = PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>;
        };
    }
    template<typename pda_t, TraceInfoType trace_info_type = TraceInfoType::Single>
    using pda_to_pautomaton_t = typename details::pda_to_pautomaton<pda_t,trace_info_type>::type;

    template<bool with_initial = true, typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type>
    void to_json_impl(json& j, const PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>& automaton) {
        j = json::object();
        size_t num_pda_states = automaton.pda().states().size();
        json j_edges = json::array();
        for (const auto& state : automaton.states()) {
            json j_from;
            if constexpr (skip_state_mapping) {
                j_from = state->_id;
            } else {
                if (auto s = automaton.get_state_optional(state->_id); s) {
                    j_from = details::label_to_string(s.value());
                } else {
                    j_from = state->_id;
                }
            }
            if constexpr(with_initial) {
                if (state->_id < num_pda_states) {
                    j["initial"].emplace_back(j_from);
                }
            }
            if (state->_accepting) {
                j["accepting"].emplace_back(j_from);
            }
            for (const auto& [to, labels] : state->_edges) {
                json j_to;
                if constexpr (skip_state_mapping) {
                    j_to = to;
                } else {
                    if (auto s = automaton.get_state_optional(to); s) {
                        j_to = details::label_to_string(s.value());
                    } else {
                        j_to = to;
                    }
                }
                for (const auto& [label,tw] : labels) {
                    j_edges.emplace_back(json::array(
                            {j_from,
                             label == automaton.epsilon ? "" : details::label_to_string(automaton.get_symbol(label)),
                             j_to}));
                }
            }
        }
        j["edges"] = j_edges;
    }
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type>
    void to_json(json& j, const PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type>& automaton) {
        to_json_impl(j,automaton);
    }

}

#endif //PDAAAL_PAUTOMATON_H
