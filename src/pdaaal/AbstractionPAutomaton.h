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
 * File:   AbstractionPAutomaton.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 14-12-2020.
 */

#ifndef PDAAAL_ABSTRACTIONPAUTOMATON_H
#define PDAAAL_ABSTRACTIONPAUTOMATON_H

#include "PAutomaton.h"

namespace pdaaal {

    template <typename T, typename W = void, typename C = std::less<W>, typename A = add<W>>
    class AbstractionPAutomaton : public PAutomaton<W,C,A> {
    private:
        using nfastate_t = typename NFA<T>::state_t;
    public:
        // TODO: This one is mostly copy-paste from PAutomaton. Clean up later...
        AbstractionPAutomaton(const AbstractionPDA<W,C>& pda, const NFA<T>& nfa, const std::vector<size_t>& states)
        : PAutomaton<W,C,A>(pda, states, nfa.empty_accept()) {
            std::unordered_map<const nfastate_t*, size_t> nfastate_to_id;
            std::vector<std::pair<const nfastate_t*,size_t>> waiting;
            auto get_nfastate_id = [this, &waiting, &nfastate_to_id](const nfastate_t* n) -> size_t {
                // Adds nfastate if not yet seen.
                size_t n_id;
                auto it = nfastate_to_id.find(n);
                if (it != nfastate_to_id.end()) {
                    n_id = it->second;
                } else {
                    n_id = this->add_state(false, n->_accepting);
                    nfastate_to_id.emplace(n, n_id);
                    waiting.emplace_back(n, n_id);
                    this->_pautomaton_to_nfastate_map.emplace(n_id, n);
                }
                return n_id;
            };

            for (const auto& i : nfa.initial()) {
                for (const auto& e : i->_edges) {
                    for (const nfastate_t* n : e.follow_epsilon()) {
                        size_t n_id = get_nfastate_id(n);
                        add_edges(states, n_id, e._negated, pda.encode_labels(e._symbols, e._negated));
                    }
                }
            }
            while (!waiting.empty()) {
                auto [top, top_id] = waiting.back();
                waiting.pop_back();
                for (const auto& e : top->_edges) {
                    for (const nfastate_t* n : e.follow_epsilon()) {
                        size_t n_id = get_nfastate_id(n);
                        add_edges(top_id, n_id, e._negated, pda.encode_labels(e._symbols, e._negated));
                    }
                }
            }
        }

        const nfastate_t* get_nfastate(size_t pautomaton_state_id) {
            auto [exists, it] = _pautomaton_to_nfastate_map.find(pautomaton_state_id);
            return exists ? it.second : nullptr;
        }

    private:
        std::unordered_map<size_t, const nfastate_t*> _pautomaton_to_nfastate_map;

    };

}

#endif //PDAAAL_ABSTRACTIONPAUTOMATON_H
