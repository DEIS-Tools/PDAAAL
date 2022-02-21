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

#include "pdaaal/internal/PAutomaton.h"
#include "AbstractionPDA.h"

namespace pdaaal {

    template <typename T, typename W = void>
    class AbstractionPAutomaton : public internal::PAutomaton<W> {
    private:
        using nfastate_t = typename NFA<T>::state_t;
    public:
        AbstractionPAutomaton(const AbstractionPDA<T,W>& pda, const NFA<T>& nfa, const std::vector<size_t>& states)
        : internal::PAutomaton<W>(pda, states, nfa.empty_accept()) {
            this->template construct<T,true,true>(nfa,states,
                    [&pda](const auto& e){ return pda.encode_labels(e._symbols, e._negated); },
                    [this](const auto& n, size_t n_id){ this->_pautomaton_to_nfastate_map.emplace(n_id, n); });
        }

        const nfastate_t* get_nfastate(size_t pautomaton_state_id) const {
            auto it = _pautomaton_to_nfastate_map.find(pautomaton_state_id);
            return it != _pautomaton_to_nfastate_map.end() ? it->second : nullptr;
        }

    private:
        std::unordered_map<size_t, const nfastate_t*> _pautomaton_to_nfastate_map;

    };

}

#endif //PDAAAL_ABSTRACTIONPAUTOMATON_H
