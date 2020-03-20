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
 * File:   Weight.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 17-03-2020.
 */

#ifndef PDAAAL_PDA_ADAPTER_H
#define PDAAAL_PDA_ADAPTER_H

#include "TypedPDA.h"

namespace pdaaal {

    template<typename T, typename W = void, typename C = std::less<W>>
    class PDA_Adapter : public TypedPDA<T,W,C> {
    public:
        explicit PDA_Adapter(const std::unordered_set<T> &all_labels) : TypedPDA<T,W,C>{all_labels} {};

        using state_t = typename TypedPDA<T,W,C>::template PDA<W,C>::state_t;

        [[nodiscard]] size_t initial() const { return 1; }
        [[nodiscard]] const std::vector<uint32_t>& initial_stack() const { return _initial_stack; }
        [[nodiscard]] size_t terminal() const { return 0; }

        void finalize() {
            for (auto& s : this->states_mutable()) {
                if (!s._pre_states.empty() && s._pre_states.front() == initial()) {
                    s._pre_states.erase(std::begin(s._pre_states));
                    s._pre_states.emplace_back(initial());
                }
            }
            for (auto& r : this->states_mutable()[initial()]._rules) {
                r._labels.clear();
                r._labels.merge(false, _initial_stack, std::numeric_limits<size_t>::max());
            }
        }

    private:
        const std::vector<uint32_t> _initial_stack{std::numeric_limits<uint32_t>::max() - 2}; // std::numeric_limits<uint32_t>::max() is reserved for epsilon transitions (and max-1 for a flag in trace_t).
    };
}

#endif //PDAAAL_PDA_ADAPTER_H
