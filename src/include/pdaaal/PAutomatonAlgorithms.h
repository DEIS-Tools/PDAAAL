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
 * File:   PAutomatonAlgorithms.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-08-2021.
 */

#ifndef PDAAAL_PAUTOMATONALGORITHMS_H
#define PDAAAL_PAUTOMATONALGORITHMS_H

#include <pdaaal/utils/workset.h>
#include <pdaaal/PAutomaton.h>

namespace pdaaal {

    template<typename W, bool indirect>
    class PAutomatonFixedPoint : public fixed_point_workset<PAutomatonFixedPoint<W, indirect>, size_t> {
        using parent_t = fixed_point_workset<PAutomatonFixedPoint<W, indirect>, size_t>;
        struct state_info {
            typename W::type weight = W::max();
            uint32_t label = std::numeric_limits<uint32_t>::max();
            size_t predecessor = std::numeric_limits<size_t>::max();
            bool in_queue = false;
        };
        const PAutomaton<W, indirect>& _automaton;
        std::vector<state_info> _states;
        size_t _min_accepting_state = std::numeric_limits<size_t>::max(); // Id of accept state with minimum path weight to it. (keep it updated).
    public:
        static constexpr size_t next_round_elem = std::numeric_limits<size_t>::max(); // Needs to be different from any proper queue element.

        explicit PAutomatonFixedPoint(const PAutomaton<W, indirect>& automaton)
        : parent_t(automaton.states().size()),
        _automaton(automaton), _states(_automaton.states().size()) {
            initialize();
        };
        void initialize() {
            for (size_t i = 0; i < _automaton.pda().states().size(); ++i) { // Iterate over initial states, i.e. the states in the PDA.
                parent_t::emplace(i);
                _states[i].weight = W::zero();
                _states[i].in_queue = true;
            }
        }
        template<bool change_is_bottom = false>
        bool step_with(size_t&& current) {
            _states[current].in_queue = false;
            const auto& current_weight = _states[current].weight;
            if (_automaton.states()[current]->_accepting) {
                if (current_weight == W::bottom()) {
                    _min_accepting_state = current;
                    return false; // Signal stop
                }
                if (_min_accepting_state == std::numeric_limits<size_t>::max() || W::less(current_weight, _states[_min_accepting_state].weight)) {
                    _min_accepting_state = current;
                }
            }
            for (const auto& [to,labels] : _automaton.states()[current]->_edges) {
                if (!labels.empty()) {
                    auto label = std::min_element(labels.begin(), labels.end(), [](const auto& a, const auto& b){ return W::less(a.second.second, b.second.second); });
                    auto to_weight = W::add(current_weight, label->second.second);
                    if (W::less(to_weight, _states[to].weight)) {
                        if constexpr(change_is_bottom) {
                            _states[to].weight = W::bottom();
                        } else {
                            _states[to].weight = to_weight;
                        }
                        _states[to].label = label->first;
                        _states[to].predecessor = current;
                        if (!_states[to].in_queue) {
                            _states[to].in_queue = true;
                            parent_t::emplace(to);
                        }
                    }
                }
            }
            return true;
        }
        [[nodiscard]] bool not_accepting() const {
            return _min_accepting_state == std::numeric_limits<size_t>::max();
        }
        [[nodiscard]] bool is_infinite() const {
            return _states[_min_accepting_state].weight == W::bottom(); // TODO: Allow for better stuff than this...
        }
        [[nodiscard]] std::tuple<std::vector<size_t>, std::vector<uint32_t>, typename W::type> get_path() const {
            // assert(this->_workset.size() == 1 && this->_workset.back() == next_round_elem);
            // Return path and stack
            std::vector<size_t> path;
            path.emplace_back(_min_accepting_state);
            std::vector<uint32_t> stack;
            size_t state = _min_accepting_state;
            while (_states[state].predecessor != std::numeric_limits<size_t>::max()) {
                assert(_states[state].label != std::numeric_limits<uint32_t>::max());
                path.emplace_back(_states[state].predecessor);
                stack.emplace_back(_states[state].label);
                state = _states[state].predecessor;
                assert(path.size() <= _states.size()); // There should be no loop here - covered elsewhere.
            }
            return {path, stack, _states[_min_accepting_state].weight};
        }
    };
    template<typename W, bool indirect>
    PAutomatonFixedPoint(const PAutomaton<W,indirect>& automaton) -> PAutomatonFixedPoint<W,indirect>;
}

#endif //PDAAAL_PAUTOMATONALGORITHMS_H
