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

#include "PAutomaton.h"
#include "pdaaal/utils/workset.h"

namespace pdaaal::internal {

    template<typename W, Trace_Type trace_type>
    class PAutomatonFixedPoint : public fixed_point_workset<PAutomatonFixedPoint<W, trace_type>, size_t> {
        static constexpr size_t uninitialized_state = std::numeric_limits<size_t>::max();
        static constexpr size_t no_state = std::numeric_limits<size_t>::max() - 1;

        using parent_t = fixed_point_workset<PAutomatonFixedPoint<W, trace_type>, size_t>;
        using solverW = solver_weight<W,trace_type>;
        struct state_info {
            typename W::type weight = solverW::max();
            uint32_t first_label = std::numeric_limits<uint32_t>::max();
            uint32_t label = std::numeric_limits<uint32_t>::max();
            size_t first_predecessor = uninitialized_state;
            size_t predecessor = uninitialized_state;
            bool in_queue = false;
        };
        const PAutomaton<W, TraceInfoType::Pair>& _automaton;
        std::vector<state_info> _states;
        size_t _min_accepting_state = uninitialized_state; // Id of accept state with minimum path weight to it. (keep it updated).
    public:
        static constexpr size_t next_round_elem = std::numeric_limits<size_t>::max(); // Needs to be different from any proper queue element.

        explicit PAutomatonFixedPoint(const PAutomaton<W, TraceInfoType::Pair>& automaton)
        : parent_t(automaton.states().size()),
        _automaton(automaton), _states(_automaton.states().size()) {
            initialize();
        };

    private:
        void initialize() {
            for (size_t i = 0; i < _automaton.pda().states().size(); ++i) { // Iterate over initial states, i.e. the states in the PDA.
                parent_t::emplace(i);
                _states[i].weight = W::zero();
                _states[i].in_queue = true;
                _states[i].first_predecessor = no_state;
                _states[i].predecessor = no_state;
            }
        }

    public:
        template<bool change_is_bottom = false>
        bool step_with(size_t&& current) {
            _states[current].in_queue = false;
            const auto& current_weight = _states[current].weight;
            if (_automaton.states()[current]->_accepting) {
                if (current_weight == solverW::bottom()) {
                    _min_accepting_state = current;
                    return false; // Signal stop
                }
                if (_min_accepting_state == uninitialized_state || solverW::less(current_weight, _states[_min_accepting_state].weight)) {
                    _min_accepting_state = current;
                }
            }
            for (const auto& [to,labels] : _automaton.states()[current]->_edges) {
                if (!labels.empty()) {
                    auto label = std::min_element(labels.begin(), labels.end(), [](const auto& a, const auto& b){ return solverW::less(a.second.second, b.second.second); });
                    auto to_weight = solverW::add(current_weight, label->second.second);
                    if (solverW::less(to_weight, _states[to].weight)) {
                        if constexpr(change_is_bottom) {
                            _states[to].weight = solverW::bottom();
                        } else {
                            _states[to].weight = to_weight;
                        }
                        if (_states[to].first_predecessor == uninitialized_state) {
                            _states[to].first_label = label->first;
                            _states[to].first_predecessor = current;
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
            return _min_accepting_state == uninitialized_state;
        }
        [[nodiscard]] typename W::type get_weight() const {
            assert(_min_accepting_state < _states.size());
            return _states[_min_accepting_state].weight;
        }
        [[nodiscard]] bool is_infinite() const {
            return get_weight() == solverW::bottom();
        }

        // If (this->is_infinite() == false) Then we can safely use get_path
        // else we might have a loop, and here get_path_with_loop is preferred.
        [[nodiscard]] auto get_path() const {
            return get_path([](size_t s){ return s; });
        }
        template<typename MapFn>
        [[nodiscard]] AutomatonPath<> get_path(MapFn&& state_map) const {
            static_assert(std::is_convertible_v<MapFn,std::function<size_t(size_t)>>);
            AutomatonPath path(state_map(_min_accepting_state));
            size_t state = _min_accepting_state;
            while (_states[state].predecessor != no_state) {
                assert(_states[state].label != std::numeric_limits<uint32_t>::max());
                path.emplace(state_map(_states[state].predecessor), _states[state].label);
                state = _states[state].predecessor;
                assert(path.edges_size() <= _states.size()); // There should be no loop here - covered elsewhere.
            }
            return path;
        }

        template<typename MapFn>
        class AutomatonTraceBack {
            const std::vector<state_info>& _states; // Note: This is ref to _states in PAutomatonFixedPoint. Beware of lifetime.
            size_t _current_state;
            MapFn _state_map;
            AutomatonPath<> _path;

        public:
            AutomatonTraceBack(const PAutomatonFixedPoint<W,trace_type>& fp, MapFn&& state_map)
            : _states(fp._states), _current_state(fp._min_accepting_state),
              _state_map(std::forward<MapFn>(state_map)), _path(_state_map(_current_state)) {
                static_assert(std::is_convertible_v<MapFn,std::function<size_t(size_t)>>);
                assert(_current_state != uninitialized_state);
            };

            template <bool avoid_loop = false>
            std::optional<uint32_t> next() {
                uint32_t label;
                if constexpr (avoid_loop) {
                    label = _states[_current_state].first_label;
                    _current_state = _states[_current_state].first_predecessor;
                } else {
                    label = _states[_current_state].label;
                    _current_state = _states[_current_state].predecessor;
                }
                if (_current_state == no_state) return std::nullopt; // Done searching.
                assert(label != std::numeric_limits<uint32_t>::max());
                assert(_current_state != uninitialized_state);
                _path.emplace(_state_map(_current_state), label);
                return label;
            }
            [[nodiscard]] size_t current_state() const {
                return _current_state;
            }
            [[nodiscard]] AutomatonPath<> get_path() const {
                return _path;
            }
        };
        // Deduction guide is needed for Clang, but not supported by GCC before 12.
        // TODO: This deduction guide should always work.
        //  Related GCC compiler bug is fixed in GCC 12. https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79501
        //  Remove #if, when upgrading to GCC 12.
        #if __clang__
        template<typename MapFn>
        AutomatonTraceBack(const PAutomatonFixedPoint<W,trace_type>& fp, MapFn&& state_map) -> AutomatonTraceBack<MapFn>;
        #endif

        [[nodiscard]] auto get_path_with_loop() {
            return get_path_with_loop([](size_t s){ return s; });
        }
        template<typename MapFn>
        AutomatonPath<> get_path_with_loop(MapFn&& state_map) {
            AutomatonTraceBack trace_back(*this, std::forward<MapFn>(state_map));
            std::unordered_set<size_t> seen;
            seen.emplace(trace_back.current_state());

            std::optional<size_t> loop_elem = std::nullopt;

            while (trace_back.next()) {
                if (auto [it,fresh] = seen.emplace(trace_back.current_state()); !fresh) {
                    loop_elem.emplace(*it);
                    break;
                }
            }
            // In case there is a loop, we take the loop one more time to ensure that there is a repeated edge.
            if (loop_elem) {
                size_t loop_elem_val = loop_elem.value();
                do {
#ifndef NDEBUG
                    auto res =
#endif
                    trace_back.next();
                    assert(res.has_value());
                } while (trace_back.current_state() != loop_elem_val);
                // Now go to the end without while avoiding the loop.
                while (trace_back.template next<true>());
            }
            return trace_back.get_path();
        }
    };

}

#endif //PDAAAL_PAUTOMATONALGORITHMS_H
