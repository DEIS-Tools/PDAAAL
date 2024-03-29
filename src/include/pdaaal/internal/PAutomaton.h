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
 * Created on 08-01-2020.
 */

#ifndef PDAAAL_INTERNAL_PAUTOMATON_H
#define PDAAAL_INTERNAL_PAUTOMATON_H

#include "PDA.h"
#include "pdaaal/NFA.h"
#include "pdaaal/AutomatonPath.h"
#include "pdaaal/utils/vector_printer.h"
#include <memory>
#include <functional>
#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <cassert>

namespace pdaaal {
    enum class Trace_Type {
        None,
        Any,
        Shortest,
        Longest,
        ShortestFixedPoint // TODO: Detect the need for fixed-point computation automatically.
    };

    enum class TraceInfoType {
        Single,
        Pair
    };
}

namespace pdaaal::internal {
    template<typename W, Trace_Type trace_type> using solver_weight = std::conditional_t<trace_type == Trace_Type::Longest, max_weight<typename W::type>, min_weight<typename W::type>>;

    struct trace_t {
        size_t _state = std::numeric_limits<size_t>::max(); // _state = p
        size_t _rule_id = std::numeric_limits<size_t>::max(); // size_t _to = pda.states()[_from]._rules[_rule_id]._to; // _to = q
        uint32_t _label = std::numeric_limits<uint32_t>::max(); // _label = \gamma
        // if is_null() { return; all is invalid. }
        // if is_pre_trace() {
        // then {use _rule_id (and potentially _state)}
        // else if is_post_epsilon_trace()
        // then {_state = q'; _rule_id invalid }
        // else {_state = p; _label = \gamma}

        constexpr trace_t() = default;

        constexpr trace_t(size_t rule_id, size_t temp_state)
                : _state(temp_state), _rule_id(rule_id), _label(std::numeric_limits<uint32_t>::max() - 1) {};

        constexpr trace_t(size_t from, size_t rule_id, uint32_t label)
                : _state(from), _rule_id(rule_id), _label(label) {};

        constexpr explicit trace_t(size_t epsilon_state)
                : _state(epsilon_state) {};

        [[nodiscard]] constexpr bool is_pre_trace() const {
            return _label == std::numeric_limits<uint32_t>::max() - 1;
        }
        [[nodiscard]] constexpr bool is_post_epsilon_trace() const {
            return _label == std::numeric_limits<uint32_t>::max();
        }
        [[nodiscard]] constexpr bool is_null() const {
            return _state == std::numeric_limits<size_t>::max() &&
                   _rule_id == std::numeric_limits<size_t>::max() &&
                   _label == std::numeric_limits<uint32_t>::max();
        }
        friend constexpr bool operator==(const trace_t& l, const trace_t& r) {
            return l._state == r._state && l._rule_id == r._rule_id && l._label && r._label;
        }
        friend constexpr bool operator!=(const trace_t& l, const trace_t& r) {
            return !(l == r);
        }

        friend constexpr bool operator<(const trace_t& l, const trace_t& r) {
            return std::tie(l._state, l._rule_id, l._label) < std::tie(r._state, r._rule_id, r._label);
        }
    };

    template <TraceInfoType trace_info_type> struct TraceInfo {};
    template <> struct TraceInfo<TraceInfoType::Single> {
        using type = trace_t;
        static constexpr type make_default() {
            return {};
        }
        static constexpr bool is_default(type t) {
            return t.is_null();
        }
    };
    template <> struct TraceInfo<TraceInfoType::Pair> {
        using type = std::pair<trace_t,trace_t>;
        static constexpr type make_default() {
            return std::make_pair(trace_t(), trace_t());
        }
        static constexpr bool is_default(type t) {
            return t.first.is_null() && t.second.is_null();
        }
    };
    template <TraceInfoType trace_info_type = TraceInfoType::Single>
    using TraceInfo_t = typename TraceInfo<trace_info_type>::type;

    template <typename W, TraceInfoType trace_info_type = TraceInfoType::Single>
    struct edge_annotation {
    private:
        using trace_info = TraceInfo<trace_info_type>;
        using trace_info_t = TraceInfo_t<trace_info_type>;
    public:
        using type = std::conditional_t<W::is_weight, std::pair<trace_info_t, typename W::type>, trace_info_t>;
        static constexpr type make_default() {
            if constexpr (W::is_weight) {
                return std::make_pair<trace_info_t, typename W::type>(trace_info::make_default(), W::zero());
            } else {
                return trace_info::make_default();
            }
        }
        static constexpr type from_trace_info(trace_info_t t) {
            if constexpr (W::is_weight) {
                return std::make_pair(t, W::zero());
            } else {
                return t;
            }
        }
        static constexpr trace_info_t get_trace_info(type t) {
            if constexpr (W::is_weight) {
                return t.first;
            } else {
                return t;
            }
        }
    };
    template <typename W, TraceInfoType trace_info_type = TraceInfoType::Single>
    using edge_annotation_t = typename edge_annotation<W,trace_info_type>::type;

    template <typename W = weight<void>, TraceInfoType trace_info_type = TraceInfoType::Single>
    class PAutomaton {
        using trace_info = TraceInfo<trace_info_type>;
        using trace_info_t = TraceInfo_t<trace_info_type>;
        using edge_anno = edge_annotation<W,trace_info_type>;
        using edge_anno_t = edge_annotation_t<W,trace_info_type>;
        using weight_or_bool_t = std::conditional_t<W::is_weight, typename W::type, bool>;
    public:
        using weight = W; // Expose the template parameter.
        static constexpr auto epsilon = std::numeric_limits<uint32_t>::max();

        struct state_t {
            bool _accepting = false;
            size_t _id;
            fut::set<std::tuple<size_t,uint32_t,edge_anno_t>, fut::type::hash, fut::type::vector> _edges;

            // TODO: Move this out to sub-class IncrementalPAutomaton
            size_t _back_edge_state = std::numeric_limits<size_t>::max(); // Use the back_edge_state to check for existence of a back_edge.
            uint32_t _back_edge_label = std::numeric_limits<uint32_t>::max()-1; // The null value should not equal epsilon (defensive defaults)
            weight_or_bool_t _best_weight;

            state_t(bool accepting, size_t id) : _accepting(accepting), _id(id) {};

            state_t(const state_t& other) = default;
        };

    public:
        // Accept one control state with given stack.
        PAutomaton(const PDA<W>& pda, size_t initial_state, const std::vector<uint32_t>& initial_stack) : _pda(pda) {
            const size_t size = pda.states().size();
            const size_t accepting = initial_stack.empty() ? initial_state : size;
            for (size_t i = 0; i < size; ++i) {
                add_state(true, i == accepting);
            }
            auto last_state = initial_state;
            for (size_t i = 0; i < initial_stack.size(); ++i) {
                auto state = add_state(false, i == initial_stack.size() - 1);
                add_edge(last_state, state, initial_stack[i]);
                last_state = state;
            }
        };

        // Construct a PAutomaton that accepts a configuration <p,w> iff states contains p and nfa accepts w.
        PAutomaton(const PDA<W>& pda, const NFA<uint32_t>& nfa, const std::vector<size_t>& states)
        : PAutomaton(pda, states, nfa.empty_accept()) {
            construct<uint32_t,false>(nfa, states, [](const auto& e){ return e._symbols; });
        }

        PAutomaton(const PDA<W>& pda, const std::vector<size_t>& special_initial_states, bool special_accepting = true) : _pda(pda) {
            assert(std::is_sorted(special_initial_states.begin(), special_initial_states.end()));
            const size_t size = pda.states().size();
            size_t j = 0;
            for (size_t i = 0; i < size; ++i) {
                if (j < special_initial_states.size() && special_initial_states[j] == i) {
                    add_state(true, special_accepting);
                    ++j;
                } else {
                    add_state(true, false);
                }
            }
        }
        PAutomaton(PAutomaton<W,trace_info_type>&& other, const PDA<W>& pda) noexcept // Move constructor, but update reference to PDA.
        : _states(std::move(other._states)), _initial(std::move(other._initial)),
          _accepting(std::move(other._accepting)), _pda(pda) {};

        PAutomaton(PAutomaton<W,trace_info_type> &&) noexcept = default;
        PAutomaton(const PAutomaton<W,trace_info_type>& other) : _pda(other._pda) {
            std::unordered_map<state_t *, state_t *> indir;
            for (auto &s : other._states) {
                _states.emplace_back(std::make_unique<state_t>(*s));
                indir[s.get()] = _states.back().get();
            }
            for (auto &s : other._accepting) {
                _accepting.push_back(indir[s]);
            }
            for (auto &s : other._initial) {
                _initial.push_back(indir[s]);
            }
        }

        template<typename C>
        bool has_negative_weights(const C& comp) const {
            if constexpr (is_weighted<W> && W::is_signed)
            {
                for (const auto& from : states()) {
                    for (const auto& [to,labels] : from->_edges) {
                        for (const auto& [label,trace] : labels) {
                            if (comp(trace.second, W::zero())) {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        [[nodiscard]] const std::vector<std::unique_ptr<state_t>>& states() const { return _states; }

        [[nodiscard]] const PDA<W>& pda() const { return _pda; }

        void or_extend(PAutomaton<W,trace_info_type>&& other) {
            assert(_pda.states().size() == other._pda.states().size()); // We compare number of PDA states, since operator== is not implemented for PDA.
            const size_t pda_size = _pda.states().size();
            const size_t automaton_size = _states.size();
            assert(automaton_size >= pda_size);
            assert(_initial.size() == pda_size);
            assert(other._states.size() >= pda_size);
            assert(other._initial.size() == pda_size);
            size_t offset = automaton_size - pda_size;
            for (size_t i = 0; i < other._states.size(); ++i) {
                assert(i >= pda_size || _initial[i]->_id == i);
                assert(i >= pda_size || other._initial[i]->_id == i);
                size_t from = i;
                if (i >= pda_size) {
                    from = add_state(false, other._states[i]->_accepting);
                } else if (!_states[i]->_accepting && other._states[i]->_accepting) {
                    _states[i]->_accepting = true;
                    _accepting.push_back(_states[i].get());
                }
                for (auto&& [to,labels] : other._states[i]->_edges) {
                    auto new_to = (to < pda_size) ? to : to + offset;
                    for (const auto& [label, trace] : labels) { // TODO: This can be done faster. Assign labels directly in some cases.
                        _states[from]->_edges.emplace(new_to, label, trace);
                    }
                }
            }
        }

        template<Trace_Type trace_type = Trace_Type::None>
        void to_dot(std::ostream &out,
                    const std::function<void(std::ostream &, const uint32_t&)>& label_printer = [](auto &s, auto &l){ s << l; },
                    const std::function<void(std::ostream &, const size_t&)>& state_printer = [](auto &s, auto &id){ s << id; }) const {
            out << "digraph NFA {\n";
            for (const auto& s : _states) {
                out << "\"";
                state_printer(out, s->_id);
                out << "\" [shape=";
                if (s->_accepting)
                    out << "double";
                out << "circle];\n";
                for (const auto& [to, labels] : s->_edges) {
                    out << "\"";
                    state_printer(out, s->_id);
                    out << "\" -> \"";
                    state_printer(out, to);
                    out << "\" [ label=\"";
                    auto has_epsilon = labels.contains(epsilon);
                    auto size = labels.size() - (has_epsilon ? 1 : 0);
                    if constexpr(W::is_weight) {
                        if (size > 0) {
                            out << "\\[";
                            bool first = true;
                            for (const auto& [l,tw] : labels) {
                                if (l == epsilon) { continue; }
                                if (!first)
                                    out << ", ";
                                first = false;
                                label_printer(out, l);
                                bool special_weight = false;
                                if (tw.second == solver_weight<W,trace_type>::max()) {
                                    out << "(-)";
                                    special_weight = true;
                                } else {
                                    if constexpr(trace_type == Trace_Type::Longest) {
                                        if (tw.second == solver_weight<W,trace_type>::bottom()) {
                                            out << "(∞)";
                                            special_weight = true;
                                        }
                                    }
                                    if constexpr(W::is_signed && (trace_type == Trace_Type::Shortest || trace_type == Trace_Type::ShortestFixedPoint)) {
                                        if (tw.second == solver_weight<W,trace_type>::bottom()) {
                                            out << "(-∞)";
                                            special_weight = true;
                                        }
                                    }
                                }
                                if (!special_weight) {
                                    if constexpr(W::is_vector) {
                                        out << "(" << vector_printer() << tw.second << ")";
                                    } else {
                                        out << "(" << tw.second << ")";
                                    }
                                }
                            }
                            out << "\\]";
                        }
                    } else {
                        if (size == number_of_labels()) {
                            out << "*";
                        } else if (size > 0) {
                            out << "\\[";
                            bool first = true;
                            for (const auto& [l,tw] : labels) {
                                if (l == epsilon) { continue; }
                                if (!first)
                                    out << ", ";
                                first = false;
                                label_printer(out, l);
                            }
                            out << "\\]";
                        }
                    }
                    if (has_epsilon) {
                        if (labels.size() > 1) out << " ";
                        out << "𝜀";
                        if constexpr(is_weighted<W>) {
                            if constexpr(W::is_vector) {
                                out << "(" << vector_printer() << labels.find(epsilon)->second.second << ")";
                            } else {
                                out << "(" << labels.find(epsilon)->second.second << ")";
                            }
                        }
                    }
                    out << "\"];\n";
                }
            }
            for (const auto& i : _initial) {
                out << "\"I";
                state_printer(out, i->_id);
                out << "\" -> \"";
                state_printer(out, i->_id);
                out << "\";\n" << "\"I";
                state_printer(out, i->_id);
                out << "\" [style=invisible];\n";
            }
            out << "}\n";
        }

        // Find_path searches for an accepting path.
        // The state_map function allows us to use this in PAutomatonProduct as well.
        [[nodiscard]] auto find_path() const { return find_path([](size_t s){ return s; }); }
        template <typename Fn> [[nodiscard]] auto find_path(Fn&& state_map) const {
            using path_state = decltype(std::declval<Fn>()(std::declval<size_t>()));
            static_assert(std::is_convertible_v<Fn, std::function<path_state(size_t)>>);
            using automaton_path_t = decltype(AutomatonPath(std::declval<path_state>()));

            // DFS search.
            std::vector<path_state> path;
            std::vector<uint32_t> label_stack;
            std::vector<std::tuple<size_t,size_t,uint32_t>> waiting; // state_id, stack_index, last_label (if stack_index > 0)
            const size_t pda_size = _pda.states().size();
            waiting.reserve(pda_size);
            for (size_t i = 0; i < pda_size; ++i) {
                if (_states[i]->_accepting) { // Initial accepting state
                    return automaton_path_t(state_map(i));
                }
                waiting.emplace_back(i, 0, std::numeric_limits<uint32_t>::max()); // Add all initial states.
            }
            std::unordered_set<size_t> seen;

            while (!waiting.empty()) {
                auto [current, stack_index, last_label] = waiting.back();
                waiting.pop_back();
                path.resize(stack_index + 2);
                label_stack.resize(stack_index + 1);
                path[stack_index] = state_map(current);
                if (stack_index > 0) {
                    label_stack[stack_index - 1] = last_label;
                }
                for (const auto &[to,labels] : _states[current]->_edges) {
                    if (!labels.empty() && seen.emplace(to).second) {
                        uint32_t label = labels[0].first;
                        if (_states[to]->_accepting) {
                            path[stack_index + 1] = state_map(to);
                            label_stack[stack_index] = label;
                            return automaton_path_t(path, label_stack); // No nice way of creating this on the fly, so we do it in the end.
                        }
                        waiting.emplace_back(to, stack_index + 1, label);
                    }
                }
            }
            return automaton_path_t();
        }

        template<bool state_pair = false>
        [[nodiscard]] auto find_path_shortest() const { return find_path_shortest<state_pair>([](size_t s){ return s; }); }

        template <bool state_pair = false, typename Fn>
        [[nodiscard]] auto find_path_shortest(Fn&& state_map) const {
            using path_state = decltype(std::declval<Fn>()(std::declval<size_t>()));
            static_assert(std::is_convertible_v<Fn, std::function<path_state(size_t)>>);
            using automaton_path_t = decltype(AutomatonPath<state_pair>(std::declval<path_state>()));

            if constexpr (is_weighted<W>) { // TODO: Consider unweighted shortest path.
                // Dijkstra.
                struct queue_elem {
                    typename W::type weight;
                    size_t state;
                    uint32_t label;
                    const queue_elem* back_pointer;
                    queue_elem(typename W::type weight, size_t state, uint32_t label, const queue_elem* back_pointer = nullptr)
                            : weight(weight), state(state), label(label), back_pointer(back_pointer) {};
                };
                struct queue_elem_comp {
                    bool operator()(const queue_elem& lhs, const queue_elem& rhs) {
                        return solver_weight<W, Trace_Type::Shortest>::less(rhs.weight, lhs.weight); // Used in a max-heap, so swap arguments to make it a min-heap.
                    }
                };
                std::priority_queue<queue_elem, std::vector<queue_elem>, queue_elem_comp> search_queue;
                std::unordered_map<size_t, typename W::type> visited;
                std::vector<std::unique_ptr<queue_elem>> pointers;
                const size_t pda_size = _pda.states().size();
                for (size_t i = 0; i < pda_size; ++i) { // Iterate over _initial ([i]->_id)
                    search_queue.emplace(W::zero(), i, std::numeric_limits<uint32_t>::max()); // No label going into initial state.
                }
                while (!search_queue.empty()) {
                    auto current = search_queue.top();
                    search_queue.pop();

                    if (_states[current.state]->_accepting) {
                        AutomatonPath<state_pair> automaton_path(state_map(current.state));
                        const queue_elem* p = &current;
                        while (p->back_pointer != nullptr) {
                            auto label = p->label;
                            p = p->back_pointer;
                            automaton_path.emplace(state_map(p->state), label);
                        }
                        return std::make_tuple(std::move(automaton_path), current.weight);
                    }

                    auto [it, fresh] = visited.emplace(current.state, current.weight);
                    if (!fresh) {
                        if (solver_weight<W, Trace_Type::Shortest>::less(current.weight, it->second)) {
                            it->second = current.weight;
                        } else {
                            continue;
                        }
                    }
                    auto u_pointer = std::make_unique<queue_elem>(current);
                    auto pointer = u_pointer.get();
                    pointers.push_back(std::move(u_pointer));
                    for (const auto& [to, labels] : _states[current.state]->_edges) {
                        if (!labels.empty()) {
                            auto label = std::min_element(labels.begin(), labels.end(), [](const auto& a, const auto& b) {
                                return solver_weight<W, Trace_Type::Shortest>::less(a.second.second, b.second.second);
                            });
                            search_queue.emplace(solver_weight<W, Trace_Type::Shortest>::add(current.weight, label->second.second), to, label->first, pointer);
                        }
                    }
                }
                return std::make_tuple(automaton_path_t(), solver_weight<W, Trace_Type::Shortest>::max());

            } else {
                assert(false);
                throw std::logic_error("Not implemented: Shortest path for unweighted automaton.");
            }
        }

        [[nodiscard]] bool accepts(size_t state, const std::vector<uint32_t> &stack) const {
            //Equivalent to (but hopefully faster than): return !_accept_path(state, stack).empty();

            if (stack.empty()) {
                return _states[state]->_accepting;
            }
            // DFS search.
            std::stack<std::pair<size_t, size_t>> search_stack;
            search_stack.emplace(state, 0);
            while (!search_stack.empty()) {
                auto current = search_stack.top();
                search_stack.pop();
                auto current_state = current.first;
                auto stack_index = current.second;
                for (const auto &[to,labels] : _states[current_state]->_edges) {
                    if (labels.contains(stack[stack_index])) {
                        if (stack_index + 1 < stack.size()) {
                            search_stack.emplace(to, stack_index + 1);
                        } else if (_states[to]->_accepting) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        template<Trace_Type trace_type = Trace_Type::Any>
        [[nodiscard]] typename std::conditional_t<trace_type == Trace_Type::Shortest && is_weighted<W>,
                std::pair<std::vector<size_t>, typename W::type>, std::vector<size_t>>
        accept_path(size_t state, const std::vector<uint32_t> &stack) const {
            if constexpr (trace_type == Trace_Type::Shortest && is_weighted<W>) { // TODO: Consider unweighted shortest path.
                if (stack.empty()) {
                    if (_states[state]->_accepting) {
                        return std::make_pair(std::vector<size_t>{state}, W::zero());
                    } else {
                        return std::make_pair(std::vector<size_t>(), solver_weight<W,trace_type>::max());
                    }
                }
                // Dijkstra.
                struct queue_elem {
                    typename W::type _weight;
                    size_t _state;
                    size_t _stack_index;
                    const queue_elem* _back_pointer;
                    queue_elem(typename W::type weight, size_t state, size_t stack_index, const queue_elem* back_pointer = nullptr)
                    : _weight(weight), _state(state), _stack_index(stack_index), _back_pointer(back_pointer) {};

                    bool operator<(const queue_elem& other) const {
                        return std::tie(_state, _stack_index) < std::tie(other._state, other._stack_index);
                    }
                    bool operator==(const queue_elem& other) const {
                        return _state == other._state && _stack_index == other._stack_index;
                    }
                    bool operator!=(const queue_elem& other) const {
                        return !(*this == other);
                    }
                };
                struct queue_elem_comp {
                    bool operator()(const queue_elem& lhs, const queue_elem& rhs){
                        return solver_weight<W,trace_type>::less(rhs._weight, lhs._weight); // Used in a max-heap, so swap arguments to make it a min-heap.
                    }
                };
                queue_elem_comp less;
                std::priority_queue<queue_elem, std::vector<queue_elem>, queue_elem_comp> search_queue;
                std::vector<queue_elem> visited;
                std::vector<std::unique_ptr<queue_elem>> pointers;
                search_queue.emplace(W::zero(), state, 0);
                while(!search_queue.empty()) {
                    auto current = search_queue.top();
                    search_queue.pop();
                    if (current._stack_index == stack.size()) {
                        std::vector<size_t> path(stack.size() + 1);
                        path[current._stack_index] = current._state;
                        for (auto p = current._back_pointer; p != nullptr; p = p->_back_pointer) {
                            path[p->_stack_index] = p->_state;
                        }
                        return std::make_pair(path, current._weight);
                    }
                    auto lb = std::lower_bound(visited.begin(), visited.end(), current);
                    if (lb != std::end(visited) && *lb == current) {
                        if (less(*lb, current)) {
                            *lb = current;
                        } else {
                            continue;
                        }
                    } else {
                        lb = visited.insert(lb, current);
                    }
                    auto u_pointer = std::make_unique<queue_elem>(*lb);
                    auto pointer = u_pointer.get();
                    pointers.push_back(std::move(u_pointer));
                    for (const auto& [to,labels] : _states[current._state]->_edges) {
                        auto label = labels.get(stack[current._stack_index]);
                        if (label != nullptr) {
                            if (current._stack_index + 1 < stack.size() || _states[to]->_accepting) {
                                search_queue.emplace(solver_weight<W,trace_type>::add(current._weight, label->second), to, current._stack_index + 1, pointer);
                            }
                        }
                        auto eps_label = labels.get(epsilon);
                        if (eps_label != nullptr) {
                            search_queue.emplace(solver_weight<W,trace_type>::add(current._weight, eps_label->second), to, current._stack_index, pointer->_back_pointer);
                        }
                    }
                }
                return std::make_pair(std::vector<size_t>(), solver_weight<W,trace_type>::max());
            } else {
                if (stack.empty()) {
                    if (_states[state]->_accepting) {
                        return std::vector<size_t>{state};
                    } else {
                        return std::vector<size_t>();
                    }
                }
                // DFS search.
                std::vector<size_t> path(stack.size() + 1);
                std::stack<std::pair<size_t, size_t>> search_stack;
                search_stack.emplace(state, 0);
                while (!search_stack.empty()) {
                    auto current = search_stack.top();
                    search_stack.pop();
                    auto current_state = current.first;
                    auto stack_index = current.second;
                    path[stack_index] = current_state;
                    for (const auto &[to,labels] : _states[current_state]->_edges) {
                        if (labels.contains(stack[stack_index])) {
                            if (stack_index + 1 < stack.size()) {
                                search_stack.emplace(to, stack_index + 1);
                            } else if (_states[to]->_accepting) {
                                path[stack_index + 1] = to;
                                return path;
                            }
                        }
                    }
                }
                return std::vector<size_t>();
            }
        }

        [[nodiscard]] trace_info_t get_trace_label(const std::tuple<size_t, uint32_t, size_t> &edge) const {
            return get_trace_label(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
        }
        [[nodiscard]] trace_info_t get_trace_label(size_t from, uint32_t label, size_t to) const {
            auto trace = _states[from]->_edges.get(to, label);
            if (trace) return edge_anno::get_trace_info(*trace);
            assert(false); // We assume the edge exists.
            return trace_info::make_default();
        }

        [[nodiscard]] size_t number_of_labels() const { return _pda.number_of_labels(); }

        [[nodiscard]] bool has_accepting_state() const {
            return !_accepting.empty();
        };
        template<Trace_Type trace_type = Trace_Type::Shortest>
        const weight_or_bool_t&
        min_accepting_weight() const {
            if constexpr (W::is_weight) {
                static const auto max = solver_weight<W,trace_type>::max();
                if (_accepting.empty()) return max;
                return (*std::min_element(_accepting.begin(), _accepting.end(), [](const auto& a, const auto& b){
                    return solver_weight<W,trace_type>::less(a->_best_weight, b->_best_weight);
                }))->_best_weight;
            } else {
                return has_accepting_state();
            }
        }

        template<bool state_pair>
        [[nodiscard]] auto get_path_shortest() const { return get_path_shortest<state_pair>([](size_t s){ return s; }); }

        template <bool state_pair, typename Fn> [[nodiscard]]
        auto get_path_shortest(Fn&& state_map) const {
            if constexpr (W::is_weight) {
                if (_accepting.empty()) return std::make_tuple(AutomatonPath<state_pair>(), solver_weight<W, Trace_Type::Shortest>::max());
                auto best_accept_state = *std::min_element(_accepting.begin(), _accepting.end(), [](const auto& a, const auto& b){
                    return solver_weight<W,Trace_Type::Shortest>::less(a->_best_weight, b->_best_weight);
                });
                AutomatonPath<state_pair> automaton_path(state_map(best_accept_state->_id));
                const state_t* p = best_accept_state;
                while (p->_back_edge_state != std::numeric_limits<size_t>::max()) {
                    auto label = p->_back_edge_label;
                    p = _states[p->_back_edge_state].get();
                    automaton_path.emplace(state_map(p->_id), label);
                }
                return std::make_tuple(std::move(automaton_path), best_accept_state->_best_weight);
            } else {
                assert(false);
                throw std::logic_error("Not implemented: Shortest path for unweighted automaton.");
            }
        }


        size_t add_state(bool initial, bool accepting) {
            auto id = next_state_id();
            _states.emplace_back(std::make_unique<state_t>(accepting, id));
            if (accepting) {
                _accepting.push_back(_states.back().get());
            }
            if (initial) {
                _initial.push_back(_states.back().get());
            }
            return id;
        }
        void set_state_accepting(size_t id) {
            assert(id < _states.size());
            if (!_states[id]->_accepting) {
                _states[id]->_accepting = true;
                _accepting.push_back(_states[id].get());
            }
        }
        [[nodiscard]] size_t next_state_id() const {
            return _states.size();
        }

        void add_epsilon_edge(size_t from, size_t to, edge_anno_t trace = edge_anno::make_default()) {
            _states[from]->_edges.emplace(to, epsilon, std::move(trace));
        }
        void add_edge(size_t from, size_t to, uint32_t label, edge_anno_t trace = edge_anno::make_default()) {
            assert(label < std::numeric_limits<uint32_t>::max() - 1);
            _states[from]->_edges.emplace(to, label, std::move(trace));
        }

        bool make_back_edge(size_t from, uint32_t label, size_t to) {
            if (_states[to]->_back_edge_state == std::numeric_limits<size_t>::max()) {
                _states[to]->_back_edge_state = from;
                _states[to]->_back_edge_label = label;
                return true;
            }
            return false;
        }
        std::optional<weight_or_bool_t>
        make_back_edge_shortest(size_t from, uint32_t label, size_t to, const weight_or_bool_t& weight) {
            if constexpr (W::is_weight) {
                if (to < _pda.states().size()) return std::nullopt; // Initial states cannot be reached by a shorter path.
                // If from is initial it has weight 0 (but we don't care about storing that).
                auto new_weight = from < _pda.states().size() ? weight : solver_weight<W,Trace_Type::Shortest>::add(_states[from]->_best_weight, weight);
                if (_states[to]->_back_edge_state == std::numeric_limits<size_t>::max()
                    || solver_weight<W,Trace_Type::Shortest>::less(new_weight, _states[to]->_best_weight)) {
                    _states[to]->_back_edge_state = from;
                    _states[to]->_back_edge_label = label;
                    _states[to]->_best_weight = std::move(new_weight);
                    return _states[to]->_best_weight;
                }
                return std::nullopt;
            } else {
                return make_back_edge(from, label, to);
            }
        }
        auto get_edge(size_t from, uint32_t label, size_t to) {
            return _states[from]->_edges.get(to, label);
        }
        auto emplace_edge(size_t from, uint32_t label, size_t to, edge_anno_t trace = edge_anno::make_default()) {
            return _states[from]->_edges.emplace(to, label, trace);
        }
        void update_edge(size_t from, size_t to, uint32_t label, edge_annotation_t<W,TraceInfoType::Single> trace) {
            auto ptr = _states[from]->_edges.get(to, label);
            assert(ptr != nullptr);
            if constexpr(trace_info_type == TraceInfoType::Single) {
                *ptr = trace;
            } else { // (trace_info_type == TraceInfoType::Pair)
                if constexpr(W::is_weight) {
                    ptr->first.second = trace.first; // We update the second trace_info in the pair, keeping the first one unchanged.
                    ptr->second = trace.second; // Update weight.
                } else {
                    // Doesn't happen (probably), but meh.
                    ptr->second = trace;
                }
            }
        }

        void add_edges(size_t from, size_t to, bool negated, const std::vector<uint32_t>& labels) {
            if (negated) {
                assert(std::is_sorted(labels.begin(), labels.end()));
                //std::sort(labels.begin(), labels.end());
                size_t i = 0;
                for (auto label : labels) {
                    for (; i < label; ++i){
                        add_edge(from, to, i);
                    }
                    ++i;
                }
                auto max = number_of_labels();
                for (; i < max; ++i) {
                    add_edge(from, to, i);
                }
            } else {
                for (auto label : labels) {
                    add_edge(from, to, label);
                }
            }
        }
        void add_edges(const std::vector<size_t>& from, size_t to, bool negated, const std::vector<uint32_t>& labels) {
            if (negated) {
                assert(std::is_sorted(labels.begin(), labels.end()));
                //std::sort(labels.begin(), labels.end());
                size_t i = 0;
                for (auto label : labels) {
                    for (; i < label; ++i){
                        for (auto f : from) {
                            add_edge(f, to, i);
                        }
                    }
                    ++i;
                }
                auto max = number_of_labels();
                for (; i < max; ++i) {
                    for (auto f : from) {
                        add_edge(f, to, i);
                    }
                }
            } else {
                for (auto f : from) {
                    for (auto label : labels) {
                        add_edge(f, to, label);
                    }
                }
            }
        }

        static constexpr trace_t new_pre_trace(size_t rule_id) {
            return {rule_id, std::numeric_limits<size_t>::max()};
        }
        static constexpr trace_t new_pre_trace(size_t rule_id, size_t temp_state) {
            return {rule_id, temp_state};
        }
        static constexpr trace_t new_post_trace(size_t from, size_t rule_id, uint32_t label) {
            return {from, rule_id, label};
        }
        static constexpr trace_t new_post_trace(size_t epsilon_state) {
            return trace_t(epsilon_state);
        }
    protected:
        template<typename T, bool use_mapping = true, bool use_new_state_action = false>
        void construct(const NFA<T>& nfa, const std::vector<size_t>& states, const std::function<std::vector<uint32_t>(const typename NFA<T>::edge_t&)>& map_edge,
                       const std::function<void(const typename NFA<T>::state_t*,size_t)>& new_state_action = [](auto,auto){}) {
            using nfastate_t = typename NFA<T>::state_t;
            std::unordered_map<const nfastate_t*, size_t> nfastate_to_id;
            std::vector<std::pair<const nfastate_t*,size_t>> waiting;
            auto get_nfastate_id = [this, &waiting, &nfastate_to_id,&new_state_action](const nfastate_t* n) -> size_t {
                // Adds nfastate if not yet seen.
                size_t n_id;
                auto it = nfastate_to_id.find(n);
                if (it != nfastate_to_id.end()) {
                    n_id = it->second;
                } else {
                    n_id = add_state(false, n->_accepting);
                    nfastate_to_id.emplace(n, n_id);
                    waiting.emplace_back(n, n_id);
                    if constexpr(use_new_state_action) {
                        new_state_action(n,n_id); // Allows subclasses to do something for each new state. Defaults to no action.
                    }
                }
                return n_id;
            };

            for (const auto& i : nfa.initial()) {
                for (const auto& e : i->_edges) {
                    for (const nfastate_t* n : e.follow_epsilon()) {
                        size_t n_id = get_nfastate_id(n);
                        if constexpr(use_mapping) {
                            add_edges(states, n_id, e._negated, map_edge(e));
                        } else {
                            add_edges(states, n_id, e._negated, e._symbols);
                        }
                    }
                }
            }
            while (!waiting.empty()) {
                auto [top, top_id] = waiting.back();
                waiting.pop_back();
                for (const auto& e : top->_edges) {
                    for (const nfastate_t* n : e.follow_epsilon()) {
                        size_t n_id = get_nfastate_id(n);
                        if constexpr(use_mapping) {
                            add_edges(top_id, n_id, e._negated, map_edge(e));
                        } else {
                            add_edges(top_id, n_id, e._negated, e._symbols);
                        }
                    }
                }
            }
        }
    private:
        std::vector<std::unique_ptr<state_t>> _states;
        std::vector<state_t *> _initial;
        std::vector<state_t *> _accepting;

        const PDA<W>& _pda;
    };

}

#endif //PDAAAL_INTERNAL_PAUTOMATON_H
