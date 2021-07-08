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

#ifndef PDAAAL_PAUTOMATON_H
#define PDAAAL_PAUTOMATON_H

#include <pdaaal/TypedPDA.h>
#include <pdaaal/utils/fut_set.h>
#include <pdaaal/NFA.h>

#include <memory>
#include <functional>
#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <cassert>
#include <boost/functional/hash.hpp>


namespace pdaaal {

    enum class Trace_Type {
        None,
        Any,
        Shortest
    };

    struct trace_t {
        size_t _state = std::numeric_limits<size_t>::max(); // _state = p
        size_t _rule_id = std::numeric_limits<size_t>::max(); // size_t _to = pda.states()[_from]._rules[_rule_id]._to; // _to = q
        uint32_t _label = std::numeric_limits<uint32_t>::max(); // _label = \gamma
        // if is_pre_trace() {
        // then {use _rule_id (and potentially _state)}
        // else if is_post_epsilon_trace()
        // then {_state = q'; _rule_id invalid }
        // else {_state = p; _label = \gamma}

        trace_t() = default;

        trace_t(size_t rule_id, size_t temp_state)
                : _state(temp_state), _rule_id(rule_id), _label(std::numeric_limits<uint32_t>::max() - 1) {};

        trace_t(size_t from, size_t rule_id, uint32_t label)
                : _state(from), _rule_id(rule_id), _label(label) {};

        explicit trace_t(size_t epsilon_state)
                : _state(epsilon_state) {};

        [[nodiscard]] bool is_pre_trace() const {
            return _label == std::numeric_limits<uint32_t>::max() - 1;
        }
        [[nodiscard]] bool is_post_epsilon_trace() const {
            return _label == std::numeric_limits<uint32_t>::max();
        }
    };

    template<typename W> using trace_ptr = std::conditional_t<is_weighted<W>, std::pair<const trace_t*, typename W::type>, const trace_t*>;
    template<typename W> inline constexpr trace_ptr<W> default_trace_ptr() {
        if constexpr (is_weighted<W>) {
            return std::make_pair<const trace_t*, typename W::type>(nullptr, W::zero());
        } else {
            return nullptr;
        }
    }
    template<typename W> inline constexpr trace_ptr<W> trace_ptr_from(const trace_t *trace) {
        if constexpr (is_weighted<W>) {
            return std::make_pair(trace, W::zero());
        } else {
            return trace;
        }
    }
    template<typename W> inline constexpr const trace_t * trace_from(trace_ptr<W> t) {
        if constexpr (is_weighted<W>) {
            return t.first;
        } else {
            return t;
        }
    }


    template <typename W = weight<void>>
    class PAutomaton {
    public:
        static constexpr auto epsilon = std::numeric_limits<uint32_t>::max();

        struct state_t {
            bool _accepting = false;
            size_t _id;
            fut::set<std::tuple<size_t,uint32_t,trace_ptr<W>>, fut::type::hash, fut::type::vector> _edges;

            state_t(bool accepting, size_t id) : _accepting(accepting), _id(id) {};

            state_t(const state_t &other) = default;
        };

    public:
        // Accept one control state with given stack.
        PAutomaton(const PDA<W> &pda, size_t initial_state, const std::vector<uint32_t> &initial_stack) : _pda(pda) {
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
        template<typename T>
        PAutomaton(const TypedPDA<T,W>& pda, const NFA<T>& nfa, const std::vector<size_t>& states)
        : PAutomaton(pda, states, nfa.empty_accept()) {
            construct<T>(nfa, states, [&pda](const auto& v){ return pda.encode_pre(v); });
        }
        // Same, but where the NFA contains the symbols mapped to ids already.
        PAutomaton(const PDA<W>& pda, const NFA<uint32_t>& nfa, const std::vector<size_t>& states)
        : PAutomaton(pda, states, nfa.empty_accept()) {
            construct<uint32_t,false>(nfa, states, [](const auto& v){ return v; });
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


        PAutomaton(PAutomaton<W> &&) noexcept = default;
        PAutomaton(const PAutomaton<W>& other) : _pda(other._pda) {
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

        [[nodiscard]] const std::vector<std::unique_ptr<state_t>> &states() const { return _states; }
        
        [[nodiscard]] const PDA<W> &pda() const { return _pda; }

        void or_extend(PAutomaton<W>&& other) {
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

        void to_dot(std::ostream &out, const std::function<void(std::ostream &, const uint32_t&)> &printer = [](auto &s, auto &l) {
                        s << l;
                    }) const {
            out << "digraph NFA {\n";
            for (auto &s : _states) {
                out << "\"" << s->_id << "\" [shape=";
                if (s->_accepting)
                    out << "double";
                out << "circle];\n";
                for (const auto &[to, labels] : s->_edges) {
                    out << "\"" << s->_id << "\" -> \"" << to << "\" [ label=\"";
                    auto has_epsilon = labels.contains(epsilon);
                    auto size = labels.size() - (has_epsilon ? 1 : 0);
                    if (size == number_of_labels()) {
                        out << "*";
                    } else if (size > 0) {
                        out << "\\[";
                        bool first = true;
                        for (const auto& [l,_] : labels) {
                            if (l == epsilon) { continue; }
                            if (!first)
                                out << ", ";
                            first = false;
                            printer(out, l);
                        }
                        out << "\\]";
                    }
                    if (labels.contains(epsilon)) {
                        if (labels.size() > 1) out << " ";
                        out << "𝜀";
                    }
                    out << "\"];\n";
                }
            }
            for (auto &i : _initial) {
                out << "\"I" << i->_id << "\" -> \"" << i->_id << "\";\n";
                out << "\"I" << i->_id << "\" [style=invisible];\n";
            }

            out << "}\n";
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
                        return std::make_pair(std::vector<size_t>(), W::max());
                    }
                }
                // Dijkstra.
                struct queue_elem {
                    typename W::type weight;
                    size_t state;
                    size_t stack_index;
                    const queue_elem *back_pointer;
                    queue_elem(typename W::type weight, size_t state, size_t stack_index, const queue_elem *back_pointer = nullptr)
                    : weight(weight), state(state), stack_index(stack_index), back_pointer(back_pointer) {};

                    bool operator<(const queue_elem &other) const {
                        if (state != other.state) {
                            return state < other.state;
                        }
                        return stack_index < other.stack_index;
                    }
                    bool operator==(const queue_elem &other) const {
                        return state == other.state && stack_index == other.stack_index;
                    }
                    bool operator!=(const queue_elem &other) const {
                        return !(*this == other);
                    }
                };
                struct queue_elem_comp {
                    bool operator()(const queue_elem &lhs, const queue_elem &rhs){
                        return W::less(rhs.weight, lhs.weight); // Used in a max-heap, so swap arguments to make it a min-heap.
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
                    if (current.stack_index == stack.size()) {
                        std::vector<size_t> path(stack.size() + 1);
                        path[current.stack_index] = current.state;
                        for (auto p = current.back_pointer; p != nullptr; p = p->back_pointer) {
                            path[p->stack_index] = p->state;
                        }
                        return std::make_pair(path, current.weight);
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
                    for (const auto &[to,labels] : _states[current.state]->_edges) {
                        auto label = labels.get(stack[current.stack_index]);
                        if (label != nullptr) {
                            if (current.stack_index + 1 < stack.size() || _states[to]->_accepting) {
                                search_queue.emplace(W::add(current.weight, label->second), to, current.stack_index + 1, pointer);
                            }
                        }
                    }
                }
                return std::make_pair(std::vector<size_t>(), W::max());
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

        [[nodiscard]] const trace_t *get_trace_label(const std::tuple<size_t, uint32_t, size_t> &edge) const {
            return get_trace_label(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
        }
        [[nodiscard]] const trace_t *get_trace_label(size_t from, uint32_t label, size_t to) const {
            auto trace = _states[from]->_edges.get(to, label);
            if (trace) return trace_from<W>(*trace);
            assert(false); // We assume the edge exists.
            return nullptr;
        }

        [[nodiscard]] size_t number_of_labels() const { return _pda.number_of_labels(); }

        [[nodiscard]] bool has_accepting_state() const {
            return !_accepting.empty();
        };

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
        [[nodiscard]] size_t next_state_id() const {
            return _states.size();
        }

        void add_epsilon_edge(size_t from, size_t to, trace_ptr<W> trace = default_trace_ptr<W>()) {
            _states[from]->_edges.emplace(to, epsilon, trace);
        }

        void add_edge(size_t from, size_t to, uint32_t label, trace_ptr<W> trace = default_trace_ptr<W>()) {
            assert(label < std::numeric_limits<uint32_t>::max() - 1);
            _states[from]->_edges.emplace(to, label, trace);
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

        const trace_t *new_pre_trace(size_t rule_id) {
            _trace_info.emplace_back(std::make_unique<trace_t>(rule_id, std::numeric_limits<size_t>::max()));
            return _trace_info.back().get();
        }
        const trace_t *new_pre_trace(size_t rule_id, size_t temp_state) {
            _trace_info.emplace_back(std::make_unique<trace_t>(rule_id, temp_state));
            return _trace_info.back().get();
        }
        const trace_t *new_post_trace(size_t from, size_t rule_id, uint32_t label) {
            _trace_info.emplace_back(std::make_unique<trace_t>(from, rule_id, label));
            return _trace_info.back().get();
        }
        const trace_t *new_post_trace(size_t epsilon_state) {
            _trace_info.emplace_back(std::make_unique<trace_t>(epsilon_state));
            return _trace_info.back().get();
        }
    private:
        template<typename T, bool use_mapping = true>
        void construct(const NFA<T>& nfa, const std::vector<size_t>& states, const std::function<std::vector<uint32_t>(const std::vector<T>&)>& map_symbols) {
            using nfastate_t = typename NFA<T>::state_t;
            std::unordered_map<const nfastate_t*, size_t> nfastate_to_id;
            std::vector<std::pair<const nfastate_t*,size_t>> waiting;
            auto get_nfastate_id = [this, &waiting, &nfastate_to_id](const nfastate_t* n) -> size_t {
                // Adds nfastate if not yet seen.
                size_t n_id;
                auto it = nfastate_to_id.find(n);
                if (it != nfastate_to_id.end()) {
                    n_id = it->second;
                } else {
                    n_id = add_state(false, n->_accepting);
                    nfastate_to_id.emplace(n, n_id);
                    waiting.emplace_back(n, n_id);
                }
                return n_id;
            };

            for (const auto& i : nfa.initial()) {
                for (const auto& e : i->_edges) {
                    for (const nfastate_t* n : e.follow_epsilon()) {
                        size_t n_id = get_nfastate_id(n);
                        if constexpr(use_mapping) {
                            add_edges(states, n_id, e._negated, map_symbols(e._symbols));
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
                            add_edges(top_id, n_id, e._negated, map_symbols(e._symbols));
                        } else {
                            add_edges(top_id, n_id, e._negated, e._symbols);
                        }
                    }
                }
            }
        }

        std::vector<std::unique_ptr<state_t>> _states;
        std::vector<state_t *> _initial;
        std::vector<state_t *> _accepting;

        std::vector<std::unique_ptr<trace_t>> _trace_info;

        const PDA<W> &_pda;
    };


}

#endif //PDAAAL_PAUTOMATON_H
