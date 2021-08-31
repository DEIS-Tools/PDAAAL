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
 * File:   PAutomatonProduct.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 15-07-2021.
 */

#ifndef PDAAAL_PAUTOMATONPRODUCT_H
#define PDAAAL_PAUTOMATONPRODUCT_H

#include <pdaaal/PAutomatonAlgorithms.h>

namespace pdaaal {

    template <typename pda_t, typename automaton_t, typename W>
    class PAutomatonProduct {
        using product_automaton_t = PAutomaton<W>; // No explicit abstraction on product automaton - this is covered by _initial and _final.
        using state_t = typename product_automaton_t::state_t;
        static constexpr auto epsilon = product_automaton_t::epsilon;
    public:
        template<typename T>
        PAutomatonProduct(const pda_t& pda, const NFA<T>& initial_nfa, const std::vector<size_t>& initial_states,
                          const NFA<T>& final_nfa, const std::vector<size_t>& final_states)
        : _pda(pda), _pda_size(_pda.states().size()),
          _initial(_pda, initial_nfa, initial_states), _final(_pda, final_nfa, final_states),
          _product(_pda, intersect_vector(initial_states, final_states), initial_nfa.empty_accept() && final_nfa.empty_accept()) {}

        PAutomatonProduct(const pda_t& pda, automaton_t&& initial, automaton_t&& final)
        : _pda(pda), _pda_size(_pda.states().size()),
          _initial(std::move(initial), _pda), _final(std::move(final), _pda),
          _product(_pda, get_initial_accepting(_initial, _final), true) {};

        // Returns whether an accepting state in the product automaton was reached.
        template<bool needs_back_lookup = false, bool ET = true>
        bool initialize_product() {
            std::vector<size_t> ids(_product.states().size());
            std::iota (ids.begin(), ids.end(), 0); // Fill with 0,1,...,size-1;
            return construct_reachable<needs_back_lookup,ET>(ids,
                                                             _swap_initial_final ? _final : _initial,
                                                             _swap_initial_final ? _initial : _final);
        }

        // Returns whether an accepting state in the product automaton was reached.
        bool add_edge_product(size_t from, uint32_t label, size_t to, trace_ptr<W> trace) {
            return add_edge(from, label, to, trace,
                            _swap_initial_final ? _final : _initial,
                            _swap_initial_final ? _initial : _final);
        }

        // This is for the dual_search mode:
        bool add_initial_edge(size_t from, uint32_t label, size_t to, trace_ptr<W> trace) {
            return add_edge<true, true>(from, label, to, trace, _initial, _final);
        }
        bool add_final_edge(size_t from, uint32_t label, size_t to, trace_ptr<W> trace) {
            return add_edge<false, true>(from, label, to, trace, _initial, _final);
        }

        automaton_t& automaton() {
            return _swap_initial_final ? _final : _initial;
        }
        const automaton_t& automaton() const {
            return _swap_initial_final ? _final : _initial;
        }

        automaton_t& initial_automaton() {
            return _initial;
        }
        const automaton_t& initial_automaton() const {
            return _initial;
        }
        automaton_t& final_automaton() {
            return _final;
        }
        const automaton_t& final_automaton() const {
            return _final;
        }
        [[nodiscard]] const product_automaton_t& product_automaton() const {
            return _product;
        }

        const pda_t& pda() const {
            return _pda;
        }

        void enable_pre_star() {
            _swap_initial_final = true;
        }

        template<bool abstraction>
        using path_state = std::conditional_t<abstraction, std::pair<size_t,size_t>, size_t>;

        template<Trace_Type trace_type = Trace_Type::Any, bool abstraction = false>
        [[nodiscard]] typename std::conditional_t<
            (trace_type == Trace_Type::Shortest || trace_type == Trace_Type::Longest || trace_type == Trace_Type::ShortestFixedPoint) && is_weighted<W>,
        std::tuple<std::vector<path_state<abstraction>>, std::vector<uint32_t>, typename W::type>,
        std::tuple<std::vector<path_state<abstraction>>, std::vector<uint32_t>>>
        find_path() const {
            if constexpr ((trace_type == Trace_Type::Longest || trace_type == Trace_Type::ShortestFixedPoint) && W::is_weight) {
                PAutomatonFixedPoint<W,true,trace_type> fixed_point(_product);
                fixed_point.run();
                if (fixed_point.not_accepting()) {
                    return {std::vector<size_t>(), std::vector<uint32_t>(), solver_weight<W,trace_type>::max()};
                }
                if (fixed_point.is_infinite()) {
                    return {std::vector<size_t>(), std::vector<uint32_t>(), solver_weight<W,trace_type>::bottom()}; // TODO: Can we provide more info than this??
                }
                return fixed_point.get_path([this](size_t state) -> size_t { return get_original_ids(state).first; });
            } else if constexpr (trace_type == Trace_Type::Shortest && is_weighted<W>) { // TODO: Consider unweighted shortest path.
                // Dijkstra.
                struct queue_elem {
                    typename W::type weight;
                    size_t state;
                    uint32_t label;
                    size_t stack_index;
                    const queue_elem *back_pointer;
                    queue_elem(typename W::type weight, size_t state, uint32_t label, size_t stack_index, const queue_elem *back_pointer = nullptr)
                            : weight(weight), state(state), label(label), stack_index(stack_index), back_pointer(back_pointer) {};

                    bool operator<(const queue_elem& other) const {
                        return std::tie(state, label) < std::tie(other.state, other.label); // TODO: Is this correct? Should it not just be state?? and better then make 'visited' std::unordered_map<size_t, typename W::type>.
                    }
                    bool operator==(const queue_elem& other) const {
                        return state == other.state && label == other.label;
                    }
                    bool operator!=(const queue_elem& other) const {
                        return !(*this == other);
                    }
                };
                struct queue_elem_comp {
                    bool operator()(const queue_elem &lhs, const queue_elem &rhs){
                        return solver_weight<W,trace_type>::less(rhs.weight, lhs.weight); // Used in a max-heap, so swap arguments to make it a min-heap.
                    }
                };
                std::priority_queue<queue_elem, std::vector<queue_elem>, queue_elem_comp> search_queue;
                std::vector<queue_elem> visited;
                std::vector<std::unique_ptr<queue_elem>> pointers;
                for (size_t i = 0; i < _pda_size; ++i) { // Iterate over _product._initial ([i]->_id)
                    search_queue.emplace(W::zero(), i, std::numeric_limits<uint32_t>::max(), 0); // No label going into initial state.
                }
                while(!search_queue.empty()) {
                    auto current = search_queue.top();
                    search_queue.pop();

                    if (_product.states()[current.state]->_accepting) {
                        std::vector<path_state<abstraction>> path(current.stack_index + 1);
                        std::vector<uint32_t> label_stack(current.stack_index);
                        const queue_elem* p = &current;
                        while (p->stack_index > 0) {
                            path[p->stack_index] = get_original_ids(p->state).first;
                            label_stack[p->stack_index - 1] = p->label;
                            p = p->back_pointer;
                        }
                        if constexpr (abstraction) {
                            path[p->stack_index] = get_original_ids(p->state).to_pair();
                        } else {
                            path[p->stack_index] = get_original_ids(p->state).first;
                        }
                        return std::make_tuple(path, label_stack, current.weight);
                    }

                    auto lb = std::lower_bound(visited.begin(), visited.end(), current);
                    if (lb != std::end(visited) && *lb == current) {
                        if (solver_weight<W,trace_type>::less(current.weight, lb->weight)) {
                            *lb = current;
                        } else {
                            continue;
                        }
                    } else {
                        lb = visited.insert(lb, current); // TODO: Consider using std::unordered_map instead...
                    }
                    auto u_pointer = std::make_unique<queue_elem>(*lb);
                    auto pointer = u_pointer.get();
                    pointers.push_back(std::move(u_pointer));
                    for (const auto& [to,labels] : _product.states()[current.state]->_edges) {
                        if (!labels.empty()) {
                            auto label = std::min_element(labels.begin(), labels.end(), [](const auto& a, const auto& b){ return solver_weight<W,trace_type>::less(a.second.second, b.second.second); });
                            search_queue.emplace(solver_weight<W,trace_type>::add(current.weight, label->second.second), to, label->first, current.stack_index + 1, pointer);
                        }
                    }
                }
                return std::make_tuple(std::vector<path_state<abstraction>>(), std::vector<uint32_t>(), solver_weight<W,trace_type>::max());
            } else {
                // DFS search.
                std::vector<path_state<abstraction>> path;
                std::vector<uint32_t> label_stack;

                std::vector<std::tuple<size_t,size_t,uint32_t>> waiting; // state_id, stack_index, last_label (if stack_index > 0)
                waiting.reserve(_pda_size);
                for (size_t i = 0; i < _pda_size; ++i) {
                    if (_product.states()[i]->_accepting) { // Initial accepting state
                        if constexpr (abstraction) {
                            path.emplace_back(i,i);
                        } else {
                            path.push_back(i);
                        }
                        return std::make_tuple(path, label_stack);
                    }
                    waiting.emplace_back(i, 0, std::numeric_limits<uint32_t>::max()); // Add all initial states in _product.
                }
                std::unordered_set<size_t> seen;

                while (!waiting.empty()) {
                    auto [current, stack_index, last_label] = waiting.back();
                    waiting.pop_back();
                    path.resize(stack_index + 2);
                    label_stack.resize(stack_index + 1);
                    if constexpr (abstraction) {
                        path[stack_index] = get_original_ids(current).to_pair();
                    } else {
                        path[stack_index] = get_original_ids(current).first;
                    }
                    if (stack_index > 0) {
                        label_stack[stack_index - 1] = last_label;
                    }
                    for (const auto &[to,labels] : _product.states()[current]->_edges) {
                        if (!labels.empty() && seen.emplace(to).second) {
                            uint32_t label = labels[0].first;
                            if (_product.states()[to]->_accepting) {
                                if constexpr (abstraction) {
                                    path[stack_index + 1] = get_original_ids(to).to_pair();
                                } else {
                                    path[stack_index + 1] = get_original_ids(to).first;
                                }
                                label_stack[stack_index] = label;
                                return std::make_tuple(path, label_stack);
                            }
                            waiting.emplace_back(to, stack_index + 1, label);
                        }
                    }
                }
                return std::make_tuple(std::vector<path_state<abstraction>>(), std::vector<uint32_t>());
            }
        }

    private:
        template<bool edge_in_first = true, bool needs_back_lookup = false>
        bool add_edge(size_t from, uint32_t label, size_t to, trace_ptr<W> trace,
                      const automaton_t& first, const automaton_t& second) { // States in first and second automaton corresponds to respectively first and second component of the states in product automaton.
            static_assert(edge_in_first || needs_back_lookup, "If you insert edge in the second automaton, then you must also enable using _id_fast_lookup_back to keep the relevant information.");
            const auto& fast_lookup = constexpr_ternary<edge_in_first>(_id_fast_lookup, _id_fast_lookup_back);
            std::vector<std::pair<size_t,size_t>> from_states;
            if (from < fast_lookup.size()) { // Avoid out-of-bounds.
                from_states = fast_lookup[from]; // Copy here, since loop-body might alter fast_lookup[from].
            }
            if (from < _pda_size) {
                from_states.emplace_back(from, from); // Initial states are not stored in _id_fast_lookup.
            }
            const auto& current = constexpr_ternary<edge_in_first>(first, second);
            const auto& other = constexpr_ternary<edge_in_first>(second, first);
            auto current_to = current.states()[to].get();
            std::vector<size_t> waiting;
            for (auto [other_from, product_from] : from_states) { // Iterate through reachable 'from-states'.
                std::vector<size_t> other_tos;
                if (label == epsilon) {
                    other_tos.emplace_back(other_from);
                } else {
                    for (const auto& [other_to,other_labels] : other.states()[other_from]->_edges) {
                        if (other_labels.contains(label)) {
                            other_tos.emplace_back(other_to);
                        }
                    }
                }
                for (auto other_to : other_tos) {
                    auto [fresh, product_to] = get_product_state<needs_back_lookup>(swap_if<!edge_in_first>(current_to, other.states()[other_to].get()));
                    if (label == epsilon) {
                        _product.add_epsilon_edge(product_from, product_to, trace);
                    } else {
                        _product.add_edge(product_from, product_to, label, trace);
                    }
                    if (_product.has_accepting_state()) {
                        return true; // Early termination
                    }
                    if (fresh) {
                        waiting.push_back(product_to); // If the 'to-state' is new (was not previously reachable), we need to continue constructing from there.
                    }
                }
            }
            return construct_reachable<needs_back_lookup>(waiting, first, second);
        }

        // Returns whether an accepting state in the product automaton was reached.
        template<bool needs_back_lookup = false, bool ET = true>
        bool construct_reachable(std::vector<size_t>& waiting, const automaton_t& initial, const automaton_t& final) {
            while (!waiting.empty()) {
                size_t top = waiting.back();
                waiting.pop_back();
                auto [i_from,f_from] = get_original_ids(top);
                for (const auto& [i_to,i_labels] : initial.states()[i_from]->_edges) {
                    if (auto it = i_labels.find(epsilon); it != i_labels.end()) {
                        auto [fresh, product_to] = get_product_state<needs_back_lookup>(initial.states()[i_to].get(), final.states()[f_from].get());
                        _product.add_epsilon_edge(top, product_to, it->second);
                        if constexpr (ET) {
                            if (_product.has_accepting_state()) {
                                return true; // Early termination
                            }
                        }
                        if (fresh) {
                            waiting.push_back(product_to);
                        }
                    }
                    for (const auto& [f_to,f_labels] : final.states()[f_from]->_edges) {
                        if (auto it = f_labels.find(epsilon); it != f_labels.end()) {
                            auto [fresh, product_to] = get_product_state<needs_back_lookup>(initial.states()[i_from].get(), final.states()[f_to].get());
                            _product.add_epsilon_edge(top, product_to, it->second);
                            if constexpr (ET) {
                                if (_product.has_accepting_state()) {
                                    return true; // Early termination
                                }
                            }
                            if (fresh) {
                                waiting.push_back(product_to);
                            }
                        }
                        std::vector<typename decltype(i_labels)::value_type> labels;
                        std::set_intersection(i_labels.begin(), i_labels.end(), f_labels.begin(), f_labels.end(), std::back_inserter(labels));
                        if (!labels.empty() && labels.size() > (labels.back() == epsilon ? 1 : 0)) {
                            auto [fresh, to_id] = get_product_state<needs_back_lookup>(initial.states()[i_to].get(), final.states()[f_to].get());
                            for (const auto& [label, trace] : labels) {
                                if (label != epsilon) {
                                    _product.add_edge(top, to_id, label, trace);
                                }
                            }
                            if constexpr (ET) {
                                if (_product.has_accepting_state()) {
                                    return true; // Early termination
                                }
                            }
                            if (fresh) {
                                waiting.push_back(to_id);
                            }
                        }
                    }
                }
            }
            return _product.has_accepting_state();
        }

        static std::vector<size_t> get_initial_accepting(const automaton_t& a1, const automaton_t& a2) {
            assert(a1.pda().states().size() == a2.pda().states().size());
            auto size = a1.pda().states().size();
            std::vector<size_t> result;
            for (size_t i = 0; i < size; ++i) {
                if (a1.states()[i]->_accepting && a2.states()[i]->_accepting) {
                    result.push_back(i);
                }
            }
            return result;
        }

        struct pair_size_t { // ptrie does not work with std::pair, so we make struct here.
            size_t first;    // We need std::has_unique_object_representations_v<pair_size_t> to be true.
            size_t second;
            [[nodiscard]] std::pair<size_t,size_t> to_pair() const {
                return std::make_pair(first, second);
            }
        };

        [[nodiscard]] pair_size_t get_original_ids(size_t id) const {
            if (id < _pda_size) {
                return {id,id};
            }
            pair_size_t res;
            _id_map.unpack(id - _pda_size, &res);
            return res;
        }
        template<bool needs_back_lookup = false>
        std::pair<bool,size_t> get_product_state(std::pair<const state_t*, const state_t*> pair) {
            return get_product_state<needs_back_lookup>(pair.first, pair.second);
        }
        template<bool needs_back_lookup = false>
        std::pair<bool,size_t> get_product_state(const state_t* a, const state_t* b) {
            if (a->_id == b->_id && a->_id < _pda_size) {
                return std::make_pair(false, a->_id);
            }
            auto [fresh, id] = _id_map.insert(pair_size_t{a->_id, b->_id});
            if (fresh) {
                size_t state_id = _product.add_state(false, a->_accepting && b->_accepting);
                assert(state_id == id + _pda_size);
                if (a->_id >= _id_fast_lookup.size()) {
                    _id_fast_lookup.resize(a->_id + 1);
                }
                _id_fast_lookup[a->_id].emplace_back(b->_id, state_id);
                if constexpr(needs_back_lookup) {
                    if (b->_id >= _id_fast_lookup_back.size()) {
                        _id_fast_lookup_back.resize(b->_id + 1);
                    }
                    _id_fast_lookup_back[b->_id].emplace_back(a->_id, state_id);
                }
                return std::make_pair(true, state_id);
            } else {
                return std::make_pair(false ,id + _pda_size);
            }
        }

        // ***
        // Utility stuff that is used in this class.
        // ***
        // Change order of two arguments based on constexpr value can be annoying.
        template<bool condition, typename T1, typename T2>
        static constexpr auto swap_if(T1&& a, T2&& b) {
            if constexpr (condition) {
                return std::make_pair(std::forward<T2>(b), std::forward<T1>(a));
            } else {
                return std::make_pair(std::forward<T1>(a), std::forward<T2>(b));
            }
        }
        // A const& needs to be initialized, and if constexpr has local scope, so this was the best solution I found
        // to make a const&, where a compile-time predicate determines where it points to.
        template<bool condition, typename T>
        static constexpr const T& constexpr_ternary(const T& a, const T& b) noexcept {
            if constexpr (condition) {
                return a;
            } else {
                return b;
            }
        }
        template<typename Elem>
        static std::vector<Elem> intersect_vector(const std::vector<Elem>& v1, const std::vector<Elem>& v2) {
            assert(std::is_sorted(v1.begin(), v1.end()));
            assert(std::is_sorted(v2.begin(), v2.end()));
            std::vector<Elem> result;
            std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(result));
            return result;
        }
        // ***

    private:
        const pda_t& _pda;
        const size_t _pda_size;
        automaton_t _initial;
        automaton_t _final;
        product_automaton_t _product;
        bool _swap_initial_final = false;
        ptrie::set_stable<pair_size_t> _id_map;
        std::vector<std::vector<std::pair<size_t,size_t>>> _id_fast_lookup; // maps initial_state -> (final_state, product_state)
        std::vector<std::vector<std::pair<size_t,size_t>>> _id_fast_lookup_back; // maps final_state -> (initial_state, product_state)  Only used in dual_search
    };

    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, bool indirect>
    PAutomatonProduct(const TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                      PAutomaton<W,indirect> initial, PAutomaton<W,indirect> final) -> PAutomatonProduct<TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>,PAutomaton<W,indirect>,W>;

}

#endif //PDAAAL_PAUTOMATONPRODUCT_H
