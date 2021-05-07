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
 * File:   Solver.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 09-03-2020.
 */

#ifndef PDAAAL_SOLVER_H
#define PDAAAL_SOLVER_H

#include "PAutomaton.h"
#include "TypedPDA.h"
#include "SolverInstance.h"

namespace pdaaal {


    class Solver {
    private:
        static constexpr auto epsilon = std::numeric_limits<uint32_t>::max();

        struct temp_edge_t {
            size_t _from = std::numeric_limits<size_t>::max();
            size_t _to = std::numeric_limits<size_t>::max();
            uint32_t _label = std::numeric_limits<uint32_t>::max();

            temp_edge_t() = default;

            temp_edge_t(size_t from, uint32_t label, size_t to)
                    : _from(from), _to(to), _label(label) {};

            bool operator<(const temp_edge_t &other) const {
                if (_from != other._from) return _from < other._from;
                if (_label != other._label) return _label < other._label;
                return _to < other._to;
            }

            bool operator==(const temp_edge_t &other) const {
                return _from == other._from && _to == other._to && _label == other._label;
            }

            bool operator!=(const temp_edge_t &other) const {
                return !(*this == other);
            }
        };
        struct temp_edge_hasher {
            std::size_t operator()(const temp_edge_t& e) const
            {
                std::size_t seed = 0;
                boost::hash_combine(seed, e._from);
                boost::hash_combine(seed, e._to);
                boost::hash_combine(seed, e._label);
                return seed;
            }
        };

        template <typename W>
        using early_termination_fn = std::function<bool(size_t,uint32_t,size_t,trace_ptr<W> trace)>;

    public:

        template <typename W, typename C, typename A>
        static bool pre_star_accepts(PAutomaton<W,C,A> &automaton, size_t state, const std::vector<uint32_t> &stack) {
            if (stack.size() == 1) {
                auto s_label = stack[0];
                return pre_star<W,C,A,true>(automaton, [&automaton, state, s_label](size_t from, uint32_t label, size_t to, trace_ptr<W> trace) -> bool {
                    return from == state && label == s_label && automaton.states()[to]->_accepting;
                });
            } else {
                return pre_star<W,C,A>(automaton) || automaton.accepts(state, stack);
            }
        }

        template <typename pda_t, typename automaton_t, typename T, typename W, typename C, typename A>
        static bool pre_star_accepts(SolverInstance_impl<pda_t,automaton_t,T,W,C,A>& instance) {
            instance.enable_pre_star();
            return instance.initialize_product() ||
                   pre_star<W,C,A,true>(instance.automaton(), [&instance](size_t from, uint32_t label, size_t to, trace_ptr<W> trace) -> bool {
                       return instance.add_edge_product(from, label, to, trace);
                   });
        }

        template <typename W, typename C, typename A, bool ET=false>
        static bool pre_star(PAutomaton<W,C,A> &automaton, const early_termination_fn<W>& early_termination = [](size_t f, uint32_t l, size_t t, trace_ptr<W> trace) -> bool { return false; }) {
            // This is an implementation of Algorithm 1 (figure 3.3) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 42)
            const auto& pda_states = automaton.pda().states();
            const auto n_pda_states = pda_states.size();
            const auto n_automaton_states = automaton.states().size();

            std::unordered_set<temp_edge_t, temp_edge_hasher> edges;
            std::stack<temp_edge_t> workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel(n_automaton_states);

            bool found = false;
            const auto insert_edge = [&found, &early_termination, &edges, &workset, &automaton](size_t from, uint32_t label, size_t to, const trace_t *trace) {
                auto res = edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U workset).
                    workset.emplace(from, label, to);
                    if (trace != nullptr) { // Don't add existing edges
                        if constexpr (ET) {
                            found = found || early_termination(from, label, to, trace_ptr_from<W>(trace));
                        }
                        automaton.add_edge(from, to, label, trace_ptr_from<W>(trace));
                    }
                }
            };
            const auto n_pda_labels = automaton.number_of_labels();
            const auto insert_edge_bulk = [&insert_edge, n_pda_labels](size_t from, const labels_t &precondition, size_t to, const trace_t *trace) {
                if (precondition.wildcard()) {
                    for (uint32_t i = 0; i < n_pda_labels; i++) {
                        insert_edge(from, i, to, trace);
                    }
                } else {
                    for (auto &label : precondition.labels()) {
                        insert_edge(from, label, to, trace);
                    }
                }
            };

            // workset := ->_0  (line 1)
            for (const auto &from : automaton.states()) {
                for (const auto &[to,labels] : from->_edges) {
                    for (const auto &[label,_] : labels) {
                        insert_edge(from->_id, label, to, nullptr);
                    }
                }
            }

            // for all <p, y> --> <p', epsilon> : workset U= (p, y, p') (line 2)
            for (size_t state = 0; state < n_pda_states; ++state) {
                size_t rule_id = 0;
                for (const auto&[rule,labels] : pda_states[state]._rules) {
                    if (rule._operation == POP) {
                        insert_edge_bulk(state, labels, rule._to, automaton.new_pre_trace(rule_id));
                    }
                    ++rule_id;
                }
            }

            // delta_prime[q] = [p,rule_id]    where states[p]._rules[rule_id]._labels.contains(y)    for each p, rule_id
            // corresponds to <p, y> --> <q, y>   (the y is the same, since we only have PUSH and not arbitrary <p, y> --> <q, y1 y2>, i.e. y==y2)
            std::vector<std::vector<std::pair<size_t, size_t>>> delta_prime(n_automaton_states);

            while (!workset.empty()) { // (line 3)
                if constexpr (ET) {
                    if (found) {
                        return true;
                    }
                }

                // pop t = (q, y, q') from workset (line 4)
                auto t = workset.top();
                workset.pop();
                // rel = rel U {t} (line 6)   (membership test on line 5 is done in insert_edge).
                rel[t._from].emplace_back(t._to, t._label);

                // (line 7-8 for \Delta')
                for (auto pair : delta_prime[t._from]) { // Loop over delta_prime (that match with t->from)
                    auto state = pair.first;
                    auto rule_id = pair.second;
                    if (pda_states[state]._rules[rule_id].second.contains(t._label)) {
                        insert_edge(state, t._label, t._to, automaton.new_pre_trace(rule_id, t._from));
                    }
                }
                // Loop over \Delta (filter rules going into q) (line 7 and 9)
                if (t._from >= n_pda_states) { continue; }
                for (auto pre_state : pda_states[t._from]._pre_states) {
                    const auto &rules = pda_states[pre_state]._rules;
                    auto lb = rules.lower_bound(details::rule_t<W,C>{t._from});
                    while (lb != rules.end() && lb->first._to == t._from) {
                        const auto &[rule, labels] = *lb;
                        size_t rule_id = lb - rules.begin();
                        ++lb;
                        switch (rule._operation) {
                            case POP:
                                break;
                            case SWAP: // (line 7-8 for \Delta)
                                if (rule._op_label == t._label) {
                                    insert_edge_bulk(pre_state, labels, t._to, automaton.new_pre_trace(rule_id));
                                }
                                break;
                            case NOOP: // (line 7-8 for \Delta)
                                if (labels.contains(t._label)) {
                                    insert_edge(pre_state, t._label, t._to, automaton.new_pre_trace(rule_id));
                                }
                                break;
                            case PUSH: // (line 9)
                                if (rule._op_label == t._label) {
                                    // (line 10)
                                    delta_prime[t._to].emplace_back(pre_state, rule_id);
                                    const trace_t *trace = nullptr;
                                    for (auto rel_rule : rel[t._to]) { // (line 11-12)
                                        if (labels.contains(rel_rule.second)) {
                                            trace = trace == nullptr ? automaton.new_pre_trace(rule_id, t._to) : trace;
                                            insert_edge(pre_state, rel_rule.second, rel_rule.first, trace);
                                        }
                                    }
                                }
                                break;
                            default:
                                assert(false);
                        }
                    }
                }
            }
            return found;
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename W, typename C, typename A>
        static bool post_star_accepts(PAutomaton<W,C,A> &automaton, size_t state, const std::vector<uint32_t> &stack) {
            if (stack.size() == 1) {
                auto s_label = stack[0];
                return post_star<trace_type,W,C,A,true>(automaton, [&automaton, state, s_label](size_t from, uint32_t label, size_t to, trace_ptr<W> trace) -> bool {
                    return from == state && label == s_label && automaton.states()[to]->_accepting;
                });
            } else {
                return post_star<trace_type,W,C,A>(automaton) || automaton.accepts(state, stack);
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename pda_t, typename automaton_t, typename T, typename W, typename C, typename A>
        static bool post_star_accepts(SolverInstance_impl<pda_t,automaton_t,T,W,C,A>& instance) {
            return instance.initialize_product() ||
                   post_star<trace_type,W,C,A,true>(instance.automaton(), [&instance](size_t from, uint32_t label, size_t to, trace_ptr<W> trace) -> bool {
                       return instance.add_edge_product(from, label, to, trace);
                   });
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename C, typename A>
        static auto get_trace(const SolverInstance<T,W,C,A>& instance) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto [path, stack, weight] = instance.template find_path<trace_type>();
                return std::make_pair(_get_trace(instance.pda(), instance.automaton(), path, stack), weight);
            } else {
                auto [path, stack] = instance.template find_path<trace_type>();
                return _get_trace(instance.pda(), instance.automaton(), path, stack);
            }
        }
        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename C, typename A>
        static auto get_rule_trace_and_paths(const AbstractionSolverInstance<T,W,C,A>& instance) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto [paths, stack, weight] = instance.template find_path<trace_type, true>();
                return std::make_pair(_get_rule_trace_and_paths(instance.automaton(), paths, stack), weight);
            } else {
                auto [paths, stack] = instance.template find_path<trace_type, true>();
                return _get_rule_trace_and_paths(instance.automaton(), paths, stack);
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename W, typename C, typename A, bool ET = false>
        static bool post_star(PAutomaton<W,C,A> &automaton, const early_termination_fn<W>& early_termination = [](size_t f, uint32_t l, size_t t, trace_ptr<W> trace) -> bool { return false; }) {
            static_assert(is_weighted<W> || trace_type != Trace_Type::Shortest, "Cannot do shortest-trace post* for PDA without weights."); // TODO: Consider: W=uin32_t, weight==1 as a default weight.
            if constexpr (is_weighted<W> && trace_type == Trace_Type::Shortest) {
                return post_star_shortest<W,C,A,true,ET>(automaton, early_termination);
            } else if constexpr (trace_type == Trace_Type::Any) {
                return post_star_any<W,C,A,ET>(automaton, early_termination);
            } else if constexpr (trace_type == Trace_Type::None) {
                return post_star_any<W,C,A,ET>(automaton, early_termination); // TODO: Implement faster no-trace option.
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename C, typename A>
        static auto get_trace(const TypedPDA<T,W,C>& pda, const PAutomaton<W,C,A>& automaton, size_t state, const std::vector<T>& stack) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            auto stack_native = pda.encode_pre(stack);
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto [path, weight] = automaton.template accept_path<trace_type>(state, stack_native);
                return std::make_pair(_get_trace(pda, automaton, path, stack_native), weight);
            } else {
                auto path = automaton.template accept_path<trace_type>(state, stack_native);
                return _get_trace(pda, automaton, path, stack_native);
            }
        }
        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename C, typename A, typename = std::enable_if_t<!std::is_same_v<T,uint32_t>>>
        static auto get_trace(const TypedPDA<T,W,C>& pda, const PAutomaton<W,C,A>& automaton, size_t state, const std::vector<uint32_t>& stack_native) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto [path, weight] = automaton.template accept_path<trace_type>(state, stack_native);
                return std::make_pair(_get_trace(pda, automaton, path, stack_native),weight);
            } else {
                auto path = automaton.template accept_path<trace_type>(state, stack_native);
                return _get_trace(pda, automaton, path, stack_native);
            }
        }

    private:
        template<typename W, typename C, typename A, bool Enable, bool ET, typename = std::enable_if_t<Enable>>
        static bool post_star_shortest(PAutomaton<W,C,A> &automaton, const early_termination_fn<W>& early_termination) {
            static_assert(is_weighted<W>);
            const A add;
            const C less;

            struct weight_edge_trace {
                W weight;
                temp_edge_t edge;
                const trace_t *trace = nullptr;
                weight_edge_trace(W weight, temp_edge_t edge, const trace_t *trace) : weight(weight), edge(edge), trace(trace) {};
                weight_edge_trace() = default;
            };
            struct weight_edge_trace_comp{
                bool operator()(const weight_edge_trace &lhs, const weight_edge_trace &rhs){
                    const C less;
                    return less(rhs.weight, lhs.weight); // Used in a max-heap, so swap arguments to make it a min-heap.
                }
            };

            // This is an implementation of Algorithm 2 (figure 3.4) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 48)
            const auto & pda_states = automaton.pda().states();
            const auto n_pda_states = pda_states.size();
            const auto n_Q = automaton.states().size();

            // for <p, y> -> <p', y1 y2> do
            //   Q' U= {q_p'y1}
            std::unordered_map<std::pair<size_t, uint32_t>, size_t, boost::hash<std::pair<size_t, uint32_t>>> q_prime{};
            for (const auto &state : pda_states) {
                for (const auto &[rule,labels] : state._rules) {
                    if (rule._operation == PUSH) {
                        auto res = q_prime.emplace(std::make_pair(rule._to, rule._op_label), automaton.next_state_id());
                        if (res.second) {
                            automaton.add_state(false, false);
                        }
                    }
                }
            }
            const auto n_automata_states = automaton.states().size();
            std::vector<W> minpath(n_automata_states - n_Q);
            for (size_t i = 0; i < minpath.size(); ++i) {
                minpath[i] = pdaaal::max<W>()();
            }

            std::unordered_map<temp_edge_t, std::pair<W,W>, temp_edge_hasher> edge_weights;
            std::priority_queue<weight_edge_trace, std::vector<weight_edge_trace>, weight_edge_trace_comp> workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel1(n_Q); // faster access for lookup _from -> (_to, _label)
            std::vector<std::vector<size_t>> rel2(n_automata_states - n_Q); // faster access for lookup _to -> _from  (when _label is uint32_t::max)
            struct rel3_elem {
                uint32_t _label;
                size_t _to;
                const trace_t *_trace;
                W _weight;

                bool operator<(const rel3_elem &other) const {
                    if (_label != other._label) return _label < other._label;
                    return _to < other._to;
                }
                bool operator==(const rel3_elem &other) const {
                    return _to == other._to && _label == other._label;
                }
                bool operator!=(const rel3_elem &other) const {
                    return !(*this == other);
                }
            };
            std::vector<std::vector<rel3_elem>> rel3(n_automata_states - n_Q);

            const auto update_edge_ = [&edge_weights](size_t from, uint32_t label, size_t to, W edge_weight, W workset_weight) -> std::pair<bool,bool> {
                auto res = edge_weights.emplace(temp_edge_t{from, label, to}, std::make_pair(edge_weight, workset_weight));
                if (!res.second) {
                    C less;
                    auto result = std::make_pair(false, false);
                    if (less(edge_weight, (*res.first).second.first)) {
                        (*res.first).second.first = edge_weight;
                        result.first = true;
                    }
                    if (less(workset_weight, (*res.first).second.second)) {
                        (*res.first).second.second = workset_weight;
                        result.second = true;
                    }
                    return result;
                }
                return std::make_pair(res.second, res.second);
            };
            const auto update_edge = [n_Q, &minpath, &workset, &update_edge_, &add](size_t from, uint32_t label, size_t to, W edge_weight, const trace_t* trace) {
                auto workset_weight = to < n_Q ? edge_weight : add(minpath[to - n_Q], edge_weight);
                if (update_edge_(from, label, to, edge_weight, workset_weight).second) {
                    workset.emplace(workset_weight, temp_edge_t{from, label, to}, trace);
                }
            };
            const auto get_weight = [&edge_weights](size_t from, uint32_t label, size_t to) -> W {
                return (*edge_weights.find(temp_edge_t{from, label, to})).second.first;
            };

            bool found = false;

            // Adds to rel.
            const auto insert_rel = [&rel1, &rel2, n_Q](size_t from, uint32_t label, size_t to){
                rel1[from].emplace_back(to, label);
                if (label == epsilon && to >= n_Q) {
                    rel2[to - n_Q].push_back(from);
                }
            };

            // workset := ->_0 intersect (P x Gamma x Q)
            // rel := ->_0 \ workset
            for (auto &from : automaton.states()) {
                for (auto &[to,labels] : from->_edges) {
                    assert(!labels.contains(epsilon)); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (auto &[label,trace] : labels) {
                        temp_edge_t temp_edge{from->_id, label, to};
                        edge_weights.emplace(temp_edge, std::make_pair(zero<W>()(), zero<W>()()));
                        if (from->_id < n_pda_states) {
                            workset.emplace(zero<W>()(), temp_edge, nullptr);
                        } else {
                            insert_rel(from->_id, label, to);
                            if constexpr (ET) {
                                found = found || early_termination(from->_id, label, to, trace);
                            }
                        }
                    }
                }
            }

            while (!workset.empty()) {
                if constexpr (ET) {
                    if (found) {
                        break;
                    }
                }
                // pop t = (q, y, q') from workset
                auto elem = workset.top();
                workset.pop();
                auto t = elem.edge;
                auto weights = (*edge_weights.find(t)).second;
                if (less(weights.second, elem.weight)) {
                    continue; // Same edge with a smaller weight was already processed.
                }
                auto t_weight = weights.first;

                // rel = rel U {t}
                insert_rel(t._from, t._label, t._to);
                if (t._label == epsilon) {
                    automaton.add_epsilon_edge(t._from, t._to, std::make_pair(elem.trace, t_weight));
                } else {
                    automaton.add_edge(t._from, t._to, t._label, std::make_pair(elem.trace, t_weight));
                }
                if constexpr (ET) {
                    found = found || early_termination(t._from, t._label, t._to, std::make_pair(elem.trace, t_weight));
                }

                // if y != epsilon
                if (t._label != epsilon) {
                    const auto &rules = pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        const auto &[rule,labels] = rules[rule_id];
                        if (!labels.contains(t._label)) { continue; }
                        auto trace = automaton.new_post_trace(t._from, rule_id, t._label);
                        auto wd = add(elem.weight, rule._weight);
                        auto wb = add(t_weight, rule._weight);
                        if (rule._operation != PUSH) {
                            uint32_t label = 0;
                            switch(rule._operation) {
                                case POP:
                                    label = epsilon;
                                    break;
                                case SWAP:
                                    label = rule._op_label;
                                    break;
                                case NOOP:
                                    label = t._label;
                                    break;
                                case PUSH:
                                default:
                                    assert(false); // Does not happen!
                            }
                            update_edge(rule._to, label, t._to, wb, trace);
                        } else { // rule._operation == PUSH
                            assert(q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(q_prime));
                            size_t q_new = q_prime[std::make_pair(rule._to, rule._op_label)];
                            auto add_to_workset = update_edge_(rule._to, rule._op_label, q_new, zero<W>()(), wd).second;
                            auto was_updated = update_edge_(q_new, t._label, t._to, wb, zero<W>()()).first;
                            if (was_updated) {
                                rel3_elem new_elem{t._label, t._to, trace, wb};
                                auto &relq = rel3[q_new - n_Q];
                                auto lb = std::lower_bound(relq.begin(), relq.end(), new_elem);
                                if (lb == std::end(relq) || *lb != new_elem) {
                                    relq.insert(lb, new_elem);
                                } else if (less(wb, lb->_weight)) {
                                    *lb = new_elem;
                                }
                            }
                            if (less(wd, minpath[q_new - n_Q])) {
                                minpath[q_new - n_Q] = wd;
                                if (add_to_workset) {
                                    workset.emplace(wd, temp_edge_t{rule._to, rule._op_label, q_new}, trace);
                                }
                            } else if (was_updated) {
                                if (!rel2[q_new - n_Q].empty()) {
                                    auto trace_q_new = automaton.new_post_trace(q_new);
                                    for (auto f : rel2[q_new - n_Q]) {
                                        update_edge(f, t._label, t._to, add(get_weight(f, epsilon, q_new), wb), trace_q_new);
                                    }
                                }
                            }
                        }
                    }
                } else {
                    if (t._to < n_Q) {
                        if (!rel1[t._to].empty()) {
                            auto trace = automaton.new_post_trace(t._to);
                            for (auto e : rel1[t._to]) {
                                assert(e.first >= n_pda_states);
                                update_edge(t._from, e.second, e.first, add(get_weight(t._to, e.second, e.first), t_weight), trace);
                            }
                        }
                    } else {
                        if (!rel3[t._to - n_Q].empty()) {
                            auto trace = automaton.new_post_trace(t._to);
                            for (auto &e : rel3[t._to - n_Q]) {
                                update_edge(t._from, e._label, e._to, add(get_weight(t._to, e._label, e._to), t_weight), trace);
                            }
                        }
                    }
                }
            }

            for (size_t i = n_Q; i < n_automata_states; ++i) {
                for (auto &e : rel3[i - n_Q]) {
                    assert(e._label != epsilon);
                    automaton.add_edge(i, e._to, e._label, std::make_pair(e._trace, e._weight));
                    if constexpr (ET) {
                        found = found || early_termination(i, e._label, e._to, std::make_pair(e._trace, e._weight));
                    }
                }
            }
            return found;
        }

        template <typename W, typename C, typename A, bool ET>
        static bool post_star_any(PAutomaton<W,C,A> &automaton, const early_termination_fn<W>& early_termination) {
            // This is an implementation of Algorithm 2 (figure 3.4) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 48)
            const auto & pda_states = automaton.pda().states();
            const auto n_pda_states = pda_states.size();
            const auto n_Q = automaton.states().size();

            // for <p, y> -> <p', y1 y2> do  (line 3)
            //   Q' U= {q_p'y1}              (line 4)
            std::unordered_map<std::pair<size_t, uint32_t>, size_t, boost::hash<std::pair<size_t, uint32_t>>> q_prime{};
            for (auto &state : pda_states) {
                for (auto &[rule, labels] : state._rules) {
                    if (rule._operation == PUSH) {
                        auto res = q_prime.emplace(std::make_pair(rule._to, rule._op_label), automaton.next_state_id());
                        if (res.second) {
                            automaton.add_state(false, false);
                        }
                    }
                }
            }
            const auto n_automata_states = automaton.states().size();

            std::unordered_set<temp_edge_t, temp_edge_hasher> edges;
            std::queue<temp_edge_t> workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel1(n_automata_states); // faster access for lookup _from -> (_to, _label)
            std::vector<std::vector<size_t>> rel2(n_automata_states - n_Q); // faster access for lookup _to -> _from  (when _label is uint32_t::max)

            bool found = false;
            const auto insert_edge = [&found, &early_termination, &edges, &workset, &rel1, &rel2, &automaton, n_Q]
                    (size_t from, uint32_t label, size_t to, const trace_t *trace, bool direct_to_rel = false) {
                auto res = edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U workset).
                    if (direct_to_rel) {
                        rel1[from].emplace_back(to, label);
                        if (label == epsilon && to >= n_Q) {
                            rel2[to - n_Q].push_back(from);
                        }
                    } else {
                        workset.emplace(from, label, to);
                    }
                    if (trace != nullptr) { // Don't add existing edges
                        if (label == epsilon) {
                            automaton.add_epsilon_edge(from, to, trace_ptr_from<W>(trace));
                        } else {
                            automaton.add_edge(from, to, label, trace_ptr_from<W>(trace));
                        }
                    }
                    if constexpr (ET) {
                        found = found || early_termination(from, label, to, trace_ptr_from<W>(trace));
                    }
                }
            };

            // workset := ->_0 intersect (P x Gamma x Q)  (line 1)
            // rel := ->_0 \ workset (line 2)
            for (const auto &from : automaton.states()) {
                for (const auto &[to,labels] : from->_edges) {
                    assert(!labels.contains(epsilon)); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (const auto &[label,_] : labels) {
                        insert_edge(from->_id, label, to, nullptr, from->_id >= n_pda_states);
                    }
                }
            }

            while (!workset.empty()) { // (line 5)
                if constexpr (ET) {
                    if (found) {
                        return true;
                    }
                }

                // pop t = (q, y, q') from workset (line 6)
                temp_edge_t t;
                t = workset.front();
                workset.pop();
                // rel = rel U {t} (line 8)   (membership test on line 7 is done in insert_edge).
                rel1[t._from].emplace_back(t._to, t._label);
                if (t._label == epsilon && t._to >= n_Q) {
                    rel2[t._to - n_Q].push_back(t._from);
                }

                // if y != epsilon (line 9)
                if (t._label != epsilon) {
                    const auto &rules = pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        const auto &[rule,labels] = rules[rule_id];
                        if (!labels.contains(t._label)) { continue; }
                        auto trace = automaton.new_post_trace(t._from, rule_id, t._label);
                        switch (rule._operation) {
                            case POP: // (line 10-11)
                                insert_edge(rule._to, epsilon, t._to, trace, false);
                                break;
                            case SWAP: // (line 12-13)
                                insert_edge(rule._to, rule._op_label, t._to, trace, false);
                                break;
                            case NOOP:
                                insert_edge(rule._to, t._label, t._to, trace, false);
                                break;
                            case PUSH: // (line 14)
                                assert(q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(q_prime));
                                size_t q_new = q_prime[std::make_pair(rule._to, rule._op_label)];
                                insert_edge(rule._to, rule._op_label, q_new, trace, false); // (line 15)
                                insert_edge(q_new, t._label, t._to, trace, true); // (line 16)
                                if (!rel2[q_new - n_Q].empty()) {
                                    auto trace_q_new = automaton.new_post_trace(q_new);
                                    for (auto f : rel2[q_new - n_Q]) { // (line 17)
                                        insert_edge(f, t._label, t._to, trace_q_new, false); // (line 18)
                                    }
                                }
                                break;
                        }
                    }
                } else {
                    if (!rel1[t._to].empty()) {
                        auto trace = automaton.new_post_trace(t._to);
                        for (auto e : rel1[t._to]) { // (line 20)
                            insert_edge(t._from, e.second, e.first, trace, false); // (line 21)
                        }
                    }
                }
            }
            return found;
        }

        template <typename T, typename W, typename C, typename A>
        static std::vector<typename TypedPDA<T>::tracestate_t> _get_trace(const TypedPDA<T,W,C> &pda, const PAutomaton<W,C,A> &automaton, const std::vector<size_t>& path, const std::vector<uint32_t>& stack) {
            using tracestate_t = typename TypedPDA<T>::tracestate_t;

            if (path.empty()) {
                return std::vector<tracestate_t>();
            }
            std::deque<std::tuple<size_t, uint32_t, size_t>> edges;
            for (size_t i = stack.size(); i > 0; --i) {
                edges.emplace_back(path[i - 1], stack[i - 1], path[i]);
            }

            auto decode_edges = [&pda](const std::deque<std::tuple<size_t, uint32_t, size_t>> &edges) -> tracestate_t {
                tracestate_t result{std::get<0>(edges.back()), std::vector<T>()};
                auto num_labels = pda.number_of_labels();
                for (auto it = edges.crbegin(); it != edges.crend(); ++it) {
                    auto label = std::get<1>(*it);
                    if (label < num_labels){
                        result._stack.emplace_back(pda.get_symbol(label));
                    }
                }
                return result;
            };

            bool post = false;

            std::vector<tracestate_t> trace;
            trace.push_back(decode_edges(edges));
            while (true) {
                auto [from, label, to] = edges.back();
                edges.pop_back();
                const trace_t *trace_label = automaton.get_trace_label(from, label, to);
                if (trace_label == nullptr) break;

                if (trace_label->is_pre_trace()) {
                    // pre* trace
                    const auto &[rule,labels] = automaton.pda().states()[from]._rules[trace_label->_rule_id];
                    switch (rule._operation) {
                        case POP:
                            break;
                        case SWAP:
                            edges.emplace_back(rule._to, rule._op_label, to);
                            break;
                        case NOOP:
                            edges.emplace_back(rule._to, label, to);
                            break;
                        case PUSH:
                            edges.emplace_back(trace_label->_state, label, to);
                            edges.emplace_back(rule._to, rule._op_label, trace_label->_state);
                            break;
                    }
                    trace.push_back(decode_edges(edges));

                } else if (trace_label->is_post_epsilon_trace()) {
                    // Intermediate post* trace
                    // Current edge is the result of merging with an epsilon edge.
                    // Reconstruct epsilon edge and the other edge.
                    edges.emplace_back(trace_label->_state, label, to);
                    edges.emplace_back(from, std::numeric_limits<uint32_t>::max(), trace_label->_state);

                } else {
                    // post* trace
                    const auto &[rule, labels] = automaton.pda().states()[trace_label->_state]._rules[trace_label->_rule_id];
                    switch (rule._operation) {
                        case POP:
                        case SWAP:
                        case NOOP:
                            edges.emplace_back(trace_label->_state, trace_label->_label, to);
                            break;
                        case PUSH:
                            auto [from2, label2, to2] = edges.back();
                            edges.pop_back();
                            auto trace_label2 = automaton.get_trace_label(from2, label2, to2);
                            edges.emplace_back(trace_label2->_state, trace_label2->_label, to2);
                            break;
                    }
                    trace.push_back(decode_edges(edges));
                    post = true;
                }
            }
            if (post) {
                std::reverse(trace.begin(), trace.end());
            }
            return trace;
        }


        template <typename W, typename C, typename A>
        static std::tuple<
                size_t, // Initial state. (State is size_t::max if no trace exists.)
                std::vector<user_rule_t<W,C>>, // Sequence of rules applied to the initial configuration to reach the final configuration.
                std::vector<uint32_t>, // Initial stack
                std::vector<uint32_t>, // Final stack
                std::vector<size_t>, // Path in initial PAutomaton (accepting initial stack)
                std::vector<size_t>  // Path in final PAutomaton (accepting final stack) (independent of whether pre* or post* was used)
                > _get_rule_trace_and_paths(const PAutomaton<W,C,A> &automaton, // The PAutomaton that has been build up (either A_pre* or A_post*)
                                           const std::vector<std::pair<size_t,size_t>>& paths, // The paths as retrieved from the product automaton. First number is the state in @automaton (A_pre* or A_post*), second number is state in goal automaton.
                                           const std::vector<uint32_t>& stack) {
            using rule_t = user_rule_t<W,C>;

            if (paths.empty()) {
                return std::make_tuple(std::numeric_limits<size_t>::max(), std::vector<rule_t>(), std::vector<uint32_t>(), std::vector<uint32_t>(), std::vector<size_t>(), std::vector<size_t>());
            }
            assert(stack.size() + 1 == paths.size());
            // Get path in goal automaton (returned in the end).
            std::vector<size_t> goal_path;
            goal_path.reserve(paths.size());
            for (auto [a,b] : paths) {
                goal_path.push_back(b);
            }
            // Build up stack of edges in the PAutomaton. Each PDA rule corresponds to changing some of the top edges.
            std::deque<std::tuple<size_t, uint32_t, size_t>> edges;
            for (size_t i = stack.size(); i > 0; --i) {
                edges.emplace_back(paths[i - 1].first, stack[i - 1], paths[i].first);
            }

            bool post = false;

            std::vector<rule_t> trace;
            while (true) {
                auto [from, label, to] = edges.back();
                const trace_t *trace_label = automaton.get_trace_label(from, label, to);
                if (trace_label == nullptr) break;
                edges.pop_back();

                if (trace_label->is_pre_trace()) {
                    // pre* trace
                    const auto &[rule,labels] = automaton.pda().states()[from]._rules[trace_label->_rule_id];
                    switch (rule._operation) {
                        case POP:
                            break;
                        case SWAP:
                            edges.emplace_back(rule._to, rule._op_label, to);
                            break;
                        case NOOP:
                            edges.emplace_back(rule._to, label, to);
                            break;
                        case PUSH:
                            edges.emplace_back(trace_label->_state, label, to);
                            edges.emplace_back(rule._to, rule._op_label, trace_label->_state);
                            break;
                    }
                    trace.emplace_back(from, label, rule);
                } else if (trace_label->is_post_epsilon_trace()) {
                    // Intermediate post* trace
                    // Current edge is the result of merging with an epsilon edge.
                    // Reconstruct epsilon edge and the other edge.
                    edges.emplace_back(trace_label->_state, label, to);
                    edges.emplace_back(from, std::numeric_limits<uint32_t>::max(), trace_label->_state);

                } else { // post* trace
                    post = true;
                    const auto &[rule, labels] = automaton.pda().states()[trace_label->_state]._rules[trace_label->_rule_id];
                    switch (rule._operation) {
                        case POP:
                        case SWAP:
                        case NOOP:
                            edges.emplace_back(trace_label->_state, trace_label->_label, to);
                            break;
                        case PUSH:
                            auto [from2, label2, to2] = edges.back();
                            edges.pop_back();
                            trace_label = automaton.get_trace_label(from2, label2, to2);
                            assert(trace_label != nullptr);
                            edges.emplace_back(trace_label->_state, trace_label->_label, to2);
                            break;
                    }
                    assert(from == rule._to);
                    trace.emplace_back(trace_label->_state, trace_label->_label, rule);
                }
            }

            // Get accepting path of initial stack (and the initial stack itself - for post*)
            std::vector<uint32_t> start_stack;
            start_stack.reserve(edges.size());
            std::vector<size_t> start_path;
            start_path.reserve(edges.size() + 1);
            start_path.push_back(std::get<0>(edges.back()));
            for (auto it = edges.crbegin(); it != edges.crend(); ++it) {
                start_path.push_back(std::get<2>(*it));
                start_stack.push_back(std::get<1>(*it));
            }

            if (post) { // post* was used
                std::reverse(trace.begin(), trace.end());
                return std::make_tuple(trace[0].from(), trace, start_stack, stack, start_path, goal_path);
            } else { // pre* was used
                return std::make_tuple(paths[0].first, trace, stack, start_stack, goal_path, start_path);
            }
        }
    };
}

#endif //PDAAAL_SOLVER_H
