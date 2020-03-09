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
 * Created on 09-03-2020.
 */

#ifndef PDAAAL_POSTSTAR_H
#define PDAAAL_POSTSTAR_H

#include "PAutomaton.h"


namespace pdaaal {

    template <typename W = void, typename C = std::less<W>, typename A = add<W>>
    class PostStar {
    private:
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

        using weight_edge_pair = std::pair<W, temp_edge_t>;
        struct weight_edge_pair_comp{
            bool operator()(const weight_edge_pair &lhs, const weight_edge_pair &rhs){
                C less;
                return less(rhs.first, lhs.first); // Used in a max-heap, so swap arguments to make it a min-heap.
            }
        };
        struct weight_edge_trace {
            W weight;
            temp_edge_t edge;
            const trace_t *trace = nullptr;
            weight_edge_trace(W weight, temp_edge_t edge, const trace_t *trace) : weight(weight), edge(edge), trace(trace) {};
            weight_edge_trace() = default;
        };
        struct weight_edge_trace_comp{
            bool operator()(const weight_edge_trace &lhs, const weight_edge_trace &rhs){
                C less;
                return less(rhs.weight, lhs.weight); // Used in a max-heap, so swap arguments to make it a min-heap.
            }
        };

    public:
        void pre_star(PAutomaton<W,C,A> &automaton) {
            // This is an implementation of Algorithm 1 (figure 3.3) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 42)
            auto& pda_states = pda().states();
            const size_t n_pda_states = pda_states.size();

            std::unordered_set<temp_edge_t, temp_edge_hasher> edges;
            std::stack<temp_edge_t> workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel(_states.size());

            auto insert_edge = [&edges, &workset, this](size_t from, uint32_t label, size_t to, const trace_t *trace) {
                auto res = edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U workset).
                    workset.emplace(from, label, to);
                    if (trace != nullptr) { // Don't add existing edges
                        this->add_edge(from, to, label, trace);
                    }
                }
            };
            const size_t n_pda_labels = this->number_of_labels();
            auto insert_edge_bulk = [&insert_edge, n_pda_labels](size_t from, const labels_t &precondition, size_t to, const trace_t *trace) {
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
            for (auto &from : this->states()) {
                for (auto &edge : from->_edges) {
                    for (auto &label : edge._labels) {
                        insert_edge(from->_id, label._label, edge._to->_id, nullptr);
                    }
                }
            }

            // for all <p, y> --> <p', epsilon> : workset U= (p, y, p') (line 2)
            for (size_t state = 0; state < n_pda_states; ++state) {
                const auto &rules = pda_states[state]._rules;
                for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                    auto &rule = rules[rule_id];
                    if (rule._operation == POP) {
                        insert_edge_bulk(state, rule._labels, rule._to, this->new_pre_trace(rule_id));
                    }
                }
            }

            // delta_prime[q] = [p,rule_id]    where states[p]._rules[rule_id]._labels.contains(y)    for each p, rule_id
            // corresponds to <p, y> --> <q, y>   (the y is the same, since we only have PUSH and not arbitrary <p, y> --> <q, y1 y2>, i.e. y==y2)
            std::vector<std::vector<std::pair<size_t, size_t>>> delta_prime(states().size());

            while (!workset.empty()) { // (line 3)
                // pop t = (q, y, q') from workset (line 4)
                auto t = workset.top();
                workset.pop();
                // rel = rel U {t} (line 6)   (membership test on line 5 is done in insert_edge).
                rel[t._from].emplace_back(t._to, t._label);

                // (line 7-8 for \Delta')
                for (auto pair : delta_prime[t._from]) { // Loop over delta_prime (that match with t->from)
                    auto state = pair.first;
                    auto rule_id = pair.second;
                    if (pda_states[state]._rules[rule_id]._labels.contains(t._label)) {
                        insert_edge(state, t._label, t._to, this->new_pre_trace(rule_id, t._from));
                    }
                }
                // Loop over \Delta (filter rules going into q) (line 7 and 9)
                if (t._from >= n_pda_states) { continue; }
                for (auto pre_state : pda_states[t._from]._pre_states) {
                    const auto &rules = pda_states[pre_state]._rules;
                    rule_t<W,C> dummy_rule{t._from, PUSH, 0}; // PUSH and 0 are the smallest w.r.t. PDA::rule_t::operator<
                    auto lb = std::lower_bound(rules.begin(), rules.end(), dummy_rule);
                    while (lb != rules.end() && lb->_to == t._from) {
                        auto &rule = *lb;
                        size_t rule_id = lb - rules.begin();
                        ++lb;
                        switch (rule._operation) {
                            case POP:
                                break;
                            case SWAP: // (line 7-8 for \Delta)
                                if (rule._op_label == t._label) {
                                    insert_edge_bulk(pre_state, rule._labels, t._to, this->new_pre_trace(rule_id));
                                }
                                break;
                            case NOOP: // (line 7-8 for \Delta)
                                if (rule._labels.contains(t._label)) {
                                    insert_edge(pre_state, t._label, t._to, this->new_pre_trace(rule_id));
                                }
                                break;
                            case PUSH: // (line 9)
                                if (rule._op_label == t._label) {
                                    // (line 10)
                                    delta_prime[t._to].emplace_back(pre_state, rule_id);
                                    for (auto rel_rule : rel[t._to]) { // (line 11-12)
                                        if (rule._labels.contains(rel_rule.second)) {
                                            insert_edge(pre_state, rel_rule.second, rel_rule.first, this->new_pre_trace(rule_id, t._to));
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
        }

        template <Trace_Type trace_type = Trace_Type::Any>
        void post_star(PAutomaton<W,C,A> &automaton) {
            if constexpr (is_weighted<W> && trace_type == Trace_Type::Shortest) {
                post_star_shortest<true>(automaton);
            } else if constexpr (trace_type == Trace_Type::Any) {
                post_star_any(automaton);
            } else if constexpr (trace_type == Trace_Type::None) {
                post_star_any(automaton); // TODO: Implement faster no-trace option.
            }
        }
    private:
        template<bool E, typename = std::enable_if_t<E>>
        void post_star_shortest(PAutomaton<W,C,A> &automaton) {
            //static_assert(is_weighted<W> || trace_type != Trace_Type::Shortest, "Cannot find shortest trace for un-weighted PDA."); // TODO: Well, weight==1 would be a good default.
            //constexpr bool shortest_trace = is_weighted<W> && trace_type == Trace_Type::Shortest;
            static_assert(shortest_trace);
            //static_assert(std::is_same_v<W, decltype(std::declval<W>() + std::declval<W>())>, "The weight type must implement operator+");
            const A add;
            const C less;

            // This is an implementation of Algorithm 2 (figure 3.4) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 48)
            auto & pda_states = pda().states();
            auto n_pda_states = pda_states.size();
            auto n_Q = _states.size();

            // for <p, y> -> <p', y1 y2> do
            //   Q' U= {q_p'y1}
            std::unordered_map<std::pair<size_t, uint32_t>, size_t, boost::hash<std::pair<size_t, uint32_t>>> q_prime{};
            for (auto &state : pda_states) {
                for (auto &rule : state._rules) {
                    if (rule._operation == PUSH) {
                        auto res = q_prime.emplace(std::make_pair(rule._to, rule._op_label), this->next_state_id());
                        if (res.second) {
                            this->add_state(false, false);
                        }
                    }
                }
            }
            auto n_automata_states = _states.size();
            std::vector<W> minpath(n_automata_states - n_pda_states);
            for (size_t i = 0; i < minpath.size(); ++i) {
                minpath[i] = i < n_Q - n_pda_states ? pdaaal::zero<W>()() : pdaaal::max<W>()();
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

            auto update_edge = [&edge_weights](size_t from, uint32_t label, size_t to, W edge_weight, W workset_weight) -> std::pair<bool,bool> {
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
            auto get_weight = [&edge_weights](size_t from, uint32_t label, size_t to) -> W {
                return (*edge_weights.find(temp_edge_t{from, label, to})).second.first;
            };
            // Adds to rel and inserts into the PAutomaton including the trace.
            auto insert_rel = [&rel1, &rel2, this, n_Q](size_t from, uint32_t label, size_t to){
                rel1[from].emplace_back(to, label);
                if (label == epsilon && to >= n_Q) {
                    rel2[to - n_Q].push_back(from);
                }
            };

            // workset := ->_0 intersect (P x Gamma x Q)
            // rel := ->_0 \ workset
            for (auto &from : this->states()) {
                for (auto &edge : from->_edges) {
                    assert(!edge.has_epsilon()); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (auto &label : edge._labels) {
                        temp_edge_t temp_edge{from->_id, label._label, edge._to->_id};
                        edge_weights.emplace(temp_edge, std::make_pair(zero<W>()(), zero<W>()()));
                        if (from->_id < n_pda_states) {
                            workset.emplace(zero<W>()(), temp_edge, nullptr);
                        } else {
                            insert_rel(from->_id, label._label, edge._to->_id);
                        }
                    }
                }
            }

            while (!workset.empty()) {
                // pop t = (q, y, q') from workset
                auto elem = workset.top();
                workset.pop();
                auto t = elem.edge;
                auto weights = (*edge_weights.find(t)).second;
                if (less(weights.second, elem.weight)) {
                    continue; // Same edge with a smaller weight was already processed.
                }
                auto b = weights.first;

                // rel = rel U {t}
                insert_rel(t._from, t._label, t._to);
                if (t._label == epsilon) {
                    this->add_epsilon_edge(t._from, t._to, elem.trace, b);
                } else {
                    this->add_edge(t._from, t._to, t._label, elem.trace, b);
                }

                // if y != epsilon
                if (t._label != epsilon) {
                    const auto &rules = pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        auto &rule = rules[rule_id];
                        if (!rule._labels.contains(t._label)) { continue; }
                        auto trace = this->new_post_trace(t._from, rule_id, t._label);
                        auto wd = add(elem.weight, rule._weight);
                        auto wb = add(b, rule._weight);
                        if (rule._operation != PUSH) {
                            uint32_t label;
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
                                    assert(false); // Does not happen!
                            }
                            if (update_edge(rule._to, label, t._to, wb, wd).second) {
                                workset.emplace(wd, temp_edge_t{rule._to, label, t._to}, trace);
                            }
                        } else { // rule._operation == PUSH
                            assert(q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(q_prime));
                            size_t q_new = q_prime[std::make_pair(rule._to, rule._op_label)];
                            auto update_t = update_edge(rule._to, rule._op_label, q_new, zero<W>()(), wd).second;
                            auto was_updated = update_edge(q_new, t._label, t._to, wb, zero<W>()()).first;
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
                            if (less(wd, minpath[q_new - n_pda_states])) {
                                minpath[q_new - n_pda_states] = wd;
                                if (update_t) {
                                    workset.emplace(wd, temp_edge_t{rule._to, rule._op_label, q_new}, trace);
                                }
                            } else if (was_updated) {
                                for (auto f : rel2[q_new - n_Q]) {
                                    auto wt = add(get_weight(f, epsilon, q_new), wb);
                                    auto dt = add(minpath[t._to - n_pda_states], wt);
                                    if (update_edge(f, t._label, t._to, wt, dt).second) {
                                        workset.emplace(dt, temp_edge_t{f, t._label, t._to}, this->new_post_trace(q_new));
                                    }
                                }
                            }
                        }
                    }
                } else {
                    if (t._to < n_Q) {
                        for (auto e : rel1[t._to]) {
                            assert(e.first >= n_pda_states);
                            auto w = add(get_weight(t._to, e.second, e.first), get_weight(t._from, t._label, t._to));
                            auto dd = add(minpath[e.first - n_pda_states], w);
                            if(update_edge(t._from, e.second, e.first, w, dd).second) {
                                workset.emplace(dd, temp_edge_t{t._from, e.second, e.first}, this->new_post_trace(t._to));
                            }
                        }
                    } else {
                        for (auto &e : rel3[t._to - n_Q]) {
                            auto w = add(get_weight(t._to, e._label, e._to), get_weight(t._from, t._label, t._to));
                            auto dd = add(minpath[e._to - n_pda_states], w);
                            if(update_edge(t._from, e._label, e._to, w, dd).second) {
                                workset.emplace(dd, temp_edge_t{t._from, e._label, e._to}, this->new_post_trace(t._to));
                            }
                        }
                    }
                }
            }

            for (size_t i = n_Q; i < n_automata_states; ++i) {
                for (auto &e : rel3[i - n_Q]) {
                    assert(e._label != epsilon);
                    this->add_edge(i, e._to, e._label, e._trace, e._weight);
                }
            }
        }


        void post_star_any(PAutomaton<W,C,A> &automaton) {
            // This is an implementation of Algorithm 2 (figure 3.4) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 48)
            auto & pda_states = pda().states();
            auto n_pda_states = pda_states.size();

            // for <p, y> -> <p', y1 y2> do  (line 3)
            //   Q' U= {q_p'y1}              (line 4)
            std::unordered_map<std::pair<size_t, uint32_t>, size_t, boost::hash<std::pair<size_t, uint32_t>>> q_prime{};
            for (auto &state : pda_states) {
                for (auto &rule : state._rules) {
                    if (rule._operation == PUSH) {
                        auto res = q_prime.emplace(std::make_pair(rule._to, rule._op_label), this->next_state_id());
                        if (res.second) {
                            this->add_state(false, false);
                        }
                    }
                }
            }

            std::unordered_set<temp_edge_t, temp_edge_hasher> edges;
            using workset_type = std::conditional_t<shortest_trace,
            std::priority_queue<weight_edge_pair, std::vector<weight_edge_pair>, weight_edge_pair_comp>,
            std::stack<temp_edge_t>>;
            using workset_elem_t = typename workset_type::value_type;
            workset_type workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel1(_states.size()); // faster access for lookup _from -> (_to, _label)
            std::vector<std::vector<size_t>> rel2(_states.size()); // faster access for lookup _to -> _from  (when _label is uint32_t::max)

            //using insert_edge_type = std::conditional_t<!shortest_trace,
            //    std::function<void(workset_elem_t, const trace_t *, bool)>,
            //      std::function<void(size_t, uint32_t, size_t, W, const trace_t *, bool)>>;
            //std::conditional_t<trace_type == Trace_Type::None,
            //std::function<void(size_t from, uint32_t label, size_t to, const trace_t *trace, bool direct_to_rel)>, // TODO: Implement no-trace version.
            //std::function<void(size_t from, uint32_t label, size_t to, const trace_t *trace, bool direct_to_rel)>>>;
            std::function<void(workset_elem_t&&, const trace_t *, bool)> insert_edge = [&edges, &workset, &rel1, &rel2, this](workset_elem_t&& edge, const trace_t *trace, bool direct_to_rel = false) {
                size_t from, to; uint32_t label;
                if constexpr (shortest_trace) {
                    from = edge.second._from;
                    label = edge.second._label;
                    to = edge.second._to;
                } else {
                    from = edge._from;
                    label = edge._label;
                    to = edge._to;
                }
                auto res = edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U workset).
                    if (direct_to_rel) {
                        rel1[from].emplace_back(to, label);
                        if (label == epsilon) {
                            rel2[to].push_back(from);
                        }
                    } else {
                        workset.push(std::move(edge));
                    }
                    if (trace != nullptr) { // Don't add existing edges
                        if (label == epsilon) {
                            this->add_epsilon_edge(from, to, trace);
                        } else {
                            this->add_edge(from, to, label, trace);
                        }
                    }
                }
            };

            // workset := ->_0 intersect (P x Gamma x Q)  (line 1)
            // rel := ->_0 \ workset (line 2)
            for (auto &from : this->states()) {
                for (auto &edge : from->_edges) {
                    assert(!edge.has_epsilon()); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (auto &label : edge._labels) {
                        temp_edge_t e{from->_id, label._label, edge._to->_id};
                        if constexpr (shortest_trace) {
                            W w; // TODO: Implement generic zero-weight.
                            insert_edge(std::make_pair(w, std::move(e)), nullptr, from->_id >= n_pda_states);
                        } else {
                            insert_edge(std::move(e), nullptr, from->_id >= n_pda_states);
                        }
                    }
                }
            }

            while (!workset.empty()) { // (line 5)
                // pop t = (q, y, q') from workset (line 6)
                temp_edge_t t;
                if constexpr (shortest_trace) {
                    bool already_processed;
                    do {
                        t = workset.top().second;
                        workset.pop();
                        already_processed = false; // TODO: Make rel membership test efficient!
                        for (auto e : rel1[t._from]) { // Check if t has already been processed. Necessary if it at some point was added with lower weight.
                            if (t._to == e.first && t._label == e.second) {
                                already_processed = true;
                                break;
                            }
                        }
                    } while (already_processed);
                } else {
                    t = workset.top();
                    workset.pop();
                }
                // rel = rel U {t} (line 8)   (membership test on line 7 is done in insert_edge).
                rel1[t._from].emplace_back(t._to, t._label);
                if (t._label == epsilon) {
                    rel2[t._to].push_back(t._from);
                }

                // if y != epsilon (line 9)
                if (t._label != epsilon) {
                    const auto &rules = pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        auto &rule = rules[rule_id];
                        if (!rule._labels.contains(t._label)) { continue; }
                        auto trace = this->new_post_trace(t._from, rule_id, t._label);
                        switch (rule._operation) {
                            case POP: // (line 10-11)
                                insert_edge(temp_edge_t{rule._to, epsilon, t._to}, trace, false);
                                break;
                            case SWAP: // (line 12-13)
                                insert_edge(temp_edge_t{rule._to, rule._op_label, t._to}, trace, false);
                                break;
                            case NOOP:
                                insert_edge(temp_edge_t{rule._to, t._label, t._to}, trace, false);
                                break;
                            case PUSH: // (line 14)
                                assert(q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(q_prime));
                                size_t q_new = q_prime[std::make_pair(rule._to, rule._op_label)];
                                insert_edge(temp_edge_t{rule._to, rule._op_label, q_new}, trace, false); // (line 15)
                                insert_edge(temp_edge_t{q_new, t._label, t._to}, trace, true); // (line 16)
                                for (auto f : rel2[q_new]) { // (line 17)
                                    insert_edge(temp_edge_t{f, t._label, t._to}, this->new_post_trace(q_new), false); // (line 18)
                                }
                                break;
                        }
                    }
                } else {
                    for (auto e : rel1[t._to]) { // (line 20)
                        insert_edge(temp_edge_t{t._from, e.second, e.first}, this->new_post_trace(t._to), false); // (line 21)
                    }
                }
            }
        }





    };








}

#endif //PDAAAL_POSTSTAR_H
