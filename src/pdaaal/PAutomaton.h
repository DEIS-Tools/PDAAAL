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

#include "WPDA.h"

#include <memory>
#include <functional>
#include <vector>
#include <stack>
#include <queue>
#include <iostream>
#include <cassert>
#include <boost/functional/hash.hpp>


namespace pdaaal {

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

    template<typename W, typename C, typename = void> struct label_with_trace_t;
    template<typename W, typename C>
    struct label_with_trace_t<W, C, std::enable_if_t<!is_weighted<W>>> {
        uint32_t _label = std::numeric_limits<uint32_t>::max();
        const trace_t *_trace = nullptr;

        explicit label_with_trace_t(uint32_t label)
                : _label(label) {};

        explicit label_with_trace_t(const trace_t *trace) // epsilon edge
                : _trace(trace) {};

        label_with_trace_t(uint32_t label, const trace_t *trace)
                : _label(label), _trace(trace) {};

        bool operator<(const label_with_trace_t &other) const {
            return _label < other._label;
        }

        bool operator==(const label_with_trace_t &other) const {
            return _label == other._label;
        }

        bool operator!=(const label_with_trace_t &other) const {
            return !(*this == other);
        }

        [[nodiscard]] bool is_epsilon() const {
            return _label == std::numeric_limits<uint32_t>::max();
        }
    };
    template<typename W, typename C>
    struct label_with_trace_t<W, C, std::enable_if_t<is_weighted<W>>> {
        uint32_t _label = std::numeric_limits<uint32_t>::max();
        const trace_t *_trace = nullptr;
        W _weight;

        explicit label_with_trace_t(uint32_t label)
                : _label(label) {};

        explicit label_with_trace_t(const trace_t *trace) // epsilon edge
                : _trace(trace) {}; // TODO: Probably remove...
        label_with_trace_t(const trace_t *trace, W weight) // epsilon edge
                : _trace(trace), _weight(weight) {};

        label_with_trace_t(uint32_t label, const trace_t *trace)
                : _label(label), _trace(trace) {}; // TODO: Probably remove...

        label_with_trace_t(uint32_t label, const trace_t *trace, W weight)
                : _label(label), _trace(trace), _weight(weight) {};

        bool operator<(const label_with_trace_t &other) const {
            return _label < other._label;
        }

        bool operator==(const label_with_trace_t &other) const {
            return _label == other._label;
        }

        bool operator!=(const label_with_trace_t &other) const {
            return !(*this == other);
        }

        [[nodiscard]] bool is_epsilon() const {
            return _label == std::numeric_limits<uint32_t>::max();
        }

        void update(const trace_t * trace, W weight) {
            C less;
            if (less(weight, _weight)) {
                _trace = trace;
                _weight = weight;
            }
        }

    };

    enum class Trace_Type {
        None,
        Any,
        Shortest
    };

    template <typename W = void, typename C = std::less<W>>
    class PAutomaton {
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
        struct state_t;

        struct edge_t {
            state_t *_to;
            std::vector<label_with_trace_t<W,C>> _labels;

            // edge with a label and optional trace
            edge_t(state_t *to, uint32_t label, W weight, const trace_t *trace = nullptr)
                    : _to(to), _labels() {
                _labels.emplace_back(label, trace, weight);
            };

            // epsilon edge with trace
            edge_t(state_t *to, W weight, const trace_t *trace) : _to(to), _labels() {
                _labels.emplace_back(trace, weight);
            };

            // wildcard (all labels), no trace
            edge_t(state_t *to, size_t all_labels) : _to(to), _labels() {
                for (uint32_t label = 0; label < all_labels; label++) {
                    _labels.emplace_back(label);
                }
            };

            void add_label(uint32_t label, const trace_t *trace, W weight) {
                label_with_trace_t<W,C> label_trace{label, trace, weight};
                auto lb = std::lower_bound(_labels.begin(), _labels.end(), label_trace);
                if (lb == std::end(_labels) || *lb != label_trace) {
                    _labels.insert(lb, label_trace);
                }
            }
            void update_label(uint32_t label, const trace_t *trace, W weight) {
                label_with_trace_t<W,C> label_trace{label, trace, weight};
                auto lb = std::lower_bound(_labels.begin(), _labels.end(), label_trace);
                if (lb == std::end(_labels) || *lb != label_trace) {
                    _labels.insert(lb, label_trace);
                } else {
                    lb->update(trace, weight);
                }
            }

            std::optional<label_with_trace_t<W,C>> find(uint32_t label) {
                label_with_trace_t<W,C> label_trace{label};
                auto lb = std::lower_bound(_labels.begin(), _labels.end(), label_trace);
                if (lb != std::end(_labels) && *lb == label_trace) {
                    return std::optional<label_with_trace_t<W,C>>(*lb);
                }
                return std::nullopt;
            }

            bool contains(uint32_t label) {
                label_with_trace_t<W,C> label_trace{label};
                auto lb = std::lower_bound(_labels.begin(), _labels.end(), label_trace);
                return lb != std::end(_labels) && *lb == label_trace;
            }

            [[nodiscard]] bool has_epsilon() const { return !_labels.empty() && _labels.back().is_epsilon(); }
            [[nodiscard]] bool has_non_epsilon() const { return !_labels.empty() && !_labels[0].is_epsilon(); }
        };

        struct state_t {
            bool _accepting = false;
            size_t _id;
            std::vector<edge_t> _edges;

            state_t(bool accepting, size_t id) : _accepting(accepting), _id(id) {};

            state_t(const state_t &other) = default;
        };

    public:
        // Accept one control state with given stack.
        PAutomaton(const WPDA<W,C> &pda, size_t initial_state, const std::vector<uint32_t> &initial_stack) : _pda(pda) {
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

        PAutomaton(PAutomaton &&) noexcept = default;

        PAutomaton(const PAutomaton &other) : _pda(other._pda) {
            std::unordered_map<state_t *, state_t *> indir;
            for (auto &s : other._states) {
                _states.emplace_back(std::make_unique<state_t>(*s));
                indir[s.get()] = _states.back().get();
            }
            // fix links
            for (auto &s : _states) {
                for (auto &e : s->_edges) {
                    e._to = indir[e._to];
                }
            }
            for (auto &s : other._accepting) {
                _accepting.push_back(indir[s]);
            }
            for (auto &s : other._initial) {
                _initial.push_back(indir[s]);
            }
        }

        void pre_star() {
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

        template <Trace_Type trace_type = Trace_Type::Shortest, typename adder = add<W>> // TODO: Move adder parameter to higher level.
        void post_star_weighted() {
            static_assert(is_weighted<W> || trace_type != Trace_Type::Shortest, "Cannot find shortest trace for un-weighted PDA."); // TODO: Well, weight==1 would be a good default.
            constexpr bool shortest_trace = is_weighted<W> && trace_type == Trace_Type::Shortest;
            static_assert(shortest_trace);
            //static_assert(std::is_same_v<W, decltype(std::declval<W>() + std::declval<W>())>, "The weight type must implement operator+");
            adder add;

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
                C less;
                if (less((*edge_weights.find(elem.edge)).second.second, elem.weight)) {
                    continue; // Same edge with a smaller weight was already processed.
                }
                auto t = elem.edge;

                // rel = rel U {t}
                insert_rel(t._from, t._label, t._to);
                if (t._label == epsilon) {
                    this->add_epsilon_edge(t._from, t._to, elem.trace, elem.weight);
                } else {
                    this->add_edge(t._from, t._to, t._label, elem.trace, elem.weight);
                }

                // if y != epsilon
                if (t._label != epsilon) {
                    const auto &rules = pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        auto &rule = rules[rule_id];
                        if (!rule._labels.contains(t._label)) { continue; }
                        auto trace = this->new_post_trace(t._from, rule_id, t._label);
                        auto wd = add(elem.weight, rule._weight);
                        auto wb = add(get_weight(t._from, t._label, t._to), rule._weight);
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
                                auto relq = rel3[q_new - n_Q];
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
                    this->add_edge(i, e._label, e._to, e._trace, e._weight);
                }
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any>
        void post_star() {
            static_assert(is_weighted<W> ||  trace_type != Trace_Type::Shortest, "Cannot find shortest trace for un-weighted PDA."); // TODO: Well, weight==1 would be a good default.
            constexpr bool shortest_trace = is_weighted<W> && trace_type == Trace_Type::Shortest;

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


        [[nodiscard]] const std::vector<std::unique_ptr<state_t>> &states() const { return _states; }
        
        [[nodiscard]] const WPDA<W,C> &pda() const { return _pda; }

        void to_dot(std::ostream &out, const std::function<void(std::ostream &, const label_with_trace_t<W,C> &)> &printer = [](auto &s, auto &e) {
                        s << e._label;
                    }) const {
            out << "digraph NFA {\n";
            for (auto &s : _states) {
                out << "\"" << s->_id << "\" [shape=";
                if (s->_accepting)
                    out << "double";
                out << "circle];\n";
                for (const edge_t &e : s->_edges) {
                    out << "\"" << s->_id << "\" -> \"" << e._to->_id << "\" [ label=\"";
                    if (e.has_non_epsilon()) {
                        out << "\\[";
                        bool first = true;
                        for (auto &l : e._labels) {
                            if (l.is_epsilon()) { continue; }
                            if (!first)
                                out << ", ";
                            first = false;
                            printer(out, l);
                        }
                        out << "\\]";
                    }
                    if (e._labels.size() == number_of_labels())
                        out << "*";
                    if (e.has_epsilon()) {
                        if (!e._labels.empty()) out << " ";
                        out << u8"𝜀";
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
                for (auto &edge : _states[current_state]->_edges) {
                    if (edge.contains(stack[stack_index])) {
                        auto to = edge._to->_id;
                        if (stack_index + 1 < stack.size()) {
                            search_stack.emplace(to, stack_index + 1);
                        } else if (edge._to->_accepting) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }
        [[nodiscard]] std::vector<size_t> accept_path(size_t state, const std::vector<uint32_t> &stack) const {
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
                for (auto &edge : _states[current_state]->_edges) {
                    if (edge.contains(stack[stack_index])) {
                        auto to = edge._to->_id;
                        if (stack_index + 1 < stack.size()) {
                            search_stack.emplace(to, stack_index + 1);
                        } else if (edge._to->_accepting) {
                            path[stack_index + 1] = to;
                            return path;
                        }
                    }
                }
            }
            return std::vector<size_t>();
        }

        [[nodiscard]] std::pair<std::vector<size_t>, W> shortest_accept_path(size_t state, const std::vector<uint32_t> &stack) const {
            if (stack.empty()) {
                if (_states[state]->_accepting) {
                    return std::make_pair(std::vector<size_t>{state}, zero<W>()());
                } else {
                    return std::make_pair(std::vector<size_t>(), max<W>()());
                }
            }
            // Dijkstra.
            struct queue_elem {
                W weight;
                size_t state;
                size_t stack_index;
                const queue_elem *back_pointer;
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
                    C less;
                    return less(rhs.weight, lhs.weight); // Used in a max-heap, so swap arguments to make it a min-heap.
                }
            };
            add<W> add;
            std::priority_queue<queue_elem, std::vector<queue_elem>, queue_elem_comp> search_queue;
            std::vector<queue_elem> visited;
            std::vector<std::unique_ptr<queue_elem>> pointers;
            search_queue.emplace(zero<W>()(), state, 0, nullptr);
            while(!search_queue.empty()) {
                auto current = search_queue.top();
                search_queue.pop();
                auto lb = std::lower_bound(visited.begin(), visited.end(), current);
                if (lb != std::end(visited) && *lb == current) {
                    if (queue_elem_comp(*lb, current)) {
                        *lb = current;
                    } else {
                        break;
                    }
                } else {
                    lb = visited.insert(lb, current);
                }
                auto u_pointer = std::make_unique(*lb);
                auto pointer = u_pointer.get();
                pointers.push_back(std::move(u_pointer));
                for (auto &edge : _states[current.state]->_edges) {
                    auto label = edge.find(stack[current.stack_index]);
                    if (label) {
                        auto to = edge._to->_id;
                        if (current.stack_index + 1 < stack.size()) {
                            search_queue.emplace(add(current.priority, label->_weight), to, current.stack_index + 1, pointer);
                        } else if (edge._to->_accepting) {
                            std::vector<size_t> path(stack.size() + 1);
                            path[current.stack_index + 1] = to;
                            for (auto p = current.back_pointer; p != nullptr; p = p->back_pointer) {
                                path[p->stack_index] = p->state;
                            }
                            return std::make_pair(path, current.weight);
                        }
                    }
                }
            }
            return std::make_pair(std::vector<size_t>(), max<W>()());
        }

        [[nodiscard]] const trace_t *get_trace_label(const std::tuple<size_t, uint32_t, size_t> &edge) const {
            return get_trace_label(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
        }
        [[nodiscard]] const trace_t *get_trace_label(size_t from, uint32_t label, size_t to) const {
            for (auto &e : _states[from]->_edges) {
                if (e._to->_id == to) {
                    label_with_trace_t<W,C> label_trace{label};
                    auto lb = std::lower_bound(e._labels.begin(), e._labels.end(), label_trace);
                    assert(lb != std::end(e._labels)); // We assume the edge exists.
                    return lb->_trace;
                }
            }
            assert(false); // We assume the edge exists.
            return nullptr;
        }

        // TODO: Implement early termination versions.
        // bool _pre_star_accepts(size_t state, const std::vector<uint32_t> &stack);
        // bool _post_star_accepts(size_t state, const std::vector<uint32_t> &stack);

    protected:
        [[nodiscard]] size_t number_of_labels() const { return _pda.number_of_labels(); }

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

        void add_epsilon_edge(size_t from, size_t to, const trace_t *trace, W weight = zero<W>()()) {
            auto &edges = _states[from]->_edges;
            for (auto &e : edges) {
                if (e._to->_id == to) {
                    if (!e._labels.back().is_epsilon()) {
                        e._labels.emplace_back(trace, weight);
                    }
                    return;
                }
            }
            edges.emplace_back(_states[to].get(), weight, trace);
        }

        void add_edge(size_t from, size_t to, uint32_t label, const trace_t *trace = nullptr, W weight = zero<W>()()) {
            assert(label < std::numeric_limits<uint32_t>::max() - 1);
            auto &edges = _states[from]->_edges;
            for (auto &e : edges) {
                if (e._to->_id == to) {
                    e.add_label(label, trace, weight);
                    return;
                }
            }
            edges.emplace_back(_states[to].get(), label, weight, trace);
        }

        void update_edge(size_t from, size_t to, uint32_t label, const trace_t *trace, W weight) {
            assert(label < std::numeric_limits<uint32_t>::max() - 1);
            auto &edges = _states[from]->_edges;
            for (auto &e : edges) {
                if (e._to->_id == to) {
                    e.update_label(label, trace, weight);
                    return;
                }
            }
            edges.emplace_back(_states[to].get(), label, trace);
        }

        void add_wildcard(size_t from, size_t to) {
            auto &edges = _states[from]->_edges;
            for (auto &e : edges) {
                if (e._to->_id == to) {
                    e._labels.clear();
                    for (uint32_t i = 0; i < number_of_labels(); i++) {
                        e._labels.emplace_back(i);
                    }
                    return;
                }
            }
            edges.emplace_back(_states[to].get(), number_of_labels());
        }

    private:
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

        std::vector<std::unique_ptr<state_t>> _states;
        std::vector<state_t *> _initial;
        std::vector<state_t *> _accepting;

        std::vector<std::unique_ptr<trace_t>> _trace_info;

        const WPDA<W,C> &_pda;
    };


}

#endif //PDAAAL_PAUTOMATON_H
