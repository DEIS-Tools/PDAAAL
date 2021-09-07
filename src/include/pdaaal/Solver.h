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

#include <pdaaal/utils/workset.h>
#include <pdaaal/AutomatonPath.h>
#include <pdaaal/PAutomaton.h>
#include <pdaaal/TypedPDA.h>
#include <pdaaal/SolverInstance.h>
#include <absl/hash/hash.h>

namespace pdaaal {

    namespace details {
        constexpr auto epsilon = std::numeric_limits<uint32_t>::max();

        struct temp_edge_t {
            size_t _from = std::numeric_limits<size_t>::max();
            size_t _to = std::numeric_limits<size_t>::max();
            uint32_t _label = std::numeric_limits<uint32_t>::max();

            temp_edge_t() = default;

            temp_edge_t(size_t from, uint32_t label, size_t to)
                    : _from(from), _to(to), _label(label) {};

            bool operator<(const temp_edge_t& other) const {
                return std::tie(_from, _label, _to) < std::tie(other._from, other._label, other._to);
            }
            bool operator==(const temp_edge_t& other) const {
                return _from == other._from && _to == other._to && _label == other._label;
            }
            bool operator!=(const temp_edge_t& other) const { return !(*this == other); }

            template <typename H>
            friend H AbslHashValue(H h, const temp_edge_t& e) {
                return H::combine(std::move(h), e._from, e._to, e._label);
            }
        };

        template <typename W>
        using early_termination_fn = std::function<bool(size_t,uint32_t,size_t,edge_annotation_t<W>)>;

        template <typename W, bool ET=false>
        class PreStarSaturation {
            using trace_info = TraceInfo<TraceInfoType::Single>;
            using edge_anno = edge_annotation<W,TraceInfoType::Single>;
            using edge_anno_t = edge_annotation_t<W,TraceInfoType::Single>;
            using p_automaton_t = PAutomaton<W>;
        public:
            explicit PreStarSaturation(PAutomaton<W> &automaton, const early_termination_fn<W>& early_termination = [](size_t f, uint32_t l, size_t t, edge_anno_t trace) -> bool { return false; })
                    : _automaton(automaton), _early_termination(early_termination), _pda_states(_automaton.pda().states()),
                      _n_pda_states(_pda_states.size()), _n_automaton_states(_automaton.states().size()),
                      _n_pda_labels(_automaton.number_of_labels()), _rel(_n_automaton_states), _delta_prime(_n_automaton_states) {
                initialize();
            };

        private:
            // This is an implementation of Algorithm 1 (figure 3.3) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 42)

            p_automaton_t& _automaton;
            const early_termination_fn<W>& _early_termination;
            const std::vector<typename PDA<W>::state_t>& _pda_states;
            const size_t _n_pda_states;
            const size_t _n_automaton_states;
            const size_t _n_pda_labels;
            std::unordered_set<temp_edge_t, absl::Hash<temp_edge_t>> _edges;
            std::stack<temp_edge_t> _workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel;
            std::vector<std::vector<std::pair<size_t, size_t>>> _delta_prime;
            bool _found = false;

            void initialize() {
                // workset := ->_0  (line 1)
                for (const auto &from : _automaton.states()) {
                    for (const auto &[to,labels] : from->_edges) {
                        for (const auto &[label,_] : labels) {
                            insert_edge(from->_id, label, to, trace_info::make_default());
                        }
                    }
                }

                // for all <p, y> --> <p', epsilon> : workset U= (p, y, p') (line 2)
                for (size_t state = 0; state < _n_pda_states; ++state) {
                    size_t rule_id = 0;
                    for (const auto&[rule,labels] : _pda_states[state]._rules) {
                        if (rule._operation == POP) {
                            insert_edge_bulk(state, labels, rule._to, p_automaton_t::new_pre_trace(rule_id));
                        }
                        ++rule_id;
                    }
                }
            }
            void insert_edge(size_t from, uint32_t label, size_t to, trace_t trace) {
                auto res = _edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U workset).
                    _workset.emplace(from, label, to);
                    if (!trace.is_null()) { // Don't add existing edges
                        if constexpr (ET) {
                            _found = _found || _early_termination(from, label, to, edge_anno::from_trace_info(trace));
                        }
                        _automaton.add_edge(from, to, label, edge_anno::from_trace_info(trace));
                    }
                }
            };
            void insert_edge_bulk(size_t from, const labels_t& precondition, size_t to, trace_t trace) {
                if (precondition.wildcard()) {
                    for (uint32_t i = 0; i < _n_pda_labels; i++) {
                        insert_edge(from, i, to, trace);
                    }
                } else {
                    for (auto label : precondition.labels()) {
                        insert_edge(from, label, to, trace);
                    }
                }
            };

        public:
            void step() {
                // pop t = (q, y, q') from workset (line 4)
                auto t = _workset.top();
                _workset.pop();
                // rel = rel U {t} (line 6)   (membership test on line 5 is done in insert_edge).
                _rel[t._from].emplace_back(t._to, t._label);

                // (line 7-8 for \Delta')
                for (auto pair : _delta_prime[t._from]) { // Loop over delta_prime (that match with t->from)
                    auto state = pair.first;
                    auto rule_id = pair.second;
                    if (_pda_states[state]._rules[rule_id].second.contains(t._label)) {
                        insert_edge(state, t._label, t._to, p_automaton_t::new_pre_trace(rule_id, t._from));
                    }
                }
                // Loop over \Delta (filter rules going into q) (line 7 and 9)
                if (t._from >= _n_pda_states) { return; }
                for (auto pre_state : _pda_states[t._from]._pre_states) {
                    const auto &rules = _pda_states[pre_state]._rules;
                    auto lb = rules.lower_bound(details::rule_t<W>{t._from});
                    while (lb != rules.end() && lb->first._to == t._from) {
                        const auto &[rule, labels] = *lb;
                        size_t rule_id = lb - rules.begin();
                        ++lb;
                        switch (rule._operation) {
                            case POP:
                                break;
                            case SWAP: // (line 7-8 for \Delta)
                                if (rule._op_label == t._label) {
                                    insert_edge_bulk(pre_state, labels, t._to, p_automaton_t::new_pre_trace(rule_id));
                                }
                                break;
                            case NOOP: // (line 7-8 for \Delta)
                                if (labels.contains(t._label)) {
                                    insert_edge(pre_state, t._label, t._to, p_automaton_t::new_pre_trace(rule_id));
                                }
                                break;
                            case PUSH: // (line 9)
                                if (rule._op_label == t._label) {
                                    // (line 10)
                                    _delta_prime[t._to].emplace_back(pre_state, rule_id);
                                    for (auto rel_rule : _rel[t._to]) { // (line 11-12)
                                        if (labels.contains(rel_rule.second)) {
                                            insert_edge(pre_state, rel_rule.second, rel_rule.first, p_automaton_t::new_pre_trace(rule_id, t._to));
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
            [[nodiscard]] bool workset_empty() const {
                return _workset.empty();
            }
            [[nodiscard]] bool found() const {
                return _found;
            }
        };

        template <typename W, bool ET=false>
        class PostStarSaturation {
            using trace_info = TraceInfo<TraceInfoType::Single>;
            using edge_anno = edge_annotation<W,TraceInfoType::Single>;
            using edge_anno_t = edge_annotation_t<W,TraceInfoType::Single>;
            using p_automaton_t = PAutomaton<W>;
        public:
            explicit PostStarSaturation(p_automaton_t& automaton, const early_termination_fn<W>& early_termination = [](size_t f, uint32_t l, size_t t, edge_anno_t trace) -> bool { return false; })
                    : _automaton(automaton), _early_termination(early_termination), _pda_states(_automaton.pda().states()),
                      _n_pda_states(_pda_states.size()), _n_Q(_automaton.states().size()) {
                initialize();
            };

        private:
            // This is an implementation of Algorithm 2 (figure 3.4) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 48)

            p_automaton_t& _automaton;
            const early_termination_fn<W>& _early_termination;
            const std::vector<typename PDA<W>::state_t>& _pda_states;
            const size_t _n_pda_states;
            const size_t _n_Q;
            std::unordered_map<std::pair<size_t, uint32_t>, size_t, absl::Hash<std::pair<size_t, uint32_t>>> _q_prime{};

            size_t _n_automaton_states{};
            std::unordered_set<temp_edge_t, absl::Hash<temp_edge_t>> _edges;
            std::queue<temp_edge_t> _workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel1; // faster access for lookup _from -> (_to, _label)
            std::vector<std::vector<size_t>> _rel2; // faster access for lookup _to -> _from  (when _label is uint32_t::max)

            bool _found = false;

            void initialize() {
                // for <p, y> -> <p', y1 y2> do  (line 3)
                //   Q' U= {q_p'y1}              (line 4)
                for (auto &state : _pda_states) {
                    for (auto &[rule, labels] : state._rules) {
                        if (rule._operation == PUSH) {
                            auto res = _q_prime.emplace(std::make_pair(rule._to, rule._op_label), _automaton.next_state_id());
                            if (res.second) {
                                _automaton.add_state(false, false);
                            }
                        }
                    }
                }
                _n_automaton_states = _automaton.states().size();
                _rel1.resize(_n_automaton_states);
                _rel2.resize(_n_automaton_states - _n_Q);

                // workset := ->_0 intersect (P x Gamma x Q)  (line 1)
                // rel := ->_0 \ workset (line 2)
                for (const auto& from : _automaton.states()) {
                    for (const auto& [to,labels] : from->_edges) {
                        assert(!labels.contains(epsilon)); // PostStar algorithm assumes no epsilon transitions in the NFA.
                        for (const auto& [label,_] : labels) {
                            insert_edge(from->_id, label, to, trace_info::make_default(), from->_id >= _n_pda_states);
                        }
                    }
                }
            }
            void insert_edge(size_t from, uint32_t label, size_t to, trace_t trace, bool direct_to_rel = false) {
                auto res = _edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U workset).
                    if (direct_to_rel) {
                        _rel1[from].emplace_back(to, label);
                        if (label == epsilon && to >= _n_Q) {
                            _rel2[to - _n_Q].push_back(from);
                        }
                    } else {
                        _workset.emplace(from, label, to);
                    }
                    if (!trace.is_null()) { // Don't add existing edges
                        if (label == epsilon) {
                            _automaton.add_epsilon_edge(from, to, edge_anno::from_trace_info(trace));
                        } else {
                            _automaton.add_edge(from, to, label, edge_anno::from_trace_info(trace));
                        }
                    }
                    if constexpr (ET) {
                        _found = _found || _early_termination(from, label, to, edge_anno::from_trace_info(trace));
                    }
                }
            };

        public:
            void step() {
                // pop t = (q, y, q') from workset (line 6)
                temp_edge_t t;
                t = _workset.front();
                _workset.pop();
                // rel = rel U {t} (line 8)   (membership test on line 7 is done in insert_edge).
                _rel1[t._from].emplace_back(t._to, t._label);
                if (t._label == epsilon && t._to >= _n_Q) {
                    _rel2[t._to - _n_Q].push_back(t._from);
                }

                // if y != epsilon (line 9)
                if (t._label != epsilon) {
                    const auto &rules = _pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        const auto &[rule,labels] = rules[rule_id];
                        if (!labels.contains(t._label)) { continue; }
                        auto trace = p_automaton_t::new_post_trace(t._from, rule_id, t._label);
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
                                assert(_q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(_q_prime));
                                size_t q_new = _q_prime[std::make_pair(rule._to, rule._op_label)];
                                insert_edge(rule._to, rule._op_label, q_new, trace, false); // (line 15)
                                insert_edge(q_new, t._label, t._to, trace, true); // (line 16)
                                if (!_rel2[q_new - _n_Q].empty()) {
                                    auto trace_q_new = p_automaton_t::new_post_trace(q_new);
                                    for (auto f : _rel2[q_new - _n_Q]) { // (line 17)
                                        insert_edge(f, t._label, t._to, trace_q_new, false); // (line 18)
                                    }
                                }
                                break;
                        }
                    }
                } else {
                    if (!_rel1[t._to].empty()) {
                        auto trace = p_automaton_t::new_post_trace(t._to);
                        for (auto e : _rel1[t._to]) { // (line 20)
                            insert_edge(t._from, e.second, e.first, trace, false); // (line 21)
                        }
                    }
                }
            }
            [[nodiscard]] bool workset_empty() const {
                return _workset.empty();
            }
            [[nodiscard]] bool found() const {
                return _found;
            }
        };

        template<typename W, bool Enable, bool ET, typename = std::enable_if_t<Enable>>
        class PostStarShortestSaturation {
            static_assert(W::is_weight);
            using solver_weight = min_weight<typename W::type>;

            using trace_info = TraceInfo<TraceInfoType::Single>;
            using edge_anno = edge_annotation<W,TraceInfoType::Single>;
            using edge_anno_t = edge_annotation_t<W,TraceInfoType::Single>;
            using p_automaton_t = PAutomaton<W>;

            struct weight_edge_trace {
                typename W::type _weight;
                temp_edge_t _edge;
                trace_t _trace;
                weight_edge_trace(typename W::type weight, temp_edge_t edge, trace_t trace) : _weight(weight), _edge(edge), _trace(trace) {};
                weight_edge_trace() = default;
            };
            struct weight_edge_trace_comp{
                bool operator()(const weight_edge_trace &lhs, const weight_edge_trace &rhs){
                    return solver_weight::less(rhs._weight, lhs._weight); // Used in a max-heap, so swap arguments to make it a min-heap.
                }
            };
            struct rel3_elem {
                uint32_t _label;
                size_t _to;
                trace_t _trace;
                typename W::type _weight;

                bool operator<(const rel3_elem& other) const {
                    if (_label != other._label) return _label < other._label;
                    return _to < other._to;
                }
                bool operator==(const rel3_elem& other) const {
                    return _to == other._to && _label == other._label;
                }
                bool operator!=(const rel3_elem& other) const {
                    return !(*this == other);
                }
            };

        public:
            PostStarShortestSaturation(p_automaton_t& automaton, const early_termination_fn<W>& early_termination)
            : _automaton(automaton), _early_termination(early_termination), _pda_states(_automaton.pda().states()),
              _n_pda_states(_pda_states.size()), _n_Q(_automaton.states().size()) {
                assert(!has_negative_weight());
                if (has_negative_weight()) {
                    throw std::runtime_error("Priority-queue based shortest trace post* algorithm does not work with negative weights.");
                }
                initialize();
            };

        private:
            p_automaton_t& _automaton;
            const early_termination_fn<W>& _early_termination;
            const std::vector<typename PDA<W>::state_t>& _pda_states;
            const size_t _n_pda_states;
            const size_t _n_Q;
            std::unordered_map<std::pair<size_t, uint32_t>, size_t, absl::Hash<std::pair<size_t, uint32_t>>> _q_prime{};

            size_t _n_automaton_states{};
            std::vector<typename W::type> _minpath;

            std::unordered_map<temp_edge_t, std::pair<typename W::type, typename W::type>, absl::Hash<temp_edge_t>> _edge_weights;
            std::priority_queue<weight_edge_trace, std::vector<weight_edge_trace>, weight_edge_trace_comp> _workset;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel1; // faster access for lookup _from -> (_to, _label)
            std::vector<std::vector<size_t>> _rel2; // faster access for lookup _to -> _from  (when _label is uint32_t::max)
            std::vector<std::vector<rel3_elem>> _rel3;

            bool _found = false;

            bool has_negative_weight() const {
                if constexpr (W::is_signed) {
                    for (const auto& state : _pda_states) {
                        for (const auto& [rule,labels] : state._rules) {
                            if (solver_weight::less(rule._weight, W::zero())) {
                                return true;
                            }
                        }
                    }
                    for (const auto& from : _automaton.states()) {
                        for (const auto& [to,labels] : from->_edges) {
                            for (const auto& [label,trace] : labels) {
                                if (solver_weight::less(trace.second, W::zero())) {
                                    return true;
                                }
                            }
                        }
                    }
                }
                return false;
            }

            void initialize() {
                // for <p, y> -> <p', y1 y2> do
                //   Q' U= {q_p'y1}
                for (const auto &state : _pda_states) {
                    for (const auto &[rule,labels] : state._rules) {
                        if (rule._operation == PUSH) {
                            auto res = _q_prime.emplace(std::make_pair(rule._to, rule._op_label), _automaton.next_state_id());
                            if (res.second) {
                                _automaton.add_state(false, false);
                            }
                        }
                    }
                }
                _n_automaton_states = _automaton.states().size();
                _minpath.resize(_n_automaton_states - _n_Q);
                for (size_t i = 0; i < _minpath.size(); ++i) {
                    _minpath[i] = solver_weight::max();
                }

                _rel1.resize(_n_Q);
                _rel2.resize(_n_automaton_states - _n_Q);
                _rel3.resize(_n_automaton_states - _n_Q);

                // workset := ->_0 intersect (P x Gamma x Q)
                // rel := ->_0 \ workset
                for (const auto& from : _automaton.states()) {
                    for (const auto& [to,labels] : from->_edges) {
                        assert(!labels.contains(epsilon)); // PostStar algorithm assumes no epsilon transitions in the NFA.
                        for (const auto& [label,trace] : labels) {
                            temp_edge_t temp_edge{from->_id, label, to};
                            _edge_weights.emplace(temp_edge, std::make_pair(W::zero(), W::zero()));
                            if (from->_id < _n_pda_states) {
                                _workset.emplace(W::zero(), temp_edge, trace_info::make_default());
                            } else {
                                insert_rel(from->_id, label, to);
                                if constexpr (ET) {
                                    _found |= _early_termination(from->_id, label, to, trace);
                                }
                            }
                        }
                    }
                }
            }

            std::pair<bool,bool> update_edge_(size_t from, uint32_t label, size_t to, typename W::type edge_weight, typename W::type workset_weight) {
                auto res = _edge_weights.emplace(temp_edge_t{from, label, to}, std::make_pair(edge_weight, workset_weight));
                if (!res.second) {
                    auto result = std::make_pair(false, false);
                    if (solver_weight::less(edge_weight, (*res.first).second.first)) {
                        (*res.first).second.first = edge_weight;
                        result.first = true;
                    }
                    if (solver_weight::less(workset_weight, (*res.first).second.second)) {
                        (*res.first).second.second = workset_weight;
                        result.second = true;
                    }
                    return result;
                }
                return std::make_pair(res.second, res.second);
            }
            void update_edge(size_t from, uint32_t label, size_t to, typename W::type edge_weight, trace_t trace) {
                auto workset_weight = to < _n_Q ? edge_weight : solver_weight::add(_minpath[to - _n_Q], edge_weight);
                if (update_edge_(from, label, to, edge_weight, workset_weight).second) {
                    _workset.emplace(workset_weight, temp_edge_t{from, label, to}, trace);
                }
            }
            typename W::type get_weight(size_t from, uint32_t label, size_t to) const {
                return (*_edge_weights.find(temp_edge_t{from, label, to})).second.first;
            }
            void insert_rel(size_t from, uint32_t label, size_t to) { // Adds to rel.
                _rel1[from].emplace_back(to, label);
                if (label == epsilon && to >= _n_Q) {
                    _rel2[to - _n_Q].push_back(from);
                }
            }

        public:
            void step() {
                // pop t = (q, y, q') from workset
                auto elem = _workset.top();
                _workset.pop();
                auto t = elem._edge;
                auto weights = (*_edge_weights.find(t)).second;
                if (solver_weight::less(weights.second, elem._weight)) {
                    return; // Same edge with a smaller weight was already processed.
                }
                auto t_weight = weights.first;

                // rel = rel U {t}
                insert_rel(t._from, t._label, t._to);
                if (t._label == epsilon) {
                    _automaton.add_epsilon_edge(t._from, t._to, std::make_pair(elem._trace, t_weight));
                } else {
                    _automaton.add_edge(t._from, t._to, t._label, std::make_pair(elem._trace, t_weight));
                }
                if constexpr (ET) {
                    _found |= _early_termination(t._from, t._label, t._to, std::make_pair(elem._trace, t_weight));
                }

                // if y != epsilon
                if (t._label != epsilon) {
                    const auto &rules = _pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        const auto &[rule,labels] = rules[rule_id];
                        if (!labels.contains(t._label)) { continue; }
                        auto trace = p_automaton_t::new_post_trace(t._from, rule_id, t._label);
                        auto wd = solver_weight::add(elem._weight, rule._weight);
                        auto wb = solver_weight::add(t_weight, rule._weight);
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
                            assert(_q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(_q_prime));
                            size_t q_new = _q_prime[std::make_pair(rule._to, rule._op_label)];
                            auto add_to_workset = update_edge_(rule._to, rule._op_label, q_new, W::zero(), wd).second;
                            auto was_updated = update_edge_(q_new, t._label, t._to, wb, W::zero()).first;
                            if (was_updated) {
                                rel3_elem new_elem{t._label, t._to, trace, wb};
                                auto &relq = _rel3[q_new - _n_Q];
                                auto lb = std::lower_bound(relq.begin(), relq.end(), new_elem);
                                if (lb == std::end(relq) || *lb != new_elem) {
                                    relq.insert(lb, new_elem);
                                } else if (solver_weight::less(wb, lb->_weight)) {
                                    *lb = new_elem;
                                }
                            }
                            if (solver_weight::less(wd, _minpath[q_new - _n_Q])) {
                                _minpath[q_new - _n_Q] = wd;
                                if (add_to_workset) {
                                    _workset.emplace(wd, temp_edge_t{rule._to, rule._op_label, q_new}, trace);
                                }
                            } else if (was_updated) {
                                if (!_rel2[q_new - _n_Q].empty()) {
                                    auto trace_q_new = p_automaton_t::new_post_trace(q_new);
                                    for (auto f : _rel2[q_new - _n_Q]) {
                                        update_edge(f, t._label, t._to, solver_weight::add(get_weight(f, epsilon, q_new), wb), trace_q_new);
                                    }
                                }
                            }
                        }
                    }
                } else {
                    if (t._to < _n_Q) {
                        if (!_rel1[t._to].empty()) {
                            auto trace = _automaton.new_post_trace(t._to);
                            for (auto e : _rel1[t._to]) {
                                assert(e.first >= _n_pda_states);
                                update_edge(t._from, e.second, e.first, solver_weight::add(get_weight(t._to, e.second, e.first), t_weight), trace);
                            }
                        }
                    } else {
                        if (!_rel3[t._to - _n_Q].empty()) {
                            auto trace = _automaton.new_post_trace(t._to);
                            for (auto &e : _rel3[t._to - _n_Q]) {
                                update_edge(t._from, e._label, e._to, solver_weight::add(get_weight(t._to, e._label, e._to), t_weight), trace);
                            }
                        }
                    }
                }
            }
            void finalize() {
                for (size_t i = _n_Q; i < _n_automaton_states; ++i) {
                    for (auto &e : _rel3[i - _n_Q]) {
                        assert(e._label != epsilon);
                        _automaton.add_edge(i, e._to, e._label, std::make_pair(e._trace, e._weight));
                        if constexpr (ET) { // TODO: We might need to do this in main loop (and update weights) to enable early termination in some cases..??
                            _found |= _early_termination(i, e._label, e._to, std::make_pair(e._trace, e._weight));
                        }
                    }
                }
            }
            [[nodiscard]] bool workset_empty() const {
                return _workset.empty();
            }
            [[nodiscard]] bool found() const {
                return _found;
            }
        };

        template<typename W, Trace_Type trace_type>
        class PreStarFixedPointSaturation : public fixed_point_workset<PreStarFixedPointSaturation<W,trace_type>, temp_edge_t> {
            using parent_t = fixed_point_workset<PreStarFixedPointSaturation<W,trace_type>, temp_edge_t>;
            static_assert(is_weighted<W>);
            using weight_t = typename W::type;
            using solverW = solver_weight<W,trace_type>;

            using trace_info = TraceInfo<TraceInfoType::Pair>;
            using trace_info_t = TraceInfo_t<TraceInfoType::Pair>;
            using edge_anno = edge_annotation<W,TraceInfoType::Pair>;
            using edge_anno_t = edge_annotation_t<W,TraceInfoType::Pair>;
            using p_automaton_t = PAutomaton<W,TraceInfoType::Pair>;

            template<bool change_is_bottom = false>
            void update_edge(size_t from, uint32_t label, size_t to, const weight_t& edge_weight, trace_t trace) {
                auto [it, fresh] = _edges.emplace(temp_edge_t{from, label, to}, edge_weight);
                bool is_changed = false;
                if (fresh) {
                    if constexpr(change_is_bottom) {
                        assert(false); // We should add all fresh edges during the first pass of rounds.
                    }
                    _rel[from].emplace_back(to, label); // Allow fast iteration over _edges matching specific from.
                    if (!trace.is_null()) {
                        _automaton.add_edge(from, to, label, std::make_pair(std::make_pair(trace,trace), edge_weight));
                    }
                    is_changed = true;
                } else {
                    if (solverW::less(edge_weight, it->second)) {
                        if constexpr(change_is_bottom) {
                            it->second = solverW::bottom();
                            _automaton.update_edge(from, to, label, std::make_pair(trace, solverW::bottom()));
                        } else {
                            it->second = edge_weight;
                            _automaton.update_edge(from, to, label, std::make_pair(trace, edge_weight));
                        }
                        is_changed = true;
                    }
                }
                if (is_changed) {
                    parent_t::emplace(from, label, to);
                }
            }
            template<bool change_is_bottom = false>
            void update_edge_bulk(size_t from, const labels_t &precondition, size_t to, const weight_t& weight, trace_t trace) {
                if (precondition.wildcard()) {
                    for (uint32_t i = 0; i < _n_pda_labels; i++) {
                        update_edge<change_is_bottom>(from, i, to, weight, trace);
                    }
                } else {
                    for (auto &label : precondition.labels()) {
                        update_edge<change_is_bottom>(from, label, to, weight, trace);
                    }
                }
            }
        public:
            explicit PreStarFixedPointSaturation(p_automaton_t& automaton)
            : parent_t(std::pow(automaton.states().size(), 2) * automaton.number_of_labels()),
              _automaton(automaton), _pda_states(_automaton.pda().states()), _n_automaton_states(_automaton.states().size()),
              _n_pda_states(_pda_states.size()), _n_pda_labels(_automaton.number_of_labels()),
              _rel(_n_automaton_states), _delta_prime(_n_automaton_states) {
                initialize();
            };
            PreStarFixedPointSaturation(p_automaton_t& automaton, size_t round_limit)
            : parent_t(round_limit),
              _automaton(automaton), _pda_states(_automaton.pda().states()), _n_automaton_states(_automaton.states().size()),
              _n_pda_states(_pda_states.size()), _n_pda_labels(_automaton.number_of_labels()),
              _rel(_n_automaton_states), _delta_prime(_n_automaton_states) {
                initialize();
            };
            static constexpr temp_edge_t next_round_elem = temp_edge_t();
        private:
            p_automaton_t& _automaton;
            const std::vector<typename PDA<W>::state_t>& _pda_states;
            const size_t _n_automaton_states;
            const size_t _n_pda_states;
            const size_t _n_pda_labels;

            std::unordered_map<temp_edge_t, weight_t, absl::Hash<temp_edge_t>> _edges;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel; // Fast access to _edges based on _from.
            std::vector<std::vector<std::tuple<size_t, size_t>>> _delta_prime;

            void initialize() {
                for (const auto& from : _automaton.states()) {
                    for (const auto& [to,labels] : from->_edges) {
                        for (const auto& [label,tw] : labels) {
                            assert(tw == std::make_pair(trace_info::make_default(), W::zero()));
                            update_edge(from->_id, label, to, W::zero(), trace_t());
                        }
                    }
                }
                // for all <p, y> --> <p', epsilon> : workset U= (p, y, p') (line 2)
                for (size_t state = 0; state < _n_pda_states; ++state) {
                    size_t rule_id = 0;
                    for (const auto& [rule,labels] : _pda_states[state]._rules) {
                        if (rule._operation == POP) {
                            update_edge_bulk(state, labels, rule._to, rule._weight, p_automaton_t::new_pre_trace(rule_id));
                        }
                        ++rule_id;
                    }
                }
            }

        public:
            template<bool change_is_bottom = false>
            bool step_with(temp_edge_t&& t) {
                assert(_edges.find(t) != _edges.end());
                auto w = _edges.find(t)->second;

                // (line 7-8 for \Delta')
                for (const auto& [state, rule_id] : _delta_prime[t._from]) { // Loop over delta_prime (that match with t->from)
                    const auto& [rule, labels] = _pda_states[state]._rules[rule_id];
                    if (labels.contains(t._label)) {
                        assert(_edges.find(temp_edge_t{rule._to, rule._op_label, t._from}) != _edges.end());
                        update_edge<change_is_bottom>(state, t._label, t._to,
                                                      solverW::add(solverW::add(rule._weight, _edges.find(temp_edge_t{rule._to, rule._op_label, t._from})->second), w),
                                                      p_automaton_t::new_pre_trace(rule_id, t._from));
                    }
                }

                if (t._from >= _n_pda_states) { return true; }
                for (auto pre_state : _pda_states[t._from]._pre_states) {
                    const auto &rules = _pda_states[pre_state]._rules;
                    auto lb = rules.lower_bound(details::rule_t<W>{t._from});
                    while (lb != rules.end() && lb->first._to == t._from) {
                        const auto &[rule, labels] = *lb;
                        size_t rule_id = lb - rules.begin();
                        ++lb;
                        switch (rule._operation) {
                            case POP:
                                break;
                            case SWAP: // (line 7-8 for \Delta)
                                if (rule._op_label == t._label) {
                                    update_edge_bulk<change_is_bottom>(pre_state, labels, t._to, solverW::add(rule._weight, w), p_automaton_t::new_pre_trace(rule_id));
                                }
                                break;
                            case NOOP: // (line 7-8 for \Delta)
                                if (labels.contains(t._label)) {
                                    update_edge<change_is_bottom>(pre_state, t._label, t._to, solverW::add(rule._weight, w), p_automaton_t::new_pre_trace(rule_id));
                                }
                                break;
                            case PUSH: // (line 9)
                                if (rule._op_label == t._label) {
                                    auto w_temp = solverW::add(rule._weight, w);
                                    // (line 10)
                                    _delta_prime[t._to].emplace_back(pre_state, rule_id); // TODO: Check existence before adding(?)
                                    for (const auto& [rel_to, rel_label] : _rel[t._to]) { // (line 11-12)
                                        if (labels.contains(rel_label)) {
                                            auto it = _edges.find(temp_edge_t{t._to, rel_label, rel_to});
                                            assert(it != _edges.end());
                                            update_edge<change_is_bottom>(pre_state, rel_label, rel_to, solverW::add(w_temp, it->second), p_automaton_t::new_pre_trace(rule_id, t._to));
                                        }
                                    }
                                }
                                break;
                            default:
                                assert(false);
                        }
                    }
                }
                return true;
            }
        };

        template <typename W, TraceInfoType trace_info_type>
        class TraceBack {
            using rule_t = user_rule_t<W>;
            using p_automaton_t = PAutomaton<W,trace_info_type>;
            using trace_info = TraceInfo<trace_info_type>;
            using trace_info_t = TraceInfo_t<trace_info_type>;
        public:
            TraceBack(const p_automaton_t& automaton, AutomatonPath&& path)
            : _automaton(automaton), _path(std::move(path)) { };
        private:
            const p_automaton_t& _automaton;
            AutomatonPath _path;
            bool _post = false;

            template <bool avoid_loop>
            auto to_trace_label(trace_info_t t) const {
                if constexpr(trace_info_type == TraceInfoType::Pair) {
                    if constexpr(avoid_loop) {
                        return t.first;
                    } else {
                        return t.second;
                    }
                } else {
                    return t;
                }
            }
            template <bool avoid_loop>
            auto get_trace_label(size_t from, uint32_t label, size_t to) const {
                return to_trace_label<avoid_loop>(_automaton.get_trace_label(from, label, to));
            }

        public:
            [[nodiscard]] bool post() const { return _post; }
            [[nodiscard]] const AutomatonPath& path() const { return _path; }
            template <bool avoid_loop = false>
            std::optional<rule_t> next() {
                while(true) { // In case of post_epsilon_trace, keep going until a rule is found or we are done.
                    if (_path.empty()) return std::nullopt;
                    auto[from, label, to] = _path.front_edge();
                    trace_t trace_label = get_trace_label<avoid_loop>(from, label, to);
                    if (trace_label.is_null()) return std::nullopt; // Done
                    _path.pop();

                    if (trace_label.is_pre_trace()) {
                        // pre* trace
                        const auto &[rule, labels] = _automaton.pda().states()[from]._rules[trace_label._rule_id];
                        switch (rule._operation) {
                            case POP:
                                break;
                            case SWAP:
                                _path.emplace(rule._to, rule._op_label);
                                break;
                            case NOOP:
                                _path.emplace(rule._to, label);
                                break;
                            case PUSH:
                                _path.emplace(trace_label._state, label);
                                _path.emplace(rule._to, rule._op_label);
                                break;
                        }
                        return rule_t(from, label, rule);
                    } else if (trace_label.is_post_epsilon_trace()) {
                        // Intermediate post* trace
                        // Current edge is the result of merging with an epsilon edge.
                        // Reconstruct epsilon edge and the other edge.
                        _path.emplace(trace_label._state, label);
                        _path.emplace(from, std::numeric_limits<uint32_t>::max());

                    } else { // post* trace
                        _post = true;
                        const auto &[rule, labels] = _automaton.pda().states()[trace_label._state]._rules[trace_label._rule_id];
                        switch (rule._operation) {
                            case POP:
                            case SWAP:
                            case NOOP:
                                _path.emplace(trace_label._state, trace_label._label);
                                break;
                            case PUSH:
                                auto[from2, label2, to2] = _path.front_edge();
                                _path.pop();
                                trace_label = get_trace_label<avoid_loop>(from2, label2, to2);
                                assert(!trace_label.is_null());
                                _path.emplace(trace_label._state, trace_label._label);
                                break;
                        }
                        assert(from == rule._to);
                        return rule_t(trace_label._state, trace_label._label, rule);
                    }
                }
            }
        };

    }

    class Solver {
    public:
        template <Trace_Type trace_type, typename pda_t, typename automaton_t, typename W>
        static bool pre_star_fixed_point_accepts(PAutomatonProduct<pda_t,automaton_t,W,TraceInfoType::Pair>& instance) {
            instance.enable_pre_star();
            pre_star_fixed_point<trace_type>(instance.automaton());
            return instance.template initialize_product<false,false>();
        }
        template <Trace_Type trace_type, typename W>
        static void pre_star_fixed_point(PAutomaton<W,TraceInfoType::Pair>& automaton) {
            details::PreStarFixedPointSaturation<W,trace_type> saturation(automaton);
            while(!saturation.done()) {
                saturation.step();
            }
            saturation.finalize();
        }

        template <typename pda_t, typename automaton_t, typename W>
        static bool dual_search_accepts(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            if (instance.template initialize_product<true>()) {
                return true;
            }
            return dual_search<W>(instance.final_automaton(), instance.initial_automaton(),
                [&instance](size_t from, uint32_t label, size_t to, edge_annotation_t<W> trace) -> bool {
                    return instance.add_final_edge(from, label, to, trace);
                },
                [&instance](size_t from, uint32_t label, size_t to, edge_annotation_t<W> trace) -> bool {
                    return instance.add_initial_edge(from, label, to, trace);
                }
            );
        }
        template <typename W, bool ET=true>
        static bool dual_search(PAutomaton<W> &pre_star_automaton, PAutomaton<W> &post_star_automaton,
                                const details::early_termination_fn<W>& pre_star_early_termination,
                                const details::early_termination_fn<W>& post_star_early_termination) {
            details::PreStarSaturation<W,ET> pre_star(pre_star_automaton, pre_star_early_termination);
            details::PostStarSaturation<W,ET> post_star(post_star_automaton, post_star_early_termination);
            if constexpr (ET) {
                if (pre_star.found() || post_star.found()) return true;
            }
            while(!pre_star.workset_empty() && !post_star.workset_empty()) {
                post_star.step();
                if constexpr (ET) {
                    if (post_star.found()) return true;
                }
                pre_star.step();
                if constexpr (ET) {
                    if (pre_star.found()) return true;
                }
            }
            return pre_star.found() || post_star.found();
        }

        template <typename W>
        static bool pre_star_accepts(PAutomaton<W> &automaton, size_t state, const std::vector<uint32_t> &stack) {
            if (stack.size() == 1) {
                auto s_label = stack[0];
                return pre_star<W,true>(automaton, [&automaton, state, s_label](size_t from, uint32_t label, size_t to, edge_annotation_t<W>) -> bool {
                    return from == state && label == s_label && automaton.states()[to]->_accepting;
                });
            } else {
                return pre_star<W>(automaton) || automaton.accepts(state, stack);
            }
        }

        template <typename pda_t, typename automaton_t, typename W>
        static bool pre_star_accepts(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            instance.enable_pre_star();
            return instance.initialize_product() ||
                   pre_star<W,true>(instance.automaton(), [&instance](size_t from, uint32_t label, size_t to, edge_annotation_t<W> trace) -> bool {
                       return instance.add_edge_product(from, label, to, trace);
                   });
        }

        template <typename W, bool ET=false>
        static bool pre_star(PAutomaton<W> &automaton,
                             const details::early_termination_fn<W>& early_termination = [](size_t, uint32_t, size_t, edge_annotation_t<W>) -> bool { return false; }) {
            details::PreStarSaturation<W,ET> saturation(automaton, early_termination);
            while(!saturation.workset_empty()) {
                if constexpr (ET) {
                    if (saturation.found()) return true;
                }
                saturation.step();
            }
            return saturation.found();
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename W>
        static bool post_star_accepts(PAutomaton<W> &automaton, size_t state, const std::vector<uint32_t> &stack) {
            if (stack.size() == 1) {
                auto s_label = stack[0];
                return post_star<trace_type,W,true>(automaton, [&automaton, state, s_label](size_t from, uint32_t label, size_t to, edge_annotation_t<W>) -> bool {
                    return from == state && label == s_label && automaton.states()[to]->_accepting;
                });
            } else {
                return post_star<trace_type,W>(automaton) || automaton.accepts(state, stack);
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename pda_t, typename automaton_t, typename W>
        static bool post_star_accepts(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            return instance.initialize_product() ||
                   post_star<trace_type,W,true>(instance.automaton(), [&instance](size_t from, uint32_t label, size_t to, edge_annotation_t<W> trace) -> bool {
                       return instance.add_edge_product(from, label, to, trace);
                   });
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename W, bool ET = false>
        static bool post_star(PAutomaton<W> &automaton,
                              const details::early_termination_fn<W>& early_termination = [](size_t, uint32_t, size_t, edge_annotation_t<W>) -> bool { return false; }) {
            static_assert(is_weighted<W> || trace_type != Trace_Type::Shortest, "Cannot do shortest-trace post* for PDA without weights."); // TODO: Consider: W=uin32_t, weight==1 as a default weight.
            if constexpr (is_weighted<W> && trace_type == Trace_Type::Shortest) {
                return post_star_shortest<W,true,ET>(automaton, early_termination);
            } else if constexpr (trace_type == Trace_Type::Any) {
                return post_star_any<W,ET>(automaton, early_termination);
            } else if constexpr (trace_type == Trace_Type::None) {
                return post_star_any<W,ET>(automaton, early_termination); // TODO: Implement faster no-trace option.
            }
        }

        template <typename pda_t, typename automaton_t, typename W>
        static bool pre_star_accepts_no_ET(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            instance.enable_pre_star();
            pre_star<W,false>(instance.automaton());
            return instance.template initialize_product<false,false>();
        }
        template <Trace_Type trace_type = Trace_Type::Any, typename pda_t, typename automaton_t, typename W>
        static bool post_star_accepts_no_ET(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            post_star<trace_type,W,false>(instance.automaton());
            return instance.template initialize_product<false,false>();
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename pda_t, typename automaton_t, typename W, TraceInfoType trace_info_type>
        static auto get_trace(const PAutomatonProduct<pda_t,automaton_t,W,trace_info_type>& instance) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest || trace_type == Trace_Type::Longest || trace_type == Trace_Type::ShortestFixedPoint) {
                auto [path, stack, weight] = instance.template find_path<trace_type>();
                return std::make_pair(_get_trace(instance.pda(), instance.automaton(), path, stack), weight);
            } else {
                auto [path, stack] = instance.template find_path<trace_type>();
                return _get_trace(instance.pda(), instance.automaton(), path, stack);
            }
        }
        template <typename pda_t, typename automaton_t, typename W>
        static auto get_trace_dual_search(const PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            auto [paths, stack] = instance.template find_path<Trace_Type::Any, true>();
            std::vector<size_t> i_path, f_path;
            for (const auto& [i_state, f_state] : paths) {
                i_path.emplace_back(i_state);
                f_path.emplace_back(f_state);
            }
            auto trace1 = _get_trace(instance.pda(), instance.initial_automaton(), i_path, stack);
            auto trace2 = _get_trace(instance.pda(), instance.final_automaton(), f_path, stack);
            assert(trace1.back()._pdastate == trace2.front()._pdastate);
            assert(trace1.back()._stack.size() == trace2.front()._stack.size()); // Should also check == for contents of stack, but T might not implement ==.
            trace1.insert(trace1.end(), trace2.begin() + 1, trace2.end());
            return trace1;
        }
        template <Trace_Type trace_type = Trace_Type::Any, bool use_dual=false, typename pda_t, typename automaton_t, typename W>
        static auto get_rule_trace_and_paths(const PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto[paths, stack, weight] = instance.template find_path<trace_type, true>();
                return std::make_pair(_get_rule_trace_and_paths(instance.automaton(), paths, stack), weight);
            } else if constexpr(use_dual) {
                auto[paths, stack] = instance.template find_path<trace_type, true>();
                return _get_rule_trace_and_paths(instance.initial_automaton(), instance.final_automaton(), paths, stack);
            } else {
                auto[paths, stack] = instance.template find_path<trace_type, true>();
                return _get_rule_trace_and_paths(instance.automaton(), paths, stack);
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W>
        static auto get_trace(const TypedPDA<T,W>& pda, const PAutomaton<W>& automaton, size_t state, const std::vector<T>& stack) {
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
        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename = std::enable_if_t<!std::is_same_v<T,uint32_t>>>
        static auto get_trace(const TypedPDA<T,W>& pda, const PAutomaton<W>& automaton, size_t state, const std::vector<uint32_t>& stack_native) {
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
        template <typename W, bool ET>
        static bool post_star_any(PAutomaton<W> &automaton, const details::early_termination_fn<W>& early_termination) {
            details::PostStarSaturation<W,ET> saturation(automaton, early_termination);
            while(!saturation.workset_empty()) {
                if constexpr (ET) {
                    if (saturation.found()) return true;
                }
                saturation.step();
            }
            return saturation.found();
        }

        template<typename W, bool Enable, bool ET, typename = std::enable_if_t<Enable>>
        static bool post_star_shortest(PAutomaton<W> &automaton, const details::early_termination_fn<W>& early_termination) {
            details::PostStarShortestSaturation<W,Enable,ET> saturation(automaton, early_termination);
            while(!saturation.workset_empty()) {
                if constexpr (ET) {
                    if (saturation.found()) break;
                }
                saturation.step();
            }
            saturation.finalize();
            return saturation.found();
        }

        template <typename T, typename W, typename S, bool ssm, TraceInfoType trace_info_type>
        static std::vector<typename TypedPDA<T,W,fut::type::vector,S,ssm>::tracestate_t>
        _get_trace(const TypedPDA<T,W,fut::type::vector,S,ssm> &pda, const PAutomaton<W,trace_info_type>& automaton,
                   const std::vector<size_t>& path, const std::vector<uint32_t>& stack) {
            using tracestate_t = typename TypedPDA<T,W,fut::type::vector,S,ssm>::tracestate_t;

            if (path.empty()) {
                return std::vector<tracestate_t>();
            }
            AutomatonPath automaton_path(path, stack);

            auto decode_edges = [&pda](const AutomatonPath& path) -> tracestate_t {
                tracestate_t result{path.front_state(), std::vector<T>()};
                auto num_labels = pda.number_of_labels();
                for (auto label : path.stack()) {
                    if (label < num_labels){
                        result._stack.emplace_back(pda.get_symbol(label));
                    }
                }
                return result;
            };

            std::vector<tracestate_t> trace;
            trace.push_back(decode_edges(automaton_path));
            details::TraceBack tb(automaton, std::move(automaton_path));
            while (tb.next()) {
                trace.push_back(decode_edges(tb.path()));
            }
            if (tb.post()) {
                std::reverse(trace.begin(), trace.end());
            }
            return trace;
        }


        template <typename W, TraceInfoType trace_info_type>
        static std::tuple<
                size_t, // Initial state. (State is size_t::max if no trace exists.)
                std::vector<user_rule_t<W>>, // Sequence of rules applied to the initial configuration to reach the final configuration.
                std::vector<uint32_t>, // Initial stack
                std::vector<uint32_t>, // Final stack
                std::vector<size_t>, // Path in initial PAutomaton (accepting initial stack)
                std::vector<size_t>  // Path in final PAutomaton (accepting final stack) (independent of whether pre* or post* was used)
                > _get_rule_trace_and_paths(const PAutomaton<W,trace_info_type>& automaton, // The PAutomaton that has been build up (either A_pre* or A_post*)
                                           const std::vector<std::pair<size_t,size_t>>& paths, // The paths as retrieved from the product automaton. First number is the state in @automaton (A_pre* or A_post*), second number is state in goal automaton.
                                           const std::vector<uint32_t>& stack) {
            using rule_t = user_rule_t<W>;

            if (paths.empty()) {
                return std::make_tuple(std::numeric_limits<size_t>::max(), std::vector<rule_t>(), std::vector<uint32_t>(), std::vector<uint32_t>(), std::vector<size_t>(), std::vector<size_t>());
            }
            assert(stack.size() + 1 == paths.size());
            // Get path in goal automaton (returned in the end).
            std::vector<size_t> goal_path;
            std::vector<uint32_t> goal_stack;
            goal_path.reserve(paths.size());
            goal_stack.reserve(stack.size());
            for (size_t i = 0; i < stack.size(); ++i) {
                if (stack[i] != std::numeric_limits<uint32_t>::max()) {
                    goal_stack.push_back(stack[i]);
                    goal_path.push_back(paths[i].second);
                }
            }
            goal_path.push_back(paths.back().second);
            // Build up stack of edges in the PAutomaton. Each PDA rule corresponds to changing some of the top edges.
            AutomatonPath automaton_path(paths.back().first);
            for (size_t i = stack.size(); i > 0; --i) {
                automaton_path.emplace(paths[i - 1].first, stack[i - 1]);
            }

            details::TraceBack tb(automaton, std::move(automaton_path));
            std::vector<rule_t> trace;
            while (auto rule = tb.next()) {
                trace.emplace_back(rule.value());
            }

            // Get accepting path of initial stack (and the initial stack itself - for post*)
            auto [start_path, start_stack] = tb.path().get_path_and_stack();

            if (tb.post()) { // post* was used
                std::reverse(trace.begin(), trace.end());
                return std::make_tuple(trace[0].from(), trace, start_stack, goal_stack, start_path, goal_path);
            } else { // pre* was used
                return std::make_tuple(paths[0].first, trace, goal_stack, start_stack, goal_path, start_path);
            }
        }

        template <typename W, TraceInfoType trace_info_type1, TraceInfoType trace_info_type2>
        static std::tuple<
                size_t, // Initial state. (State is size_t::max if no trace exists.)
                std::vector<user_rule_t<W>>, // Sequence of rules applied to the initial configuration to reach the final configuration.
        std::vector<uint32_t>, // Initial stack
        std::vector<uint32_t>, // Final stack
        std::vector<size_t>, // Path in initial PAutomaton (accepting initial stack)
        std::vector<size_t>  // Path in final PAutomaton (accepting final stack) (independent of whether pre* or post* was used)
        > _get_rule_trace_and_paths(const PAutomaton<W,trace_info_type1>& initial_automaton, const PAutomaton<W,trace_info_type2>& final_automaton,
                                    const std::vector<std::pair<size_t,size_t>>& paths, // The paths as retrieved from the product automaton. First number is the state in initial_automaton, second number is state in final_automaton.
                                    const std::vector<uint32_t>& stack) {
            using rule_t = user_rule_t<W>;

            if (paths.empty()) {
                return std::make_tuple(std::numeric_limits<size_t>::max(), std::vector<rule_t>(), std::vector<uint32_t>(), std::vector<uint32_t>(), std::vector<size_t>(), std::vector<size_t>());
            }
            assert(stack.size() + 1 == paths.size());
            // Build up stack of edges in the PAutomaton. Each PDA rule corresponds to changing some of the top edges.
            AutomatonPath initial_automaton_path(paths.back().first);
            for (size_t i = stack.size(); i > 0; --i) {
                initial_automaton_path.emplace(paths[i - 1].first, stack[i - 1]);
            }
            auto [trace, initial_stack, initial_path] = _get_trace_stack_path(initial_automaton, std::move(initial_automaton_path));

            AutomatonPath final_automaton_path(paths.back().second);
            for (size_t i = stack.size(); i > 0; --i) {
                final_automaton_path.emplace(paths[i - 1].second, stack[i - 1]);
            }
            auto [trace2, final_stack, final_path] = _get_trace_stack_path(final_automaton, std::move(final_automaton_path));
            // Concat traces
            trace.insert(trace.end(), trace2.begin(), trace2.end());
            return std::make_tuple(trace[0].from(), trace, initial_stack, final_stack, initial_path, final_path);
        }

        template <typename W, TraceInfoType trace_info_type>
        static std::tuple<std::vector<user_rule_t<W>>, std::vector<uint32_t>, std::vector<size_t>>
        _get_trace_stack_path(const PAutomaton<W,trace_info_type>& automaton, AutomatonPath&& automaton_path) {
            std::vector<user_rule_t<W>> trace;
            details::TraceBack tb(automaton, std::move(automaton_path));
            while(auto rule = tb.next()) {
                trace.emplace_back(rule.value());
            }
            if (tb.post()) {
                std::reverse(trace.begin(), trace.end());
            }
            // Get accepting path of initial stack (and the initial stack itself - for post*)
            auto [path, stack] = tb.path().get_path_and_stack();
            return {trace, stack, path};
        }
    };

}

#endif //PDAAAL_SOLVER_H
