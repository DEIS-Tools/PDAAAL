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
 * File:   PdaSaturation.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 18-02-2022.
 */

#ifndef PDAAAL_PDASATURATION_H
#define PDAAAL_PDASATURATION_H

#include "PAutomaton.h"
#include "pdaaal/utils/workset.h"

namespace pdaaal::internal {
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
    template <typename W>
    using early_termination_fn2 = std::function<bool(size_t,uint32_t,size_t,edge_annotation_t<W>,std::conditional_t<W::is_weight, typename W::type, bool> const&)>;

    template <typename W, bool ET=false, bool SHORTEST=false>
    class PreStarSaturation {
    private:
        using weight_t = std::conditional_t<W::is_weight,typename W::type,int>;
        using solver_weight = min_weight<typename W::type>;

        struct weighted_edge_t : public temp_edge_t { // maybe reuse for poststar?
            weighted_edge_t(size_t from, uint32_t label, size_t to, weight_t&& weight, trace_t trace)
            : temp_edge_t(from, label, to), _weight(std::move(weight)), _trace(trace) {}
            weight_t _weight;
            trace_t _trace;
        };

        struct weighted_edge_t_comp {
            bool operator()(const weighted_edge_t &lhs, const weighted_edge_t &rhs){
                if constexpr (W::is_weight) return solver_weight::less(rhs._weight, lhs._weight);
                else return false;
            }
        };

        static inline const weight_t& get_weight(const pda_rule_t<W>& rule) {
            static const int dummy = 1;
            if constexpr(W::is_weight) return rule._weight;
            else return dummy;
        }


        using trace_info = TraceInfo<TraceInfoType::Single>;
        using edge_anno = edge_annotation<W,TraceInfoType::Single>;
        using edge_anno_t = edge_annotation_t<W,TraceInfoType::Single>;
        using p_automaton_t = PAutomaton<W>;
        using edge_t = std::conditional_t<SHORTEST,weighted_edge_t,temp_edge_t>;
        using workset_t = std::conditional_t<SHORTEST,
                                                    std::priority_queue<weighted_edge_t,std::vector<weighted_edge_t>,weighted_edge_t_comp>,
                                                    std::stack<temp_edge_t>
                                          >;

    public:
        explicit PreStarSaturation(PAutomaton<W> &automaton, const early_termination_fn2<W>& early_termination)
                : _automaton(automaton), _early_termination(early_termination), _pda_states(_automaton.pda().states()),
                  _n_pda_states(_pda_states.size()), _n_automaton_states(_automaton.states().size()),
                  _n_pda_labels(_automaton.number_of_labels()), _rel(_n_automaton_states),
                  _delta_prime(_n_automaton_states),_popped(_n_pda_states) {
            initialize();
        };

        // This is an implementation of Algorithm 1 (figure 3.3) in:
        // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
        // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 42)

        p_automaton_t& _automaton;
        const early_termination_fn2<W>& _early_termination;
        const std::vector<typename PDA<W>::state_t>& _pda_states;
        const size_t _n_pda_states;
        const size_t _n_automaton_states;
        const size_t _n_pda_labels;
        workset_t _workset;
        std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel;
        std::vector<std::vector<std::pair<size_t, size_t>>> _delta_prime;
        bool _found = false;
        std::vector<weight_t> _minpath;
        std::vector<bool> _popped;
        std::unordered_set<temp_edge_t,absl::Hash<temp_edge_t>> _seen;

        bool has_negative_weight() const {
            if constexpr (W::is_signed) {
                for (const auto& state : _pda_states)
                    if(state.has_negative_weight(solver_weight::less)) return true;
                if(_automaton.has_negative_weights(solver_weight::less)) return true;
            }
            return false;
        }

        void add_pop(size_t state) {
            if(state >= _n_pda_states) return;
            if(_popped[state]) return;
            for (auto pre_state : _pda_states[state]._pre_states) {
                const auto& rules = _pda_states[pre_state]._rules;
                auto lb = rules.lower_bound(pda_rule_t<W>{state});
                while (lb != rules.end() && lb->first._to == state) {
                    const auto& [rule, labels] = *lb;
                    size_t rule_id = lb - rules.begin();
                    ++lb;
                    if(rule._operation == POP)
                    {
                        if constexpr (W::is_weight && SHORTEST)
                            insert_edge_bulk(pre_state, labels, state, p_automaton_t::new_pre_trace(rule_id), rule._weight);
                        else
                            insert_edge_bulk(pre_state, labels, state, p_automaton_t::new_pre_trace(rule_id), weight_t{});
                    }
                }
            }
            _popped[state] = true;
        }

        void initialize() {
            if constexpr (SHORTEST && W::is_weight) {
                _minpath.resize(_automaton.states().size());
                std::fill(_minpath.begin(), _minpath.end(), solver_weight::max());
            }

            // workset := ->_0  (line 1)
            for (const auto& from : _automaton.states()) {
                for (const auto& [to, labels] : from->_edges) {
                    for (const auto& [label, _] : labels) {
                        if constexpr (W::is_weight && SHORTEST) {
                            // anything inside the automaton is reachable with weight zero.
                            _minpath[from->_id] = solver_weight::zero();
                            _minpath[to] = solver_weight::zero();
                            _workset.emplace(from->_id, label, to, solver_weight::zero(), trace_t());
                        } else if constexpr (!SHORTEST) {
                            _workset.emplace(from->_id, label, to);
                        } else throw std::logic_error("Shortest traces for void-weights is not supported.");
                    }
                }
                if(from->_accepting) add_pop(from->_id);
            }
        }


        void insert_edge(size_t from, uint32_t label, size_t to, trace_t trace, [[maybe_unused]] weight_t weight) {

            if constexpr (SHORTEST && W::is_weight) {
                auto res = solver_weight::add(weight, _minpath[to]);
                assert(res != solver_weight::max());
                assert(weight != solver_weight::max());

                auto [it, fresh] = _automaton.emplace_edge(from, label, to, std::make_pair(trace, weight));
                if (fresh) { // New edge is not already in edges (rel U workset).
                    if(solver_weight::less(res, _minpath[from])) _minpath[from] = res;
                    _workset.emplace(from, label, to, std::move(res), trace);
                } else {
                    if(solver_weight::less(res, _minpath[from])) {
                        // we need to bump it up on the queue
                        _minpath[from] = res;
                        _workset.emplace(from, label, to, std::move(res), trace);
                    }
                    if(solver_weight::less(weight, it->second.second)) {
                        // update the weight (but no need to re-add to queue)
                        it->second.second = std::move(weight);
                    }
                }
            } else {
                if (_automaton.emplace_edge(from, label, to, edge_anno::from_trace_info(trace)).second) {
                    _workset.emplace(from, label, to);
                    if constexpr (ET) {
                        _found = _found || _early_termination(from, label, to, edge_anno::from_trace_info(trace), weight_t{});
                    }
                }
            }
        }

        void insert_edge_bulk(size_t from, const labels_t& labels, size_t to, trace_t trace, weight_t weight) {
            if (labels.wildcard()) {
                for (uint32_t i = 0; i < _n_pda_labels; i++) {
                    insert_edge(from, i, to, trace, weight);
                }
            } else {
                for (auto label : labels.labels()) {
                    insert_edge(from, label, to, trace, weight);
                }
            }
        }

        weight_t get_pautomata_edge_weight(auto from, auto label, auto to)
        {
            if constexpr (ET && W::is_weight)
                return  _automaton.get_edge(from, label, to)->second;
            return weight_t{};
        }


    public:

        void step() {
            // pop t = (q, y, q') from workset (line 4)
            auto t = _workset.top();
            _workset.pop();

            [[maybe_unused]] const auto w = get_pautomata_edge_weight(t._from, t._label, t._to);

            if constexpr (SHORTEST && W::is_weight) {
                if(!_seen.emplace(t).second)
                    return;
                assert(t._weight != solver_weight::max());
                if constexpr (ET) {
                    _found = _early_termination(t._from, t._label, t._to, std::make_pair(t._trace, w), t._weight) || _found;
                }
            }
            // rel = rel U {t} (line 6)   (membership test on line 5 is done in insert_edge).
            _rel[t._from].emplace_back(t._to, t._label);
            // (line 7-8 for \Delta')
            for (const auto& [state, rule_id] : _delta_prime[t._from]) { // Loop over delta_prime (that match with t->from)
                auto& [rule, labels] = _pda_states[state]._rules[rule_id];
                if (labels.contains(t._label)) {
                    if constexpr (W::is_weight && SHORTEST) {
                        auto nw = solver_weight::add(solver_weight::add(rule._weight, _automaton.get_edge(rule._to, rule._op_label, t._from)->second), w);
                        insert_edge(state, t._label, t._to, p_automaton_t::new_pre_trace(rule_id, t._from), nw);
                    } else {
                        insert_edge(state, t._label, t._to, p_automaton_t::new_pre_trace(rule_id, t._from), weight_t{});
                    }
                }
            }

            // Loop over \Delta (filter rules going into q) (line 7 and 9)
            if (t._from >= _n_pda_states) { return; }
            for (auto pre_state : _pda_states[t._from]._pre_states) {
                const auto& rules = _pda_states[pre_state]._rules;
                auto lb = rules.lower_bound(pda_rule_t<W>{t._from});
                while (lb != rules.end() && lb->first._to == t._from) {
                    const auto& [rule, labels] = *lb;
                    size_t rule_id = lb - rules.begin();
                    ++lb;
                    switch (rule._operation) {
                        case POP:
                            // check if weight is lower here, maybe?
                            if (!_popped[t._from]) {
                                if constexpr (W::is_weight && SHORTEST)
                                    insert_edge_bulk(pre_state, labels, t._from, p_automaton_t::new_pre_trace(rule_id), rule._weight);
                                else
                                    insert_edge_bulk(pre_state, labels, t._from, p_automaton_t::new_pre_trace(rule_id), weight_t{});
                            }
                            break;
                        case SWAP: // (line 7-8 for \Delta)
                            if (rule._op_label == t._label) {
                                if constexpr (W::is_weight && SHORTEST)
                                    insert_edge_bulk(pre_state, labels, t._to, p_automaton_t::new_pre_trace(rule_id), solver_weight::add(w, rule._weight));
                                else
                                    insert_edge_bulk(pre_state, labels, t._to, p_automaton_t::new_pre_trace(rule_id), weight_t{});
                            }
                            break;
                        case NOOP: // (line 7-8 for \Delta)
                            if (labels.contains(t._label)) {
                                if constexpr (W::is_weight && SHORTEST)
                                    insert_edge(pre_state, t._label, t._to, p_automaton_t::new_pre_trace(rule_id), solver_weight::add(w, rule._weight));
                                else
                                    insert_edge(pre_state, t._label, t._to, p_automaton_t::new_pre_trace(rule_id), weight_t{});
                            }
                            break;
                        case PUSH: // (line 9)
                            if (rule._op_label == t._label) {
                                // (line 10)
                                _delta_prime[t._to].emplace_back(pre_state, rule_id);
                                for (const auto& [to, label] : _rel[t._to]) { // (line 11-12)
                                    if (labels.contains(label)) {
                                        if constexpr (W::is_weight && SHORTEST) {
                                            auto nw = solver_weight::add(solver_weight::add(_automaton.get_edge(t._to, label, to)->second, w), rule._weight);
                                            insert_edge(pre_state, label, to, p_automaton_t::new_pre_trace(rule_id, t._to), nw);
                                        } else
                                            insert_edge(pre_state, label, to, p_automaton_t::new_pre_trace(rule_id, t._to), weight_t{});
                                    }
                                }
                            }
                            break;
                        default:
                            assert(false);
                    }
                }
            }
            _popped[t._from] = true;

            if constexpr (ET && SHORTEST && W::is_weight)
            {
                // flush the solution in case we got a trace, but the intersection has no trace shorter than the "longest shortest trace" in the pautomata
                if(workset_empty())
                    _found = _early_termination(t._from, t._label, t._to, std::make_pair(t._trace, w), solver_weight::max());
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
        using p_automaton_t = PAutomaton<W>;
        static constexpr auto epsilon = p_automaton_t::epsilon;
    public:
        explicit PostStarSaturation(p_automaton_t& automaton, const early_termination_fn<W>& early_termination = [](size_t, uint32_t, size_t, edge_annotation_t<W>) -> bool { return false; })
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
        std::queue<temp_edge_t> _workset;
        std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel1; // faster access for lookup _from -> (_to, _label)
        std::vector<std::vector<size_t>> _rel2; // faster access for lookup _to -> _from  (when _label is epsilon)

        bool _found = false;

        void initialize() {
            // for <p, y> -> <p', y1 y2> do  (line 3)
            //   Q' U= {q_p'y1}              (line 4)
            for (const auto& state : _pda_states) {
                for (const auto& [rule, labels] : state._rules) {
                    if (rule._operation == PUSH) {
                        auto [it, fresh] = _q_prime.emplace(std::make_pair(rule._to, rule._op_label), _automaton.next_state_id());
                        if (fresh) {
                            _automaton.add_state(false, false);
                        }
                    }
                }
            }
            _n_automaton_states = _automaton.states().size();
            _rel1.resize(_n_automaton_states - _n_pda_states);
            _rel2.resize(_n_automaton_states - _n_Q);

            // workset := ->_0 intersect (P x Gamma x Q)  (line 1)
            // rel := ->_0 \ workset (line 2)
            for (const auto& from : _automaton.states()) {
                for (const auto& [to, labels] : from->_edges) {
                    assert(!labels.contains(epsilon)); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (const auto& [label, _] : labels) {
                        if (from->_id < _n_pda_states) {
                            _workset.emplace(from->_id, label, to);
                        } else {
                            insert_rel(from->_id, label, to);
                        }
                    }
                }
            }
        }

        void insert_rel(size_t from, uint32_t label, size_t to) {
            if (from >= _n_pda_states) {
                _rel1[from - _n_pda_states].emplace_back(to, label);
            }
            if (label == epsilon && to >= _n_Q) {
                _rel2[to - _n_Q].push_back(from);
            }
        }
        void insert_edge(size_t from, uint32_t label, size_t to, trace_t trace) {
            if (_automaton.emplace_edge(from, label, to, edge_anno::from_trace_info(trace)).second) {
                _workset.emplace(from, label, to);
                if constexpr (ET) {
                    _found = _found || _early_termination(from, label, to, edge_anno::from_trace_info(trace));
                }
            }
        }
        void insert_edge_rel(size_t from, uint32_t label, size_t to, trace_t trace) {
            if (_automaton.emplace_edge(from, label, to, edge_anno::from_trace_info(trace)).second) {
                insert_rel(from, label, to);
                if constexpr (ET) {
                    _found = _found || _early_termination(from, label, to, edge_anno::from_trace_info(trace));
                }
            }
        }

    public:
        void step() {
            // pop t = (q, y, q') from workset (line 6)
            temp_edge_t t;
            t = _workset.front();
            _workset.pop();
            assert(t._to >= _n_pda_states); // This should be an invariant of post*
            // rel = rel U {t} (line 8)   (membership test on line 7 is done in insert_edge).
            insert_rel(t._from, t._label, t._to);

            // if y != epsilon (line 9)
            if (t._label != epsilon) {
                const auto& rules = _pda_states[t._from]._rules;
                for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                    const auto& [rule, labels] = rules[rule_id];
                    if (!labels.contains(t._label)) { continue; }
                    auto trace = p_automaton_t::new_post_trace(t._from, rule_id, t._label);
                    switch (rule._operation) {
                        case POP: // (line 10-11)
                            insert_edge(rule._to, epsilon, t._to, trace);
                            break;
                        case SWAP: // (line 12-13)
                            insert_edge(rule._to, rule._op_label, t._to, trace);
                            break;
                        case NOOP:
                            insert_edge(rule._to, t._label, t._to, trace);
                            break;
                        case PUSH: // (line 14)
                            assert(_q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(_q_prime));
                            size_t q_new = _q_prime[std::make_pair(rule._to, rule._op_label)];
                            insert_edge(rule._to, rule._op_label, q_new, trace); // (line 15)
                            insert_edge_rel(q_new, t._label, t._to, trace); // (line 16)
                            if (!_rel2[q_new - _n_Q].empty()) {
                                auto trace_q_new = p_automaton_t::new_post_trace(q_new);
                                for (auto f : _rel2[q_new - _n_Q]) { // (line 17)
                                    insert_edge(f, t._label, t._to, trace_q_new); // (line 18)
                                }
                            }
                            break;
                    }
                }
            } else {
                for (const auto& [to, label] : _rel1[t._to - _n_pda_states]) { // (line 20)
                    assert(to >= _n_pda_states);
                    insert_edge(t._from, label, to, p_automaton_t::new_post_trace(t._to)); // (line 21)
                }
            }
        }
        [[nodiscard]] bool workset_empty() const {
            return _workset.empty();
        }
        [[nodiscard]] bool found() const {
            return _found;
        }
        bool run() {
            while(!workset_empty()) {
                if constexpr (ET) {
                    if (found()) return true;
                }
                step();
            }
            return found();
        }
    };

    template<typename W, bool Enable, bool ET, typename = std::enable_if_t<Enable>>
    class PostStarShortestSaturation {
        static_assert(W::is_weight);
        using solver_weight = min_weight<typename W::type>;

        using trace_info = TraceInfo<TraceInfoType::Single>;
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

    public:
        explicit PostStarShortestSaturation(p_automaton_t& automaton, const early_termination_fn2<W>& early_termination = [](size_t, uint32_t, size_t, edge_annotation_t<W>,auto) -> bool { return false; })
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
        const early_termination_fn2<W>& _early_termination;
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

        bool _found = false;

        bool has_negative_weight() const {
            if constexpr (W::is_signed) {
                for (const auto& state : _pda_states)
                    if(state.has_negative_weight(solver_weight::less)) return true;
                if(_automaton.has_negative_weights(solver_weight::less)) return true;
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
            std::fill(_minpath.begin(), _minpath.end(), solver_weight::max());

            _rel1.resize(_n_automaton_states - _n_pda_states);
            _rel2.resize(_n_automaton_states - _n_Q);

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
                                _found |= _early_termination(from->_id, label, to, trace, W::zero());
                            }
                        }
                    }
                }
            }
        }

        std::pair<bool,bool> update_edge_(size_t from, uint32_t label, size_t to, typename W::type edge_weight, typename W::type workset_weight) {
            auto [it, fresh] = _edge_weights.emplace(temp_edge_t{from, label, to}, std::make_pair(edge_weight, workset_weight));
            if (!fresh) {
                auto result = std::make_pair(false, false);
                if (solver_weight::less(edge_weight, it->second.first)) {
                    it->second.first = edge_weight;
                    result.first = true;
                }
                if (solver_weight::less(workset_weight, it->second.second)) {
                    it->second.second = workset_weight;
                    result.second = true;
                }
                return result;
            }
            return std::make_pair(fresh, fresh);
        }
        void update_edge(size_t from, uint32_t label, size_t to, typename W::type edge_weight, trace_t trace) {
            auto workset_weight = to < _n_Q ? edge_weight : solver_weight::add(_minpath[to - _n_Q], edge_weight);
            if (update_edge_(from, label, to, edge_weight, workset_weight).second) {
                _workset.emplace(workset_weight, temp_edge_t{from, label, to}, trace);
            }
        }
        void insert_rel(size_t from, uint32_t label, size_t to) { // Adds to rel.
            if (from >= _n_pda_states) {
                _rel1[from - _n_pda_states].emplace_back(to, label);
            }
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
            auto weights = _edge_weights.find(t)->second;
            if (solver_weight::less(weights.second, elem._weight)) {
                return; // Same edge with a smaller weight was already processed.
            }
            auto t_weight = weights.first;
            assert(t._to >= _n_pda_states); // This should be an invariant of post*

            // rel = rel U {t}
            insert_rel(t._from, t._label, t._to);
            if (t._label == epsilon) {
                _automaton.add_epsilon_edge(t._from, t._to, std::make_pair(elem._trace, t_weight));
            } else {
                _automaton.add_edge(t._from, t._to, t._label, std::make_pair(elem._trace, t_weight));
            }
            if constexpr (ET) {
                _found |= _early_termination(t._from, t._label, t._to, std::make_pair(elem._trace, t_weight), elem._weight);
            }
            if (t._from >= _n_pda_states) {
                assert(t._from >= _n_Q);
                for (auto f : _rel2[t._from - _n_Q]) {
                    assert(_automaton.get_edge(f, epsilon, t._from) != nullptr);
                    update_edge(f, t._label, t._to, solver_weight::add(t_weight, _automaton.get_edge(f, epsilon, t._from)->second), p_automaton_t::new_post_trace(t._from));
                }
                return;
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
                    switch(rule._operation) {
                        case POP:
                            update_edge(rule._to, epsilon, t._to, wb, trace);
                            break;
                        case SWAP:
                            update_edge(rule._to, rule._op_label, t._to, wb, trace);
                            break;
                        case NOOP:
                            update_edge(rule._to, t._label, t._to, wb, trace);
                            break;
                        case PUSH: {
                            assert(_q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(_q_prime));
                            size_t q_new = _q_prime[std::make_pair(rule._to, rule._op_label)];
                            auto add_to_workset = update_edge_(rule._to, rule._op_label, q_new, W::zero(), wd).second;
                            if (solver_weight::less(wd, _minpath[q_new - _n_Q])) {
                                _minpath[q_new - _n_Q] = wd;
                                if (add_to_workset) {
                                    _workset.emplace(wd, temp_edge_t{rule._to, rule._op_label, q_new}, trace);
                                }
                            }
                            update_edge(q_new, t._label, t._to, wb, trace); // Add to queue, since we do Dijkstra -> Transitions in Pautomaton and product are only added once they are on the search-frontier.
                            break;
                        }
                    }
                }
            } else {
                for (const auto& [to, label] : _rel1[t._to - _n_pda_states]) {
                    assert(to >= _n_pda_states);
                    assert(_automaton.get_edge(t._to, label, to) != nullptr);
                    update_edge(t._from, label, to, solver_weight::add(_automaton.get_edge(t._to, label, to)->second, t_weight), p_automaton_t::new_post_trace(t._to));
                }
            }
        }
        [[nodiscard]] bool workset_empty() const {
            return _workset.empty();
        }
        [[nodiscard]] bool found() const {
            return _found;
        }
        bool run() {
            while(!workset_empty()) {
                if constexpr (ET) {
                    if (found()) return true;
                }
                step();
            }
            return found();
        }
    };

    template<typename W, Trace_Type trace_type>
    class PreStarFixedPointSaturation : public fixed_point_workset<PreStarFixedPointSaturation<W,trace_type>, temp_edge_t> {
        using parent_t = fixed_point_workset<PreStarFixedPointSaturation<W,trace_type>, temp_edge_t>;
        static_assert(W::is_weight);
        using weight_t = typename W::type;
        using solverW = solver_weight<W,trace_type>;

        using trace_info = TraceInfo<TraceInfoType::Pair>;
        using p_automaton_t = PAutomaton<W,TraceInfoType::Pair>;

        template<bool change_is_bottom = false>
        void update_edge(size_t from, uint32_t label, size_t to, const weight_t& edge_weight, trace_t trace) {
            bool is_changed = false;
            if constexpr(change_is_bottom) {
                auto ptr = _automaton.get_edge(from, label, to);
                assert(ptr != nullptr);
                if (solverW::less(edge_weight, ptr->second)) {
                    ptr->first.second = trace; // We update the second trace_info in the pair, keeping the first one unchanged.
                    ptr->second = solverW::bottom(); // Update weight.
                    is_changed = true;
                }
            } else {
                auto [it, fresh] = _automaton.emplace_edge(from, label, to, std::make_pair(std::make_pair(trace,trace), edge_weight));
                if (fresh) {
                    ++_count_transitions; parent_t::set_round_limit(_count_transitions);
                    _rel[from].emplace_back(to, label); // Allow fast iteration over _edges matching specific from.
                    is_changed = true;
                } else {
                    if (solverW::less(edge_weight, it->second.second)) {
                        it->second.first.second = trace;
                        it->second.second = edge_weight;
                        is_changed = true;
                    }
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
    private:
        p_automaton_t& _automaton;
        const std::vector<typename PDA<W>::state_t>& _pda_states;
        const size_t _n_automaton_states;
        const size_t _n_pda_states;
        const size_t _n_pda_labels;

        std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel; // Fast access to _edges based on _from.
        std::vector<fut::vector_set<std::pair<size_t, size_t>>> _delta_prime;

        size_t _count_transitions = 0;

        void initialize() {
            for (const auto& from : _automaton.states()) {
                for (const auto& [to,labels] : from->_edges) {
                    for (const auto& [label,tw] : labels) {
                        assert(tw == std::make_pair(trace_info::make_default(), W::zero()));
                        _rel[from->_id].emplace_back(to, label); // Allow fast iteration over _edges matching specific from.
                        parent_t::emplace(from->_id, label, to);
                        ++_count_transitions;
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
            parent_t::set_round_limit(_count_transitions);
        }

    public:
        template<bool change_is_bottom = false>
        bool step_with(temp_edge_t&& t) {
            assert(_automaton.get_edge(t._from, t._label, t._to) != nullptr);
            auto w = _automaton.get_edge(t._from, t._label, t._to)->second;

            // (line 7-8 for \Delta')
            for (const auto& [state, rule_id] : _delta_prime[t._from]) { // Loop over delta_prime (that match with t->from)
                const auto& [rule, labels] = _pda_states[state]._rules[rule_id];
                if (labels.contains(t._label)) {
                    assert(_automaton.get_edge(rule._to, rule._op_label, t._from) != nullptr);
                    update_edge<change_is_bottom>(state, t._label, t._to,
                                                  solverW::add(solverW::add(rule._weight, _automaton.get_edge(rule._to, rule._op_label, t._from)->second), w),
                                                  p_automaton_t::new_pre_trace(rule_id, t._from));
                }
            }

            if (t._from >= _n_pda_states) { return true; }
            for (auto pre_state : _pda_states[t._from]._pre_states) {
                const auto &rules = _pda_states[pre_state]._rules;
                auto lb = rules.lower_bound(pda_rule_t<W>{t._from});
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
                                _delta_prime[t._to].emplace(pre_state, rule_id);
                                for (const auto& [rel_to, rel_label] : _rel[t._to]) { // (line 11-12)
                                    if (labels.contains(rel_label)) {
                                        assert(_automaton.get_edge(t._to, rel_label, rel_to) != nullptr);
                                        update_edge<change_is_bottom>(pre_state, rel_label, rel_to,
                                                solverW::add(w_temp, _automaton.get_edge(t._to, rel_label, rel_to)->second),
                                                p_automaton_t::new_pre_trace(rule_id, t._to));
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

    template<typename W, Trace_Type trace_type>
    class PostStarFixedPointSaturation : public fixed_point_workset<PostStarFixedPointSaturation<W,trace_type>, temp_edge_t> {
        using parent_t = fixed_point_workset<PostStarFixedPointSaturation<W,trace_type>, temp_edge_t>;
        static_assert(W::is_weight);
        using weight_t = typename W::type;
        using solverW = solver_weight<W,trace_type>;
        using trace_info = TraceInfo<TraceInfoType::Pair>;
        using p_automaton_t = PAutomaton<W,TraceInfoType::Pair>;

    public:
        explicit PostStarFixedPointSaturation(p_automaton_t& automaton)
        : parent_t(std::pow(automaton.states().size(), 2) * automaton.number_of_labels()),
          _automaton(automaton), _pda_states(_automaton.pda().states()),
          _n_pda_states(_pda_states.size()), _n_Q(_automaton.states().size()) {
            initialize();
        };
        PostStarFixedPointSaturation(p_automaton_t& automaton, size_t round_limit)
        : parent_t(round_limit),
          _automaton(automaton), _pda_states(_automaton.pda().states()),
          _n_pda_states(_pda_states.size()), _n_Q(_automaton.states().size()) {
            initialize();
        };
    private:
        p_automaton_t& _automaton;
        const std::vector<typename PDA<W>::state_t>& _pda_states;
        const size_t _n_pda_states;
        const size_t _n_Q;
        std::unordered_map<std::pair<size_t, uint32_t>, size_t, absl::Hash<std::pair<size_t, uint32_t>>> _q_prime{};

        size_t _n_automaton_states{};
        std::vector<std::vector<std::pair<size_t,uint32_t>>> _rel1; // faster access for lookup _from -> (_to, _label)
        std::vector<std::vector<size_t>> _rel2; // faster access for lookup _to -> _from  (when _label is epsilon)

        size_t _count_transitions = 0;

        void initialize() {
            // for <p, y> -> <p', y1 y2> do  (line 3)
            //   Q' U= {q_p'y1}              (line 4)
            for (const auto& state : _pda_states) {
                for (const auto& [rule, labels] : state._rules) {
                    if (rule._operation == PUSH) {
                        auto [it, fresh] = _q_prime.emplace(std::make_pair(rule._to, rule._op_label), _automaton.next_state_id());
                        if (fresh) {
                            _automaton.add_state(false, false);
                        }
                    }
                }
            }

            _n_automaton_states = _automaton.states().size();
            _rel1.resize(_n_automaton_states - _n_pda_states);
            _rel2.resize(_n_automaton_states - _n_Q);

            // workset := ->_0 intersect (P x Gamma x Q)  (line 1)
            // rel := ->_0 \ workset (line 2)
            for (const auto& from : _automaton.states()) {
                for (const auto& [to, labels] : from->_edges) {
                    assert(!labels.contains(epsilon)); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (const auto& [label, _] : labels) {
                        if (from->_id < _n_pda_states) {
                            parent_t::emplace(from->_id, label, to);
                            ++_count_transitions;
                        } else {
                            insert_rel(from->_id, label, to);
                        }
                    }
                }
            }
            parent_t::set_round_limit(_count_transitions);
        }

        void insert_rel(size_t from, uint32_t label, size_t to) {
            if (from >= _n_pda_states) {
                _rel1[from - _n_pda_states].emplace_back(to, label);
            }
            if (label == epsilon && to >= _n_Q) {
                _rel2[to - _n_Q].push_back(from);
            }
        }

        template<bool change_is_bottom = false>
        bool update_edge(size_t from, uint32_t label, size_t to, const weight_t& edge_weight, trace_t trace) {
            bool is_changed = false;
            if constexpr(change_is_bottom) {
                auto ptr = _automaton.get_edge(from, label, to);
                assert(ptr != nullptr);
                if (solverW::less(edge_weight, ptr->second)) {
                    ptr->first.second = trace; // We update the second trace_info in the pair, keeping the first one unchanged.
                    ptr->second = solverW::bottom(); // Update weight.
                    is_changed = true;
                }
            } else {
                auto [it, fresh] = _automaton.emplace_edge(from, label, to, std::make_pair(std::make_pair(trace,trace), edge_weight));
                if (fresh) {
                    ++_count_transitions; parent_t::set_round_limit(_count_transitions);
                    insert_rel(from, label, to);
                    is_changed = true;
                } else if (solverW::less(edge_weight, it->second.second)) {
                    it->second.first.second = trace;
                    it->second.second = edge_weight;
                    is_changed = true;
                }
            }
            if (is_changed && from < _n_pda_states) {
                parent_t::emplace(from, label, to);
            }
            return is_changed;
        }

    public:
        template<bool change_is_bottom = false>
        bool step_with(temp_edge_t&& t) {
            assert(t._from < _n_pda_states); // This should be an invariant of the way we use the workset.
            assert(t._to >= _n_pda_states); // This should be an invariant of post*

            assert(_automaton.get_edge(t._from, t._label, t._to) != nullptr);
            auto w = _automaton.get_edge(t._from, t._label, t._to)->second;

            if (t._label != epsilon) {
                const auto& rules = _pda_states[t._from]._rules;
                for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                    const auto& [rule, labels] = rules[rule_id];
                    if (!labels.contains(t._label)) { continue; }
                    auto trace = p_automaton_t::new_post_trace(t._from, rule_id, t._label);
                    auto wb = solverW::add(w, rule._weight);
                    switch (rule._operation) {
                        case POP:
                            update_edge<change_is_bottom>(rule._to, epsilon, t._to, wb, trace);
                            break;
                        case SWAP:
                            update_edge<change_is_bottom>(rule._to, rule._op_label, t._to, wb, trace);
                            break;
                        case NOOP:
                            update_edge<change_is_bottom>(rule._to, t._label, t._to, wb, trace);
                            break;
                        case PUSH: {
                            assert(_q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(_q_prime));
                            size_t q_new = _q_prime[std::make_pair(rule._to, rule._op_label)];
                            update_edge<change_is_bottom>(rule._to, rule._op_label, q_new, W::zero(), trace);
                            if (update_edge<change_is_bottom>(q_new, t._label, t._to, wb, trace)) {
                                assert(q_new >= _n_Q);
                                for (auto f: _rel2[q_new - _n_Q]) {
                                    assert(_automaton.get_edge(f, epsilon, q_new) != nullptr);
                                    update_edge<change_is_bottom>(f, t._label, t._to, solverW::add(wb, _automaton.get_edge(f, epsilon, q_new)->second), p_automaton_t::new_post_trace(q_new));
                                }
                            }
                            break;
                        }
                    }
                }
            } else {
                for (const auto& [to, label]: _rel1[t._to - _n_pda_states]) {
                    assert(_automaton.get_edge(t._to, label, to) != nullptr);
                    update_edge<change_is_bottom>(t._from, label, to, solverW::add(_automaton.get_edge(t._to, label, to)->second, w), p_automaton_t::new_post_trace(t._to));
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
        TraceBack(const p_automaton_t& automaton, AutomatonPath<>&& path)
                : _automaton(automaton), _path(std::move(path)) { };
    private:
        const p_automaton_t& _automaton;
        AutomatonPath<> _path;
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
        [[nodiscard]] const AutomatonPath<>& path() const { return _path; }
        [[nodiscard]] std::optional<std::tuple<size_t,uint32_t,size_t>> current_state() const {
            if (_path.empty()) {
                return std::nullopt;
            } else {
                return _path.front_edge();
            }
        }
        template <bool avoid_loop = false>
        std::optional<rule_t> next() {
            while(true) { // In case of post_epsilon_trace, keep going until a rule is found, or we are done.
                if (_path.empty()) return std::nullopt;
                auto [from, label, to] = _path.front_edge();
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


#endif //PDAAAL_PDASATURATION_H
