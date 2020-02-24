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

    struct label_with_trace_t {
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

    template <typename W = void, typename C = std::less<W>>
    class PAutomaton {
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

    public:
        struct state_t;

        struct edge_t {
            state_t *_to;
            std::vector<label_with_trace_t> _labels;

            // edge with a label and optional trace
            edge_t(state_t *to, uint32_t label, const trace_t *trace = nullptr)
                    : _to(to), _labels() {
                _labels.emplace_back(label, trace);
            };

            // epsilon edge with trace
            edge_t(state_t *to, const trace_t *trace) : _to(to), _labels() {
                _labels.emplace_back(trace);
            };

            // wildcard (all labels), no trace
            edge_t(state_t *to, size_t all_labels) : _to(to), _labels() {
                for (uint32_t label = 0; label < all_labels; label++) {
                    _labels.emplace_back(label);
                }
            };

            void add_label(uint32_t label, const trace_t *trace) {
                label_with_trace_t label_trace{label, trace};
                auto lb = std::lower_bound(_labels.begin(), _labels.end(), label_trace);
                if (lb == std::end(_labels) || *lb != label_trace) {
                    _labels.insert(lb, label_trace);
                }
            }

            bool contains(uint32_t label) {
                label_with_trace_t label_trace{label};
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
            std::stack<temp_edge_t> trans;
            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel(_states.size());

            auto insert_edge = [&edges, &trans, this](size_t from, uint32_t label, size_t to, const trace_t *trace) {
                auto res = edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U trans).
                    trans.emplace(from, label, to);
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

            // trans := ->_0  (line 1)
            for (auto &from : this->states()) {
                for (auto &edge : from->_edges) {
                    for (auto &label : edge._labels) {
                        insert_edge(from->_id, label._label, edge._to->_id, nullptr);
                    }
                }
            }

            // for all <p, y> --> <p', epsilon> : trans U= (p, y, p') (line 2)
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

            while (!trans.empty()) { // (line 3)
                // pop t = (q, y, q') from trans (line 4)
                auto t = trans.top();
                trans.pop();
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

        void post_star() {
            // This is an implementation of Algorithm 2 (figure 3.4) in:
            // Schwoon, Stefan. Model-checking pushdown systems. 2002. PhD Thesis. Technische Universität München.
            // http://www.lsv.fr/Publis/PAPERS/PDF/schwoon-phd02.pdf (page 48)
            auto & pda_states = pda().states();
            auto n_pda_states = pda_states.size();

            std::unordered_set<temp_edge_t, temp_edge_hasher> edges;
            std::stack<temp_edge_t> trans;

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

            std::vector<std::vector<std::pair<size_t,uint32_t>>> rel1(_states.size()); // faster access for lookup _from -> (_to, _label)
            std::vector<std::vector<size_t>> rel2(_states.size()); // faster access for lookup _to -> _from  (when _label is uint32_t::max)

            auto insert_edge = [&edges, &trans, &rel1, &rel2, this](size_t from, uint32_t label, size_t to,
                                                                    const trace_t *trace,
                                                                    bool direct_to_rel = false) {
                auto res = edges.emplace(from, label, to);
                if (res.second) { // New edge is not already in edges (rel U trans).
                    if (direct_to_rel) {
                        rel1[from].emplace_back(to, label);
                        if (label == std::numeric_limits<uint32_t>::max()) {
                            rel2[to].push_back(from);
                        }
                    } else {
                        trans.emplace(from, label, to);
                    }
                    if (trace != nullptr) { // Don't add existing edges
                        if (label == std::numeric_limits<uint32_t>::max()) {
                            this->add_epsilon_edge(from, to, trace);
                        } else {
                            this->add_edge(from, to, label, trace);
                        }
                    }
                }
            };

            // trans := ->_0 intersect (P x Gamma x Q)  (line 1)
            // rel := ->_0 \ trans (line 2)
            for (auto &from : this->states()) {
                for (auto &edge : from->_edges) {
                    assert(!edge.has_epsilon()); // PostStar algorithm assumes no epsilon transitions in the NFA.
                    for (auto &label : edge._labels) {
                        insert_edge(from->_id, label._label, edge._to->_id, nullptr, from->_id >= n_pda_states);
                    }
                }
            }

            while (!trans.empty()) { // (line 5)
                // pop t = (q, y, q') from trans (line 6)
                auto t = trans.top();
                trans.pop();
                // rel = rel U {t} (line 8)   (membership test on line 7 is done in insert_edge).
                rel1[t._from].emplace_back(t._to, t._label);
                if (t._label == std::numeric_limits<uint32_t>::max()) {
                    rel2[t._to].push_back(t._from);
                }

                // if y != epsilon (line 9)
                if (t._label != std::numeric_limits<uint32_t>::max()) {
                    const auto &rules = pda_states[t._from]._rules;
                    for (size_t rule_id = 0; rule_id < rules.size(); ++rule_id) {
                        auto &rule = rules[rule_id];
                        if (!rule._labels.contains(t._label)) { continue; }
                        auto trace = this->new_post_trace(t._from, rule_id, t._label);
                        switch (rule._operation) {
                            case POP: // (line 10-11)
                                insert_edge(rule._to, std::numeric_limits<uint32_t>::max(), t._to, trace);
                                break;
                            case SWAP: // (line 12-13)
                                insert_edge(rule._to, rule._op_label, t._to, trace);
                                break;
                            case NOOP:
                                insert_edge(rule._to, t._label, t._to, trace);
                                break;
                            case PUSH: // (line 14)
                                assert(q_prime.find(std::make_pair(rule._to, rule._op_label)) != std::end(q_prime));
                                size_t q_new = q_prime[std::make_pair(rule._to, rule._op_label)];
                                insert_edge(rule._to, rule._op_label, q_new, trace); // (line 15)
                                insert_edge(q_new, t._label, t._to, trace, true); // (line 16)
                                for (auto f : rel2[q_new]) { // (line 17)
                                    insert_edge(f, t._label, t._to, this->new_post_trace(q_new)); // (line 18)
                                }
                                break;
                        }
                    }
                } else {
                    for (auto e : rel1[t._to]) { // (line 20)
                        insert_edge(t._from, e.second, e.first, this->new_post_trace(t._to)); // (line 21)
                    }
                }
            }
        };

        [[nodiscard]] const std::vector<std::unique_ptr<state_t>> &states() const { return _states; }
        
        [[nodiscard]] const WPDA<W,C> &pda() const { return _pda; }

        void to_dot(std::ostream &out, const std::function<void(std::ostream &, const label_with_trace_t &)> &printer = [](auto &s, auto &e) {
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

        [[nodiscard]] const trace_t *get_trace_label(const std::tuple<size_t, uint32_t, size_t> &edge) const {
            return get_trace_label(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
        }
        [[nodiscard]] const trace_t *get_trace_label(size_t from, uint32_t label, size_t to) const {
            for (auto &e : _states[from]->_edges) {
                if (e._to->_id == to) {
                    label_with_trace_t label_trace{label};
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

        void add_epsilon_edge(size_t from, size_t to, const trace_t *trace) {
            auto &edges = _states[from]->_edges;
            for (auto &e : edges) {
                if (e._to->_id == to) {
                    if (!e._labels.back().is_epsilon()) {
                        e._labels.emplace_back(trace);
                    }
                    return;
                }
            }
            edges.emplace_back(_states[to].get(), trace);
        }

        void add_edge(size_t from, size_t to, uint32_t label, const trace_t *trace = nullptr) {
            assert(label < std::numeric_limits<uint32_t>::max() - 1);
            auto &edges = _states[from]->_edges;
            for (auto &e : edges) {
                if (e._to->_id == to) {
                    e.add_label(label, trace);
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
