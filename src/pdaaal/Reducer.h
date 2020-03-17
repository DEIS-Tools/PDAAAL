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
 *  Copyright Peter G. Jensen
 *  Modified by Morten K. Schou
 */

/*
 * File:   Reducer.h
 * Author: Peter G. Jensen <root@petergjoel.dk>
 *
 * Created on August 21, 2019, 2:47 PM
 */

#ifndef PDAAAL_REDUCER_H
#define PDAAAL_REDUCER_H

#include <queue>
#include "WPDA.h"

namespace pdaaal {

    class Reducer {
    private:
        struct tos_t {
            std::vector<uint32_t> _tos; // TODO: some symbolic representation would be better here
            std::vector<uint32_t> _stack; // TODO: some symbolic representation would be better here

            enum waiting_t {
                NOT_IN_STACK = 0, TOS = 8, BOS = 16, BOTH = TOS | BOS
            };
            waiting_t _in_waiting = NOT_IN_STACK;

            bool update_state(const std::pair<bool, bool>& new_state);

            bool empty_tos() const;

            bool forward_stack(const tos_t& prev, size_t all_labels);

            template <typename W, typename C>
            bool active(const tos_t& prev, const rule_t<W,C>& rule) const {
                if (rule._labels.empty()) {
                    return false;
                }
                if (rule._labels.wildcard()) {
                    return true;
                }
                auto rit = rule._labels.labels().begin();
                bool match = false;
                for (auto& s : prev._tos) {
                    while (rit != std::end(rule._labels.labels()) && *rit < s) ++rit;
                    if (rit == std::end(rule._labels.labels())) {
                        break;
                    }
                    if (*rit == s) {
                        match = true;
                        break;
                    }
                }
                return match;
            }

            template <typename W, typename C>
            std::pair<bool, bool> merge_pop(const tos_t& prev, const rule_t<W,C>& rule, bool dual_stack, size_t all_labels) {
                if (!active(prev, rule)) {
                    return std::make_pair(false, false);
                }
                bool changed = false;
                bool stack_changed = false;
                if (!dual_stack) {
                    if (_tos.size() != all_labels) {
                        _tos.resize(all_labels);
                        for (size_t i = 0; i < all_labels; ++i) _tos[i] = i;
                        changed = true;
                    }
                }
                else {
                    // move stack->stack
                    stack_changed |= forward_stack(prev, all_labels);
                    // move stack -> TOS
                    if (_tos.size() != all_labels) {
                        if (prev._stack.size() == all_labels) {
                            changed = true;
                            _tos = prev._stack;
                        }
                        else {
                            auto it = _tos.begin();
                            for (auto s : prev._stack) {
                                while (it != std::end(_tos) && *it < s) ++it;
                                if (it != std::end(_tos) && *it == s) continue;
                                it = _tos.insert(it, s);
                                changed = true;
                            }
                        }
                    }
                }
                return std::make_pair(changed, stack_changed);
            }

            template <typename W, typename C>
            std::pair<bool, bool> merge_noop(const tos_t& prev, const rule_t<W,C>& rule, bool dual_stack, size_t all_labels) {
                if (rule._labels.empty()) return std::make_pair(false, false);
                bool changed = false;
                {
                    auto iit = _tos.begin();
                    for (auto& symbol : rule._labels.wildcard() ? prev._tos : rule._labels.labels()) {
                        while (iit != _tos.end() && *iit < symbol) ++iit;
                        if (iit != _tos.end() && *iit == symbol) {
                            ++iit;
                            continue;
                        }
                        changed = true;
                        iit = _tos.insert(iit, symbol);
                        ++iit;
                    }
                }
                bool stack_changed = false;
                if (dual_stack) {
                    stack_changed |= forward_stack(prev, all_labels);
                }
                return std::make_pair(changed, stack_changed);
            }

            template <typename W, typename C>
            std::pair<bool, bool> merge_swap(const tos_t& prev, const rule_t<W,C>& rule, bool dual_stack, size_t all_labels) {
                if (!active(prev, rule))
                    return std::make_pair(false, false); // we know that there is a match!
                bool changed = false;
                {
                    auto lb = std::lower_bound(_tos.begin(), _tos.end(), rule._op_label);
                    if (lb == std::end(_tos) || *lb != rule._op_label) {
                        changed = true;
                        _tos.insert(lb, rule._op_label);
                    }
                }
                bool stack_changed = false;
                if (dual_stack) {
                    stack_changed |= forward_stack(prev, all_labels);
                }
                return std::make_pair(changed, stack_changed);
            }

            template <typename W, typename C>
            std::pair<bool, bool> merge_push(const tos_t& prev, const rule_t<W,C>& rule, bool dual_stack, size_t all_labels) {
                // similar to swap!
                auto changed = merge_swap(prev, rule, dual_stack, all_labels);
                if (dual_stack) {
                    // but we also push all TOS labels down
                    if (_stack.size() != all_labels) {
                        if (prev._tos.size() == all_labels) {
                            changed.second = true;
                            _stack = prev._tos;
                        }
                        else {
                            auto it = _stack.begin();
                            for (auto& s : prev._tos) {
                                while (it != _stack.end() && *it < s) ++it;
                                if (it != std::end(_stack) && *it == s) continue;
                                it = _stack.insert(it, s);
                                changed.second = true;
                            }
                        }
                    }
                }
                return changed;
            }
        };

    public:
        template <typename W, typename C>
        static std::pair<size_t, size_t> reduce(WPDA<W,C> &pda, int aggresivity, size_t initial_id, size_t terminal_id) {
            size_t cnt = pda.size();
            if (aggresivity == 0)
                return std::make_pair(cnt, cnt);

            Reducer::forwards_prune(pda, initial_id);
            Reducer::backwards_prune(pda, terminal_id);
            std::queue<size_t> waiting;
            auto ds = (aggresivity >= 2);
            std::vector<tos_t> approximation(pda.states().size());
            // initialize
            for (auto& r : pda.states()[initial_id]._rules) {
                if (r._to == terminal_id) continue;
                if (r._labels.empty()) continue;
                auto& ss = approximation[r._to];
                std::pair<bool, bool> tmp(true, true);
                if (ss._in_waiting == tos_t::NOT_IN_STACK) {
                    ss.update_state(tmp);
                    waiting.push(r._to);
                }
                assert(r._operation == PUSH);
                auto lb = std::lower_bound(ss._tos.begin(), ss._tos.end(), r._op_label);
                if (lb == std::end(ss._tos) || *lb != r._op_label)
                    ss._tos.insert(lb, r._op_label);
            }

            if (aggresivity == 3) {
                Reducer::target_tos_prune(pda, terminal_id);
            }
            // saturate
            while (!waiting.empty()) {
                auto el = waiting.front();
                waiting.pop();
                auto& ss = approximation[el];
                auto& state = pda.states()[el];
                auto fit = state._rules.begin();
                ss._in_waiting = tos_t::NOT_IN_STACK;
                while (fit != std::end(state._rules)) {
                    if (fit->_to == terminal_id) {
                        ++fit;
                        continue;
                    }
                    if (fit->_labels.empty()) {
                        ++fit;
                        continue;
                    }
                    auto& to = approximation[fit->_to];
                    // handle dots!
                    std::pair<bool, bool> change;
                    switch (fit->_operation) {
                        case POP:
                            change = to.merge_pop(ss, *fit, ds, pda.number_of_labels());
                            break;
                        case NOOP:
                            change = to.merge_noop(ss, *fit, ds, pda.number_of_labels());
                            break;
                        case PUSH:
                            change = to.merge_push(ss, *fit, ds, pda.number_of_labels());
                            break;
                        case SWAP:
                            change = to.merge_swap(ss, *fit, ds, pda.number_of_labels());
                            break;
                        default:
                            throw std::logic_error("Unknown PDA operation");
                            break;
                    }
                    if (to.update_state(change)) {
                        waiting.push(fit->_to);
                    }
                    ++fit;
                }
            }
            // DO PRUNING!
            for (size_t i = 1; i < pda.states().size(); ++i) {
                if(i == initial_id) continue;
                auto& app = approximation[i];
                auto& state = pda.states()[i];
                size_t br = 0;
                for (size_t r = 0; r < state._rules.size(); ++r) {
                    // check rule
                    auto& rule = state._rules[r];
                    if (rule._to == terminal_id || rule._labels.intersect(app._tos, pda.number_of_labels())) {
                        if (br != r) {
                            std::swap(state._rules[br], state._rules[r]);
                        }
                        ++br;
                    }
                }
                state._rules.resize(br);
                if (state._rules.empty()) {
                    pda.clear_state(i);
                }
            }

            Reducer::backwards_prune(pda, terminal_id);
            if (aggresivity == 3) {
                // it could potentially work as fixpoint; not sure if it has any effect.
                Reducer::target_tos_prune(pda, terminal_id);
            }

            size_t after_cnt = pda.size();
            return std::make_pair(cnt, after_cnt);
        }

        template <typename W, typename C>
        static void forwards_prune(WPDA<W,C> &pda, size_t initial_id) {
            std::queue<size_t> waiting;
            std::vector<bool> seen(pda.states().size());
            waiting.push(initial_id);
            seen[initial_id] = true;
            while (!waiting.empty()) {
                auto el = waiting.front();
                waiting.pop();
                for (auto& r : pda.states()[el]._rules) {
                    if (!seen[r._to] && !r._labels.empty()) {
                        waiting.push(r._to);
                        seen[r._to] = true;
                    }
                }
            }
            for (size_t s = 0; s < pda.states().size(); ++s) {
                if (!seen[s]) {
                    pda.clear_state(s);
                }
            }
        }

        template <typename W, typename C>
        static void backwards_prune(WPDA<W,C> &pda, size_t terminal) {
            std::queue<size_t> waiting;
            std::vector<bool> seen(pda.states().size());
            waiting.push(terminal);
            seen[terminal] = true;
            while (!waiting.empty()) {
                // backward
                auto el = waiting.front();
                waiting.pop();
                for (auto& s2 : pda.states()[el]._pre_states) {
                    if (!seen[s2]) {
                        waiting.push(s2);
                        seen[s2] = true;
                    }
                }
            }
            for (size_t s = 0; s < pda.states().size(); ++s) {
                if (!seen[s]) {
                    pda.clear_state(s);
                }
            }
        }

        template <typename W, typename C>
        static void target_tos_prune(WPDA<W,C> &pda, size_t terminal_id) {
            std::queue<size_t> waiting;
            std::vector<bool> in_waiting(pda.states().size());
            for (size_t t = 0; t < pda.states().size(); ++t) {
                in_waiting[t] = true;
                waiting.push(t);
            }

            while (!waiting.empty()) {
                auto s = waiting.front();
                waiting.pop();
                if (s == terminal_id) continue; // We don't prune terminal.
                in_waiting[s] = false;
                std::set<uint32_t> usefull_tos;
                bool cont = false;
                for (auto& r : pda.states()[s]._rules) {
                    if (!r._labels.wildcard()) {
                        usefull_tos.insert(r._labels.labels().begin(), r._labels.labels().end());
                    }
                    else {
                        cont = true;
                    }
                    if (usefull_tos.size() == pda.number_of_labels() || cont) {
                        cont = true;
                        break;
                    }
                }
                if (cont)
                    continue;
                for (auto& pres : pda.states()[s]._pre_states) {
                    auto& state = pda.states()[pres];
                    for (auto& r : state._rules) {
                        if (r._to == s) {
                            switch (r._operation) {
                                case SWAP:
                                case PUSH:
                                    if (!r._labels.empty() && usefull_tos.count(r._op_label) == 0) {
                                        r._labels.clear();
                                        if (!in_waiting[pres])
                                            waiting.push(pres);
                                        in_waiting[pres] = true;
                                    }
                                    break;
                                case NOOP:
                                {
                                    bool changed = r._labels.noop_pre_filter(usefull_tos);
                                    if (changed && !in_waiting[pres]) {
                                        waiting.push(pres);
                                        in_waiting[pres] = true;
                                    }
                                    break;
                                }
                                case POP:
                                    // we cant really prune this one.
                                    // it fans out if we try; i.e. no local computation.
                                    break;
                            }
                        }
                    }
                }
            }
        }
    };
}

#endif //PDAAAL_REDUCER_H
