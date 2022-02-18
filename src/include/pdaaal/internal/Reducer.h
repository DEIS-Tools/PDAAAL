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
#include "PDA.h"

namespace pdaaal::internal {

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

            [[nodiscard]] bool empty_tos() const;

            bool forward_stack(const tos_t& prev, size_t all_labels);

            static bool active(const tos_t& prev, const labels_t& labels) ;

            std::pair<bool, bool> merge_pop(const tos_t& prev, const labels_t& labels, bool dual_stack, size_t all_labels);

            std::pair<bool, bool> merge_noop(const tos_t& prev, const labels_t& labels, bool dual_stack, size_t all_labels);

            std::pair<bool, bool> merge_swap(const tos_t& prev, uint32_t op_label, const labels_t& labels, bool dual_stack, size_t all_labels);

            std::pair<bool, bool> merge_push(const tos_t& prev, uint32_t op_label, const labels_t& labels, bool dual_stack, size_t all_labels);
        };

    public:
        template <typename W>
        static std::pair<size_t, size_t> reduce(PDA<W> &pda, int aggresivity, size_t initial_id, size_t terminal_id) {
            size_t cnt = Reducer::size(pda, initial_id, terminal_id);
            if (aggresivity == 0)
                return std::make_pair(cnt, cnt);

            Reducer::forwards_prune(pda, initial_id);
            Reducer::backwards_prune(pda, terminal_id);
            std::queue<size_t> waiting;
            auto ds = (aggresivity == 2 || aggresivity == 4);
            std::vector<tos_t> approximation(pda.states().size());
            // initialize
            for (const auto& [r,labels] : pda.states()[initial_id]._rules) {
                if (r._to == terminal_id) continue;
                if (labels.empty()) continue;
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

            if (aggresivity >= 3) {
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
                    if (fit->first._to == terminal_id) {
                        ++fit;
                        continue;
                    }
                    if (fit->second.empty()) {
                        ++fit;
                        continue;
                    }
                    auto& to = approximation[fit->first._to];
                    // handle dots!
                    std::pair<bool, bool> change;
                    switch (fit->first._operation) {
                        case POP:
                            change = to.merge_pop(ss, fit->second, ds, pda.number_of_labels());
                            break;
                        case NOOP:
                            change = to.merge_noop(ss, fit->second, ds, pda.number_of_labels());
                            break;
                        case PUSH:
                            change = to.merge_push(ss, fit->first._op_label, fit->second, ds, pda.number_of_labels());
                            break;
                        case SWAP:
                            change = to.merge_swap(ss, fit->first._op_label, fit->second, ds, pda.number_of_labels());
                            break;
                        default:
                            throw std::logic_error("Unknown PDA operation");
                            break;
                    }
                    if (to.update_state(change)) {
                        waiting.push(fit->first._to);
                    }
                    ++fit;
                }
            }
            // DO PRUNING!
            for (size_t i = 1; i < pda.states().size(); ++i) {
                if(i == initial_id) continue;
                auto& app = approximation[i];
                auto& state = pda.states_mutable()[i];
                size_t br = 0;
                for (size_t r = 0; r < state._rules.size(); ++r) {
                    // check rule
                    auto& [rule,labels] = state._rules[r];
                    if (rule._to == terminal_id || labels.intersect(app._tos, pda.number_of_labels())) {
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
            if (aggresivity >= 3) {
                // it could potentially work as fixpoint; not sure if it has any effect.
                Reducer::target_tos_prune(pda, terminal_id);
            }

            size_t after_cnt = Reducer::size(pda, initial_id, terminal_id);
            return std::make_pair(cnt, after_cnt);
        }

        template <typename W>
        static void forwards_prune(PDA<W> &pda, size_t initial_id) {
            std::queue<size_t> waiting;
            std::vector<bool> seen(pda.states().size());
            waiting.push(initial_id);
            seen[initial_id] = true;
            while (!waiting.empty()) {
                auto el = waiting.front();
                waiting.pop();
                for (const auto& [r,labels] : pda.states()[el]._rules) {
                    if (!seen[r._to] && !labels.empty()) {
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

        template <typename W>
        static void backwards_prune(PDA<W> &pda, size_t terminal) {
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

        template <typename W>
        static void target_tos_prune(PDA<W> &pda, size_t terminal_id) {
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
                for (const auto& [r,labels] : pda.states()[s]._rules) {
                    if (!labels.wildcard()) {
                        usefull_tos.insert(labels.labels().begin(), labels.labels().end());
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
                    auto& state = pda.states_mutable()[pres];
                    for (auto& [r,labels] : state._rules) {
                        if (r._to == s) {
                            switch (r._operation) {
                                case SWAP:
                                case PUSH:
                                    if (!labels.empty() && usefull_tos.count(r._op_label) == 0) {
                                        labels.clear();
                                        if (!in_waiting[pres])
                                            waiting.push(pres);
                                        in_waiting[pres] = true;
                                    }
                                    break;
                                case NOOP:
                                {
                                    bool changed = labels.noop_pre_filter(usefull_tos);
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

    private:
        template <typename W>
        static size_t size(const PDA<W> &pda, size_t initial_id, size_t terminal_id)
        {
            size_t cnt = 1;
            // lets start by the initial transitions
            cnt += pda.states()[initial_id]._rules.size();
            for (size_t sid = 1; sid < pda.states().size(); ++sid) {
                for (auto& [r,labels] : pda.states()[sid]._rules) {
                    if (r._to == terminal_id) {
                        ++cnt;
                        continue;
                    }
                    if (labels.empty()) continue;
                    cnt += labels.wildcard() ? pda.number_of_labels() : labels.labels().size();
                }
            }
            return cnt;
        }
    };
}

#endif //PDAAAL_REDUCER_H
