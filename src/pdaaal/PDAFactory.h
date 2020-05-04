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
 */

/* 
 * File:   PDAFactory.h
 * Author: Peter G. Jensen <root@petergjoel.dk>
 *
 * Created on July 18, 2019, 3:24 PM
 */

#ifndef PDAFACTORY_H
#define PDAFACTORY_H

#include <string>
#include <sstream>
#include <vector>
#include <ostream>
#include <unordered_set>

#include "NFA.h"
#include "PDAAdapter.h"

namespace pdaaal {

    template<typename T, typename W = void, typename C = std::less<W>>
    class PDAFactory {
    private:
        using Temp_PDA = PDAAdapter<T,W,C,fut::type::hash>;
    public:
        using nfastate_t = typename NFA<T>::state_t;

        struct state_t {
            size_t _ptr;
            bool _indirect;
        };
    protected:
        template <typename WT, typename = void> struct rule_t_;
        template <typename WT>
        struct rule_t_<WT, std::enable_if_t<!is_weighted<WT>>> {
            op_t _op = POP;
            T _pre;
            size_t _dest = std::numeric_limits<size_t>::max();
            T _op_label;
        };

        template <typename WT>
        struct rule_t_<WT, std::enable_if_t<is_weighted<WT>>> {
            op_t _op = POP;
            T _pre;
            size_t _dest = std::numeric_limits<size_t>::max();
            T _op_label;
            WT _weight;
        };
        using rule_t = rule_t_<W>;
        
    public:

        PDAFactory(NFA<T>& prestack, NFA<T>& poststack, std::unordered_set<T>&& all_labels)
        : _cons_stack(prestack), _des_stack(poststack), _all_labels(all_labels) {
            _cons_stack.compile();
            _des_stack.compile();
        };

        PDAAdapter<T,W,C> compile() {
            Temp_PDA temp_result(_all_labels);
            bool cons_empty_accept = false;
            bool des_empty_accept = empty_desctruction_accept();
            
            // we need to know the number of "other states" to create indexes
            // for the NFA-states
            build_pda(temp_result, des_empty_accept);

            // CONSTRUCTION-HEADER
            {
                std::unordered_set<const nfastate_t*> seen;
                std::vector<const nfastate_t*> waiting;

                // compute initial for construction
                cons_empty_accept = initialize_construction(temp_result, seen, waiting);

                // saturate reachable rules from initial construction
                build_construction(temp_result, seen, waiting);
            }

            // trivially empty
            if (cons_empty_accept && empty_accept() && des_empty_accept) {
                std::vector<T> empty;
                T lbl;
                temp_result.add_rule(temp_result.initial(), temp_result.terminal(), NOOP, lbl, true, empty);
            }

            // Destruct the stack!
            build_destruction(temp_result);
            PDAAdapter<T,W,C> result(std::move(temp_result));
            result.finalize();
            return result;
        }

    protected:
        bool initialize_construction(Temp_PDA& result, std::unordered_set<const nfastate_t*>& seen, std::vector<const nfastate_t*>& waiting) {
            bool has_empty_accept = false;
            std::vector<T> empty;
            for (auto& i : _cons_stack.initial()) {
                // We do the first step here.
                if (i->_accepting) {
                    has_empty_accept = true;
                }
                for (auto& e : i->_edges) {
                    std::vector<nfastate_t*> next{e._destination};
                    NFA<T>::follow_epsilon(next);
                    for (auto n : next) {
                        result.add_rules(result.initial(), nfa_id(n), PUSH, e._negated, e._symbols, true, empty);
                        if (seen.count(n) == 0) {
                            seen.insert(n);
                            waiting.push_back(n);
                        }
                        if(n->_accepting)
                        {
                            for (auto s : initial()) {
                                result.add_rules(result.initial(), s, PUSH, e._negated, e._symbols, true, empty);
                            }
                            // we can pass directly to destruction
                            if (empty_accept()) {
                                for (auto s : _des_stack.initial()) {
                                    result.add_rules(result.initial(), nfa_id(s), PUSH, e._negated, e._symbols, true, empty);
                                }
                            }                            
                        }
                    }
                }
            }
            return has_empty_accept;
        }

        void build_construction(Temp_PDA& result, std::unordered_set<const nfastate_t*>& seen, std::vector<const nfastate_t*>& waiting) {
            while (!waiting.empty()) {
                auto top = waiting.back();
                waiting.pop_back();
                // we need to know the set of pre-edges to be able to handle the TOS correctly
                auto pre = top->prelabels(_all_labels);

                for (auto& e : top->_edges) {
                    std::vector<nfastate_t*> next{e._destination};
                    NFA<T>::follow_epsilon(next);
                    for (auto n : next) {
                        result.add_rules(nfa_id(top), nfa_id(n), PUSH, e._negated, e._symbols, false, pre);
                        if (seen.count(n) == 0) {
                            seen.insert(n);
                            waiting.push_back(n);
                        }
                        if(n->_accepting)
                        {
                            for (auto s : initial()) {
                                auto id = nfa_id(top);
                                result.add_rules(id, s, PUSH, e._negated, e._symbols, false, pre);
                            }
                            // we can pass directly to destruction
                            if (empty_accept()) {
                                for (auto s : _des_stack.initial()) {
                                    result.add_rules(nfa_id(top), nfa_id(s), PUSH, e._negated, e._symbols, false, pre);
                                }
                            }                            
                        }
                    }
                }
            }            
        }

        bool empty_desctruction_accept() {
            for (auto s : _des_stack.initial()) {
                if (s->_accepting) {
                    return true;
                }
            }
            return false;
        }

        void build_pda(Temp_PDA& result, bool des_empty_accept) {
            auto pdawaiting = initial();
            std::unordered_set<size_t> pdaseen(pdawaiting.begin(), pdawaiting.end());
            std::vector<T> empty;
            std::vector<size_t> accepting_states;
            _num_pda_states = 2; // 0 and 1 is allready counted,
            while (!pdawaiting.empty()) {
                auto top = pdawaiting.back();
                if(top > 1)
                    ++_num_pda_states;
                else
                {
                    assert(false);
                }
                pdawaiting.pop_back();
                if (accepting(top)) {
                    accepting_states.push_back(top);
                }
                for (auto &r : rules(top)) {
                    // translate rules into PDA rules
                    std::vector<T> pre{r._pre};
                    assert(_all_labels.count(r._pre) == 1);
                    if constexpr (is_weighted<W>) {
                        result.add_rule(top, r._dest, r._op, r._op_label, false, pre, r._weight);
                    } else {
                        result.add_rule(top, r._dest, r._op, r._op_label, false, pre);
                    }
                    if (pdaseen.count(r._dest) == 0) {
                        pdaseen.insert(r._dest);
                        pdawaiting.push_back(r._dest);
                    }
                }
            }
            
            // we need to know the number of PDA states first to be able to 
            // correctly index the NFA-states
            T none;
            for(auto a : accepting_states)
            {
                // any label could really be on the top of the stack here
                // do first step of DES
                if (des_empty_accept) {
                    // empty accept in destruction, just go directly.
                    result.add_rule(a, result.terminal(), NOOP, none, true, empty);
                }
                // link with destruction-header
                for (auto ds : _des_stack.initial()) {

                    for (auto& e : ds->_edges) {
                        std::vector<nfastate_t*> next{e._destination};
                        NFA<T>::follow_epsilon(next);
                        for (auto n : next) {
                            result.add_rule(a, nfa_id(n), POP, none, e._negated, e._symbols);
                        }
                    }
                }                
            }
        }

        void build_destruction(Temp_PDA& result) {
            std::vector<nfastate_t*> waiting_next = _des_stack.initial();
            std::unordered_set<nfastate_t*> seen_next(waiting_next.begin(), waiting_next.end());
            std::vector<T> empty;
            T none;
            while (!waiting_next.empty()) {
                auto top = waiting_next.back();
                waiting_next.pop_back();
                if (top->_accepting) {
                    result.add_rule(nfa_id(top), result.terminal(), NOOP, none, true, empty);
                }
                for (auto& e : top->_edges) {
                    std::vector<nfastate_t*> next{e._destination};
                    NFA<T>::follow_epsilon(next);
                    for (auto n : next) {
                        result.add_rule(nfa_id(top), nfa_id(n), POP, none, e._negated, e._symbols);
                        if (seen_next.count(n) == 0) {
                            waiting_next.push_back(n);
                            seen_next.insert(n);
                        }
                    }
                }
            }
        }

        virtual const std::vector<size_t>& initial() = 0;
        virtual bool empty_accept() const = 0;
        virtual bool accepting(size_t) = 0;
        virtual std::vector<rule_t> rules(size_t) = 0;

    protected:
        size_t nfa_id(const nfastate_t* state)
        {
            auto id = _ptr_to_state.insert(state);
            if(id.first)
            {
                size_t nid = _nfamap.size() + _num_pda_states;
                _ptr_to_state.get_data(id.second) = nid;
                _nfamap.push_back(state);
                return nid;
            }
            return _ptr_to_state.get_data(id.second);
        }
        NFA<T> _cons_stack, _des_stack;
        std::unordered_set<T> _all_labels;
        size_t _num_pda_states = 0;
        std::vector<const nfastate_t*> _nfamap;
        ptrie::map<const nfastate_t*, size_t> _ptr_to_state;
    private:

    };
}

#endif /* PDAFACTORY_H */