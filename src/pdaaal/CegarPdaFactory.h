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
 * File:   CegarPdaFactory.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 11-12-2020.
 */

#ifndef PDAAAL_CEGARPDAFACTORY_H
#define PDAAAL_CEGARPDAFACTORY_H

#include "AbstractionMapping.h"
#include "PDA.h"
#include "AbstractionPDA.h"
#include "AbstractionPAutomaton.h"
#include "Solver.h"
#include <utility>

namespace pdaaal {

    // *NOTES*
    //
    // Concrete rule: (e,l,e',w) s.t. (e',w) \in \tau(e,l).
    // Need fast lookup for (e,l). Use \tau?? (no - think about failures, path etc...)
    // so maybe fut::set<(e,l,e',w),hash,vector> or fut::set<(l,e,e',w),hash,vector>
    //
    // Abstract rule is: from_state, rule_t, label.
    // This can be determined when constructing the rules, and it can be calculated when reconstructing the traces.

    // We need to store ids of abstract states a'la ptrie, so we can reuse states.
    // Essentially _abstract_values and _map_fn,
    // but for the state abstraction we maybe don't need the mapping the other way..??
    // ... well, we do need it for finding C_0 from <p_0,w_0> (the p_0 part), so ...
    //

    // Instead of current trace: vector of <p,w>, we need a:
    // rule trace: <p_0,w_0> + vector<abstract_rule>
    // Can we calculate one from the other, and vice versa? - yes
    // But rule trace is more concise, and can be directly used here. So that is what we will do.
    //

    // For label abstraction, we just need a TypedAbstractionPDA instead of TypedPDA.
    // In TypedAbstractionPDA concrete labels can map to the same label in the PDA.
    // Maybe use this AbstractionMapping<Query::label_t, uint32_t>
    // If we can be sure that the AbstractType values produced by _map_fn are consecutive 0..n,
    // then we can skip the _abstract_values part. But lets not make that restriction yet.


    // wildcard_header helps efficiently representing a partially specified stack, and ensuring that the instantiation of wildcards are sound, and that the concrete stack conforms to initial and final nfa.
    // wildcard_header::header_t contains the state needed for representing the header in the search.
    template <typename Label, typename W = void, typename C = std::less<W>>
    class wildcard_header {
        using nfa_state_t = typename NFA<Label>::state_t;
        using pda_t = AbstractionPDA<Label,W,C,fut::type::vector>; // Hmm, not able to decouple this one...
    public:
        wildcard_header(const NFA<Label>& initial_nfa, const NFA<Label>& final_nfa, const pda_t& pda,
                        std::vector<const nfa_state_t*>&& initial_path,
                        std::vector<const nfa_state_t*>&& final_path,
                        const std::vector<uint32_t>& initial_abstract_stack,
                        const std::vector<uint32_t>& final_abstract_stack)
                : _initial_nfa(initial_nfa), _final_nfa(final_nfa), _pda(pda),
                  _initial_path(std::move(initial_path)), _final_path(std::move(final_path)),
                  _initial_abstract_stack(initial_abstract_stack), _final_abstract_stack(final_abstract_stack) {
            assert(_initial_abstract_stack.size() == _initial_path.size());
            assert(_final_abstract_stack.size() == _final_path.size());
        };

        struct header_t {
            size_t count_wildcards = 0;
            std::vector<Label> concrete_part;
            // The header goes from bottom to top, where first count_wildcards labels are wildcards and the rest is concrete_part;
            [[nodiscard]] size_t size() const {
                return count_wildcards + concrete_part.size();
            }
            [[nodiscard]] bool top_is_concrete() const noexcept {
                return !concrete_part.empty();
            }
            [[nodiscard]] bool empty() const noexcept {
                return count_wildcards == 0 && concrete_part.empty();
            }
            [[nodiscard]] bool is_concrete() const noexcept {
                return count_wildcards == 0;
            }
            void pop() {
                if (concrete_part.empty()) {
                    if (count_wildcards > 0) {
                        --count_wildcards;
                    } else {
                        // Empty pop does nothing
                        assert(false); // But we want to detect it.
                    }
                } else {
                    concrete_part.pop_back();
                }
            }
            void append(const std::vector<Label>& labels) {
                concrete_part.insert(concrete_part.end(), labels.begin(), labels.end());
            }
        };

        header_t initial_header() const {
            return header_t{_initial_path.size(), std::vector<Label>()};
        }
        /**
         * Updates header by popping pre and pushing post.
         * Before popping, we check if pre matches current labels and (when relevant) is a valid concreterization of wildcard label.
         * @param header
         * @param pre is top to bottom vector.
         * @param post is bottom to top vector.
         * @return The updated header if valid, or std::nullopt if pre was not valid.
         */
        std::optional<header_t> update(const header_t& input_header, const std::vector<Label>& pre, const std::vector<Label>& post) const {
            auto header = input_header;
            assert(pre.size() <= header.size());
            for (const auto& label : pre) {
                if (header.empty()) {
                    assert(false); // TODO: Would this be an implementation bug or a legitimate error state?
                    return std::nullopt;
                }
                if (header.top_is_concrete()
                    ? header.concrete_part.back() == label
                    : check_nfa_path(_initial_nfa, _initial_path, label, _initial_path.size() - header.count_wildcards)) {
                    // label is fine as pre, so we can pop header.
                    header.pop();
                } else {
                    // pre label violates some condition for the header (match header value or be okay with initial_nfa_path), so return error value.
                    return std::nullopt;
                }
            }
            header.append(post);
            return header;
        }

        // Returns the concrete final header if possible, or provides refinement info if this is a spurious counterexample.
        // The refinement info is two disjoint sets (vectors) of labels that map to the same abstract label, but should map to different labels in the refined abstraction.
        std::variant<header_t, std::pair<std::vector<Label>,std::vector<Label>>> finalize_header(const header_t& input_header) const {
            auto header = input_header; // Copy
            assert(header.size() == _final_path.size());
            size_t i = 0;
            while (header.top_is_concrete()) {
                if (!check_nfa_path(_final_nfa, _final_path, header.concrete_part.back(), i)) {
                    auto labels = _pda.get_concrete_labels(_final_abstract_stack[i]);
                    std::sort(labels.begin(), labels.end());
                    return std::make_pair(                                              // Create refinement info
                            std::vector<Label>{header.concrete_part.back()},            // The label in concrete header, not matching edge
                            intersect_edge_labels(_final_nfa, _final_path, labels, i)); // All the matching concrete labels that maps to the same abstract label.
                }
                ++i;
                header.pop();
            }
            auto res = concreterize_wildcards(std::move(header));
            if (std::holds_alternative<header_t>(res)) {
                std::get<header_t>(res).append(input_header.concrete_part); // Add already concrete part, so the returned header is the full concrete final header.
            }
            return res;
        }

    private:
        std::variant<header_t, std::pair<std::vector<Label>,std::vector<Label>>> concreterize_wildcards(header_t&& header) const {
            assert(header.concrete_part.empty());
            std::vector<Label> concrete_stack;
            concrete_stack.reserve(header.count_wildcards);
            for(; header.count_wildcards > 0; --header.count_wildcards) {
                std::vector<Label> initial_ok_labels;
                std::vector<Label> final_ok_labels;
                auto abstract_label = _initial_abstract_stack[_initial_abstract_stack.size() - header.count_wildcards];
                assert(abstract_label == _final_abstract_stack[_final_abstract_stack.size() - header.count_wildcards]); // This position is wildcard in concrete_stack, so the trace did not touch this part of the stack, hence they must be equal.
                auto&[label_it, label_end] = _pda.get_concrete_labels_range(abstract_label);
                Label label;
                for (; label_it == label_end; ++label_it) {
                    label = *label_it;
                    bool initial_found = check_nfa_path(_initial_nfa, _initial_path, label, _initial_path.size() - header.count_wildcards);
                    bool final_found = check_nfa_path(_final_nfa, _final_path, label, _final_path.size() - header.count_wildcards);
                    if (initial_found && final_found) {
                        break;
                    }
                    if (initial_found) {
                        initial_ok_labels.push_back(label);
                    }
                    if (final_found) {
                        final_ok_labels.push_back(label);
                    }
                }
                if (label_it == label_end) {
                    assert(!initial_ok_labels.empty());
                    assert(!final_ok_labels.empty()); // Both initial and final PAutomaton contains an edge here. Some concrete label must correspond to this.
                    // Failed.
                    // initial_ok_labels and final_ok_labels are disjoint, but all map to the same abstract label.
                    // Use this split for refinement.
                    return std::make_pair(initial_ok_labels, final_ok_labels);
                } else {
                    concrete_stack.push_back(label);
                }
            }
            return header_t{0, std::vector<Label>(concrete_stack.rbegin(), concrete_stack.rend())}; // Reverse stack to make it bottom to top.
        }

        bool check_nfa_path(const NFA<Label>& nfa, const std::vector<const nfa_state_t*>& nfa_path, Label label, size_t i) const {
            return i == 0
                   ? NFA<Label>::has_as_successor(nfa.initial(), label, nfa_path[i])
                   : NFA<Label>::has_as_successor(nfa_path[i-1], label, nfa_path[i]);
        }
        std::vector<Label> intersect_edge_labels(const NFA<Label>& nfa, const std::vector<const nfa_state_t*>& nfa_path, const std::vector<Label>& labels, size_t i) const {
            return i == 0
                   ? NFA<Label>::intersect_edge_labels(nfa.initial(), nfa_path[i], labels)
                   : NFA<Label>::intersect_edge_labels(nfa_path[i-1], nfa_path[i], labels);
        }

        const NFA<Label>& _initial_nfa;
        const NFA<Label>& _final_nfa;
        const pda_t& _pda;
        const std::vector<const nfa_state_t*> _initial_path;
        const std::vector<const nfa_state_t*> _final_path;
        const std::vector<uint32_t>& _initial_abstract_stack;
        const std::vector<uint32_t>& _final_abstract_stack;
    };


    // NOTE: CEGAR construction with weights is not yet implemented.
    template <typename Label, typename State, typename AbstractState, typename conf_it, typename W = void, typename C = std::less<W>, typename A = add<W>>
    class CegarPdaFactory {
    private:
        using builder_pda_t = AbstractionPDA<Label,W,C,fut::type::hash>; // We optimize for set insertion while building,
        using pda_t = AbstractionPDA<Label,W,C,fut::type::vector>;       // and then optimize for iteration when analyzing.
        using nfa_state_t = typename NFA<Label>::state_t;
        using automaton_t = AbstractionPAutomaton<Label,W,C,A>;
    protected:
        using configuration_t = typename std::iterator_traits<conf_it>::value_type;
        using header_wrapper_t = wildcard_header<Label,W,C>;
        using header_t = typename header_wrapper_t::header_t;
    public:
        using abstract_rule_t = user_rule_t<W,C>; // FIXME: This does not yet work for weighted rules.

        // Who needs structs, when you can have tuples? :-D
        // It simplifies conversion to/from byte-vector for using with ptrie, so this is how we do it for now.
        using concrete_rule_t = std::tuple<State, Label, State, op_t, Label>;
        static constexpr concrete_rule_t concrete_rule(State _from, Label _pre, State _to, op_t _op, Label _op_label) {
            return {_from,_pre,_to,_op,_op_label};
        }
        static constexpr const State& from(const concrete_rule_t& rule) noexcept { return std::get<0>(rule); }
        static constexpr const Label& pre(const concrete_rule_t& rule) noexcept { return std::get<1>(rule); }
        static constexpr const State& to(const concrete_rule_t& rule) noexcept { return std::get<2>(rule); }
        static constexpr const op_t& op(const concrete_rule_t& rule) noexcept { return std::get<3>(rule); }
        static constexpr const Label& op_label(const concrete_rule_t& rule) noexcept { return std::get<4>(rule); }

    private:
        //using rule_iterator = typename AbstractionMapping<concrete_rule_t, abstract_rule_t>::iterator;
        //using label_iterator = typename AbstractionMapping<Label, uint32_t>::iterator;
    public:

        // Requirements:
        // if state_abstraction_fn(a) == state_abstraction_fn(b) then accepting(a) == accepting(b)
        explicit CegarPdaFactory(std::function<uint32_t(const Label&)>&& label_abstraction_fn/*,
                        std::function<AbstractState(const State&)>&& state_abstraction_fn*/)
                : _temp_pda(std::move(label_abstraction_fn)) { };
                /*, _state_abstraction(std::move(state_abstraction_fn)),
                  _rule_abstraction([this](const concrete_rule_t& r) {
                      abstract_rule_t res;
                      res._from = this->_state_abstraction.insert(from(r)).second;
                      res._to = this->_state_abstraction.insert(to(r)).second;
                      res._pre = this->_temp_pda.insert_label(pre(r)).second;
                      res._op_label = (op(r) == PUSH || op(r) == SWAP) ? this->_temp_pda.insert_label(op_label(r)).second : std::numeric_limits<uint32_t>::max();
                      res._op = op(r);
                      return res;
                  }) { };*/

        // NFAs must be already compiled before passing them to this function.
        AbstractionSolverInstance<Label,W,C,A> compile(const NFA<Label>& initial_headers, const NFA<Label>& final_headers) {
            build_pda();
            return AbstractionSolverInstance<Label,W,C,A>(pda_t{std::move(_temp_pda)}, initial_headers, initial(), final_headers, accepting());
        }

        void reconstruct_trace(const AbstractionSolverInstance<Label,W,C,A>& instance, const NFA<Label>& initial_headers, const NFA<Label>& final_headers) {
            static_assert(!is_weighted<W>, "Not yet supported.");
            if constexpr (is_weighted<W>) {
                assert(false); // Not yet supported.
            } else {
                const auto[init_state, trace, initial_abstract_stack, final_abstract_stack, initial_path, final_path] = Solver::get_rule_trace_and_paths(instance);
                assert(init_state != std::numeric_limits<size_t>::max()); // Assume that a trace exists (otherwise don't call this function!)

                auto build_nfa_path = [](const automaton_t& automaton, const std::vector<size_t>& path) -> std::vector<const nfa_state_t*> {
                    assert(automaton.get_nfastate(path[0]) == nullptr); // First state of path is initial (does not exists in NFA)
                    std::vector<const nfa_state_t*> result;
                    for (size_t i = 1; i < path.size(); ++i) {
                        auto nfa_s = automaton.get_nfastate(path[i]);
                        assert(nfa_s != nullptr); // All other states (than the first) must exist in NFA.
                        result.push_back(nfa_s);
                    }
                    return result;
                };

                header_wrapper_t header_handler(initial_headers, final_headers, instance.pda(),
                                                build_nfa_path(instance.initial_automaton(), initial_path),
                                                build_nfa_path(instance.final_automaton(), final_path),
                                                initial_abstract_stack, final_abstract_stack);
                // Keep track of outcome of search, and info for doing refinement if needed.
                size_t max_depth = 0;
                std::vector<configuration_t> current_deepest_states;
                std::optional<std::pair<std::vector<Label>,std::vector<Label>>> header_refinement_info;
                std::optional<header_t> final_header;

                // Depth first search, using this as states:
                struct search_state_t {
                    conf_it _it;
                    conf_it _end;
                    size_t _trace_id;
                    search_state_t(std::pair<conf_it,conf_it> confs, size_t trace_id) : _it(confs.first), _end(confs.second), _trace_id(trace_id) {};
                    [[nodiscard]] bool end() const noexcept {
                        return _it == _end;
                    }
                    void operator++() noexcept {
                        ++_it;
                    }
                };
                std::vector<search_state_t> search_stack;
                // Initialize
                search_stack.emplace_back(first_confs(header_handler, initial_abstract_stack, trace[0]), 0);
                // Search
                while (!search_stack.empty()) {
                    if (search_stack.back().end()) { // No more rules at this level
                        search_stack.pop_back(); // Backtrack search one level
                        if (!search_stack.empty()) {
                            ++search_stack.back(); // Go to next rule at the lower level
                        }
                        continue;
                    }

                    // Keep track of states at the deepest level
                    auto next_trace_id = search_stack.back()._trace_id + 1;
                    if (next_trace_id > max_depth) {
                        current_deepest_states.clear();
                        max_depth = next_trace_id;
                    }
                    auto conf = *search_stack.back()._it;
                    if (next_trace_id == max_depth) {
                        current_deepest_states.push_back(conf);
                    }

                    if (next_trace_id < trace.size()) {
                        // Search at next level
                        search_stack.emplace_back(next_confs(header_handler, conf, trace[next_trace_id]), next_trace_id);
                        continue;
                    } else {
                        // Done with trace, see if the final header can be valid.
                        auto res = header_handler.finalize_header(get_header(conf));
                        if (std::holds_alternative<header_t>(res)) {
                            final_header.emplace(std::get<header_t>(res));
                            break; // Yeah, we are done searching.
                        } else {
                            header_refinement_info.emplace(std::get<1>(res));
                            ++search_stack.back(); // No, keep searching.
                            continue;
                        }
                    }
                }

                if (max_depth < trace.size()) {
                    // We did not reach a final header.
                    // Refine based on configurations at deepest level of search.
                    if (max_depth == 0) {
                        // Special case since we did not even apply a first rule, so current_deepest_states is empty.
                        // TODO: ...

                    } else {
                        // TODO: Refine...
                        find_refinement(current_deepest_states, trace[max_depth]);

                    }
                } else if (final_header) {
                    // TODO: Success. What to return??
                    auto header = final_header.value();

                } else if (header_refinement_info) {
                    // TODO: Perform label refinement...
                    auto[label_set1, label_set2] = header_refinement_info.value();

                } else {
                    assert(false); // If we got through the trace, but have no final header, we must have header_refinement_info.
                }

            }
        }


    private:
        /*void build_pda(builder_pda_t& pda) {
            // Build up PDA by searching through reachable states from initial states.
            // Derived class must define initial states, successor function (rules), and accepting state predicate.

            std::vector<State> waiting = initial();
            std::unordered_set<State> seen(waiting.begin(), waiting.end());
            for (const auto& concrete_initial_state : waiting) {
                auto [fresh, id] = _state_abstraction.insert(concrete_initial_state);
                if (fresh) {
                    _initial_states.emplace(id);
                }
            }
            std::unordered_set<size_t> accepting_states_set;
            while (!waiting.empty()) {
                auto from = waiting.back();
                waiting.pop_back();
                if (accepting(from)) {
                    auto [exists, from_id] = _state_abstraction.exists(from);
                    assert(exists);
                    accepting_states_set.emplace(from_id);
                }
                for (const auto& [ignore_concrete, r] : rules(from)) {
                    assert(from == r._from);

                    auto [fresh,ar_id] = _rule_abstraction.insert(r, ignore_concrete);
                    if (fresh) {
                        auto abstract_r = _rule_abstraction.get_abstract_value(ar_id);
                        pda.add_rule(abstract_r);
                    }

                    if (seen.emplace(r._to).second) {
                        waiting.push_back(r._to);
                    }
                }
            }
            std::sort(_initial_states.begin(), _initial_states.end());
            _accepting_states.assign(accepting_states_set.begin(), accepting_states_set.end());
            std::sort(_accepting_states.begin(), _accepting_states.end());
        }*/

    protected:

        void add_rule(const abstract_rule_t& rule) {
            _temp_pda.add_rule(rule);
        }
        virtual void build_pda() = 0;
        virtual const std::vector<size_t>& initial() = 0;
        virtual const std::vector<size_t>& accepting() = 0;

        /*abstract_rule_t make_abstract_rule(const concrete_rule_t& r) {
            abstract_rule_t res;
            res._from = _state_abstraction.insert(r._from).second;
            res._to = _state_abstraction.insert(r._to).second;
            res._pre = _temp_pda.insert_label(r._pre).second;
            res._op_label = (r._op == PUSH || r._op == SWAP) ? _temp_pda.insert_label(r._op_label).second : std::numeric_limits<uint32_t>::max();
            res._op = r._op;
            return res;
        }*/

        //virtual const std::vector<State>& initial() = 0;
        //virtual bool accepting(const State&) = 0;
        //virtual std::vector<std::pair<bool,concrete_rule_t>> rules(const State&) = 0;

        //virtual bool is_initial(const State&) = 0;


        virtual std::pair<conf_it,conf_it> first_confs(const header_wrapper_t&, std::vector<uint32_t>, const abstract_rule_t&) = 0;
        virtual std::pair<conf_it,conf_it> next_confs(const header_wrapper_t&, const configuration_t&, const abstract_rule_t&) = 0;
        virtual void // TODO: What should be return type of this?
        find_refinement(const std::vector<configuration_t>&, const abstract_rule_t&) = 0;
        virtual header_t get_header(const configuration_t&) = 0;


    private:
        builder_pda_t _temp_pda;
        //AbstractionMapping<State, AbstractState> _state_abstraction;
        //AbstractionMapping<concrete_rule_t, abstract_rule_t> _rule_abstraction;

        //std::vector<size_t> _initial_states;
        //std::vector<size_t> _accepting_states;
    };

}

#endif //PDAAAL_CEGARPDAFACTORY_H
