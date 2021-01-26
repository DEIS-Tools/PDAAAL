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
    template <typename label_t, typename W, typename C>
    class wildcard_header {
        using nfa_state_t = typename NFA<label_t>::state_t;
        using pda_t = AbstractionPDA<label_t,W,C,fut::type::vector>; // Hmm, not able to decouple this one...
    public:
        wildcard_header(const NFA<label_t>& initial_nfa, const NFA<label_t>& final_nfa, const pda_t& pda,
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
            std::vector<label_t> concrete_part;
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
            void append(const std::vector<label_t>& labels) {
                concrete_part.insert(concrete_part.end(), labels.begin(), labels.end());
            }
        };

        header_t initial_header() const {
            return header_t{_initial_path.size(), std::vector<label_t>()};
        }

        /**
         * Updates header by popping the label pre and 'additional_pops' without checks, and then pushing post.
         * Before popping, we check if pre matches current labels and (when relevant) is a valid concreterization of wildcard label.
         * @param header
         * @param pre is label.
         * @param post is bottom to top vector.
         * @param additional_pops is the number of pops after popping pre. These pops are not checked (i.e. we assume they match)
         * @return The updated header if valid, or std::nullopt if pre was not valid.
         */
        std::optional<header_t> update(const header_t& input_header, const label_t& pre, const std::vector<label_t>& post, size_t additional_pops = 0) const {
            auto header = input_header;
            assert(1 + additional_pops <= header.size());
            if (header.top_is_concrete()
                ? header.concrete_part.back() == pre
                : check_nfa_path(_initial_nfa, _initial_path, pre, _initial_path.size() - header.count_wildcards)) {
                // label is fine as pre, so we can pop header.
                header.pop();
            } else {
                // pre label violates some condition for the header (match header value or be okay with initial_nfa_path), so return error value.
                return std::nullopt;
            }
            for (size_t i = 0; i < additional_pops; ++i) {
                assert(!header.empty());
                header.pop();
            }
            header.append(post);
            return header;
        }
        // Returns the labels, pre, for which update(header, pre, ...) succeeds. Can be used when computing refinement.
        std::vector<label_t> pre_labels(const header_t& header) const {
            if (header.empty()) return std::vector<label_t>();
            if (header.top_is_concrete()) return std::vector<label_t>{header.concrete_part.back()};
            size_t i = _initial_path.size() - header.count_wildcards;
            auto labels = _pda.get_concrete_labels(_initial_abstract_stack[i]);
            std::sort(labels.begin(), labels.end());
            return intersect_edge_labels(_initial_nfa, _initial_path, labels, i);
        }

        // Returns the concrete final header if possible, or provides refinement info if this is a spurious counterexample.
        // The refinement info is two disjoint sets (vectors) of labels that map to the same abstract label, but should map to different labels in the refined abstraction.
        std::variant<header_t, Refinement<label_t>> finalize_header(const header_t& input_header) const {
            auto header = input_header; // Copy
            assert(header.size() == _final_path.size());
            size_t i = 0;
            while (header.top_is_concrete()) {
                if (!check_nfa_path(_final_nfa, _final_path, header.concrete_part.back(), i)) {
                    auto labels = _pda.get_concrete_labels(_final_abstract_stack[i]);
                    std::sort(labels.begin(), labels.end());
                    return  Refinement<label_t>(                                       // Create refinement info
                            std::vector<label_t>{header.concrete_part.back()},         // The label in concrete header, not matching edge
                            intersect_edge_labels(_final_nfa, _final_path, labels, i), // All the matching concrete labels that maps to the same abstract label.
                            _final_abstract_stack[i]);
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

        /*
        std::vector<label_t> get_concrete_labels(size_t label) const {
            return _pda.get_concrete_labels(label);
        }
        auto get_concrete_labels_range(size_t label) const {
            return _pda.get_concrete_labels_range(label);
        }
        */
        bool maps_to(const label_t& label, size_t id) const {
            return _pda.maps_to(label, id);
        }

    private:
        std::variant<header_t, Refinement<label_t>> concreterize_wildcards(header_t&& header) const {
            assert(header.concrete_part.empty());
            std::vector<label_t> concrete_stack;
            concrete_stack.reserve(header.count_wildcards);
            for(; header.count_wildcards > 0; --header.count_wildcards) {
                std::vector<label_t> initial_ok_labels;
                std::vector<label_t> final_ok_labels;
                auto abstract_label = _initial_abstract_stack[_initial_abstract_stack.size() - header.count_wildcards];
                assert(abstract_label == _final_abstract_stack[_final_abstract_stack.size() - header.count_wildcards]); // This position is wildcard in concrete_stack, so the trace did not touch this part of the stack, hence they must be equal.
                bool found = false;
                for (const auto& label : _pda.get_concrete_labels_range(abstract_label)) {
                    bool initial_found = check_nfa_path(_initial_nfa, _initial_path, label, _initial_path.size() - header.count_wildcards);
                    bool final_found = check_nfa_path(_final_nfa, _final_path, label, _final_path.size() - header.count_wildcards);
                    if (initial_found && final_found) {
                        found = true;
                        concrete_stack.push_back(label);
                        break;
                    }
                    if (initial_found) {
                        initial_ok_labels.push_back(label);
                    }
                    if (final_found) {
                        final_ok_labels.push_back(label);
                    }
                }
                if (!found) {
                    assert(!initial_ok_labels.empty());
                    assert(!final_ok_labels.empty()); // Both initial and final PAutomaton contains an edge here. Some concrete label must correspond to this.
                    // Failed.
                    // initial_ok_labels and final_ok_labels are disjoint, but all map to the same abstract label.
                    // Use this split for refinement.
                    return Refinement<label_t>(std::move(initial_ok_labels), std::move(final_ok_labels), abstract_label);
                }
            }
            return header_t{0, std::vector<label_t>(concrete_stack.rbegin(), concrete_stack.rend())}; // Reverse stack to make it bottom to top.
        }

        bool check_nfa_path(const NFA<label_t>& nfa, const std::vector<const nfa_state_t*>& nfa_path, label_t label, size_t i) const {
            return i == 0
                   ? NFA<label_t>::has_as_successor(nfa.initial(), label, nfa_path[i])
                   : NFA<label_t>::has_as_successor(nfa_path[i-1], label, nfa_path[i]);
        }
        std::vector<label_t> intersect_edge_labels(const NFA<label_t>& nfa, const std::vector<const nfa_state_t*>& nfa_path, const std::vector<label_t>& labels, size_t i) const {
            return i == 0
                   ? NFA<label_t>::intersect_edge_labels(nfa.initial(), nfa_path[i], labels)
                   : NFA<label_t>::intersect_edge_labels(nfa_path[i-1], nfa_path[i], labels);
        }

        const NFA<label_t>& _initial_nfa;
        const NFA<label_t>& _final_nfa;
        const pda_t& _pda;
        const std::vector<const nfa_state_t*> _initial_path;
        const std::vector<const nfa_state_t*> _final_path;
        const std::vector<uint32_t>& _initial_abstract_stack;
        const std::vector<uint32_t>& _final_abstract_stack;
    };


    // NOTE: CEGAR construction with weights is not yet implemented.
    template <typename label_t, typename W = void, typename C = std::less<W>, typename A = add<W>>
    class CegarPdaFactory {
    private:
        using builder_pda_t = AbstractionPDA<label_t,W,C,fut::type::hash>; // We optimize for set insertion while building,
        using pda_t = AbstractionPDA<label_t,W,C,fut::type::vector>;       // and then optimize for iteration when analyzing.
    protected:
        using solver_instance_t = AbstractionSolverInstance<label_t,W,C,A>;
    public:
        using abstract_rule_t = user_rule_t<W,C>; // FIXME: This does not yet work for weighted rules.

    public:
        template<typename abstract_label_t>
        CegarPdaFactory(const std::unordered_set<label_t>& all_labels, std::function<abstract_label_t(const label_t&)>&& label_abstraction_fn)
        : _temp_pda(all_labels, std::move(label_abstraction_fn)) { }

        void reset_pda(RefinementMapping<label_t>&& mapping) {
            _temp_pda = builder_pda_t(std::move(mapping));
        }

        // NFAs must be already compiled before passing them to this function.
        solver_instance_t compile(const NFA<label_t>& initial_headers, const NFA<label_t>& final_headers) {
            build_pda();
            return solver_instance_t(pda_t{std::move(_temp_pda)}, initial_headers, initial(), final_headers, accepting());
        }

    protected:
        void add_rule(const abstract_rule_t& rule) {
            _temp_pda.add_rule(rule);
        }
        std::pair<bool,size_t> abstract_label(const label_t& label){
            return _temp_pda.abstract_label(label);
        }

        virtual void build_pda() = 0;
        virtual const std::vector<size_t>& initial() = 0;
        virtual const std::vector<size_t>& accepting() = 0;

    private:
        builder_pda_t _temp_pda;
    };

    template <typename label_t, typename state_t, typename configuration_range_t, typename concrete_trace_t, typename W = void, typename C = std::less<W>, typename A = add<W>>
    class CegarPdaReconstruction {
    private:
        using configuration_t = typename configuration_range_t::value_type;
        using header_wrapper_t = wildcard_header<label_t,W,C>;

        using pda_t = AbstractionPDA<label_t,W,C,fut::type::vector>;
        using nfa_state_t = typename NFA<label_t>::state_t;
        using automaton_t = AbstractionPAutomaton<label_t,W,C,A>;
        using solver_instance_t = AbstractionSolverInstance<label_t,W,C,A>;
    public:
        using header_t = typename header_wrapper_t::header_t;
        using state_refinement_t = Refinement<state_t>;
        using label_refinement_t = Refinement<label_t>;
        using header_refinement_t = HeaderRefinement<label_t>;
        using refinement_t = std::pair<state_refinement_t, label_refinement_t>;
        using abstract_rule_t = user_rule_t<W,C>; // FIXME: This does not yet work for weighted rules.

    public:

        std::variant<concrete_trace_t,// Concrete trace (of configurations) and final concrete header.
                refinement_t,
                header_refinement_t>
        reconstruct_trace(const solver_instance_t& instance, const NFA<label_t>& initial_headers, const NFA<label_t>& final_headers) {
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
                header_refinement_t header_refinement_info;
                std::optional<header_t> final_header;

                // Depth first search, using this as states:
                struct search_state_t {
                    decltype(std::declval<const configuration_range_t&>().begin()) _it;
                    decltype(std::declval<const configuration_range_t&>().end()) _end;
                    size_t _trace_id;
                    search_state_t(const configuration_range_t& config_range, size_t trace_id)
                            : _it(config_range.begin()), _end(config_range.end()), _trace_id(trace_id) {};
                    [[nodiscard]] bool end() const noexcept {
                        return _it == _end;
                    }
                    void operator++() noexcept {
                        ++_it;
                    }
                };
                std::vector<search_state_t> search_stack;
                if (!trace.empty()) { // Initialize
                    search_stack.emplace_back(initial_concrete_rules(trace[0], header_handler), 0);
                } else {
                    // Special case: Empty trace. Immediately find concrete header.
                    auto res = header_handler.finalize_header(header_handler.initial_header());
                    if (std::holds_alternative<header_t>(res)) {
                        final_header.emplace(std::get<header_t>(std::move(res)));
                    } else {
                        header_refinement_info.combine(std::get<1>(std::move(res)));
                    }
                }
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
                        search_stack.emplace_back(search_concrete_rules(trace[next_trace_id], conf, header_handler), next_trace_id);
                        continue;
                    } else {
                        // Done with trace, see if the final header can be valid.
                        auto res = header_handler.finalize_header(get_header(conf));
                        if (std::holds_alternative<header_t>(res)) {
                            final_header.emplace(std::get<header_t>(res));
                            break; // Yeah, we are done searching.
                        } else {
                            header_refinement_info.combine(std::get<1>(std::move(res)));
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
                        return find_initial_refinement(trace[0], header_handler);
                    } else {
                        return find_refinement(current_deepest_states, trace[max_depth], header_handler);
                    }
                } else if (final_header) {
                    // Success. Reconstruct concrete trace from search stack and return it together with the final concrete header.
                    std::vector<configuration_t> concrete_trace;
                    for (const auto& search_state : search_stack) {
                        concrete_trace.emplace_back(*search_state._it);
                    }
                    assert(final_header->count_wildcards == 0);
                    return get_concrete_trace(std::move(concrete_trace), std::move(final_header->concrete_part), initial_path[0]);
                } else if (!header_refinement_info.empty()) {
                    // Return Label refinement info.
                    return header_refinement_info;
                } else {
                    assert(false); // If we got through the trace, but have no final header, we must have header_refinement_info.
                }
            }
            // I don't know, sort of an empty return on failure...
            assert(false);
            return header_refinement_t();
        }

    protected:
        virtual const configuration_range_t& initial_concrete_rules(const abstract_rule_t&, const header_wrapper_t&) = 0;
        virtual refinement_t find_initial_refinement(const abstract_rule_t& abstract_rule, const header_wrapper_t&  header_handler) = 0;
        virtual const configuration_range_t& search_concrete_rules(const abstract_rule_t&, const configuration_t&, const header_wrapper_t&) = 0;
        virtual refinement_t find_refinement(const std::vector<configuration_t>&, const abstract_rule_t&, const header_wrapper_t&) = 0;
        virtual header_t get_header(const configuration_t&) = 0;
        virtual concrete_trace_t get_concrete_trace(std::vector<configuration_t>&&, std::vector<label_t>&&, size_t) = 0;
    };



    template <typename Factory, typename Reconstruction>
    class CEGAR {
        static_assert(std::is_same_v<typename Factory::label_t, typename Reconstruction::label_t>);
        using label_t = typename Reconstruction::label_t;
        using concrete_trace_t = typename Reconstruction::concrete_trace_t;
        using refinement_t = typename Reconstruction::refinement_t;
        using header_refinement_t = typename Reconstruction::header_refinement_t;
    public:
        std::optional<concrete_trace_t> cegar_solve(Factory&& factory, const NFA<label_t>& initial_headers, const NFA<label_t>& final_headers) {
            while(true) {
                auto instance = factory.compile(initial_headers, final_headers);

                bool result = Solver::post_star_accepts(instance);
                if (!result) {
                    return std::nullopt; // No trace.
                }

                Reconstruction reconstruction(factory);

                auto res = reconstruction.reconstruct_trace(instance, initial_headers, final_headers);

                if (std::holds_alternative<concrete_trace_t>(res)) {
                    return std::get<concrete_trace_t>(res);
                } else if (std::holds_alternative<refinement_t>(res)) {
                    factory.reset_pda(instance.move_pda_refinement_mapping(std::get<refinement_t>(res).second));
                    factory.refine(std::get<refinement_t>(std::move(res)));
                } else {
                    assert(std::holds_alternative<header_refinement_t>(res));
                    factory.reset_pda(instance.move_pda_refinement_mapping(std::get<header_refinement_t>(res)));
                    factory.refine(std::get<header_refinement_t>(std::move(res)));
                }
            }
        }
    };

}

#endif //PDAAAL_CEGARPDAFACTORY_H
