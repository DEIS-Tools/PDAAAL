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

#include "AutomatonPath.h"
#include "PDA.h"
#include "SolverInstance.h"
#include "pdaaal/internal/PdaSaturation.h"
#include <absl/hash/hash.h>

namespace pdaaal {

    class Solver {
    public:
        template <Trace_Type trace_type, typename pda_t, typename automaton_t, typename W>
        static bool pre_star_fixed_point_accepts(PAutomatonProduct<pda_t,automaton_t,W,TraceInfoType::Pair>& instance) {
            instance.enable_pre_star();
            pre_star_fixed_point<trace_type>(instance.automaton());
            return instance.template initialize_product<false,false>();
        }
        template <Trace_Type trace_type, typename W>
        static void pre_star_fixed_point(internal::PAutomaton<W,TraceInfoType::Pair>& automaton) {
            internal::PreStarFixedPointSaturation<W,trace_type> saturation(automaton);
            saturation.run();
        }

        template <typename pda_t, typename automaton_t, typename W>
        static bool dual_search_accepts(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            if (instance.template initialize_product<true>()) {
                return true;
            }
            return dual_search<W>(instance.final_automaton(), instance.initial_automaton(),
                [&instance](size_t from, uint32_t label, size_t to, internal::edge_annotation_t<W> trace) -> bool {
                    return instance.add_final_edge(from, label, to, trace);
                },
                [&instance](size_t from, uint32_t label, size_t to, internal::edge_annotation_t<W> trace) -> bool {
                    return instance.add_initial_edge(from, label, to, trace);
                }
            );
        }
        template <typename W, bool ET=true>
        static bool dual_search(internal::PAutomaton<W> &pre_star_automaton, internal::PAutomaton<W> &post_star_automaton,
                                const internal::early_termination_fn<W>& pre_star_early_termination,
                                const internal::early_termination_fn<W>& post_star_early_termination) {
            internal::PreStarSaturation<W,ET> pre_star(pre_star_automaton, pre_star_early_termination);
            internal::PostStarSaturation<W,ET> post_star(post_star_automaton, post_star_early_termination);
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
        static bool pre_star_accepts(internal::PAutomaton<W> &automaton, size_t state, const std::vector<uint32_t> &stack) {
            if (stack.size() == 1) {
                auto s_label = stack[0];
                return pre_star<W,true>(automaton, [&automaton, state, s_label](size_t from, uint32_t label, size_t to, internal::edge_annotation_t<W>) -> bool {
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
                   pre_star<W,true>(instance.automaton(), [&instance](size_t from, uint32_t label, size_t to, internal::edge_annotation_t<W> trace) -> bool {
                       return instance.add_edge_product(from, label, to, trace);
                   });
        }

        template <typename W, bool ET=false>
        static bool pre_star(internal::PAutomaton<W> &automaton,
                             const internal::early_termination_fn<W>& early_termination = [](size_t, uint32_t, size_t, internal::edge_annotation_t<W>) -> bool { return false; }) {
            internal::PreStarSaturation<W,ET> saturation(automaton, early_termination);
            while(!saturation.workset_empty()) {
                if constexpr (ET) {
                    if (saturation.found()) return true;
                }
                saturation.step();
            }
            return saturation.found();
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename W>
        static bool post_star_accepts(internal::PAutomaton<W> &automaton, size_t state, const std::vector<uint32_t> &stack) {
            if (stack.size() == 1) {
                auto s_label = stack[0];
                return post_star<trace_type,W,true>(automaton, [&automaton, state, s_label](size_t from, uint32_t label, size_t to, internal::edge_annotation_t<W>) -> bool {
                    return from == state && label == s_label && automaton.states()[to]->_accepting;
                });
            } else {
                return post_star<trace_type,W>(automaton) || automaton.accepts(state, stack);
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename pda_t, typename automaton_t, typename W>
        static bool post_star_accepts(PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            return instance.initialize_product() ||
                   post_star<trace_type,W,true>(instance.automaton(), [&instance](size_t from, uint32_t label, size_t to, internal::edge_annotation_t<W> trace) -> bool {
                       return instance.add_edge_product(from, label, to, trace);
                   });
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename W, bool ET = false>
        static bool post_star(internal::PAutomaton<W> &automaton,
                              const internal::early_termination_fn<W>& early_termination = [](size_t, uint32_t, size_t, internal::edge_annotation_t<W>) -> bool { return false; }) {
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
            if constexpr (trace_type == Trace_Type::Longest || trace_type == Trace_Type::ShortestFixedPoint) {
                using return_type = decltype(_get_trace(instance.pda(), instance.initial_automaton(), std::declval<AutomatonPath<>>()));
                auto [automaton_path, weight] = instance.template find_path_fixed_point<trace_type>();
                if (weight != internal::solver_weight<W,trace_type>::bottom()) { // Not infinite. Use standard _get_trace.
                    return std::make_pair(_get_trace(instance.pda(), instance.automaton(), automaton_path), weight);
                }
                // Infinite trace.
                assert(!automaton_path.is_null());
                // TODO: Implement infinite trace structure.
                return std::make_pair(return_type{}, weight);
            } else if constexpr (trace_type == Trace_Type::Shortest) {
                auto [automaton_path, weight] = instance.template find_path<trace_type>();
                return std::make_pair(_get_trace(instance.pda(), instance.automaton(), automaton_path), weight);
            } else {
                auto automaton_path = instance.template find_path<trace_type>();
                return _get_trace(instance.pda(), instance.automaton(), automaton_path);
            }
        }
        template <typename pda_t, typename automaton_t, typename W>
        static auto get_trace_dual_search(const PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            auto paths = instance.template find_path<Trace_Type::Any, true>();
            using return_type = decltype(_get_trace(instance.pda(), instance.initial_automaton(), std::declval<AutomatonPath<>>()));
            if (paths.is_null()) {
                return return_type{};
            }
            auto [i_path, f_path] = paths.split();
            auto trace1 = _get_trace(instance.pda(), instance.initial_automaton(), i_path);
            auto trace2 = _get_trace(instance.pda(), instance.final_automaton(), f_path);
            assert(trace1.back()._pdastate == trace2.front()._pdastate);
            assert(trace1.back()._stack.size() == trace2.front()._stack.size()); // Should also check == for contents of stack, but T might not implement ==.
            trace1.insert(trace1.end(), trace2.begin() + 1, trace2.end());
            return trace1;
        }
        template <Trace_Type trace_type = Trace_Type::Any, bool use_dual=false, typename pda_t, typename automaton_t, typename W>
        static auto get_rule_trace_and_paths(const PAutomatonProduct<pda_t,automaton_t,W>& instance) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto[paths, weight] = instance.template find_path<trace_type, true>();
                return std::make_pair(_get_rule_trace_and_paths(instance.automaton(), paths), weight);
            } else if constexpr(use_dual) {
                auto paths = instance.template find_path<trace_type, true>();
                return _get_rule_trace_and_paths(instance.initial_automaton(), instance.final_automaton(), paths);
            } else {
                auto paths = instance.template find_path<trace_type, true>();
                return _get_rule_trace_and_paths(instance.automaton(), paths);
            }
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W>
        static auto get_trace(const PDA<T,W>& pda, const internal::PAutomaton<W>& automaton, size_t state, const std::vector<T>& stack) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            auto stack_native = pda.encode_pre(stack);
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto [path, weight] = automaton.template accept_path<trace_type>(state, stack_native);
                return std::make_pair(_get_trace(pda, automaton, AutomatonPath(path, stack_native)), weight);
            } else {
                auto path = automaton.template accept_path<trace_type>(state, stack_native);
                return _get_trace(pda, automaton, AutomatonPath(path, stack_native));
            }
        }
        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename = std::enable_if_t<!std::is_same_v<T,uint32_t>>>
        static auto get_trace(const PDA<T,W>& pda, const internal::PAutomaton<W>& automaton, size_t state, const std::vector<uint32_t>& stack_native) {
            static_assert(trace_type != Trace_Type::None, "If you want a trace, don't ask for none.");
            if constexpr (trace_type == Trace_Type::Shortest) {
                auto [path, weight] = automaton.template accept_path<trace_type>(state, stack_native);
                return std::make_pair(_get_trace(pda, automaton, AutomatonPath(path, stack_native)), weight);
            } else {
                auto path = automaton.template accept_path<trace_type>(state, stack_native);
                return _get_trace(pda, automaton, AutomatonPath(path, stack_native));
            }
        }

    private:
        template <typename W, bool ET>
        static bool post_star_any(internal::PAutomaton<W> &automaton, const internal::early_termination_fn<W>& early_termination) {
            internal::PostStarSaturation<W,ET> saturation(automaton, early_termination);
            while(!saturation.workset_empty()) {
                if constexpr (ET) {
                    if (saturation.found()) return true;
                }
                saturation.step();
            }
            return saturation.found();
        }

        template<typename W, bool Enable, bool ET, typename = std::enable_if_t<Enable>>
        static bool post_star_shortest(internal::PAutomaton<W> &automaton, const internal::early_termination_fn<W>& early_termination) {
            internal::PostStarShortestSaturation<W,Enable,ET> saturation(automaton, early_termination);
            while(!saturation.workset_empty()) {
                if constexpr (ET) {
                    if (saturation.found()) break;
                }
                saturation.step();
            }
            saturation.finalize();
            return saturation.found();
        }

        template <typename T, typename W, typename S, bool ssm>
        static typename PDA<T,W,fut::type::vector,S,ssm>::tracestate_t
        _decode_edges(const PDA<T,W,fut::type::vector,S,ssm> &pda, const AutomatonPath<>& path) {
            using tracestate_t = typename PDA<T,W,fut::type::vector,S,ssm>::tracestate_t;
            tracestate_t result{path.front_state(), std::vector<T>()};
            auto num_labels = pda.number_of_labels();
            for (auto label : path.stack()) {
                if (label < num_labels){
                    result._stack.emplace_back(pda.get_symbol(label));
                }
            }
            return result;
        }

        template <typename T, typename W, typename S, bool ssm, TraceInfoType trace_info_type>
        static std::vector<typename PDA<T,W,fut::type::vector,S,ssm>::tracestate_t>
        _get_trace(const PDA<T,W,fut::type::vector,S,ssm> &pda, const internal::PAutomaton<W,trace_info_type>& automaton,
                   AutomatonPath<> automaton_path) {
            using tracestate_t = typename PDA<T,W,fut::type::vector,S,ssm>::tracestate_t;

            if (automaton_path.is_null()) {
                return std::vector<tracestate_t>();
            }

            std::vector<tracestate_t> trace;
            trace.push_back(_decode_edges(pda, automaton_path));
            internal::TraceBack tb(automaton, std::move(automaton_path));
            while (tb.next()) {
                trace.push_back(_decode_edges(pda, tb.path()));
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
                > _get_rule_trace_and_paths(const internal::PAutomaton<W,trace_info_type>& automaton, // The PAutomaton that has been build up (either A_pre* or A_post*)
                                            const AutomatonPath<true>& paths) {
            using rule_t = user_rule_t<W>;

            if (paths.is_null()) {
                return std::make_tuple(std::numeric_limits<size_t>::max(), std::vector<rule_t>(), std::vector<uint32_t>(), std::vector<uint32_t>(), std::vector<size_t>(), std::vector<size_t>());
            }
            // The paths was retrieved from the product automaton. First number is the state in @automaton (A_pre* or A_post*), second number is state in goal automaton.
            auto [automaton_path, goal] = paths.split();
            // Get path in goal automaton (returned in the end).
            auto [goal_path, goal_stack] = goal.get_path_and_stack();

            internal::TraceBack tb(automaton, std::move(automaton_path));
            std::vector<rule_t> trace;
            while (auto rule = tb.next()) {
                trace.emplace_back(rule.value());
            }

            // Get accepting path of initial stack (and the initial stack itself - for post*)
            auto [start_path, start_stack] = tb.path().get_path_and_stack();

            if (tb.post()) { // post* was used
                std::reverse(trace.begin(), trace.end());
                return std::make_tuple(trace[0].from(), std::move(trace),
                                       std::move(start_stack), std::move(goal_stack),
                                       std::move(start_path), std::move(goal_path));
            } else { // pre* was used
                return std::make_tuple(paths.front_state().first, std::move(trace),
                                       std::move(goal_stack), std::move(start_stack),
                                       std::move(goal_path), std::move(start_path));
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
        > _get_rule_trace_and_paths(const internal::PAutomaton<W,trace_info_type1>& initial_automaton,
                                    const internal::PAutomaton<W,trace_info_type2>& final_automaton,
                                    const std::optional<AutomatonPath<true>>& paths) {
            using rule_t = user_rule_t<W>;

            if (!paths.has_value()) {
                return std::make_tuple(std::numeric_limits<size_t>::max(), std::vector<rule_t>(), std::vector<uint32_t>(), std::vector<uint32_t>(), std::vector<size_t>(), std::vector<size_t>());
            }
            // The paths was retrieved from the product automaton. First number is the state in initial_automaton, second number is state in final_automaton.
            auto [initial_automaton_path, final_automaton_path] = paths.value().split();

            auto [trace, initial_stack, initial_path] = _get_trace_stack_path(initial_automaton, std::move(initial_automaton_path));
            auto [trace2, final_stack, final_path] = _get_trace_stack_path(final_automaton, std::move(final_automaton_path));
            // Concat traces
            trace.insert(trace.end(), trace2.begin(), trace2.end());
            return std::make_tuple(trace[0].from(), std::move(trace),
                                   std::move(initial_stack), std::move(final_stack),
                                   std::move(initial_path), std::move(final_path));
        }

        template <typename W, TraceInfoType trace_info_type>
        static std::tuple<std::vector<user_rule_t<W>>, std::vector<uint32_t>, std::vector<size_t>>
        _get_trace_stack_path(const internal::PAutomaton<W,trace_info_type>& automaton, AutomatonPath<>&& automaton_path) {
            std::vector<user_rule_t<W>> trace;
            internal::TraceBack tb(automaton, std::move(automaton_path));
            while(auto rule = tb.next()) {
                trace.emplace_back(rule.value());
            }
            if (tb.post()) {
                std::reverse(trace.begin(), trace.end());
            }
            // Get accepting path of initial stack (and the initial stack itself - for post*)
            auto [path, stack] = tb.path().get_path_and_stack();
            return {std::move(trace), std::move(stack), std::move(path)};
        }
    };

}

#endif //PDAAAL_SOLVER_H
