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
 * File:   Solver_Adapter.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 12-02-2020.
 */

#ifndef SOLVER_H
#define SOLVER_H

#include "PAutomaton.h"
#include "PDAAdapter.h"
#include "Solver.h"

namespace pdaaal {

    /**
     * This is an apapter from the general Solver interface, to the (legacy) PDA design, with initial and terminal states, currently used in AalWiNes.
     */
    class SolverAdapter {
    public:
        SolverAdapter() = default;

        virtual ~SolverAdapter() = default;

        template <typename W, typename C, typename A>
        using trace_info = typename std::pair<std::unique_ptr<PAutomaton<W,C,A>>, size_t>;
        template <typename W, typename C, typename A>
        using res_type = typename std::pair<bool, trace_info<W,C,A>>;

        template<typename T, typename W, typename C, typename A = add<W>>
        res_type<W,C,A> pre_star(const PDAAdapter<T,W,C>& pda, bool build_trace) {
            auto automaton = std::make_unique<PAutomaton<W,C,A>>(pda, pda.terminal(), pda.initial_stack());
            bool result = Solver::pre_star_accepts(*automaton, pda.initial(), pda.initial_stack()); // TODO: implement no-trace version. pre_star(build_trace)
            return std::make_pair(result, std::make_pair(std::move(automaton), pda.initial()));
        }

        template<Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename C, typename A = add<W>>
        res_type<W,C,A> post_star(const PDAAdapter<T,W,C>& pda) {
            auto automaton = std::make_unique<PAutomaton<W,C,A>>(pda, pda.initial(), pda.initial_stack());
            bool result = Solver::post_star_accepts<trace_type>(*automaton, pda.terminal(), pda.initial_stack());
            return std::make_pair(result, std::make_pair(std::move(automaton), pda.terminal()));
        }

        template <Trace_Type trace_type = Trace_Type::Any, typename T, typename W, typename C, typename A>
        [[nodiscard]] std::vector<typename TypedPDA<T>::tracestate_t> get_trace(const PDAAdapter<T,W,C>& pda, trace_info<W,C,A> info) const {
            auto trace = Solver::get_trace<trace_type>(pda, *info.first, info.second, pda.initial_stack());
            trace.pop_back(); // Removes terminal state from trace.
            return trace;
        }
    };
}

#endif /* SOLVER_H */

