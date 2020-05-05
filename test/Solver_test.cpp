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
 * File:   Solver_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 10-03-2020.
 */

#define BOOST_TEST_MODULE Solver

#include <boost/test/unit_test.hpp>
#include <pdaaal/Solver.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(SolverTest1)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, std::array<double, 3>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', false, 'A', w);
    pda.add_rule(0, 0, POP , '*', false, 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', false, 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', false, 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', false, 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    auto trace = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 1, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(SolverTest2)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<uint32_t> labels{2,3,4};
    TypedPDA<uint32_t, std::array<double, 3>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 3, false, 2, w);
    pda.add_rule(0, 0, POP , '*', false, 3, w);
    pda.add_rule(1, 3, SWAP, 2, false, 3, w);
    pda.add_rule(2, 0, SWAP, 3, false, 4, w);
    pda.add_rule(3, 2, PUSH, 4, false, 2, w);

    std::vector<uint32_t> init_stack{2, 2};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<uint32_t> test_stack_reachable{3, 2, 2, 2};
    auto trace = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 1, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(SolverTest3)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, std::array<double, 3>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', false, 'A', w);
    pda.add_rule(0, 0, POP , '*', false, 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', false, 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', false, 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', false, 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    auto stack_native = pda.encode_pre(test_stack_reachable);
    auto trace = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 1, stack_native);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(EarlyTerminationPostStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, std::array<double, 3>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', false, 'A', w);
    pda.add_rule(0, 0, POP , '*', false, 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', false, 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', false, 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', false, 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    auto stack_native = pda.encode_pre(test_stack_reachable);
    auto result = Solver::post_star_accepts(automaton, 1, stack_native);
    BOOST_CHECK_EQUAL(result, true);

    auto trace = Solver::get_trace(pda, automaton, 1, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(EarlyTerminationPreStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, std::array<double, 3>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', false, 'A', w);
    pda.add_rule(0, 0, POP , '*', false, 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', false, 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', false, 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', false, 'A', w);

    std::vector<char> init_stack{'B', 'A', 'A', 'A'};
    PAutomaton automaton(pda, 1, pda.encode_pre(init_stack));

    std::vector<char> test_stack_reachable{'A'};
    auto result = Solver::pre_star_accepts(automaton, 0, pda.encode_pre(test_stack_reachable));
    BOOST_CHECK_EQUAL(result, true);

    auto trace = Solver::get_trace(pda, automaton, 0, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 12);
}