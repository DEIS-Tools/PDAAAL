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
#include <pdaaal/Solver_Adapter.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(SolverTest1)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, std::array<double, 3>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', w, false, 'A');
    pda.add_rule(0, 0, POP , '*', w, false, 'B');
    pda.add_rule(1, 3, SWAP, 'A', w, false, 'B');
    pda.add_rule(2, 0, SWAP, 'B', w, false, 'C');
    pda.add_rule(3, 2, PUSH, 'C', w, false, 'A');

    //std::vector<char> init_stack{'A', 'A'};
    //PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver_Adapter solver;

//    solver.post_star(pda); // Does not work yet. The new WPDA is not compatible was the initial/terminal PDA design from AalWiNes. Refactoring needed.

    //PostStar::post_star<Trace_Type::Shortest>(automaton);

    /*std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(0, pda.encode_pre(test_stack_unreachable)), false);*/
}