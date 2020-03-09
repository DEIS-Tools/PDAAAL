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
 * File:   PAutomaton_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 19-02-2020.
 */

#define BOOST_TEST_MODULE PAutomaton

#include <boost/test/unit_test.hpp>
#include <pdaaal/PAutomaton.h>
#include <pdaaal/TypedPDA.h>
#include <pdaaal/PostStar.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(UnweightedPreStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', false, 'A');
    pda.add_rule(0, 0, POP, '*', false, 'B');
    pda.add_rule(1, 3, SWAP, 'A', false, 'B');
    pda.add_rule(2, 0, SWAP, 'B', false, 'C');
    pda.add_rule(3, 2, PUSH, 'C', false, 'A');

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    PostStar::pre_star(automaton);

    std::vector<char> test_stack_reachable{'C', 'B', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'C', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_unreachable)), false);
}

BOOST_AUTO_TEST_CASE(UnweightedPostStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', false, 'A');
    pda.add_rule(0, 0, POP, '*', false, 'B');
    pda.add_rule(1, 3, SWAP, 'A', false, 'B');
    pda.add_rule(2, 0, SWAP, 'B', false, 'C');
    pda.add_rule(3, 2, PUSH, 'C', false, 'A');

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    PostStar::post_star(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(0, pda.encode_pre(test_stack_unreachable)), false);
}

BOOST_AUTO_TEST_CASE(WeightedPreStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, int> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', 1, false, 'A');
    pda.add_rule(0, 0, POP , '*', 1, false, 'B');
    pda.add_rule(1, 3, SWAP, 'A', 1, false, 'B');
    pda.add_rule(2, 0, SWAP, 'B', 1, false, 'C');
    pda.add_rule(3, 2, PUSH, 'C', 1, false, 'A');

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    PostStar::pre_star(automaton);

    std::vector<char> test_stack_reachable{'C', 'B', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'C', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_unreachable)), false);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar)
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

    std::vector<char> init_stack{'A', 'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    PostStar::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(0, pda.encode_pre(test_stack_unreachable)), false);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar2)
{
    std::unordered_set<char> labels{'A', 'B'};
    TypedPDA<char, int> pda(labels);

    pda.add_rule(1, 2, POP, '*', 1, false, 'A');
    pda.add_rule(1, 3, PUSH , 'B', 3, false, 'A');
    pda.add_rule(1, 3, SWAP, 'A', 2, false, 'B');
    pda.add_rule(2, 1, POP, '*', 4, false, 'B');
    std::vector<char> pre{'A', 'B'};
    pda.add_rule(2, 2, PUSH, 'B', 5, false, pre);
    pda.add_rule(3, 1, POP, '*', 1, false, 'B');

    std::vector<char> init_stack{'A', 'B', 'A'};
    PAutomaton automaton(pda, 1, pda.encode_pre(init_stack));

    PostStar::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar3)
{
    std::unordered_set<char> labels{'A'};
    TypedPDA<char, int> pda(labels);
    std::vector<char> pre{'A'};

    pda.add_rule(1, 2, PUSH, 'A', 16, false, 'A');
    pda.add_rule(1, 3, PUSH , 'A', 1, false, 'A');
    pda.add_rule(3, 3, PUSH , 'A', 2, false, 'A');
    pda.add_rule(3, 2, POP , 'A', 1, false, 'A');

    std::vector<char> init_stack{'A'};
    PAutomaton automaton(pda, 1, pda.encode_pre(init_stack));

    PostStar::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'A','A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar4)
{
    std::unordered_set<char> labels{'A'};
    TypedPDA<char, int> pda(labels);

    pda.add_rule(0, 3, PUSH, 'A', 4, false, 'A');
    pda.add_rule(0, 1, PUSH , 'A', 1, false, 'A');
    pda.add_rule(3, 1, PUSH , 'A', 8, false, 'A');
    pda.add_rule(1, 2, POP , 'A', 2, false, 'A');
    pda.add_rule(2, 4, POP , 'A', 16, false, 'A');

    std::vector<char> init_stack{'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    PostStar::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(4, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStarPerformance)
{
    std::unordered_set<int> labels;
    int alphabet_size = 1000;

    //Insert labels alphabet
    for(int i = 0; i < alphabet_size; i++){
        labels.insert(i);
    }

    TypedPDA<int, int> pda(labels);

    for(int i = 0; i < alphabet_size; i++){
        pda.add_rule(0, 1, SWAP, i, 1, false, 0);
        pda.add_rule(1, 2, SWAP, 0, i, false, i);
        pda.add_rule(2, 3, PUSH, i, 1, false, 0);
    }
    std::vector<int> init_stack;
    init_stack.push_back(0);
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    PostStar::post_star<Trace_Type::Shortest>(automaton);

    std::vector<int> test_stack_reachable;
    test_stack_reachable.push_back(0);
    test_stack_reachable.push_back(0);
    BOOST_CHECK_EQUAL(automaton.accepts(3, pda.encode_pre(test_stack_reachable)), true);
}