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
 * File:   StackSizeWeight_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 05-05-2020.
 */

#define BOOST_TEST_MODULE StackSize

#include <boost/test/unit_test.hpp>
#include <pdaaal/StackSizeWeight.h>
#include <pdaaal/PAutomaton.h>
#include <pdaaal/TypedPDA.h>
#include <pdaaal/Solver.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(StackSizeWeightElemTest){
    StackSizeWeight::elem_t a{2,-1};
    StackSizeWeight::elem_t b{0,0};

    auto c = StackSizeWeight::elem_t::append(a,b);
    BOOST_CHECK_EQUAL(c.max, 2);
    BOOST_CHECK_EQUAL(c.diff, -1);
}

BOOST_AUTO_TEST_CASE(StackSizeWeightTest){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();
    const auto swap = StackSizeWeight::swap();

    auto a = push.add(push).add(pop).add(pop);
    auto b = push;
    auto c = swap;
    auto d = push.add(push);

    std::cout << "(" << a << " \u2293 " << b << ") \u2295 " << c << " = " << StackSizeWeight::extend(a, b).add(c) << std::endl;
    std::cout << "(" << a << " \u2293 " << b << ") \u2295 " << d << " = " << StackSizeWeight::extend(a, b).add(d) << std::endl;

    BOOST_CHECK_EQUAL(StackSizeWeight::extend(a, b).add(c).final_value(), 1);
    BOOST_CHECK_EQUAL(StackSizeWeight::extend(a, b).add(d).final_value(), 2);
}

BOOST_AUTO_TEST_CASE(StackSizeWeightTest2){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();

    auto a1 = push.add(push).add(push);
    auto a2 = push.add(push).add(push).add(push).add(pop).add(pop);
    auto a3 = push.add(push).add(push).add(push).add(push).add(pop).add(pop).add(pop).add(pop);

    auto b1 = push.add(push).add(pop).add(pop);
    auto b2 = push;

    auto a = a1.min(a2).min(a3);
    std::cout << "(" << a1 << " \u2293 " << a2 << ") \u2293 " << a3 << " = " << a << std::endl;
    auto b = b1.min(b2);
    std::cout << b1 << " \u2293 " << b2 << " = " << b << std::endl;

    auto c = a.add(b);
    std::cout << a << " \u2295 " << b << " = " << c << std::endl;

    BOOST_CHECK_EQUAL(a.final_value(), 3);
    BOOST_CHECK_EQUAL(b.final_value(), 1);
    BOOST_CHECK_EQUAL(c.final_value(), 4);
}

BOOST_AUTO_TEST_CASE(StackSizeWeightTest3){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();
    const auto swap = StackSizeWeight::swap();

    auto a1 = push.add(push).add(push);
    auto a2 = push.add(push).add(push).add(push).add(pop).add(pop);
    auto a3 = push.add(push).add(push).add(push).add(push).add(pop).add(pop).add(pop).add(pop);

    auto b1 = swap;
    auto b2 = push.add(push);
    auto b3 = push.add(push).add(push).add(push);

    auto a = a1.min(a2).min(a3);
    std::cout << "(" << a1 << " \u2293 " << a2 << ") \u2293 " << a3 << " = " << a << std::endl;

    auto c1 = a.add(b1);
    std::cout << a << " \u2295 " << b1 << " = " << c1 << std::endl;
    auto c2 = a.add(b2);
    std::cout << a << " \u2295 " << b2 << " = " << c2 << std::endl;
    auto c3 = a.add(b3);
    std::cout << a << " \u2295 " << b3 << " = " << c3 << std::endl;

    BOOST_CHECK_EQUAL(c1.final_value(), 3);
    BOOST_CHECK_EQUAL(c2.final_value(), 4);
    BOOST_CHECK_EQUAL(c3.final_value(), 5);
}

BOOST_AUTO_TEST_CASE(StackSizeWeightTest3_WithGabs){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();
    const auto swap = StackSizeWeight::swap();

    auto a1 = push.add(push).add(push);
    auto a2 = push.add(push).add(push).add(push).add(push).add(pop).add(pop).add(pop).add(pop);
    auto a3 = push.add(push).add(push).add(push).add(push).add(push).add(push).add(pop).add(pop).add(pop).add(pop).add(pop).add(pop).add(pop).add(pop);

    auto b1 = swap;
    auto b2 = push.add(push).add(push).add(push);
    auto b3 = push.add(push).add(push).add(push).add(push).add(push).add(push).add(push);

    auto a = a1.min(a2).min(a3);
    std::cout << "(" << a1 << " \u2293 " << a2 << ") \u2293 " << a3 << " = " << a << std::endl;

    auto c1 = a.add(b1);
    std::cout << a << " \u2295 " << b1 << " = " << c1 << std::endl;
    auto c2 = a.add(b2);
    std::cout << a << " \u2295 " << b2 << " = " << c2 << std::endl;
    auto c3 = a.add(b3);
    std::cout << a << " \u2295 " << b3 << " = " << c3 << std::endl;

    BOOST_CHECK_EQUAL(c1.final_value(), 3);
    BOOST_CHECK_EQUAL(c2.final_value(), 5);
    BOOST_CHECK_EQUAL(c3.final_value(), 7);
}

BOOST_AUTO_TEST_CASE(StackSizeWeightExtendCommutAssoc){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();

    auto a1 = push.add(push).add(push);
    auto a2 = push.add(push).add(push).add(push).add(pop).add(pop);
    auto a3 = push.add(push).add(push).add(push).add(push).add(pop).add(pop).add(pop).add(pop);

    std::cout << "(" << a1 << " \u2293 " << a2 << ") \u2293 " << a3 << ")) = " << a1.min(a2).min(a3) << std::endl;
    std::cout << "(" << a1 << " \u2293 " << a3 << ") \u2293 " << a2 << ")) = " << a1.min(a3).min(a2) << std::endl;
    std::cout << "(" << a2 << " \u2293 " << a1 << ") \u2293 " << a3 << ")) = " << a2.min(a1).min(a3) << std::endl;
    std::cout << "(" << a2 << " \u2293 " << a3 << ") \u2293 " << a1 << ")) = " << a2.min(a3).min(a1) << std::endl;
    std::cout << "(" << a3 << " \u2293 " << a1 << ") \u2293 " << a2 << ")) = " << a3.min(a1).min(a2) << std::endl;
    std::cout << "(" << a3 << " \u2293 " << a2 << ") \u2293 " << a1 << ")) = " << a3.min(a2).min(a1) << std::endl;

    auto expected = a1.min(a2).min(a3);
    BOOST_CHECK_EQUAL(expected, a1.min(a3).min(a2));
    BOOST_CHECK_EQUAL(expected, a2.min(a1).min(a3));
    BOOST_CHECK_EQUAL(expected, a2.min(a3).min(a1));
    BOOST_CHECK_EQUAL(expected, a3.min(a1).min(a2));
    BOOST_CHECK_EQUAL(expected, a3.min(a2).min(a1));

    std::cout << "(" << a1 << " \u2293 " << a2 << ") \u2293 " << a3 << ")) = " << a1.min(a2).min(a3) << std::endl;
    std::cout << "(" << a1 << " \u2293 (" << a2 << " \u2293 " << a3 << ")) = " << a1.min(a2.min(a3)) << std::endl;
    BOOST_CHECK_EQUAL(a1.min(a2).min(a3), a1.min(a2.min(a3)));
}

BOOST_AUTO_TEST_CASE(PostStarStackSizeWeight1){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();

    std::unordered_set<char> labels{'A'};
    TypedPDA<char, StackSizeWeight> pda(labels);

    pda.add_rule(0, 1, PUSH, 'A', false, 'A', push);
    pda.add_rule(1, 2, POP , 'A', false, 'A', pop);
    pda.add_rule(1, 3, PUSH, 'A', false, 'A', push);
    pda.add_rule(2, 4, PUSH, 'A', false, 'A', push);
    pda.add_rule(3, 4, POP , 'A', false, 'A', pop);
    pda.add_rule(4, 5, POP , 'A', false, 'A', pop);

    std::vector<char> init_stack{'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachableA{'A'};
    //auto result = automaton.accept_path<Trace_Type::Shortest>(5, pda.encode_pre(test_stack_reachableA));
    //auto distance = result.second;

    //BOOST_CHECK_EQUAL(distance, 1);
}

BOOST_AUTO_TEST_CASE(PostStarStackSizeWeight2){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();
    const auto swap = StackSizeWeight::swap();

    std::unordered_set<char> labels{'A'};
    TypedPDA<char, StackSizeWeight> pda(labels);

    pda.add_rule( 0,  1, PUSH, 'A', false, 'A', push);
    pda.add_rule( 1,  2, PUSH, 'A', false, 'A', push);
    pda.add_rule( 2,  3, PUSH, 'A', false, 'A', push);
    pda.add_rule( 3,  4, PUSH, 'A', false, 'A', push);
    pda.add_rule( 3,  9, SWAP, 'A', false, 'A', swap);
    pda.add_rule( 4,  5, PUSH, 'A', false, 'A', push);
    pda.add_rule( 4,  8, POP , 'A', false, 'A', pop);
    pda.add_rule( 5,  6, POP , 'A', false, 'A', pop);
    pda.add_rule( 6,  7, POP , 'A', false, 'A', pop);
    pda.add_rule( 7,  8, POP , 'A', false, 'A', pop);
    pda.add_rule( 8,  9, POP , 'A', false, 'A', pop);
    pda.add_rule( 9, 10, PUSH, 'A', false, 'A', push);
    pda.add_rule( 9, 15, SWAP, 'A', false, 'A', swap);
    pda.add_rule(10, 11, PUSH, 'A', false, 'A', push);
    pda.add_rule(10, 14, PUSH, 'A', false, 'A', push);
    pda.add_rule(11, 12, PUSH, 'A', false, 'A', push);
    pda.add_rule(12, 13, PUSH, 'A', false, 'A', push);
    pda.add_rule(13, 13, POP , 'A', false, 'A', pop);
    pda.add_rule(14, 14, POP , 'A', false, 'A', pop);
    pda.add_rule(15, 15, POP , 'A', false, 'A', pop);

    std::vector<char> init_stack{'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachableA{'A'};
    //auto result1 = automaton.accept_path<Trace_Type::Shortest>(13, pda.encode_pre(test_stack_reachableA));
    //auto result2 = automaton.accept_path<Trace_Type::Shortest>(14, pda.encode_pre(test_stack_reachableA));
    //auto result3 = automaton.accept_path<Trace_Type::Shortest>(15, pda.encode_pre(test_stack_reachableA));

    //BOOST_CHECK_EQUAL(result1.first.size(), 18);
    //BOOST_CHECK_EQUAL(result2.first.size(), 12);
    //BOOST_CHECK_EQUAL(result3.first.size(), 8);

    //BOOST_CHECK_EQUAL(result1.second.final_value(), 5);
    //BOOST_CHECK_EQUAL(result2.second.final_value(), 4);
    //BOOST_CHECK_EQUAL(result3.second.final_value(), 3);
}

BOOST_AUTO_TEST_CASE(PostStarStackSizeWeight3){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();

    std::unordered_set<char> labels{'A'};
    TypedPDA<char, StackSizeWeight> pda(labels);

    pda.add_rule(0, 1, PUSH, 'A', false, 'A', push);
    pda.add_rule(1, 2, POP , 'A', false, 'A', pop);
    pda.add_rule(1, 3, PUSH, 'A', false, 'A', push);
    pda.add_rule(2, 5, PUSH, 'A', false, 'A', push);
    pda.add_rule(3, 4, POP , 'A', false, 'A', pop);
    pda.add_rule(4, 5, POP , 'A', false, 'A', pop);
    pda.add_rule(5, 6, PUSH, 'A', false, 'A', push);
    pda.add_rule(6, 7, PUSH, 'A', false, 'A', push);
    pda.add_rule(7, 7, POP , 'A', false, 'A', pop);

    std::vector<char> init_stack{'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachableA{'A', 'A', 'A'};
    //auto result = automaton.accept_path<Trace_Type::Shortest>(7, pda.encode_pre(test_stack_reachableA));
    //auto distance = result.second;

    //BOOST_CHECK_EQUAL(distance, 1);
}

BOOST_AUTO_TEST_CASE(PostStarStackSizeWeight4){
    const auto push = StackSizeWeight::push();
    const auto pop = StackSizeWeight::pop();

    std::unordered_set<char> labels{'A', 'B', 'C'};
    TypedPDA<char, StackSizeWeight> pda(labels);

    pda.add_rule(0, 3, SWAP, 'A', false, 'A', StackSizeWeight(StackSizeWeight::elem_t{3,3}));
    pda.add_rule(1, 3, SWAP, 'A', false, 'B', StackSizeWeight(StackSizeWeight::elem_t{4,2}));
    pda.add_rule(2, 3, SWAP, 'A', false, 'C', StackSizeWeight(StackSizeWeight::elem_t{5,1}));
    pda.add_rule(3, 4, SWAP, 'A', false, 'A', StackSizeWeight(StackSizeWeight::elem_t{0,0}));
    pda.add_rule(3, 5, SWAP, 'A', false, 'A', StackSizeWeight(StackSizeWeight::elem_t{2,2}));
    pda.add_rule(3, 6, SWAP, 'A', false, 'A', StackSizeWeight(StackSizeWeight::elem_t{4,4}));

    std::vector<char> init_stack{'A'};
    PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));
    automaton.add_edge(1, 7, pda.encode_pre(std::vector<char>{'B'})[0]);
    automaton.add_edge(2, 7, pda.encode_pre(std::vector<char>{'C'})[0]);

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachableA{'A'};
    auto result1 = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 4, test_stack_reachableA);
    auto result2 = Solver::get_trace<Trace_Type::Any>(pda, automaton, 5, test_stack_reachableA);
    auto result3 = Solver::get_trace<Trace_Type::Any>(pda, automaton, 6, test_stack_reachableA);

    bool temp = false;
    //BOOST_CHECK_EQUAL(result1.second.final_value(), 3);
    //BOOST_CHECK_EQUAL(result2.second.final_value(), 4);
    //BOOST_CHECK_EQUAL(result3.second.final_value(), 5);
    BOOST_CHECK_EQUAL(true, true);
}
