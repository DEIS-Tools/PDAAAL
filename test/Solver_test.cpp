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

#include <pdaaal/Solver.h>
#include <boost/test/unit_test.hpp>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(SolverTest1)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char, weight<std::array<double, 3>>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', 'A', w);
    pda.add_rule(0, 0, POP , '*', 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    auto [trace,weight] = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 1, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(SolverTest2)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<uint32_t> labels{2,3,4};
    PDA<uint32_t, weight<std::array<double, 3>>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 3, 2, w);
    pda.add_rule(0, 0, POP , '*', 3, w);
    pda.add_rule(1, 3, SWAP, 2, 3, w);
    pda.add_rule(2, 0, SWAP, 3, 4, w);
    pda.add_rule(3, 2, PUSH, 4, 2, w);

    std::vector<uint32_t> init_stack{2, 2};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<uint32_t> test_stack_reachable{3, 2, 2, 2};
    auto [trace,weight] = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 1, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(SolverTest3)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char, weight<std::array<double, 3>>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', 'A', w);
    pda.add_rule(0, 0, POP , '*', 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    auto stack_native = pda.encode_pre(test_stack_reachable);
    auto [trace,weight] = Solver::get_trace<Trace_Type::Shortest>(pda, automaton, 1, stack_native);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(EarlyTerminationPostStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char, weight<std::array<double, 3>>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', 'A', w);
    pda.add_rule(0, 0, POP , '*', 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    auto stack_native = pda.encode_pre(test_stack_reachable);
    PAutomatonProduct instance(pda, std::move(automaton), internal::PAutomaton(pda, 1, stack_native));
    auto result = Solver::post_star_accepts(instance);
    BOOST_CHECK_EQUAL(result, true);

    auto trace = Solver::get_trace(pda, instance.automaton(), 1, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 7);
}

BOOST_AUTO_TEST_CASE(EarlyTerminationPreStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char, weight<std::array<double, 3>>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', 'A', w);
    pda.add_rule(0, 0, POP , '*', 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', 'A', w);

    std::vector<char> init_stack{'B', 'A', 'A', 'A'};
    internal::PAutomaton automaton(pda, 1, pda.encode_pre(init_stack));

    std::vector<char> test_stack_reachable{'A'};
    auto result = Solver::pre_star_accepts<Trace_Type::None>(automaton, 0, pda.encode_pre(test_stack_reachable));
    BOOST_REQUIRE_EQUAL(result, true);

    auto trace = Solver::get_trace(pda, automaton, 0, test_stack_reachable);
    BOOST_CHECK_EQUAL(trace.size(), 12);
}

BOOST_AUTO_TEST_CASE(Post_0AApop1A)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 1, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(std::unordered_set<char>{'A'});
    std::vector<size_t> final_states{1};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::post_star_accepts(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
}

BOOST_AUTO_TEST_CASE(POST_0Apop1)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 1, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(true);
    std::vector<size_t> final_states{1};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::post_star_accepts(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
}

BOOST_AUTO_TEST_CASE(Pre_0AApop1A)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 1, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(std::unordered_set<char>{'A'});
    std::vector<size_t> final_states{1};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::pre_star_accepts<Trace_Type::None>(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
}

BOOST_AUTO_TEST_CASE(Pre_0Apop1)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 1, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(true);
    std::vector<size_t> final_states{1};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::pre_star_accepts<Trace_Type::None>(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
}

BOOST_AUTO_TEST_CASE(Pre_0AApop0)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 0, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(true);
    std::vector<size_t> final_states{0};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::pre_star_accepts<Trace_Type::None>(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 3);
}

BOOST_AUTO_TEST_CASE(Pre_0AApop0A)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 0, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(std::unordered_set<char>{'A'});
    std::vector<size_t> final_states{0};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::pre_star_accepts<Trace_Type::None>(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 3);
}

BOOST_AUTO_TEST_CASE(Post_0AApop0)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 0, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(true);
    std::vector<size_t> final_states{0};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::post_star_accepts(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 3);
}

BOOST_AUTO_TEST_CASE(Post_0AAApop0A)
{
    std::unordered_set<char> labels{'A'};
    PDA<char> pda(labels);
    pda.add_rule(0, 0, POP, '*', 'A');

    NFA<char> initial_nfa(std::unordered_set<char>{'A'});
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    initial_nfa.concat(NFA<char>(std::unordered_set<char>{'A'}));
    std::vector<size_t> initial_states{0};
    NFA<char> final_nfa(std::unordered_set<char>{'A'});
    std::vector<size_t> final_states{0};
    PAutomatonProduct instance(pda, initial_nfa, initial_states, final_nfa, final_states);

    bool result = Solver::post_star_accepts(instance);
    BOOST_CHECK(result);
    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 3);
}
