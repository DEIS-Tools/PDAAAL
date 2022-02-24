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

#include <pdaaal/PAutomaton.h>
#include <pdaaal/PDA.h>
#include <pdaaal/Solver.h>
#include <boost/test/unit_test.hpp>
#include <chrono>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(Dijkstra_Test_1)
{
    using trace_t = internal::trace_t;

    std::unordered_set<char> labels{'A'};
    PDA<char, weight<int>> pda(labels);
    pda.add_rule(0, 0, POP, '*', 'A', 0);

    internal::PAutomaton automaton(pda, std::vector<size_t>());
    auto id1 = automaton.add_state(false, false);
    auto id2 = automaton.add_state(false, false);
    auto id3 = automaton.add_state(false, false);
    auto id4 = automaton.add_state(false, true);

    automaton.add_edge(0, id1, 0, std::make_pair(trace_t(), 1));
    automaton.add_edge(0, id2, 0, std::make_pair(trace_t(), 2));
    automaton.add_edge(id1, id3, 0, std::make_pair(trace_t(), 2));
    automaton.add_edge(id3, id4, 0, std::make_pair(trace_t(), 2));

    std::vector<char> test_stack{'A', 'A', 'A'};
    std::vector<size_t> correct_path{0,id1,id3,id4};
    auto [path, w] = automaton.template accept_path<Trace_Type::Shortest>(0, pda.encode_pre(test_stack));
    BOOST_CHECK_EQUAL(w, 5);
    BOOST_CHECK_EQUAL_COLLECTIONS(path.begin(), path.end(), correct_path.begin(), correct_path.end());

    automaton.add_edge(id2, id3, 0, std::make_pair(trace_t(), 2));

    // A bug in the implementation of Dijkstra in PAutomaton caused the following to fail. It is now fixed.
    auto [path2, w2] = automaton.template accept_path<Trace_Type::Shortest>(0, pda.encode_pre(test_stack));
    BOOST_CHECK_EQUAL(w2, 5);
    BOOST_CHECK_EQUAL_COLLECTIONS(path2.begin(), path2.end(), correct_path.begin(), correct_path.end());
}

BOOST_AUTO_TEST_CASE(UnweightedPreStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', 'A');
    pda.add_rule(0, 0, POP, '*', 'B');
    pda.add_rule(1, 3, SWAP, 'A', 'B');
    pda.add_rule(2, 0, SWAP, 'B', 'C');
    pda.add_rule(3, 2, PUSH, 'C', 'A');

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::pre_star(automaton);

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
    PDA<char> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', 'A');
    pda.add_rule(0, 0, POP, '*', 'B');
    pda.add_rule(1, 3, SWAP, 'A', 'B');
    pda.add_rule(2, 0, SWAP, 'B', 'C');
    pda.add_rule(3, 2, PUSH, 'C', 'A');

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(0, pda.encode_pre(test_stack_unreachable)), false);

    automaton.to_dot(std::cout, [&pda](auto &s, auto &l) { s << pda.get_symbol(l); });

}

BOOST_AUTO_TEST_CASE(UnweightedPostStarPath)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', 'A');
    pda.add_rule(0, 0, POP, '*', 'B');
    pda.add_rule(1, 3, SWAP, 'A', 'B');
    pda.add_rule(2, 0, SWAP, 'B', 'C');
    pda.add_rule(3, 2, PUSH, 'C', 'A');

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star(automaton);

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    BOOST_CHECK_EQUAL(automaton.accept_path(1, pda.encode_pre(test_stack_reachable)).size(), 5);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accept_path(0, pda.encode_pre(test_stack_unreachable)).size(), 0);
}

BOOST_AUTO_TEST_CASE(WeightedPreStar)
{
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char, weight<std::vector<int>>> pda(labels);
    std::vector<int> w{1};
    pda.add_rule(0, 1, PUSH, 'B', 'A', w);
    pda.add_rule(0, 0, POP , '*', 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', 'A', w);

    std::vector<char> init_stack{'A', 'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::pre_star(automaton);

    std::vector<char> test_stack_reachable{'C', 'B', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'C', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_unreachable)), false);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar4EarlyTermination)
{
    std::unordered_set<char> labels{'A'};
    PDA<char, weight<int>> pda(labels);

    pda.add_rule(0, 3, PUSH, 'A', 'A', 4);
    pda.add_rule(0, 1, PUSH , 'A', 'A', 1);
    pda.add_rule(3, 1, PUSH , 'A', 'A', 8);
    pda.add_rule(1, 2, POP , 'A', 'A', 2);
    pda.add_rule(2, 4, POP , 'A', 'A', 16);

    std::vector<char> init_stack{'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    std::vector<char> test_stack_reachable{'A'};
    BOOST_CHECK_EQUAL(Solver::post_star_accepts<Trace_Type::Shortest>(automaton, 4, pda.encode_pre(test_stack_reachable)), true);

    auto result4A = automaton.accept_path<Trace_Type::Shortest>(4, pda.encode_pre(test_stack_reachable));
    auto distance4A = result4A.second;
    BOOST_CHECK_EQUAL(distance4A, 30);          //Example Derived on whiteboard
}

BOOST_AUTO_TEST_CASE(WeightedPostStar)
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
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(0, pda.encode_pre(test_stack_unreachable)), false);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar2)
{
    std::unordered_set<char> labels{'A', 'B'};
    PDA<char, weight<int>> pda(labels);

    pda.add_rule(1, 2, POP, '*', 'A', 1);
    pda.add_rule(1, 3, PUSH , 'B', 'A', 3);
    pda.add_rule(1, 3, SWAP, 'A',  'B', 2);
    pda.add_rule(2, 1, POP, '*',  'B', 4);
    std::vector<char> pre{'A', 'B'};
    pda.add_rule(2, 2, PUSH, 'B', false, pre, 5);
    pda.add_rule(3, 1, POP, '*', 'B', 1);

    std::vector<char> init_stack{'A', 'B', 'A'};
    internal::PAutomaton automaton(pda, 1, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar3)
{
    std::unordered_set<char> labels{'A'};
    PDA<char, weight<int>> pda(labels);
    std::vector<char> pre{'A'};

    pda.add_rule(1, 2, PUSH, 'A', 'A', 16);
    pda.add_rule(1, 3, PUSH , 'A', 'A', 1);
    pda.add_rule(3, 3, PUSH , 'A', 'A', 2);
    pda.add_rule(3, 2, POP , 'A', 'A', 1);

    std::vector<char> init_stack{'A'};
    internal::PAutomaton automaton(pda, 1, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'A','A'};
    BOOST_CHECK_EQUAL(automaton.accepts(2, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStar4)
{
    std::unordered_set<char> labels{'A'};
    PDA<char, weight<int>> pda(labels);

    pda.add_rule(0, 3, PUSH, 'A', 'A', 4);
    pda.add_rule(0, 1, PUSH , 'A', 'A', 1);
    pda.add_rule(3, 1, PUSH , 'A', 'A', 8);
    pda.add_rule(1, 2, POP , 'A', 'A', 2);
    pda.add_rule(2, 4, POP , 'A', 'A', 16);

    std::vector<char> init_stack{'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachable{'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(4, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStarResult)
{
    std::unordered_set<char> labels{'A'};
    PDA<char, weight<int>> pda(labels);

    pda.add_rule(0, 3, PUSH, 'A', 'A', 4);
    pda.add_rule(0, 1, PUSH , 'A', 'A', 1);
    pda.add_rule(3, 1, PUSH , 'A', 'A', 8);
    pda.add_rule(1, 2, POP , 'A', 'A', 2);
    pda.add_rule(2, 4, POP , 'A', 'A', 16);

    std::vector<char> init_stack{'A'};
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<char> test_stack_reachableA{'A'};
    auto result4A = automaton.accept_path<Trace_Type::Shortest>(4, pda.encode_pre(test_stack_reachableA));
    auto distance4A = result4A.second;

    std::vector<char> test_stack_reachableAA{'A','A'};
    auto result2AA = automaton.accept_path<Trace_Type::Shortest>(2, pda.encode_pre(test_stack_reachableAA));
    auto distance2AA = result2AA.second;

    BOOST_CHECK_EQUAL(distance4A, 30);          //Example Derived on whiteboard
    BOOST_CHECK_EQUAL(distance2AA, 14);         //Example Derived on whiteboard
}

PDA<int,weight<int>> create_syntactic_network_broad(int network_size = 2) {
    std::unordered_set<int> labels{0,1,2};
    int start_state = 0;
    int states = 4;
    int end_state = 4;

    PDA<int, weight<int>> pda(labels);

    for (int j = 0; j < network_size; j++) {
        pda.add_rule(start_state, 1 + start_state, PUSH, 0, 0, 0);
        pda.add_rule(start_state, 1 + start_state, PUSH, 1, 0, 1);
        pda.add_rule(start_state, 1 + start_state, PUSH, 2, 0, 1);
        pda.add_rule(start_state, 2 + start_state, PUSH, 0, 2, 0);
        pda.add_rule(start_state, 3 + start_state, POP, 0, 1, 1);

        pda.add_rule(1 + start_state, 3 + start_state, PUSH, 1, 2, 1);
        pda.add_rule(1 + start_state, end_state, PUSH, 0, 0, 1);
        pda.add_rule(1 + start_state, end_state, PUSH, 1, 1, 1);

        for (size_t i = 0; i < labels.size(); i++) {
            pda.add_rule(2 + start_state, 2 + start_state, POP, 0, i, 5);
        }

        pda.add_rule(2 + start_state, end_state, PUSH, 0, 0, 1);

        pda.add_rule(3 + start_state, 2 + start_state, POP, 0, 2, 1);
        pda.add_rule(3 + start_state, end_state, PUSH, 2, 0, 1);
        pda.add_rule(3 + start_state, end_state, PUSH, 2, 1, 1);

        start_state = end_state;
        end_state = end_state + states;
    }
    return pda;
}

PDA<int,weight<int>> create_syntactic_network_deep(int network_size = 2){
    std::unordered_set<int> labels{0,1,2};
    PDA<int, weight<int>> pda(labels);
    int start_state = 0;
    int new_start_state = 4;
    int end_state = 2;
    int new_end_state = 6;

    for(int j = 0; j < network_size; j++){
        pda.add_rule(start_state, 1+start_state, POP, 2, 1, 1);

        pda.add_rule(1+start_state, end_state, SWAP, 2, 0, 1);

        pda.add_rule(end_state, 3+start_state, POP, 1, 2, 1);

        pda.add_rule(3+start_state, start_state, SWAP, 1, 0, 1);

        for(size_t i = 0; i < labels.size(); i++){
            for(size_t k = 0; k < labels.size(); k++) {
                pda.add_rule(new_start_state, start_state, PUSH, i, k, 1);
            }
        }
        for(size_t i = 0; i < labels.size(); i++){
            for(size_t k = 0; k < labels.size(); k++) {
                pda.add_rule(end_state, new_end_state, PUSH, i, k, 1);
            }
        }
        start_state = new_start_state;
        end_state = new_end_state;
        new_start_state = new_start_state + 4;
        new_end_state = new_end_state + 4;
    }
    return pda;
}

BOOST_AUTO_TEST_CASE(WeightedPostStarSyntacticModel)
{
    PDA<int,weight<int>> pda = create_syntactic_network_broad(1);
    std::vector<int> init_stack;
    init_stack.push_back(0);
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));

    Solver::post_star<Trace_Type::Shortest>(automaton);

    std::vector<int> test_stack_reachable;
    test_stack_reachable.push_back(0);
    test_stack_reachable.push_back(0);
    test_stack_reachable.push_back(0);

    BOOST_CHECK_EQUAL(automaton.accepts(4, pda.encode_pre(test_stack_reachable)), true);
}

BOOST_AUTO_TEST_CASE(WeightedPostStarVSPostUnorderedPerformance)
{
    std::unordered_set<int> labels;
    int alphabet_size = 10000;

    //Insert labels alphabet
    for(int i = 0; i < alphabet_size; i++){
        labels.insert(i);
    }

    PDA<int, weight<int>> pda(labels);

    for(int i = 0; i < alphabet_size; i++){
        pda.add_rule(0, 1, SWAP, i, 0, 1);
        pda.add_rule(1, 2, SWAP, 0, i, i);
        pda.add_rule(2, 3, PUSH, i, 0, 1);
    }

    std::vector<int> init_stack;
    init_stack.push_back(0);

    std::vector<int> test_stack_reachable;
    test_stack_reachable.push_back(0);

    internal::PAutomaton shortest_automaton(pda, 0, pda.encode_pre(init_stack));
    auto t1 = std::chrono::high_resolution_clock::now();
    Solver::post_star<Trace_Type::Shortest>(shortest_automaton);
    auto t2 = std::chrono::high_resolution_clock::now();
    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));
    auto t3 = std::chrono::high_resolution_clock::now();
    Solver::post_star<Trace_Type::Any>(automaton);
    auto t4 = std::chrono::high_resolution_clock::now();

    auto duration_short_post = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    auto duration_post = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();

    BOOST_TEST_MESSAGE( "ShortestTrace: " << std::to_string(duration_short_post) << " PostStar: " << std::to_string(duration_post));
}


BOOST_AUTO_TEST_CASE(WeightedShortestPerformance)
{
    PDA<int, weight<int>> pda = create_syntactic_network_deep(200);

    std::vector<int> init_stack;
    init_stack.push_back(0);

    std::vector<int> test_stack_reachable;
    test_stack_reachable.push_back(0);
    test_stack_reachable.push_back(0);
    test_stack_reachable.push_back(0);
    test_stack_reachable.push_back(0);

    internal::PAutomaton shortest_automaton(pda, 0, pda.encode_pre(init_stack));
    auto t1 = std::chrono::high_resolution_clock::now();
    Solver::post_star<Trace_Type::Shortest>(shortest_automaton);
    auto t2 = std::chrono::high_resolution_clock::now();

    internal::PAutomaton automaton(pda, 0, pda.encode_pre(init_stack));
    auto t3 = std::chrono::high_resolution_clock::now();
    Solver::post_star<Trace_Type::Any>(automaton);
    auto t4 = std::chrono::high_resolution_clock::now();

    auto duration_short_post = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    auto duration_post = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();

    BOOST_TEST_MESSAGE( "ShortestTrace: " << std::to_string(duration_short_post) << " PostStar: " << std::to_string(duration_post));
}

