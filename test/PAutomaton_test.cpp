//
// Created by Morten on 19-02-2020.
//
#define BOOST_TEST_MODULE PAutomaton

#include <boost/test/unit_test.hpp>
#include <pdaaal/PAutomaton.h>
#include <pdaaal/TypedPDA.h>

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

    automaton.pre_star();

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

    automaton.post_star();

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

    automaton.pre_star();

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

    automaton.post_star();

    std::vector<char> test_stack_reachable{'B', 'A', 'A', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(1, pda.encode_pre(test_stack_reachable)), true);

    std::vector<char> test_stack_unreachable{'A', 'A', 'B', 'A'};
    BOOST_CHECK_EQUAL(automaton.accepts(0, pda.encode_pre(test_stack_unreachable)), false);
}

