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
 * File:   NFA_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 21-12-2020.
 */

#define BOOST_TEST_MODULE NFA

#include <pdaaal/NFA.h>
#include <boost/test/unit_test.hpp>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(NFA_Test_1)
{
    // A+ || (A (^A)* .)
    // Alphabet: {A,B,C}
    NFA<char> nfa_1(std::unordered_set<char>{'A'});
    nfa_1.plus_extend();
    NFA<char> nfa_2(std::unordered_set<char>{'A'});
    NFA<char> nfa_3(std::unordered_set<char>{'A'}, true); // [B,C] = ^A
    nfa_3.star_extend();
    nfa_2.concat(std::move(nfa_3));
    NFA<char> nfa_4(std::unordered_set<char>{'A', 'B', 'C'});
    nfa_2.concat(std::move(nfa_4));
    nfa_1.or_extend(std::move(nfa_2));

    nfa_1.compile();

    BOOST_CHECK(!nfa_1.empty_accept());
    auto states1 = NFA<char>::successor(nfa_1.initial(), 'A');
    BOOST_CHECK(std::any_of(states1.begin(), states1.end(), [](const auto& state){ return state->_accepting; }));
    std::vector<const NFA<char>::state_t*> const_states1(states1.begin(), states1.end()); // Checking that both std::vector<state_t*> and std::vector<const state_t*> works.
    auto states2 = NFA<char>::successor(const_states1, 'A');
    BOOST_CHECK(std::any_of(states2.begin(), states2.end(), [](const auto& state){ return state->_accepting; }));
    auto states3 = NFA<char>::successor(states2, 'B');
    BOOST_CHECK(!std::any_of(states3.begin(), states3.end(), [](const auto& state){ return state->_accepting; }));
}

BOOST_AUTO_TEST_CASE(NFA_Test_2)
{
    // A B
    // Alphabet: {A,B,C}
    NFA<char> nfa_1(std::unordered_set<char>{'A'});
    NFA<char> nfa_2(std::unordered_set<char>{'B'});
    nfa_1.concat(std::move(nfa_2));

    nfa_1.compile();

    auto states = NFA<char>::successor(nfa_1.initial(), 'A');
    BOOST_CHECK(!states.empty());
    BOOST_CHECK(NFA<char>::has_as_successor(nfa_1.initial(), 'A', states[0]));
    BOOST_CHECK(NFA<char>::has_as_successor(nfa_1.initial()[0], 'A', states[0]));

}

BOOST_AUTO_TEST_CASE(NFA_Test_3)
{
    // [A,B] [^B,C] D
    // Alphabet: {A,B,C,D}
    NFA<char> nfa_1(std::unordered_set<char>{'A','B'});
    NFA<char> nfa_2(std::unordered_set<char>{'B', 'C'}, true);
    NFA<char> nfa_3(std::unordered_set<char>{'D'});
    nfa_1.concat(std::move(nfa_2));
    nfa_1.concat(std::move(nfa_3));

    nfa_1.compile();

    // Define vectors that are used below.
    std::vector<char> a_b_c_d{'A', 'B', 'C', 'D'};
    std::vector<char> a{'A'};
    std::vector<char> a_b{'A', 'B'};
    std::vector<char> a_c{'A', 'C'};
    std::vector<char> a_d{'A', 'D'};

    BOOST_CHECK(!nfa_1.empty_accept());
    auto states1 = NFA<char>::successor(nfa_1.initial(), 'A');
    BOOST_CHECK(!std::any_of(states1.begin(), states1.end(), [](const auto& state){ return state->_accepting; }));
    BOOST_CHECK(!states1.empty());
    auto labels1 = NFA<char>::intersect_edge_labels(nfa_1.initial(), states1[0], a_b_c_d);
    BOOST_CHECK_EQUAL_COLLECTIONS(labels1.begin(), labels1.end(), a_b.begin(), a_b.end()); // We assume that all labels are on the same edge (to states1[0]). Might not be the case for a very different implementation...
    auto labels2 = NFA<char>::intersect_edge_labels(nfa_1.initial(), states1[0], a_c);
    BOOST_CHECK_EQUAL_COLLECTIONS(labels2.begin(), labels2.end(), a.begin(), a.end());

    BOOST_CHECK(NFA<char>::successor(states1, 'B').empty());
    auto states2 = NFA<char>::successor(states1, 'A');
    BOOST_CHECK(!std::any_of(states2.begin(), states2.end(), [](const auto& state){ return state->_accepting; }));
    BOOST_CHECK(!states2.empty());
    auto labels3 = NFA<char>::intersect_edge_labels(states1, states2[0], a_b_c_d);
    BOOST_CHECK_EQUAL_COLLECTIONS(labels3.begin(), labels3.end(), a_d.begin(), a_d.end()); // We assume that all labels are on the same edge (to states1[0]). Might not be the case for a very different implementation...
    auto labels4 = NFA<char>::intersect_edge_labels(states1, states2[0], a_c);
    BOOST_CHECK_EQUAL_COLLECTIONS(labels4.begin(), labels4.end(), a.begin(), a.end());

    auto states3 = NFA<char>::successor(states2, 'D');
    BOOST_CHECK(std::any_of(states3.begin(), states3.end(), [](const auto& state){ return state->_accepting; }));
}