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
 * File:   testmain.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 03-01-2020.
 */
#define BOOST_TEST_MODULE Test1

#include <boost/test/unit_test.hpp>

#include "TestPDAFactory.h"
#include <pdaaal/Solver.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(PassTest)
{
    BOOST_CHECK_EQUAL(true, true);
}

BOOST_AUTO_TEST_CASE(TestFactoryTest)
{
    TestPDAFactory testFactory{};
    TypedPDA<char> testPDA = testFactory.compile();

    std::vector<char> stack{'A', 'A'};
    PAutomaton test{testPDA, 0, testPDA.encode_pre(stack)};
    auto test2 = test;

    Solver::pre_star(test);

    Solver::post_star(test2);

    std::vector<char> test_stack{'B', 'A', 'A', 'A'};
    auto trace = Solver::get_trace(testPDA, test, 1, test_stack);
    BOOST_CHECK_EQUAL(trace.size(), 0);

    auto trace2 = Solver::get_trace(testPDA, test2, 1, test_stack);
    BOOST_CHECK_EQUAL(trace2.size(), 7);
}
