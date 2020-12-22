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
 * File:   ParsingPDAFactory_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 22-12-2020.
 */

#define BOOST_TEST_MODULE ParsingPDAFactory

#include <boost/test/unit_test.hpp>
#include <pdaaal/ParsingPDAFactory.h>
#include <pdaaal/Solver.h>
#include <iostream>
#include <sstream>

using namespace pdaaal;

template <typename T>
void print_trace(std::vector<typename TypedPDA<T>::tracestate_t> trace, std::ostream& s = std::cout) {
    for (const auto& conf : trace) {
        s << conf._pdastate << ";[";
        for (size_t i = 0; i < conf._stack.size(); ++i) {
            s << conf._stack[i];
            if (i + 1 < conf._stack.size()) {
                s << ",";
            }
        }
        s << "]" << std::endl;
    }
    s << std::endl;
}

BOOST_AUTO_TEST_CASE(ParsingPDAFactory_Test_1)
{
    std::istringstream i_stream(
R"(
# You can make a comment like this

# Labels
A,B
# Initial states
0
# Accepting states
0
# Rules
0 A -> 2 B
0 B -> 0 A
0 A -> 1 -
1 B -> 2 +B
2 B -> 0 -

# POP  rules use -
# PUSH rules use +LABEL
# SWAP rules use LABEL
)");
    auto factory = ParsingPDAFactory<>::make_parsing_pda_factory(i_stream);

    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp(std::unordered_set<std::string>{"B"});
    initial.concat(std::move(temp));
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    auto instance = factory.compile(initial, final);
    bool result = Solver::post_star_accepts(instance);
    BOOST_CHECK(result);

    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);

    print_trace<std::string>(trace);
}
