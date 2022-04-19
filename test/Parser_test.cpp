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
 * File:   Parser_test
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 24-02-2022.
 */

#define BOOST_TEST_MODULE Parser_test

#include <parsing/PAutomatonJsonParser.h>
#include <pdaaal/Solver.h>
#include <boost/test/unit_test.hpp>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(PAutomatonFromJson_OldFormat_Test)
{
    std::unordered_set<std::string> labels{"A"};
    PDA<std::string> pda(labels);
    pda.add_rule(0, 0, POP, "*", "A");
    std::istringstream automaton_stream(R"({"P-automaton":{
        "states":[
            {"edges":[{"label":"A","to":1}],"initial":true},
            {"edges":[{"label":"A","to":2}]},
            {"accepting":true,"edges":[]}
        ]
    }})");
    auto automaton = parsing::PAutomatonJsonParser_Old::parse<>(automaton_stream, pda);
    BOOST_CHECK_EQUAL(automaton.states().size(), 3);
    std::vector<uint32_t> stack; stack.emplace_back(0);
    bool result = Solver::post_star_accepts(automaton, 0, stack);
    BOOST_CHECK(result);
}

BOOST_AUTO_TEST_CASE(PAutomatonFromJson_NewFormat_Test)
{
    std::unordered_set<std::string> labels{"A"};
    PDA<std::string> pda(labels);
    pda.add_rule(0, 0, POP, "*", "A");
    std::istringstream automaton_stream(R"({"P-automaton":{
        "initial":[0],
        "accepting":[2],
        "edges":[
            [0,"A",1],
            [1,"A",2]
        ]
    }})");
    auto automaton = parsing::PAutomatonJsonParser::parse<>(automaton_stream, pda);
    BOOST_CHECK_EQUAL(automaton.states().size(), 3);
    std::vector<uint32_t> stack; stack.emplace_back(0);
    bool result = Solver::post_star_accepts(automaton, 0, stack);
    BOOST_CHECK(result);
}

BOOST_AUTO_TEST_CASE(PAutomatonToJsonTest)
{
    std::unordered_set<std::string> labels{"A"};
    PDA<std::string> pda(labels);
    pda.add_rule(0, 0, POP, "*", "A");
    std::vector<std::string> init_stack{"A", "A"};
    PAutomaton automaton(pda, 0, init_stack);
    auto j = automaton.to_json();
    BOOST_TEST_MESSAGE(j.dump());
    BOOST_CHECK_EQUAL(j.dump(), R"({"P-automaton":{"accepting":[2],"edges":[[0,"A",1],[1,"A",2]],"initial":[0]}})");
}
