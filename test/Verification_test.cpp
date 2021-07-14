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
 * File:   Verification_test
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 14-07-2021.
 */

#define BOOST_TEST_MODULE Verification_test

#include <boost/test/unit_test.hpp>

#include <pdaaal/parsing/PAutomatonParser.h>
#include <pdaaal/parsing/PdaJsonParser.h>
#include <pdaaal/SolverInstance.h>
#include <pdaaal/Solver.h>

using namespace pdaaal;

template<typename Trace, typename Instance>
void print_trace(const Trace& trace, const Instance& instance, std::ostream& s = std::cout) {
    for (const auto& trace_state : trace) {
        s << "< " << instance.pda().get_state(trace_state._pdastate) << ", [";
        bool first = true;
        for (const auto& label : trace_state._stack) {
            if (first) {
                first = false;
            } else {
                s << ", ";
            }
            s << label;
        }
        s << "] >" << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(Verification_Test_1)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "Zero": {
            "A": {"to": "Two", "swap": "B", "weight": 2}
          },
          "One": {
            "B": {"to": "Two", "push": "B", "weight": 1}
          },
          "Two": {}
        }
      }
    })");
//    std::stringstream warnings;
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    std::string initial_automaton_string = "< [Zero, One] , ([A]?[B])* >";
    std::string final_automaton_string = " < [Two] , [B] [B] [B] > ";
    auto initial_p_automaton = PAutomatonParser::parse_string(initial_automaton_string, pda);
    auto final_p_automaton = PAutomatonParser::parse_string(final_automaton_string, pda);
    SolverInstance instance(std::move(pda), std::move(initial_p_automaton), std::move(final_p_automaton));

    bool result = Solver::post_star_accepts<Trace_Type::Shortest>(instance);

    BOOST_TEST(result);

    auto [trace, weight] = Solver::get_trace<Trace_Type::Shortest>(instance);

    BOOST_TEST(weight == 1);
    BOOST_TEST(trace.size() == 2);

    std::cout << "Weight: " << weight << std::endl;
    print_trace(trace, instance);
}
