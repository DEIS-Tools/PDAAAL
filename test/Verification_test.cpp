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

#include <parsing/PAutomatonParser.h>
#include <parsing/PdaJsonParser.h>
#include <pdaaal/SolverInstance.h>
#include <pdaaal/Solver.h>

using namespace pdaaal;

template<typename pda_t>
void print_trace(const std::vector<typename pda_t::tracestate_t>& trace, const pda_t& pda, std::ostream& s = std::cout) {
    for (const auto& trace_state : trace) {
        s << "< " << pda.get_state(trace_state._pdastate) << ", [";
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
template<typename Automaton, typename pda_t>
void print_automaton(const Automaton& automaton, const pda_t& pda, std::ostream& s = std::cout) {
    automaton.to_dot(s,
        [&pda](std::ostream& s, const uint32_t& label){ s << pda.get_symbol(label); },
        [&pda](std::ostream& s, const size_t& state_id){
            if (state_id < pda.states().size()) {
                s << pda.get_state(state_id);
            } else {
                s << state_id;
            }
        }
    );
}
template<typename label_t, typename state_t, typename W, bool ssm, bool indirect>
auto get_edge(const PAutomaton<W,indirect>& automaton, const TypedPDA<label_t,W,fut::type::vector,state_t,ssm>& pda, const state_t& from, const label_t& label, const state_t& to) {
    BOOST_TEST(pda.exists_state(from).first);
    auto from_id = pda.exists_state(from).second;
    BOOST_TEST(pda.exists_label(label).first);
    auto label_id = pda.exists_label(label).second;
    BOOST_TEST(pda.exists_state(to).first);
    auto to_id = pda.exists_state(to).second;
    return automaton.states()[from_id]->_edges.get(to_id, label_id);
}

BOOST_AUTO_TEST_CASE(Verification_Test_1)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "Zero": { "A": {"to": "Two", "swap": "B", "weight": 2} },
          "One": { "B": {"to": "Two", "push": "B", "weight": 1} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto initial_p_automaton = PAutomatonParser::parse_string("< [Zero, One] , ([A]?[B])* >", pda);
    auto final_p_automaton = PAutomatonParser::parse_string("< [Two] , [B] [B] [B] >", pda);
    PAutomatonProduct instance(pda, std::move(initial_p_automaton), std::move(final_p_automaton));

    bool result = Solver::post_star_accepts<Trace_Type::Shortest>(instance);

    BOOST_TEST(result);

    auto [trace, weight] = Solver::get_trace<Trace_Type::Shortest>(instance);

    BOOST_TEST(weight == 1);
    BOOST_TEST(trace.size() == 2);

    std::cout << "Weight: " << weight << std::endl;
    print_trace(trace, pda);
}

BOOST_AUTO_TEST_CASE(Verification_negative_weight_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p":  { "X":[{"to": "p'", "swap": "Y", "weight": 1},
                       {"to": "q", "swap": "Y", "weight": 1}],
                  "Y": {"to": "p", "pop": "", "weight": 1} },
          "p'": { "Y": {"to": "p", "push": "X", "weight": 0} },
          "q":  { "Y": {"to": "q", "pop": "", "weight": -2} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton = PAutomatonParser::parse_string("< [q] , >", pda);

    Solver::pre_star_fixed_point(p_automaton);

    std::stringstream s;
    print_automaton(p_automaton, pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto pXq = get_edge<std::string,std::string>(p_automaton, pda, "p", "X", "q");
    BOOST_TEST(pXq != nullptr);
    BOOST_TEST(pXq->second == weight<int32_t>::bottom());

    auto pYp = get_edge<std::string,std::string>(p_automaton, pda, "p", "Y", "p");
    BOOST_TEST(pYp != nullptr);
    BOOST_TEST(pYp->second == 1);

    auto qYq = get_edge<std::string,std::string>(p_automaton, pda, "q", "Y", "q");
    BOOST_TEST(qYq != nullptr);
    BOOST_TEST(qYq->second == -2);
}
