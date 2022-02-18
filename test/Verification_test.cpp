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

#include <parsing/PAutomatonParser.h>
#include <parsing/PdaJsonParser.h>
#include <pdaaal/SolverInstance.h>
#include <pdaaal/Solver.h>
#include <boost/test/unit_test.hpp>

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
template<Trace_Type trace_type = Trace_Type::Any, typename Automaton, typename pda_t>
void print_automaton(const Automaton& automaton, const pda_t& pda, std::ostream& s = std::cout) {
    automaton.template to_dot<trace_type>(s,
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
template <typename pda_t>
std::ostream& print_state(std::ostream& s, size_t state_id, const pda_t& pda) {
    if (state_id < pda.states().size()) {
        s << pda.get_state(state_id);
    } else {
        s << state_id;
    }
    return s;
}
template <typename pda_t>
void print_conf_path(std::ostream& s, const AutomatonPath<>& automaton_path, const pda_t& pda) {
    s << "< ";
    print_state(s, automaton_path.front_state(), pda) << ", [";
    bool first = true;
    for (auto l : automaton_path.stack()) {
        if (first) {
            first  = false;
        } else {
            s << ", ";
        }
        s << pda.get_symbol(l);
    }
    s << "] >" << std::endl;
}
template <typename pda_t>
void print_edges_path(std::ostream& s, const AutomatonPath<>& automaton_path, const pda_t& pda) {
    auto [path, stack] = automaton_path.get_path_and_stack();
    assert(path.size() == stack.size() + 1);
    print_state(s, path[0], pda);
    for (size_t i = 0; i < stack.size(); ++i) {
        s << " --" << pda.get_symbol(stack[i]) << "-> ";
        print_state(s, path[i+1], pda);
    }
    s << std::endl;
}
template<typename label_t, typename state_t, typename W, bool ssm, TraceInfoType trace_info_type>
auto get_edge(const internal::PAutomaton<W,trace_info_type>& automaton, const PDA<label_t,W,fut::type::vector,state_t,ssm>& pda, const state_t& from, const label_t& label, const state_t& to) {
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

    BOOST_CHECK_EQUAL(weight, 1);
    BOOST_CHECK_EQUAL(trace.size(), 2);

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
    auto p_automaton = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [q] , >", pda);

    Solver::pre_star_fixed_point<Trace_Type::Shortest>(p_automaton);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(p_automaton, pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto pXq = get_edge<std::string,std::string>(p_automaton, pda, "p", "X", "q");
    BOOST_CHECK_NE(pXq, nullptr);
    BOOST_CHECK_EQUAL(pXq->second, min_weight<int32_t>::bottom());

    auto pYp = get_edge<std::string,std::string>(p_automaton, pda, "p", "Y", "p");
    BOOST_CHECK_NE(pYp, nullptr);
    BOOST_CHECK_EQUAL(pYp->second, 1);

    auto qYq = get_edge<std::string,std::string>(p_automaton, pda, "q", "Y", "q");
    BOOST_CHECK_NE(qYq, nullptr);
    BOOST_CHECK_EQUAL(qYq->second, -2);
}

BOOST_AUTO_TEST_CASE(Verification_negative_weight_loop_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p":  { "X":{"to": "p", "pop":"", "weight": -1} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , >", pda);

    Solver::pre_star_fixed_point<Trace_Type::Shortest>(p_automaton);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(p_automaton, pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto pXp = get_edge<std::string,std::string>(p_automaton, pda, "p", "X", "p");
    BOOST_CHECK_NE(pXp, nullptr);
    BOOST_CHECK_EQUAL(pXp->second, -1);
}

BOOST_AUTO_TEST_CASE(Verification_negative_weight_loop_path_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p":  { "X":{"to": "p", "pop":"", "weight": -1} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , .* >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    Solver::pre_star_fixed_point_accepts<Trace_Type::Shortest>(instance);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(instance.automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Shortest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [path, w] = instance.find_path<Trace_Type::ShortestFixedPoint>();
    BOOST_CHECK_EQUAL(w, min_weight<int32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Verification_negative_weight_loop_path2_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p":  { "X":{"to": "p", "pop":"", "weight": -1} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , .+ >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    Solver::pre_star_fixed_point_accepts<Trace_Type::Shortest>(instance);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(instance.automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Shortest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [path, w] = instance.find_path<Trace_Type::ShortestFixedPoint>();
    BOOST_CHECK_EQUAL(w, min_weight<int32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Verification_negative_weight_loop_not_accepting_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p":  { "X":{"to": "p", "pop":"", "weight": -1} },
          "q":  { "Y":{"to": "p", "swap":"X", "weight": 1} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [q] , [Y] .+ >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    Solver::pre_star_fixed_point_accepts<Trace_Type::Shortest>(instance);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [path, w] = instance.find_path<Trace_Type::ShortestFixedPoint>();
    BOOST_CHECK_EQUAL(w, min_weight<int32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Verification_negative_weight_finite_path_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "a":  { "X":[{"to": "b", "push":"X", "weight": -1},
                       {"to": "b", "push":"Y", "weight": -4}] },
          "b":  { "X":{"to": "c", "push":"X", "weight": -1},
                  "Y":{"to": "c", "push":"Y", "weight": -1} },
          "c":  { "X":{"to": "c", "pop":"", "weight": -2},
                  "Y":{"to": "c", "pop":"", "weight": -1} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [a] , [X] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [c] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    Solver::pre_star_fixed_point_accepts<Trace_Type::Shortest>(instance);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(instance.product_automaton(), pda, s);

    auto [path, w] = instance.find_path<Trace_Type::ShortestFixedPoint>();
    BOOST_CHECK_EQUAL(w, -9);

    auto [trace, weight] = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
    BOOST_CHECK_EQUAL(w, weight);

    BOOST_CHECK_EQUAL(trace.size(), 6);

    s << std::endl;
    print_trace(trace, pda, s);
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_poststar_pop_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X": {"to":"p", "pop":""} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<void>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string("< [p] , [X] [X] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string("< [p] , [X] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::post_star_accepts(instance);
    BOOST_TEST(result);

    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
    BOOST_CHECK_EQUAL(trace[0]._stack.size(), 2);
    BOOST_CHECK_EQUAL(trace[1]._stack.size(), 1);

    std::stringstream s;
    print_trace(trace, pda, s);
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_poststar_empty_final_stack_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X": {"to":"p", "pop":""} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<void>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string("< [p] , [X] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::post_star_accepts(instance);
    BOOST_TEST(result);

    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
    BOOST_CHECK_EQUAL(trace[0]._stack.size(), 1);
    BOOST_CHECK_EQUAL(trace[1]._stack.size(), 0);

    std::stringstream s;
    print_trace(trace, pda, s);
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_poststar_no_ET_empty_final_stack_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X": {"to":"p", "pop":""} }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<void>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string("< [p] , [X] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::post_star_accepts_no_ET(instance);
    BOOST_TEST(result);

    auto trace = Solver::get_trace(instance);
    BOOST_CHECK_EQUAL(trace.size(), 2);
    BOOST_CHECK_EQUAL(trace[0]._stack.size(), 1);
    BOOST_CHECK_EQUAL(trace[1]._stack.size(), 0);

    std::stringstream s;
    print_trace(trace, pda, s);
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_negative_ring_push_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X1":[{"to":"p", "pop":"", "weight": -1},
                       {"to":"q", "swap":"X2", "weight": 0}],
                 "X2": {"to":"q", "swap":"X3", "weight": 0},
                 "X3": {"to":"q", "swap":"Xn", "weight": 0},
                 "Xn": {"to":"q", "swap":"X1", "weight": 0}
               },
          "q": { "X1": {"to":"p", "push":"X1", "weight": 0},
                 "X2": {"to":"p", "push":"X2", "weight": 0},
                 "X3": {"to":"p", "push":"X3", "weight": 0},
                 "Xn": {"to":"p", "push":"Xn", "weight": 0}
               }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , [X1] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , [X1] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Shortest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [path, w] = instance.find_path<Trace_Type::ShortestFixedPoint>();
    BOOST_CHECK_EQUAL(w, min_weight<int32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Verification_negative_ring_swap_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X1":[{"to":"p", "pop":"", "weight": -1},
                       {"to":"p", "push":"X2", "weight": 0}],
                 "X2": {"to":"p", "swap":"X3", "weight": 0},
                 "X3": {"to":"p", "swap":"X4", "weight": 0},
                 "X4": {"to":"p", "swap":"X5", "weight": 0},
                 "X5": {"to":"p", "swap":"Xn", "weight": 0},
                 "Xn": {"to":"p", "swap":"X1", "weight": 0}
               }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<int32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , [X1] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Shortest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Shortest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [path, w] = instance.find_path<Trace_Type::ShortestFixedPoint>();
    BOOST_CHECK_EQUAL(w, min_weight<int32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X1":[{"to":"p", "pop":"", "weight": 1},
                       {"to":"p", "push":"X2", "weight": 0}],
                 "X2": {"to":"p", "swap":"X3", "weight": 0},
                 "X3": {"to":"p", "swap":"X4", "weight": 0},
                 "X4": {"to":"p", "swap":"X5", "weight": 0},
                 "X5": {"to":"p", "swap":"Xn", "weight": 0},
                 "Xn": {"to":"p", "swap":"X1", "weight": 0}
               }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , [X1] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Incremental_Parsing_vs_Wildcard_test)
{
    std::istringstream pda_stream(R"({
        "pda":{
            "states":{
                "p0":{"A":{"pop":"","to":"p0"}}
            }
        }
    })");
    auto pda = PdaJSONParser::parse(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string("< [p0], [B] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string("< [p0], >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::post_star_accepts(instance);
    BOOST_TEST(!result);
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_arithmetic_3_5_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "s" : { "S": {"to":"p1", "swap":"X", "weight": 1}},
          "p1": { "X": {"to":"p2", "push":"X", "weight": 1} },
          "p2": { "X": {"to":"p3", "push":"X", "weight": 1} },
          "p3": { "X":[{"to":"p1", "push":"X", "weight": 1},
                       {"to":"q1", "swap":"X", "weight": 1}] },
          "q1": { "X": {"to":"q2", "pop":"", "weight": 1} },
          "q2": { "X": {"to":"q3", "pop":"", "weight": 1} },
          "q3": { "X": {"to":"q4", "pop":"", "weight": 1} },
          "q4": { "X": {"to":"q5", "pop":"", "weight": 1} },
          "q5": { "X":[{"to":"q1", "pop":"", "weight": 1},
                       {"to":"f", "swap":"F", "weight": 1}]},
          "f": { }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [s] , [S] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [f] , [F] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    s << std::endl;

    auto [automaton_path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);

    BOOST_CHECK(!automaton_path.is_null());
    // TODO: Test instead of printing
    internal::TraceBack tb(instance.automaton(), std::move(automaton_path));
    std::unordered_set<std::tuple<size_t,uint32_t,size_t>, absl::Hash<std::tuple<size_t,uint32_t,size_t>>> seen_front;
    while (seen_front.emplace(tb.path().front_edge()).second) {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
        tb.next();
    }
    do {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
    } while (tb.next<true>());
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_arithmetic_3_3_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "s" : { "S": {"to":"p1", "swap":"X", "weight": 1}},
          "p1": { "X": {"to":"p2", "push":"X", "weight": 1} },
          "p2": { "X": {"to":"p3", "push":"X", "weight": 1} },
          "p3": { "X":[{"to":"p1", "push":"X", "weight": 1},
                       {"to":"q2", "swap":"X", "weight": 1}] },
          "q1": { "X": {"to":"q2", "pop":"", "weight": 1} },
          "q2": { "X": {"to":"q3", "pop":"", "weight": 1} },
          "q3": { "X":[{"to":"q1", "pop":"", "weight": 1},
                       {"to":"f", "swap":"F", "weight": 1}]},
          "f": { }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [s] , [S] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [f] , [F] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(!result);
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_arithmetic_3_3or5_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "s" : { "S": {"to":"p1", "swap":"X", "weight": 1}},
          "p1": { "X": {"to":"p2", "push":"X", "weight": 1} },
          "p2": { "X": {"to":"p3", "push":"X", "weight": 1} },
          "p3": { "X":[{"to":"p1", "push":"X", "weight": 1},
                       {"to":"q3", "swap":"X", "weight": 1},
                       {"to":"r1", "swap":"X", "weight": 1}] },
          "q1": { "X": {"to":"q2", "pop":"", "weight": 1} },
          "q2": { "X": {"to":"q3", "pop":"", "weight": 1} },
          "q3": { "X":[{"to":"q1", "pop":"", "weight": 1},
                       {"to":"f", "swap":"F", "weight": 1}]},
          "r1": { "X": {"to":"r2", "pop":"", "weight": 1} },
          "r2": { "X": {"to":"r3", "pop":"", "weight": 1} },
          "r3": { "X": {"to":"r4", "pop":"", "weight": 1} },
          "r4": { "X": {"to":"r5", "pop":"", "weight": 1} },
          "r5": { "X":[{"to":"r1", "pop":"", "weight": 1},
                       {"to":"f", "swap":"F", "weight": 1}]},
          "f": { }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [s] , [S] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [f] , [F] | ([F] [X] [X] [X] [X] [X] [X] [X] [X] [X] [X] [X] [X] [X] [X] [X]) >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    s << std::endl;

    auto [automaton_path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);

    BOOST_CHECK(!automaton_path.is_null());
    // TODO: Test instead of printing
    internal::TraceBack tb(instance.automaton(), std::move(automaton_path));
    std::unordered_set<std::tuple<size_t,uint32_t,size_t>, absl::Hash<std::tuple<size_t,uint32_t,size_t>>> seen_front;
    while (seen_front.emplace(tb.path().front_edge()).second) {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
        tb.next();
    }
    do {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
    } while (tb.next<true>());
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_which_pop_seq_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "s" : { "S": {"to":"p3", "swap":"X", "weight": 1}},
          "p1": { "X": {"to":"p2", "push":"X", "weight": 1} },
          "p2": { "X": {"to":"p3", "push":"X", "weight": 1} },
          "p3": { "X":[{"to":"p1", "push":"X", "weight": 1},
                       {"to":"q", "swap":"X", "weight": 1}] },
          "q" : { "X":[{"to":"q1", "pop":"", "weight": 1},
                       {"to":"q2", "pop":"", "weight": 1},
                       {"to":"q3", "pop":"", "weight": 1}] },
          "q1": { "X": {"to":"q2", "pop":"", "weight": 1} },
          "q2": { "X": {"to":"q3", "pop":"", "weight": 1} },
          "q3": { "X": {"to":"r1", "swap":"X", "weight": 1} },
          "r1": { "X": {"to":"r2", "pop":"", "weight": 1} },
          "r2": { "X": {"to":"r3", "pop":"", "weight": 1} },
          "r3": { "X":[{"to":"r1", "pop":"", "weight": 1},
                       {"to":"f", "swap":"F", "weight": 1}] },
          "f": { }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [s] , [S] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [f] , [F] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    s << std::endl;

    auto [automaton_path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);

    BOOST_CHECK(!automaton_path.is_null());
    // TODO: Test instead of printing
    internal::TraceBack tb(instance.automaton(), std::move(automaton_path));
    std::unordered_set<std::tuple<size_t,uint32_t,size_t>, absl::Hash<std::tuple<size_t,uint32_t,size_t>>> seen_front;
    while (seen_front.emplace(tb.path().front_edge()).second) {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
        tb.next();
    }
    do {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
    } while (tb.next<true>());
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_arithmetic_pop3loop_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "s" : { "S": {"to":"p1", "swap":"X", "weight": 1}},
          "p1": { "X": {"to":"p2", "pop":"", "weight": 1} },
          "p2": { "X": {"to":"p3", "pop":"", "weight": 1} },
          "p3": { "X":[{"to":"p1", "pop":"", "weight": 1},
                       {"to":"f", "swap":"F", "weight": 1}] },
          "f": { }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [s] , [S] [X]* >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [f] , [F] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    BOOST_TEST_MESSAGE(s.str());

    auto [automaton_path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_hill_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "p": { "X":[{"to":"p", "push":"X", "weight": 1},
                      {"to":"q", "swap":"X", "weight": 1}]},
          "q": { "X": {"to":"q", "pop":"", "weight": 1}}
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [p] , [X] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [q] , >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Longest>(instance.automaton(), pda, s);
    s << std::endl;

    auto [automaton_path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    auto pXq = get_edge<std::string,std::string>(instance.automaton(), pda, "p", "X", "q");
    BOOST_CHECK_NE(pXq, nullptr);
    BOOST_CHECK_EQUAL(pXq->second, max_weight<uint32_t>::bottom());

    BOOST_CHECK(!automaton_path.is_null());
    // TODO: Test instead of printing
    internal::TraceBack tb(instance.automaton(), std::move(automaton_path));
    std::unordered_set<std::tuple<size_t,uint32_t,size_t>, absl::Hash<std::tuple<size_t,uint32_t,size_t>>> seen_front;
    while (seen_front.emplace(tb.path().front_edge()).second) {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
        tb.next();
    }
    do {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
    } while (tb.next<true>());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);

    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(Verification_longest_trace_start_hill_end_test)
{
    std::istringstream pda_stream(R"({
      "pda": {
        "states": {
          "s": { "S": {"to":"p", "swap":"X", "weight": 1}},
          "p": { "X":[{"to":"p", "push":"X", "weight": 1},
                      {"to":"q", "swap":"X", "weight": 1}]},
          "q": { "X":[{"to":"q", "pop":"", "weight": 1},
                      {"to":"f", "swap":"F", "weight": 1}]},
          "f": { }
        }
      }
    })");
    auto pda = PdaJSONParser::parse<weight<uint32_t>,true>(pda_stream, std::cerr);
    auto p_automaton_i = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [s] , [S] >", pda);
    auto p_automaton_f = PAutomatonParser::parse_string<TraceInfoType::Pair>("< [f] , [F] >", pda);
    PAutomatonProduct instance(pda, std::move(p_automaton_i), std::move(p_automaton_f));

    auto result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
    BOOST_TEST(result);

    std::stringstream s;
    print_automaton<Trace_Type::Longest>(instance.product_automaton(), pda, s);
    s << std::endl;
    print_automaton<Trace_Type::Longest>(instance.automaton(), pda, s);
    s << std::endl;

    auto [automaton_path, w] = instance.find_path<Trace_Type::Longest>();
    BOOST_CHECK_EQUAL(w, max_weight<uint32_t>::bottom());

    BOOST_CHECK(!automaton_path.is_null());
    // TODO: Test instead of printing
    internal::TraceBack tb(instance.automaton(), std::move(automaton_path));
    std::unordered_set<std::tuple<size_t,uint32_t,size_t>, absl::Hash<std::tuple<size_t,uint32_t,size_t>>> seen_front;
    while (seen_front.emplace(tb.path().front_edge()).second) {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
        tb.next();
    }
    do {
        print_conf_path(s, tb.path(), pda);
        print_edges_path(s, tb.path(), pda);
    } while (tb.next<true>());

    auto [trace, weight] = Solver::get_trace<Trace_Type::Longest>(instance);
    BOOST_CHECK_EQUAL(w, weight);

    BOOST_TEST_MESSAGE(s.str());
}
