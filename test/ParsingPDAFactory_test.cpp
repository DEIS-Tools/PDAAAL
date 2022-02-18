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
#include "ParsingPDAFactory.h"
#include <pdaaal/Solver.h>
#include <iostream>
#include <sstream>

using namespace pdaaal;

template <typename tracestate_t>
void print_trace(const std::vector<tracestate_t>& trace, std::ostream& s = std::cout) {
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

BOOST_AUTO_TEST_CASE(NewPDAFactory_Test)
{
    std::istringstream i_stream(R"(
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
    auto factory = ParsingPDAFactory<>::create(i_stream);

    // initial stack: [A,B]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp(std::unordered_set<std::string>{"B"});
    initial.concat(std::move(temp));
    // final stack: [A]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    // Yeah, a small regex -> NFA parser could be nice here...

    auto instance = factory.compile(initial, final);

    bool result = Solver::post_star_accepts(*instance);
    BOOST_CHECK(result);

    auto trace = Solver::get_trace(*instance);
    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);

    print_trace<decltype(trace)::value_type>(trace);
}

BOOST_AUTO_TEST_CASE(NewPDAFactory_Weighted_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B
# Initial states
0
# Accepting states
0
# Rules | with weights
0 A -> 2 B | 3
0 B -> 0 A | 1
0 A -> 1 - | 1
1 B -> 2 +B | 1
2 B -> 0 - | 1
)");
    auto factory = ParsingPDAFactory<weight<unsigned int>>::create(i_stream);

    // initial stack: [A,B]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp(std::unordered_set<std::string>{"B"});
    initial.concat(std::move(temp));
    // final stack: [A]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});

    auto instance = factory.compile(initial, final);

    bool result = Solver::post_star_accepts<Trace_Type::Shortest>(*instance);
    BOOST_CHECK(result);

    auto [trace, weight] = Solver::get_trace<Trace_Type::Shortest>(*instance);
    BOOST_CHECK_EQUAL(weight, 4);
    BOOST_CHECK_EQUAL(trace.size(), 5);

    print_trace<decltype(trace)::value_type>(trace);
}


BOOST_AUTO_TEST_CASE(CegarPdaFactory_Simple_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto& label){ return (int)label[0]; }, // No abstraction (use first character of strings A,B and C).
                                                    [](const auto& s){ return s; }); // No abstraction

    // initial stack: [A,B,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"C"});
    initial.concat(std::move(temp1));
    initial.concat(std::move(temp2));
    // final stack: [A,C]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp3(std::unordered_set<std::string>{"C"});
    final.concat(std::move(temp3));
    // Yeah, a small regex -> NFA parser could be nice here...

    auto instance = factory.compile(initial, final);

    bool result = Solver::post_star_accepts(*instance);
    BOOST_CHECK(result);

    ParsingCegarPdaReconstruction<> reconstruction(std::move(factory), *instance, initial, final);
    auto res = reconstruction.reconstruct_trace();
    BOOST_CHECK(res.index() == 0);

    auto trace = std::get<0>(res);
    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);
    BOOST_CHECK(std::all_of(trace.begin(), trace.end(), [](const auto& trace_state){ return trace_state._stack.back() == "C"; }));

    print_trace<decltype(trace)::value_type>(trace);
}


BOOST_AUTO_TEST_CASE(CegarPdaFactory_Empty_Trace_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto& label){ return (int)label[0]; }, // No abstraction (use first character of strings A,B and C).
                                                    [](const auto& s){ return s; }); // No abstraction

    // initial stack: [A,B] or [A,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B", "C"});
    initial.concat(std::move(temp1));
    // final stack: [A,A] or [A,B]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"A", "B"});
    final.concat(std::move(temp2));
    // Yeah, a small regex -> NFA parser could be nice here...

    auto instance = factory.compile(initial, final);

    bool result = Solver::post_star_accepts(*instance);
    BOOST_CHECK(result);
    ParsingCegarPdaReconstruction<> reconstruction(std::move(factory), *instance, initial, final);
    auto res = reconstruction.reconstruct_trace();
    BOOST_CHECK(res.index() == 0);
    auto trace = std::get<0>(res);
    print_trace<decltype(trace)::value_type>(trace);
}


BOOST_AUTO_TEST_CASE(CegarPdaFactory_Full_Abstraction_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto&){ return 0; }, // All labels map to 0.
                                                    [](const auto&){ return 0; }); // All states map to 0.

    // initial stack: [A,B,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"C"});
    initial.concat(std::move(temp1));
    initial.concat(std::move(temp2));
    // final stack: [A,C]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp3(std::unordered_set<std::string>{"C"});
    final.concat(std::move(temp3));
    // Yeah, a small regex -> NFA parser could be nice here...

    auto instance = factory.compile(initial, final);

    bool result = Solver::post_star_accepts(*instance); // NOTE: This test depends on the trace returned by post*, but with the current implementation we don't get a 'lucky' trace.
    BOOST_CHECK(result);

    ParsingCegarPdaReconstruction<> reconstruction(std::move(factory), *instance, initial, final);
    auto res = reconstruction.reconstruct_trace();
    BOOST_CHECK(res.index() == 2);

    auto header_refinement = std::get<2>(res);
    BOOST_CHECK(!header_refinement.empty());
    // TODO: More test...
}

BOOST_AUTO_TEST_CASE(CegarPdaFactory_State_Abstraction_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
# Initial states
0
# Accepting states
1
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto& label){ return (int)label[0]; }, // No label abstraction.
                                                    [](const auto&){ return 0; }); // All states map to 0.

    // initial stack: [A,B,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"C"});
    initial.concat(std::move(temp1));
    initial.concat(std::move(temp2));
    // final stack: [A,C]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp3(std::unordered_set<std::string>{"C"});
    final.concat(std::move(temp3));
    // Yeah, a small regex -> NFA parser could be nice here...

    auto instance = factory.compile(initial, final);

    bool result = Solver::post_star_accepts(*instance); // NOTE: This test depends on the trace returned by post*, but with the current implementation we don't get a 'lucky' trace.
    BOOST_CHECK(result);

    ParsingCegarPdaReconstruction<> reconstruction(std::move(factory), *instance, initial, final);
    auto res = reconstruction.reconstruct_trace();
    BOOST_CHECK(res.index() == 1);

    auto [state_refinement, label_refinement] = std::get<0>(std::get<1>(res));
    BOOST_CHECK(!state_refinement.empty() || !label_refinement.empty());
    // TODO: More test...
}

BOOST_AUTO_TEST_CASE(Complete_CEGAR_Full_Abstraction_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto&){ return 0; }, // All labels map to 0.
                                                    [](const auto&){ return 0; }); // All states map to 0.

    // initial stack: [A,B,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"C"});
    initial.concat(std::move(temp1));
    initial.concat(std::move(temp2));
    // final stack: [A,C]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp3(std::unordered_set<std::string>{"C"});
    final.concat(std::move(temp3));
    // Yeah, a small regex -> NFA parser could be nice here...

    CEGAR<ParsingCegarPdaFactory<>,ParsingCegarPdaReconstruction<>> cegar;
    auto res = cegar.cegar_solve(std::move(factory), initial, final);
    BOOST_CHECK(res.has_value());
    auto trace = res.value();

    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);
    BOOST_CHECK(std::all_of(trace.begin(), trace.end(), [](const auto& trace_state){ return trace_state._stack.back() == "C"; }));
    print_trace<decltype(trace)::value_type>(trace);
}

BOOST_AUTO_TEST_CASE(Complete_CEGAR_prestar_Full_Abstraction_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto&){ return 0; }, // All labels map to 0.
                                                    [](const auto&){ return 0; }); // All states map to 0.

    // initial stack: [A,B,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"C"});
    initial.concat(std::move(temp1));
    initial.concat(std::move(temp2));
    // final stack: [A,C]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp3(std::unordered_set<std::string>{"C"});
    final.concat(std::move(temp3));
    // Yeah, a small regex -> NFA parser could be nice here...

    CEGAR<ParsingCegarPdaFactory<>,ParsingCegarPdaReconstruction<>> cegar;
    auto res = cegar.cegar_solve<true>(std::move(factory), initial, final);
    BOOST_CHECK(res.has_value());
    auto trace = res.value();

    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);
    BOOST_CHECK(std::all_of(trace.begin(), trace.end(), [](const auto& trace_state){ return trace_state._stack.back() == "C"; }));
    print_trace<decltype(trace)::value_type>(trace);
}


BOOST_AUTO_TEST_CASE(DualSearch_Test)
{
    std::istringstream i_stream(R"(
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
    auto factory = ParsingPDAFactory<>::create(i_stream);

    // initial stack: [A,B]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp(std::unordered_set<std::string>{"B"});
    initial.concat(std::move(temp));
    // final stack: [A]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    // Yeah, a small regex -> NFA parser could be nice here...

    auto instance = factory.compile(initial, final);

    bool result = Solver::dual_search_accepts(*instance);
    BOOST_CHECK(result);

    auto trace = Solver::get_trace_dual_search(*instance);
    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);

    print_trace<decltype(trace)::value_type>(trace);
}

BOOST_AUTO_TEST_CASE(Complete_CEGAR_dualsearch_Full_Abstraction_Test)
{
    std::istringstream i_stream(R"(
# Labels
A,B,C
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
    auto factory = ParsingCegarPdaFactory<>::create(i_stream,
                                                    [](const auto&){ return 0; }, // All labels map to 0.
                                                    [](const auto&){ return 0; }); // All states map to 0.

    // initial stack: [A,B,C]
    NFA<std::string> initial(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp1(std::unordered_set<std::string>{"B"});
    NFA<std::string> temp2(std::unordered_set<std::string>{"C"});
    initial.concat(std::move(temp1));
    initial.concat(std::move(temp2));
    // final stack: [A,C]
    NFA<std::string> final(std::unordered_set<std::string>{"A"});
    NFA<std::string> temp3(std::unordered_set<std::string>{"C"});
    final.concat(std::move(temp3));
    // Yeah, a small regex -> NFA parser could be nice here...

    CEGAR<ParsingCegarPdaFactory<>,ParsingCegarPdaReconstruction<>> cegar;
    auto res = cegar.cegar_solve<false,true>(std::move(factory), initial, final);
    BOOST_CHECK(res.has_value());
    auto trace = res.value();

    BOOST_CHECK_GE(trace.size(), 4);
    BOOST_CHECK_LE(trace.size(), 5);
    BOOST_CHECK(std::all_of(trace.begin(), trace.end(), [](const auto& trace_state){ return trace_state._stack.back() == "C"; }));
    print_trace<decltype(trace)::value_type>(trace);
}