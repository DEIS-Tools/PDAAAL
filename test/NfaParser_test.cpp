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
 * File:   NfaParser_test
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 04-07-2021.
 */

#define BOOST_TEST_MODULE NfaParser_test

#include <pdaaal/parsing/NfaParser.h>
#include <pdaaal/parsing/NfaParserGrammar.h>
#include <pdaaal/utils/ptrie_interface.h>
#include <tao/pegtl/contrib/analyze.hpp>
#include <tao/pegtl/contrib/trace.hpp>
#include <boost/test/unit_test.hpp>

namespace pegtl = tao::pegtl;

BOOST_AUTO_TEST_CASE(NfaParser_grammar_analyze)
{
    using grammar = pdaaal::nfa_file<>;
    BOOST_TEST(pegtl::analyze<grammar>(1) == 0);
}

BOOST_AUTO_TEST_CASE(NfaParser_test1_tracer)
{
    using grammar = pdaaal::nfa_file<>;
    pegtl::string_input in("[abcd123, dcba]* [qq] | ([^abcd123] [hjk])+", "Test 1");
    BOOST_TEST(pegtl::standard_trace< grammar >(in));
}

BOOST_AUTO_TEST_CASE(NfaParser_input_test1)
{
    using grammar = pdaaal::nfa_file<>;
    pegtl::string_input in("[abcd123, dcba]* [qq] | ( [^abcd123] [hjk] )+", "Test 1");
    try {
        BOOST_TEST(pegtl::parse<grammar>(in));
    } catch (const pegtl::parse_error& e) {
        // This catch block needs access to the input
        const auto p = e.positions().front();
        std::stringstream s;
        s << e.what() << std::endl
          << in.line_at(p) << std::endl
          << std::setw(p.column) << '^' << std::endl;
        BOOST_TEST_MESSAGE(s.str());
        BOOST_TEST(false);
    }
}

BOOST_AUTO_TEST_CASE(NfaParser_input_test2)
{
    using grammar = pdaaal::nfa_file<>;
    pegtl::string_input in(".* .+.?", "Test 2");
    try {
        BOOST_TEST(pegtl::parse<grammar>(in));
    } catch (const pegtl::parse_error& e) {
        // This catch block needs access to the input
        const auto p = e.positions().front();
        std::stringstream s;
        s << e.what() << std::endl
          << in.line_at(p) << std::endl
          << std::setw(p.column) << '^' << std::endl;
        BOOST_TEST_MESSAGE(s.str());
        BOOST_TEST(false);
    }
}

BOOST_AUTO_TEST_CASE(NfaParser_input_test3)
{
    // This should fail.
    using grammar = pdaaal::nfa_file<>;
    pegtl::string_input in("[,a]", "Test 3");
    try {
        BOOST_TEST(!pegtl::parse<grammar>(in));
    } catch (const pegtl::parse_error& e) {
        // This catch block needs access to the input
        const auto p = e.positions().front();
        std::stringstream s;
        s << e.what() << std::endl
          << in.line_at(p) << std::endl
          << std::setw(p.column) << '^' << std::endl;
        BOOST_TEST_MESSAGE(s.str());
        BOOST_TEST(true);
    }
}

BOOST_AUTO_TEST_CASE(NfaParser_parse_test1)
{
    pdaaal::utils::ptrie_set<std::string> labels;
    auto f = [&labels](const std::string& s) -> size_t {
        return labels.insert(s).second;
    };
    auto nfa = pdaaal::NfaParser::parse_string("[abcd123, dcba]* [qq] | ( [^abcd123] [hjk] )+  # A Comment ", f);
    nfa.compile();
    std::stringstream s;
    nfa.to_dot(s, [&labels](std::ostream& s, const size_t& label){ s << labels.at(label); });
    BOOST_TEST_MESSAGE(s.str());
}

BOOST_AUTO_TEST_CASE(NfaParser_parse_test2)
{
    pdaaal::utils::ptrie_set<std::string> labels;
    auto f = [&labels](const std::string& s) -> size_t {
        return labels.insert(s).second;
    };
    auto nfa = pdaaal::NfaParser::parse_string("[a] | [b]", f);
    nfa.compile();
    std::stringstream s;
    nfa.to_dot(s, [&labels](std::ostream& s, const size_t& label){ s << labels.at(label); });
    BOOST_TEST_MESSAGE(s.str());
}
