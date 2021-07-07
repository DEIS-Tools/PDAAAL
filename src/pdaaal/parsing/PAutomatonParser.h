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
 * File:   PautomataRegexParser.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 04-07-2021.
 */

#ifndef PDAAAL_PAUTOMATAREGEXPARSER_H
#define PDAAAL_PAUTOMATAREGEXPARSER_H

#include <pdaaal/parsing/NfaParserGrammar.h>
#include <pdaaal/PAutomaton.h>
#include <tao/pegtl/contrib/unescape.hpp>

namespace pdaaal {

    struct escaped_c : pegtl::one< '"', '\\' > {};
    struct character : pegtl::if_then_else< pegtl::one< '\\' >, escaped_c, pegtl::utf8::range< 0x20, 0x10FFFF > > {};
    struct string_state : public std::string {
        template<typename ParseInput, typename... States >
        explicit string_state(const ParseInput& in, States&&... st) { }
        template<typename ParseInput, typename... States >
        void success(const ParseInput& in, States&&... st) {
            (st.accept_string(*this), ...);
        }
    };
    struct string_literal : pegtl::state<string_state, pegtl::one< '"' >, pegtl::until< pegtl::one< '"' >, character > > {};


    template<typename State>
    struct p_automaton_initial : label_set_or_singleton<State, comment> {};
    template<typename State>
    struct p_automaton_expr : pegtl::seq<pegtl::one<'<'>, pegtl::pad<p_automaton_initial<State>, ignored<comment>>, pegtl::one<','>, nfa_expr_default, pegtl::one<'>'>> {};
    struct p_automaton_state_1 : pegtl::sor<pegtl::plus<pegtl::digit>, pegtl::identifier> {};
    struct p_automaton_state : pegtl::sor<p_automaton_state_1, string_literal> {};

    struct p_automaton_file : pegtl::must<pegtl::pad<p_automaton_expr<p_automaton_state>, ignored<comment>>, pegtl::eof> {};
    
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping>
    class PAutomatonBuilder : public NfaBuilder<uint32_t> {
    public:
        explicit PAutomatonBuilder(TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                                   const std::function<state_t(const std::string&)>& state_mapping)
                : NfaBuilder<uint32_t>([&pda](const std::string& label) -> uint32_t { return pda.insert_label(label); }),
                  _pda(pda), _state_mapping(state_mapping) {};
        PAutomaton<W> get_p_automaton() {
            return PAutomaton<W>(_pda, get_nfa(), _states);
        }
        bool add_state(const std::string& state_name) {
            auto [found, id] = _pda.exists_state(_state_mapping(state_name));
            if (found) {
                _states.emplace_back(id);
            }
            return found;
        }
        void accept_string(const std::string& s) {
            add_state(s);
        }
    private:
        std::vector<size_t> _states;
        const TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& _pda;
        const std::function<state_t(const std::string&)>& _state_mapping;
    };
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping>
    PAutomatonBuilder(TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                      const std::function<state_t(const std::string&)>& state_mapping) -> PAutomatonBuilder<label_t,W,state_t,skip_state_mapping>;

    template<typename Rule> struct p_automaton_build_action : nfa_build_action<Rule> { };
    template<> struct p_automaton_build_action< pegtl::utf8::range< 0x20, 0x10FFFF > > : pegtl::unescape::append_all {};
    template<> struct p_automaton_build_action< escaped_c > : pegtl::unescape::unescape_c< escaped_c, '"', '\\' > {};
    template<> struct p_automaton_build_action<p_automaton_state_1> {
        template<typename ActionInput, typename T, typename W, typename S, bool ssm> static void apply(const ActionInput& in, PAutomatonBuilder<T,W,S,ssm>& v) {
            if (!v.add_state(in.string())) {
                throw pegtl::parse_error("state is not found in the PDA.", in);
            }
        }
    };

    class PAutomatonParser {
    public:
        template <typename pda_t>
        static auto parse_file(const std::string& file, pda_t& pda) {
            std::filesystem::path file_path(file);
            pegtl::file_input in(file_path);
            return parse(in, pda);
        }
        template <typename pda_t>
        static auto parse_string(const std::string& content, pda_t& pda) {
            pegtl::memory_input in(content, "");
            return parse(in, pda);
        }

    private:
        template <typename Input, typename W>
        static PAutomaton<W> parse(Input& in, TypedPDA<std::string,W,fut::type::vector, std::string>& pda) {
            return parse<std::string>(in, pda, [](const std::string& s){ return s; });
        }
        template <typename Input, typename W>
        static PAutomaton<W> parse(Input& in, TypedPDA<std::string,W,fut::type::vector, size_t>& pda) {
            return parse<size_t>(in, pda, [](const std::string& s) -> size_t { return std::stoul(s); });
        }
        template <typename state_t, typename Input, typename pda_t>
        static auto parse(Input& in, pda_t& pda, const std::function<state_t(const std::string&)>& state_mapping) {
            PAutomatonBuilder p_automaton_builder(pda, state_mapping);
            try {
                pegtl::parse<p_automaton_file,p_automaton_build_action>(in, p_automaton_builder);
            } catch (const pegtl::parse_error& e) {
                std::stringstream s;
                const auto p = e.positions().front();
                s << e.what() << std::endl
                  << in.line_at(p) << std::endl
                  << std::setw(p.column) << '^' << std::endl;
                throw std::runtime_error(s.str());
            }
            return p_automaton_builder.get_p_automaton();
        }
    };

}

#endif //PDAAAL_PAUTOMATAREGEXPARSER_H
