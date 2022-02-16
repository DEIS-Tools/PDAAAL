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
#include <pdaaal/TypedPAutomaton.h>
#include <tao/pegtl/contrib/unescape.hpp>

namespace pdaaal {

    // First define grammar for PAutomaton.
    // Escaped string
    struct escaped_c : pegtl::one< '"', '\\' > {};
    struct character : pegtl::if_then_else< pegtl::one< '\\' >, escaped_c, pegtl::utf8::range< 0x20, 0x10FFFF > > {};
    struct string_state : public std::string {
        template<typename ParseInput, typename... States >
        explicit string_state(const ParseInput&, States&&...) { }
        template<typename ParseInput, typename... States >
        void success(const ParseInput&, States&&... st) {
            (st.accept_string(*this), ...);
        }
    };
    struct string_literal : pegtl::state<string_state, pegtl::one< '"' >, pegtl::until< pegtl::one< '"' >, character > > {};
    // PAutomaton grammar (using the NFA grammar)
    template<typename State>
    struct p_automaton_initial
        : pegtl::sor<
            pegtl::if_must<pegtl::one<'['>, pegtl::list_must<State, pegtl::one<','>, ignored<comment>>, pegtl::one<']'>>,
            State
        > {};
    struct p_automaton_or_symbol : pegtl::one<'|'> {}; // We want a different action here than for NFA or_symbol, so we define a new rule.
    template<typename State>
    struct p_automaton_atom : pegtl::seq<pegtl::one<'<'>, pegtl::pad<p_automaton_initial<State>, ignored<comment>>, pegtl::one<','>, pegtl::state<NfaBuilder<uint32_t>, nfa_expr_default>, pegtl::one<'>'>> {};
    template<typename State>
    struct p_automaton_expr : pegtl::list_must<p_automaton_atom<State>, p_automaton_or_symbol, ignored<comment>> {};

    struct p_automaton_state_1 : pegtl::sor<pegtl::plus<pegtl::digit>, pegtl::identifier> {};
    struct p_automaton_state : pegtl::sor<p_automaton_state_1, string_literal> {};
    // Top-level rule
    struct p_automaton_file : pegtl::must<pegtl::pad<p_automaton_expr<p_automaton_state>, ignored<comment>>, pegtl::eof> {};

    // The State object that gets passed around by the parser is a builder that constructs the PAutomaton.
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, bool indirect>
    class PAutomatonBuilder {
        using automaton_t = TypedPAutomaton<label_t,W,state_t,skip_state_mapping,indirect>;
    public:
        explicit PAutomatonBuilder(TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                                   const std::function<state_t(const std::string&)>& state_mapping)
        : _pda(pda), _state_mapping(state_mapping),
          _label_mapping([&pda](const std::string& label) -> uint32_t { return pda.insert_label(label); }) {};
        automaton_t get_p_automaton() {
            return _current_p_automaton.value();
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
        [[nodiscard]] const std::function<uint32_t(const std::string&)>& get_label_map() const {
            return _label_mapping;
        }
        void accept_nfa(NFA<uint32_t>&& nfa) {
            _current_nfa = std::move(nfa);
            _current_nfa.compile();
        }
        void finish_atom() {
            std::sort(_states.begin(), _states.end()); // Sort and remove duplicates. TODO: Consider using unordered_set instead...
            _states.erase(std::unique(_states.begin(), _states.end()), _states.end());
            automaton_t new_p_automaton(_pda, _current_nfa, _states);
            if (_current_p_automaton) {
                _current_p_automaton.value().or_extend(std::move(new_p_automaton));
            } else {
                _current_p_automaton.emplace(std::move(new_p_automaton));
            }
            _states.clear();
        }
    private:
        std::vector<size_t> _states;
        NFA<uint32_t> _current_nfa;
        std::optional<automaton_t> _current_p_automaton;

        const TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& _pda;
        const std::function<state_t(const std::string&)>& _state_mapping;
        std::function<uint32_t(const std::string&)> _label_mapping;
    };
    template<bool indirect, typename label_t, typename W, typename state_t, bool skip_state_mapping>
    inline auto make_PAutomatonBuilder(TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                                       const std::function<state_t(const std::string&)>& state_mapping) {
        return PAutomatonBuilder<label_t,W,state_t,skip_state_mapping,indirect>(pda, state_mapping);
    }
    // CTAD guide
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping>
    PAutomatonBuilder(TypedPDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>& pda,
                      const std::function<state_t(const std::string&)>& state_mapping) -> PAutomatonBuilder<label_t,W,state_t,skip_state_mapping,true>;

    // Definition of the actions applied by the parser to the builder state object.
    // Combine NFA action, unescape action, and a new action that adds states.
    template<typename Rule> struct p_automaton_build_action : nfa_build_action<Rule> { };
    template<> struct p_automaton_build_action< pegtl::utf8::range< 0x20, 0x10FFFF > > : pegtl::unescape::append_all {};
    template<> struct p_automaton_build_action< escaped_c > : pegtl::unescape::unescape_c< escaped_c, '"', '\\' > {};
    template<> struct p_automaton_build_action<p_automaton_state_1> {
        template<typename ActionInput, typename T, typename W, typename S, bool ssm, bool i> static void apply(const ActionInput& in, PAutomatonBuilder<T,W,S,ssm,i>& v) {
            if (!v.add_state(in.string())) {
                throw pegtl::parse_error("state is not found in the PDA.", in);
            }
        }
    };
    template<typename State> struct p_automaton_build_action<p_automaton_atom<State>> {
        template<typename T, typename W, typename S, bool ssm, bool i> static void apply0(PAutomatonBuilder<T,W,S,ssm,i>& v) {
            v.finish_atom();
        }
    };

    // Final parser class.
    class PAutomatonParser {
    public:
        template <bool indirect = true, typename pda_t>
        static auto parse_file(const std::string& file, pda_t& pda) {
            std::filesystem::path file_path(file);
            pegtl::file_input in(file_path);
            return parse<indirect>(in, pda);
        }
        template <bool indirect = true, typename pda_t>
        static auto parse_string(const std::string& content, pda_t& pda) {
            pegtl::memory_input in(content, "");
            return parse<indirect>(in, pda);
        }

    private:
        template <bool indirect, typename Input, typename W>
        static auto parse(Input& in, TypedPDA<std::string,W,fut::type::vector, std::string>& pda) {
            return parse<indirect, std::string>(in, pda, [](const std::string& s){ return s; });
        }
        template <bool indirect, typename Input, typename W, bool ssm>
        static auto parse(Input& in, TypedPDA<std::string,W,fut::type::vector, size_t, ssm>& pda) {
            return parse<indirect, size_t>(in, pda, [](const std::string& s) -> size_t { return std::stoul(s); });
        }
        template <bool indirect, typename state_t, typename Input, typename pda_t>
        static auto parse(Input& in, pda_t& pda, const std::function<state_t(const std::string&)>& state_mapping) {
            auto p_automaton_builder = make_PAutomatonBuilder<indirect>(pda, state_mapping);
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
