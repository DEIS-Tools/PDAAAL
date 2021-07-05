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
 * File:   NfaParserGrammar.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 05-07-2021.
 */

#ifndef PDAAAL_NFAPARSERGRAMMAR_H
#define PDAAAL_NFAPARSERGRAMMAR_H

#include <pdaaal/NFA.h>
#include <tao/pegtl.hpp>

namespace pegtl = tao::pegtl;

namespace pdaaal {

    struct negation : pegtl::one<'^'> {};
    struct wildcard : pegtl::one<'.'> {};
    struct bracket_open : pegtl::one<'('> {};
    struct bracket_close : pegtl::one<')'> {};
    struct star_symbol : pegtl::one<'*'> {};
    struct plus_symbol : pegtl::one<'+'> {};
    struct question_symbol : pegtl::one<'?'> {};
    struct or_symbol : pegtl::one<'|'> {};
    template<typename Comment>
    struct ignored : pegtl::sor<pegtl::space, Comment> {};
    template<typename L, typename C>
    struct nfa_expr;
    template<typename L, typename C>
    struct bracket : pegtl::if_must<bracket_open, pegtl::pad<nfa_expr<L, C>, ignored<C>>, bracket_close> {};
    template<typename L, typename C>
    struct value_expr : pegtl::sor<wildcard, bracket < L, C>, L> {};
    template<typename L, typename C>
    struct unary_expr : pegtl::seq<value_expr<L, C>, pegtl::opt<pegtl::sor<star_symbol, plus_symbol, question_symbol>>> {};
    template<typename L, typename C>
    struct seq_expr : pegtl::seq<unary_expr<L, C>, pegtl::star<pegtl::seq<pegtl::star<ignored<C>>,unary_expr<L, C>>>> {};
    template<typename L, typename C>
    struct nfa_expr : pegtl::list<seq_expr<L, C>, or_symbol, ignored < C>> {};


    template<typename Label, typename C>
    struct label_set : pegtl::if_must<pegtl::one<'['>, pegtl::opt<negation>, pegtl::list_must<Label, pegtl::one<','>, ignored<C>>, pegtl::one<']'>> {};
    template<typename Label, typename C>
    struct label_set_or_singleton : pegtl::sor<label_set<Label, C>, Label> {};
    // We take the type of Label and Comment as parameters, to allow for various cases.
    struct label : pegtl::plus<pegtl::identifier_other> {};
    struct comment : pegtl::seq<pegtl::one < '#'>, pegtl::until<pegtl::eolf>>{};

    template<typename L = label_set<label, comment>, typename C = comment>
    struct nfa_file : pegtl::must<pegtl::pad < nfa_expr<L, C>, ignored < C>>, pegtl::eof> {};

    enum class nfa_op_tag {BRACKET, OR, CONCAT};

    template<typename T>
    class NfaBuilder { // This one is a bit ugly, but it works...
    public:
        explicit NfaBuilder(const std::function<T(const std::string&)>& label_function) : _label_function(label_function) {};

        void open() {
            _op_type_stack.emplace_back(nfa_op_tag::BRACKET);
        }
        void close() {
            assert(!_op_type_stack.empty());
            while (_op_type_stack.back() != nfa_op_tag::BRACKET) {
                if (_op_type_stack.back() == nfa_op_tag::OR) {
                    pop_op<nfa_op_tag::OR>();
                } else {
                    pop_op<nfa_op_tag::CONCAT>();
                }
                assert(!_op_type_stack.empty());
            }
            _op_type_stack.pop_back();
        }
        void set_negation() {
            _negated = true;
        }
        void add_label(std::string&& label) {
            _label_set.emplace(_label_function(std::move(label)));
        }
        void use_label_set() {
            emplace_nfa(std::move(_label_set), _negated);
            _label_set = std::unordered_set<T>();
            _negated = false;
        }
        template<typename... Args>
        void emplace_nfa(Args&& ... args) {
            _nfa_stack.emplace_back(std::forward<Args>(args)...);
        }
        NFA <T>& current_nfa() {
            return _nfa_stack.back();
        }
        template<nfa_op_tag op>
        void reduce() {
            if constexpr(op == nfa_op_tag::CONCAT) {
                assert(!_op_type_stack.empty() && _op_type_stack.back() == nfa_op_tag::CONCAT);
                _op_type_stack.pop_back();
            }
            while (!_op_type_stack.empty() && _op_type_stack.back() == op) {
                pop_op<op>();
            }
        }
        void push_op(nfa_op_tag op) {
            _op_type_stack.emplace_back(op);
        }
        NFA <T> get() {
            assert(_nfa_stack.size() == 1);
            return std::move(_nfa_stack.back());
        }

    private:
        template<nfa_op_tag op>
        void pop_op() {
            assert(_op_type_stack.back() == op);
            if constexpr(op != nfa_op_tag::BRACKET) {
                assert(_nfa_stack.size() > 1);
                auto temp = std::move(_nfa_stack.back());
                _nfa_stack.pop_back();
                if constexpr(op == nfa_op_tag::OR) {
                    _nfa_stack.back().or_extend(std::move(temp));
                } else {
                    _nfa_stack.back().concat(std::move(temp));
                };
            }
            assert(!_op_type_stack.empty());
            _op_type_stack.pop_back();
        }

        std::function<T(const std::string&)> _label_function;
        bool _negated = false;
        std::unordered_set <T> _label_set;
        std::vector <NFA<T>> _nfa_stack;
        std::vector <nfa_op_tag> _op_type_stack;
    };

    template<typename Rule> struct nfa_build_action : pegtl::nothing<Rule> { };

    template<> struct nfa_build_action<label> {
        template<typename ActionInput, typename T> static void apply(const ActionInput& in, NfaBuilder<T>& v) {
            v.add_label(in.string());
        }
    };
    template<typename L, typename C> struct nfa_build_action<label_set<L, C>> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.use_label_set();
        }
    };
    template<> struct nfa_build_action<wildcard> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.emplace_nfa(std::unordered_set<T>(), true);
        }
    };
    template<> struct nfa_build_action<bracket_open> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.open();
        }
    };
    template<> struct nfa_build_action<bracket_close> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.close();
        }
    };
    template<> struct nfa_build_action<negation> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.set_negation();
        }
    };
    template<> struct nfa_build_action<star_symbol> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.current_nfa().star_extend();
        }
    };
    template<> struct nfa_build_action<plus_symbol> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.current_nfa().plus_extend();
        }
    };
    template<> struct nfa_build_action<question_symbol> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.current_nfa().question_extend();
        }
    };
    template<typename L, typename C> struct nfa_build_action<unary_expr<L, C>> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.push_op(nfa_op_tag::CONCAT);
        }
    };
    template<typename L, typename C> struct nfa_build_action<seq_expr<L, C>> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.template reduce<nfa_op_tag::CONCAT>();
        }
    };
    template<> struct nfa_build_action<or_symbol> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.push_op(nfa_op_tag::OR);
        }
    };
    template<typename L, typename C> struct nfa_build_action<nfa_expr<L, C>> {
        template<typename T> static void apply0(NfaBuilder<T>& v) {
            v.template reduce<nfa_op_tag::OR>();
        }
    };

}

#endif //PDAAAL_NFAPARSERGRAMMAR_H
