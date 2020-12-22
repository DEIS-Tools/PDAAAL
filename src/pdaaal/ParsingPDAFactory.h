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
 * File:   ParsingPDAFactory.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 22-12-2020.
 */

#ifndef PDAAAL_PARSINGPDAFACTORY_H
#define PDAAAL_PARSINGPDAFACTORY_H

#include "NewPDAFactory.h"
#include "CegarPdaFactory.h"
#include <istream>
#include <algorithm>
#include <vector>

namespace pdaaal {

    // This class enables constructing PDAs by parsing from a simple file format.
    // The main motivation is to enable easier testing of the PDA construction and verification implementations.
    // PDAParser provides static methods for parsing.
    // ParsingPDAFactory (below) implements the PDA construction using PDAParser.
    class PDAParser {
    public:
        static std::unordered_set<std::string> parse_all_labels(std::istream& input) {
            std::unordered_set<std::string> all_labels;
            std::stringstream s_line(next_line(input));
            std::string label;
            while(std::getline(s_line, label, ',')) {
                all_labels.emplace(label);
            }
            return all_labels;
        }

        static std::vector<size_t> parse_states(std::istream& input) {
            std::vector<size_t> result;
            std::stringstream s_line(next_line(input));
            size_t s;
            while (s_line >> s) {
                result.push_back(s);
            }
            std::sort(result.begin(), result.end());
            return result;
        }

        template <typename rule_t, typename W>
        static std::pair<bool, rule_t> parse_rule(std::istream& input) {
            std::string line = next_line(input);
            if (line.empty()) return std::make_pair(false, rule_t{});
            std::string delimiter = "->";
            rule_t rule;
            std::string op_string;
            auto pos = line.find(delimiter);
            std::stringstream first(line.substr(0, pos));
            first >> rule._from >> rule._pre;

            size_t wpos = std::string::npos;
            if constexpr (is_weighted<W>) {
                wpos = line.find("|", pos);
            }
            std::stringstream second(line.substr(pos + delimiter.length(), wpos));
            second >> rule._to >> op_string;
            if (op_string.empty()) {
                throw std::runtime_error("Invalid PDA input: Op_string is empty.");
            }
            if (op_string[0] == '-') {
                rule._op = POP;
                rule._op_label.clear();
            } else if (op_string[0] == '+') {
                rule._op = PUSH;
                rule._op_label = op_string.substr(1);
            } else {
                rule._op = SWAP;
                rule._op_label = std::move(op_string);
            }
            if constexpr (is_weighted<W>) {
                std::stringstream w_stream(line.substr(wpos + 1));
                w_stream >> rule._weight;
            }
            return std::make_pair(true, rule);
        }

        static std::string next_line(std::istream& input) {
            if (skip_empty_and_comment_lines(input)) {
                std::string line;
                std::getline(input, line);
                return line;
            }
            return std::string();
        }

        static bool skip_empty_and_comment_lines(std::istream& input) { // Implement simple commenting functionality.
            if (!input) return false; // Already at EOF.
            do {
                auto next_c = input.peek();
                if (next_c != '#' && next_c != input.widen('\n')) { // Skip if comment line or empty line, exit otherwise
                    return true; // Not reached EOF.
                }
            } while(input.ignore(std::numeric_limits<std::streamsize>::max(), input.widen('\n'))); // Skip line. Exit if EOF reached.
            return false; // EOF reached
        }
    };

    template <typename W = void, typename C = std::less<W>, typename A = add<W>>
    class ParsingPDAFactory : public NewPDAFactory<std::string, W, C, A> {
    public:
        static ParsingPDAFactory<W,C,A> create(std::istream& input) {
            auto all_labels = PDAParser::parse_all_labels(input);
            return ParsingPDAFactory<W,C,A>(input, std::move(all_labels));
        }
    private:
        ParsingPDAFactory(std::istream& input, std::unordered_set<std::string>&& all_labels)
                : NewPDAFactory<std::string, W, C, A>(std::move(all_labels)), _input(input) {
            initialize();
        };
        using rule_t = typename NewPDAFactory<std::string, W, C, A>::rule_t;
    protected:
        const std::vector<size_t>& initial() override {
            return _initial;
        }
        bool accepting(size_t s) override {
            auto it = std::lower_bound(_accepting.begin(), _accepting.end(), s);
            return it != _accepting.end() && *it == s;
        }
        std::vector<rule_t> rules(size_t s) override {
            // No lazy loading. Parsed in initialize(), so just return here.
            if (s < _rules.size()) {
                return _rules[s];
            }
            return std::vector<rule_t>();
        }
    private:
        void initialize() {
            _initial = PDAParser::parse_states(_input);
            _accepting = PDAParser::parse_states(_input);

            while (true) {
                auto [keep_going, rule] = PDAParser::parse_rule<rule_t, W>(_input);
                if (!keep_going) break;
                _rules.resize(std::max(rule._from, rule._to) + 1);
                _rules[rule._from].push_back(rule);
            }
        }

    private:
        std::istream& _input;
        std::vector<size_t> _initial;
        std::vector<size_t> _accepting;
        std::vector<std::vector<rule_t>> _rules;
    };


    template <typename W, typename C>
    using ParsingCegarPdaFactory_configuration = std::pair<size_t, typename wildcard_header<std::string,W,C>::header_t>;
    template <typename W = void, typename C = std::less<W>, typename A = add<W>>
    class ParsingCegarPdaFactory : public CegarPdaFactory<std::string, size_t /* State */, size_t /* AbstractState */,
            typename std::vector<ParsingCegarPdaFactory_configuration<W,C>>::iterator /* conf_it */, W, C, A> {
    public:
        static ParsingCegarPdaFactory<W,C,A> create(std::istream& input) {
            auto all_labels = PDAParser::parse_all_labels(input);
            return ParsingCegarPdaFactory<W,C,A>(input, std::move(all_labels));
        }
    private:
        using state_t = size_t;
        using abstract_state_t = size_t;
        using conf_it = typename std::vector<ParsingCegarPdaFactory_configuration<W,C>>::iterator;
        using parent_t = CegarPdaFactory<std::string, state_t, abstract_state_t, conf_it, W, C, A>;
        using configuration_t = typename parent_t::configuration_t;
        using header_wrapper_t = typename parent_t::header_wrapper_t;
        using header_t = typename parent_t::header_t;
        using pda_t = typename parent_t::pda_t;
        using refinement_info_t = typename parent_t::refinement_info_t;

        ParsingCegarPdaFactory(std::istream& input, std::unordered_set<std::string>&& all_labels)
        : parent_t([](const std::string& label){ return (uint32_t)label[0]; }), // Cast first character... TODO: Stuff
          _input(input), _all_labels(std::move(all_labels)), _state_abstraction([](size_t s){ return s; }) {  // No state abstraction either, for now...
            initialize();
        };
        using rule_t = typename TypedPDA<std::string, W, C>::rule_t; // For concrete rules we just use this one.
        using abstract_rule_t = typename parent_t::abstract_rule_t;
    protected:

        void build_pda() override {
            for (const auto& rules : _rules) { // rules for each from state.
                for (const auto& rule : rules) {
                    this->add_rule(abstract_rule(rule));
                }
            }
        }
        const std::vector<size_t>& initial() override {
            return _initial;
        }
        const std::vector<size_t>& accepting() override {
            return _accepting;
        }

        // TODO: Coroutines might make this a lot simpler...
        std::pair<conf_it,conf_it> first_confs(const abstract_rule_t& rule, const std::vector<uint32_t>& initial_abstract_stack, const header_wrapper_t& header_handler) override {
            auto from_states = _state_abstraction.get_concrete_values(rule._from);
            auto header = header_handler.initial_header();
            _temps.emplace_back(); // We store all configurations in _temps. TODO: Make efficient range implementation. That was the hole point of returning iterators.
            make_configurations(_temps.back(), rule, from_states, header, header_handler);
            return std::make_pair(_temps.back().begin(), _temps.back().end());
        }
        std::pair<conf_it,conf_it> next_confs(const abstract_rule_t& rule, const configuration_t& conf, const header_wrapper_t&  header_handler) override {
            _temps.emplace_back(); // We store all configurations in _temps. TODO: Make efficient range implementation. That was the hole point of returning iterators.
            make_configurations(_temps.back(), rule, std::vector<state_t>{conf.first}, conf.second, header_handler );
            return std::make_pair(_temps.back().begin(), _temps.back().end());
        }
        refinement_info_t find_refinement(const std::vector<configuration_t>&, const abstract_rule_t&) override {
            std::vector<std::string> labels_a, labels_b;
            std::vector<state_t> states_a, states_b;
            // TODO:: Implement
            return std::make_pair(std::make_pair(labels_a, labels_b), std::make_pair(states_a, states_b));
        }
        header_t get_header(const configuration_t& conf) override {
            return conf.second;
        }

    private:

        std::vector<rule_t> get_rules(size_t from) const {
            if (from < _rules.size()) {
                return _rules[from];
            }
            return std::vector<rule_t>();
        }

        void make_configurations(std::vector<configuration_t>& result, const abstract_rule_t& abstract_rule, const std::vector<state_t>& from_states, const header_t& header, const header_wrapper_t& header_handler) const {
            for (const state_t& from_state : from_states) {
                for (const auto& rule : get_rules(from_state)) {
                    std::vector<std::string> pre{rule._pre}; // Here only one label, but we support multiple pre labels - useful e.g. if concrete rule translates to multiple abstract rules.
                    std::vector<std::string> post;
                    switch (rule._op) {
                        case POP:
                            // empty post
                            break;
                        case NOOP:
                            post = pre;
                            break;
                        case SWAP:
                            post = std::vector<std::string>{rule._op_label};
                            break;
                        case PUSH:
                            post = std::vector<std::string>{rule._pre, rule._op_label};
                            break;
                    }
                    auto new_header = header_handler.update(header, pre, post);
                    if (new_header) {
                        result.emplace_back(rule._to, new_header.value());
                    }
                }
            }
        }
        void initialize() {
            _initial = abstract_states(PDAParser::parse_states(_input));
            _accepting = abstract_states(PDAParser::parse_states(_input));

            while (true) {
                auto [keep_going, rule] = PDAParser::parse_rule<rule_t, W>(_input);
                if (!keep_going) break;
                _rules.resize(std::max(rule._from, rule._to) + 1);
                _rules[rule._from].push_back(rule);
            }
        }
        std::vector<size_t> abstract_states(std::vector<state_t>&& concrete_states) {
            std::vector<size_t> abstract_states;
            for (const auto& state : concrete_states) {
                auto [fresh, id] = _state_abstraction.insert(state);
                abstract_states.push_back(id);
            }
            std::sort(abstract_states.begin(), abstract_states.end());
            abstract_states.erase(std::unique(abstract_states.begin(), abstract_states.end()), abstract_states.end());
            return abstract_states;
        }
        abstract_rule_t abstract_rule(const rule_t& r) {
            abstract_rule_t res;
            res._from = _state_abstraction.insert(r._from).second;
            res._to = _state_abstraction.insert(r._to).second;
            res._pre = this->insert_label(r._pre).second;
            res._op_label = (r._op == PUSH || r._op == SWAP) ? this->insert_label(r._op_label).second
                                                             : std::numeric_limits<uint32_t>::max();
            res._op = r._op;
            return res;
        }

    private:
        std::istream& _input;
        std::unordered_set<std::string> _all_labels;
        AbstractionMapping<state_t, abstract_state_t> _state_abstraction; // <size_t, size_t> is kind of a simple case, but fine for now
        std::vector<size_t> _initial; // Abstracted initial states
        std::vector<size_t> _accepting; // Abstracted final states
        std::vector<std::vector<rule_t>> _rules; // Concrete rules. Here stored explicitly, but CegarPdaFactory supports generating them on the fly.

        std::vector<std::vector<configuration_t>> _temps; // This is where all configurations are stored. Super inefficient, but I don't have better option yet. Implementing ranges would take some time...
    };

}

#endif //PDAAAL_PARSINGPDAFACTORY_H
