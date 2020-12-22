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
        static ParsingPDAFactory<W,C,A> make_parsing_pda_factory(std::istream& input) {
            auto all_labels = PDAParser::parse_all_labels(input);
            return ParsingPDAFactory<W,C,A>(input, std::move(all_labels));
        }
    private:
        explicit ParsingPDAFactory(std::istream& input, std::unordered_set<std::string>&& all_labels)
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

}

#endif //PDAAAL_PARSINGPDAFACTORY_H
