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


    template <typename W = void, typename C = std::less<W>, typename A = add<W>>
    class ParsingCegarPdaFactory : public CegarPdaFactory<
        std::string, // label_t
        int, // abstract_label_t
        size_t, // state_t
        std::vector< // configuration_range_t        In this case configuration_t consists of:
                std::pair<typename TypedPDA<std::string, W, C>::rule_t, // Our concrete rule_t
                          typename wildcard_header<std::string,int,W,C>::header_t> // header_t
                >,
        std::vector<typename TypedPDA<std::string>::tracestate_t>, // concrete_trace_t
        W, C, A> {
    private:
        using label_t = std::string;
        using abstract_label_t = int;
        using state_t = size_t;
        using header_wrapper_t = wildcard_header<label_t,abstract_label_t,W,C>;
        using header_t = typename header_wrapper_t::header_t;
        using rule_t = typename TypedPDA<std::string, W, C>::rule_t; // For concrete rules we just use this one.
        using configuration_t = std::pair<rule_t, header_t>;
        using configuration_range_t = std::vector<configuration_t>;
        using concrete_trace_t = std::vector<typename TypedPDA<label_t>::tracestate_t>;
        using abstract_state_t = state_t;
        using parent_t = CegarPdaFactory<label_t, abstract_label_t, state_t, configuration_range_t, concrete_trace_t, W, C, A>;
        using refinement_info_t = typename parent_t::refinement_info_t;
        using abstract_rule_t = typename parent_t::abstract_rule_t;

        ParsingCegarPdaFactory(std::istream& input, std::unordered_set<std::string>&& all_labels,
                               std::function<abstract_label_t(const label_t&)>&& label_abstraction_fn,
                               std::function<abstract_state_t(const state_t&)>&& state_abstraction_fn)
        : parent_t(std::move(all_labels), std::move(label_abstraction_fn)),
          _input(input), _state_abstraction(std::move(state_abstraction_fn)) {
            initialize();
        };
    public:
        static ParsingCegarPdaFactory<W,C,A> create(std::istream& input,
            std::function<abstract_label_t(const label_t&)>&& label_abstraction_fn,
            std::function<abstract_state_t(const state_t&)>&& state_abstraction_fn)
        {
            auto all_labels = PDAParser::parse_all_labels(input);
            return ParsingCegarPdaFactory<W,C,A>(input, std::move(all_labels), std::move(label_abstraction_fn), std::move(state_abstraction_fn));
        }

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
        const configuration_range_t& initial_concrete_rules(const abstract_rule_t& rule, const header_wrapper_t& header_handler) override {
            auto from_states = _state_abstraction.get_concrete_values(rule._from);
            auto header = header_handler.initial_header();
            _temps.emplace_back(); // We store all configurations in _temps. TODO: Make efficient range implementation. That was the hole point of returning iterators.
            make_configurations(_temps.back(), rule, from_states, header, header_handler);
            return _temps.back();
        }
        const configuration_range_t& search_concrete_rules(const abstract_rule_t& rule, const configuration_t& conf, const header_wrapper_t&  header_handler) override {
            _temps.emplace_back(); // We store all configurations in _temps. TODO: Make efficient range implementation. That was the hole point of returning iterators.
            make_configurations(_temps.back(), rule, std::vector<state_t>{conf.first._to}, conf.second, header_handler);
            return _temps.back();
        }
        refinement_info_t find_refinement(const std::vector<configuration_t>& configurations, const abstract_rule_t& abstract_rule, const header_wrapper_t&  header_handler) override {
            RefinementInfo<label_t> label_refinement;
            RefinementInfo<state_t> state_refinement;
            for (const auto& [c_rule, header] : configurations) {
                auto state = c_rule._to;
                if (_state_abstraction.maps_to(state, abstract_rule._from) && // Check if configuration matches abstract_rule.
                    (!header.top_is_concrete() || header_handler.maps_to(header.concrete_part.back(), abstract_rule._pre))) {
                    for (const auto& from_state : _state_abstraction.get_concrete_values(abstract_rule._from)) { // Search through concrete rules corresponding to abstract_rule
                        for (const auto& rule : get_rules(from_state)) {
                            if (!rule_match(rule, abstract_rule, header_handler)) continue;
                            if (header.top_is_concrete()) { // We must have violation of either state or label
                                assert(state != rule._from || header.concrete_part.back() != rule._pre);
                                if (state != rule._from) {
                                    state_refinement.add(state, rule._from);
                                }
                                if (header.concrete_part.back() != rule._pre) {
                                    label_refinement.add(header.concrete_part.back(), rule._pre);
                                }
                            } else { // Header top is wildcard, so violation must be due to state
                                assert(state != rule._from);
                                state_refinement.add(state, rule._from);
                            }
                        }
                    }
                }
            }
            label_refinement.remove_duplicates();
            state_refinement.remove_duplicates();
            assert(!label_refinement.empty() || !state_refinement.empty());
            return std::make_pair(label_refinement, state_refinement);
        }
        header_t get_header(const configuration_t& conf) override {
            return conf.second;
        }

        concrete_trace_t get_concrete_trace(std::vector<configuration_t>&& configurations, std::vector<label_t>&& final_header, size_t initial_abstract_state) override {
            concrete_trace_t trace;
            for (auto it = configurations.crbegin(); it < configurations.crend(); ++it) {
                trace.emplace_back();
                trace.back()._pdastate = it->first._to;
                trace.back()._stack.assign(final_header.rbegin(), final_header.rend());
                switch (it->first._op) {
                    case POP:
                        final_header.push_back(it->first._pre);
                        break;
                    case NOOP:
                        break;
                    case SWAP:
                        final_header.back() = it->first._pre;
                        break;
                    case PUSH:
                        final_header.pop_back();
                        break;
                }
            }
            trace.emplace_back();
            if (configurations.empty()) {
                auto range = _state_abstraction.get_concrete_values_range(initial_abstract_state);
                assert(range.begin() != range.end());
                trace.back()._pdastate = *range.begin();
            } else {
                trace.back()._pdastate = configurations[0].first._from;
            }
            trace.back()._stack.assign(final_header.rbegin(), final_header.rend());
            std::reverse(trace.begin(), trace.end());
            return trace;
        }

    private:

        std::vector<rule_t> get_rules(size_t from) const {
            if (from < _rules.size()) {
                return _rules[from];
            }
            return std::vector<rule_t>();
        }

        bool rule_match(const rule_t& rule, const abstract_rule_t& abstract_rule, const header_wrapper_t& header_handler) const {
            assert(_state_abstraction.maps_to(rule._from, abstract_rule._from)); // Matching _from is a precondition where this is used.
            return rule._op == abstract_rule._op &&
                   _state_abstraction.maps_to(rule._to, abstract_rule._to) &&
                   header_handler.maps_to(rule._pre, abstract_rule._pre) &&
                   (abstract_rule._op == POP || abstract_rule._op == NOOP ||
                    header_handler.maps_to(rule._op_label, abstract_rule._op_label));
        }

        void make_configurations(std::vector<configuration_t>& result, const abstract_rule_t& abstract_rule, const std::vector<state_t>& from_states, const header_t& header, const header_wrapper_t& header_handler) const {
            for (const state_t& from_state : from_states) {
                for (const auto& rule : get_rules(from_state)) {
                    if (!rule_match(rule, abstract_rule, header_handler)) continue;
                    std::vector<std::string> pre{rule._pre}; // Here only one label, but we support multiple pre labels - useful e.g. if concrete rule translates to multiple abstract rules.
                    std::vector<std::string> post;
                    switch (abstract_rule._op) {
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
                        result.emplace_back(rule, new_header.value());
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
        AbstractionMapping<state_t, abstract_state_t> _state_abstraction; // <size_t, size_t> is kind of a simple case, but fine for now
        std::vector<size_t> _initial; // Abstracted initial states
        std::vector<size_t> _accepting; // Abstracted final states
        std::vector<std::vector<rule_t>> _rules; // Concrete rules. Here stored explicitly, but CegarPdaFactory supports generating them on the fly.

        std::vector<std::vector<configuration_t>> _temps; // This is where all configurations are stored. Super inefficient, but I don't have better option yet. Implementing ranges would take some time...
    };

}

#endif //PDAAAL_PARSINGPDAFACTORY_H
