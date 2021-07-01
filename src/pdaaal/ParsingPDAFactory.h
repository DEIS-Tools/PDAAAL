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

#include "PDAFactory.h"
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

    template <typename W = weight<void>>
    class ParsingPDAFactory : public DFS_PDAFactory<std::string, W> {
    public:
        static ParsingPDAFactory<W> create(std::istream& input) {
            auto all_labels = PDAParser::parse_all_labels(input);
            return ParsingPDAFactory<W>(input, std::move(all_labels));
        }
    private:
        ParsingPDAFactory(std::istream& input, std::unordered_set<std::string>&& all_labels)
        : DFS_PDAFactory<std::string, W>(std::move(all_labels), "."), _input(input) {
            initialize();
        };
        using rule_t = typename DFS_PDAFactory<std::string, W>::rule_t;
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


    template <typename W> class ParsingCegarPdaReconstruction;

    template <typename W = weight<void>>
    class ParsingCegarPdaFactory : public CegarPdaFactory<std::string, W> {
        friend class ParsingCegarPdaReconstruction<W>;
    public:
        using label_t = std::string;
    private:
        using abstract_label_t = int;
        using state_t = size_t;
        using rule_t = typename TypedPDA<std::string, W>::rule_t; // For concrete rules we just use this one.
        using abstract_state_t = state_t;
        using parent_t = CegarPdaFactory<label_t, W>;
        using abstract_rule_t = typename parent_t::abstract_rule_t;
        using solver_instance_t = typename parent_t::solver_instance_t;
    public:
        struct ConcretePDA {
            explicit ConcretePDA(std::istream& input) {
                _initial = PDAParser::parse_states(input);
                _accepting = PDAParser::parse_states(input);
                while (true) {
                    auto [keep_going, rule] = PDAParser::parse_rule<rule_t, W>(input);
                    if (!keep_going) break;
                    _rules.resize(std::max(rule._from, rule._to) + 1);
                    _rules[rule._from].push_back(rule);
                }
            };
            std::vector<state_t> _initial; // Concrete initial states
            std::vector<state_t> _accepting; // Concrete final states
            std::vector<std::vector<rule_t>> _rules; // Concrete rules. Here stored explicitly, but CegarPdaFactory supports generating them on the fly.
        };

    private:
        ParsingCegarPdaFactory(std::istream& input, std::unordered_set<std::string>&& all_labels,
                               std::function<abstract_label_t(const label_t&)>&& label_abstraction_fn,
                               std::function<abstract_state_t(const state_t&)>&& state_abstraction_fn)
        : parent_t(all_labels, std::move(label_abstraction_fn)), _concrete_pda(input) {
            AbstractionMapping<state_t, abstract_state_t> builder_mapping(std::move(state_abstraction_fn));
            _initial = abstract_states(_concrete_pda._initial, builder_mapping);
            _accepting = abstract_states(_concrete_pda._accepting, builder_mapping);
            for (const auto& rules : _concrete_pda._rules) { // rules for each from state.
                for (const auto& rule : rules) {
                    builder_mapping.insert(rule._from);
                    builder_mapping.insert(rule._to);
                }
            }
            _state_abstraction = RefinementMapping<state_t>(std::move(builder_mapping));
        };

    public:
        void refine(typename ParsingCegarPdaReconstruction<W>::refinement_t&& refinement) {
            assert(refinement.index() == 0);
            _state_abstraction.refine(std::get<0>(refinement).first);
        }
        void refine(typename ParsingCegarPdaReconstruction<W>::header_refinement_t&& refinement) {
            // No refinement of states.
        }

        static ParsingCegarPdaFactory<W> create(std::istream& input,
            std::function<abstract_label_t(const label_t&)>&& label_abstraction_fn,
            std::function<abstract_state_t(const state_t&)>&& state_abstraction_fn)
        {
            auto all_labels = PDAParser::parse_all_labels(input);
            return ParsingCegarPdaFactory<W>(input, std::move(all_labels), std::move(label_abstraction_fn), std::move(state_abstraction_fn));
        }

    protected:
        void build_pda() override {
            if (!first) {
                _initial = abstract_states(_concrete_pda._initial);
                _accepting = abstract_states(_concrete_pda._accepting);
            }
            first = false;
            for (const auto& rules : _concrete_pda._rules) { // rules for each from state.
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

    private:
        std::vector<size_t> abstract_states(const std::vector<state_t>& concrete_states, AbstractionMapping<state_t, abstract_state_t>& builder_mapping) {
            std::vector<size_t> abstract_states;
            for (const auto& state : concrete_states) {
                auto [fresh, id] = builder_mapping.insert(state);
                abstract_states.push_back(id);
            }
            std::sort(abstract_states.begin(), abstract_states.end());
            abstract_states.erase(std::unique(abstract_states.begin(), abstract_states.end()), abstract_states.end());
            return abstract_states;
        }
        std::vector<size_t> abstract_states(const std::vector<state_t>& concrete_states) {
            std::vector<size_t> abstract_states;
            for (const auto& state : concrete_states) {
                auto [found, id] = _state_abstraction.exists(state);
                assert(found);
                abstract_states.push_back(id);
            }
            std::sort(abstract_states.begin(), abstract_states.end());
            abstract_states.erase(std::unique(abstract_states.begin(), abstract_states.end()), abstract_states.end());
            return abstract_states;
        }
        abstract_rule_t abstract_rule(const rule_t& r) {
            abstract_rule_t res;
            assert(_state_abstraction.exists(r._from).first);
            res._from = _state_abstraction.exists(r._from).second;
            assert(_state_abstraction.exists(r._to).first);
            res._to = _state_abstraction.exists(r._to).second;
            assert(this->abstract_label(r._pre).first);
            res._pre = this->abstract_label(r._pre).second;
            assert(!(r._op == PUSH || r._op == SWAP) || this->abstract_label(r._op_label).first);
            res._op_label = (r._op == PUSH || r._op == SWAP) ? this->abstract_label(r._op_label).second
                                                             : std::numeric_limits<uint32_t>::max();
            res._op = r._op;
            return res;
        }

    private:
        RefinementMapping<state_t> _state_abstraction; // <size_t, size_t> is kind of a simple case, but fine for now
        std::vector<size_t> _initial; // Abstracted initial states
        std::vector<size_t> _accepting; // Abstracted final states
        bool first = true;
        ConcretePDA _concrete_pda;
    };


    template <typename W = weight<void>>
    class ParsingCegarPdaReconstruction : public CegarPdaReconstruction<
            std::string, // label_t
            size_t, // state_t
            const std::vector< // configuration_range_t        In this case configuration_t consists of:
                    std::pair<typename TypedPDA<std::string, W>::rule_t, // Our concrete rule_t
                              Header<std::string>> // header_t
            >&,
            std::vector<typename TypedPDA<std::string>::tracestate_t>, // concrete_trace_t
            W> {
        friend class ParsingCegarPdaFactory<W>;
    public:
        using label_t = std::string;
        using concrete_trace_t = std::vector<typename TypedPDA<label_t>::tracestate_t>;
    private:
        using state_t = size_t;
        using header_t = Header<label_t>;
        using rule_t = typename TypedPDA<std::string, W>::rule_t; // For concrete rules we just use this one.
        using configuration_t = std::pair<rule_t, header_t>;
        using configuration_range_t = const std::vector<configuration_t>&;
        using parent_t = CegarPdaReconstruction<label_t, state_t, configuration_range_t, concrete_trace_t, W>;
        using abstract_rule_t = typename parent_t::abstract_rule_t;
        using solver_instance_t = typename parent_t::solver_instance_t;
    public:
        using refinement_t = typename parent_t::refinement_t;
        using header_refinement_t = typename parent_t::header_refinement_t;

        explicit ParsingCegarPdaReconstruction(const ParsingCegarPdaFactory<W>& factory, const solver_instance_t& instance,
                                               const NFA<label_t>& initial_headers, const NFA<label_t>& final_headers)
        : parent_t(instance, initial_headers, final_headers),
          _state_abstraction(factory._state_abstraction), _rules(factory._concrete_pda._rules), _initial_states(factory._concrete_pda._initial) {};

    protected:

        // TODO: Coroutines might make this a lot simpler...
        configuration_range_t initial_concrete_rules(const abstract_rule_t& rule) override {
            auto from_states = _state_abstraction.get_concrete_values(rule._from);
            auto header = this->initial_header();
            _temps.emplace_back(); // We store all configurations in _temps. TODO: Make efficient range implementation. That was the hole point of returning iterators.
            make_configurations(_temps.back(), rule, from_states, header);
            return _temps.back();
        }
        configuration_range_t search_concrete_rules(const abstract_rule_t& rule, const configuration_t& conf) override {
            _temps.emplace_back(); // We store all configurations in _temps. TODO: Make efficient range implementation. That was the hole point of returning iterators.
            make_configurations(_temps.back(), rule, std::vector<state_t>{conf.first._to}, conf.second);
            return _temps.back();
        }
        refinement_t find_initial_refinement(const abstract_rule_t& abstract_rule) override {
            std::vector<std::pair<state_t,label_t>> X, Y;

            auto labels = this->pre_labels(this->initial_header());
            for (const auto& initial_state : _initial_states) {
                if (_state_abstraction.maps_to(initial_state, abstract_rule._from)) {
                    for (const auto& label : labels) {
                        if (this->label_maps_to(label, abstract_rule._pre)) {
                            X.emplace_back(initial_state, label);
                        }
                    }
                }
            }
            for (const auto& from_state : _state_abstraction.get_concrete_values(abstract_rule._from)) {
                for (const auto& rule : get_rules(from_state)) {
                    if (rule_match(rule, abstract_rule)){
                        Y.emplace_back(rule._from, rule._pre);
                    }
                }
            }
            return make_refinement<refinement_option_t::best_refinement>(std::move(X), std::move(Y), abstract_rule._from, abstract_rule._pre);
        }
        refinement_t find_refinement(const abstract_rule_t& abstract_rule, const std::vector<configuration_t>& configurations) override {
            std::vector<std::pair<state_t,label_t>> X, Y;

            for (const auto& [c_rule, header] : configurations) {
                auto state = c_rule._to;
                if (_state_abstraction.maps_to(state, abstract_rule._from)) {
                    for (const auto& label : this->pre_labels(header)) {
                        if (this->label_maps_to(label, abstract_rule._pre)) {
                            X.emplace_back(state, label);
                        }
                    }
                }
            }
            for (const auto& from_state : _state_abstraction.get_concrete_values(abstract_rule._from)) {
                for (const auto& rule : get_rules(from_state)) {
                    if (rule_match(rule, abstract_rule)){
                        Y.emplace_back(rule._from, rule._pre);
                    }
                }
            }
            return make_refinement<refinement_option_t::best_refinement>(std::move(X), std::move(Y), abstract_rule._from, abstract_rule._pre);
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

        bool rule_match(const rule_t& rule, const abstract_rule_t& abstract_rule) const {
            assert(_state_abstraction.maps_to(rule._from, abstract_rule._from)); // Matching _from is a precondition where this is used.
            return rule._op == abstract_rule._op &&
                   _state_abstraction.maps_to(rule._to, abstract_rule._to) &&
                   this->label_maps_to(rule._pre, abstract_rule._pre) &&
                   (abstract_rule._op == POP || abstract_rule._op == NOOP ||
                    this->label_maps_to(rule._op_label, abstract_rule._op_label));
        }

        void make_configurations(std::vector<configuration_t>& result, const abstract_rule_t& abstract_rule, const std::vector<state_t>& from_states, const header_t& header) const {
            for (const state_t& from_state : from_states) {
                for (const auto& rule : get_rules(from_state)) {
                    if (!rule_match(rule, abstract_rule)) continue;
                    std::vector<std::string> post;
                    switch (abstract_rule._op) {
                        case POP:
                            // empty post
                            break;
                        case NOOP:
                            post = std::vector<std::string>{rule._pre};
                            break;
                        case SWAP:
                            post = std::vector<std::string>{rule._op_label};
                            break;
                        case PUSH:
                            post = std::vector<std::string>{rule._pre, rule._op_label};
                            break;
                    }
                    auto new_header = this->update_header(header, rule._pre, post);
                    if (new_header) {
                        result.emplace_back(rule, new_header.value().first);
                    }
                }
            }
        }

    private:
        const RefinementMapping<state_t>& _state_abstraction; // <size_t, size_t> is kind of a simple case, but fine for now
        const std::vector<std::vector<rule_t>>& _rules;
        const std::vector<size_t>& _initial_states;
        std::vector<std::vector<configuration_t>> _temps; // This is where all configurations are stored. Super inefficient, but I don't have better option yet. Implementing ranges would take some time...
    };

}

#endif //PDAAAL_PARSINGPDAFACTORY_H
