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
 * File:   Parsing.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-06-2021.
 */

#ifndef PDAAAL_PARSING_H
#define PDAAAL_PARSING_H

#include "PdaJsonParser.h"
#include "SolverInstanceJsonParser.h"
#include "PAutomatonParser.h"
#include <utils/stopwatch.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>

namespace po = boost::program_options;

namespace pdaaal::parsing {

    class Parsing {
    public:
        using pda_variant_t = std::variant<
                PdaSaxHandler<weight<void>, true>::pda_t,
                PdaSaxHandler<weight<void>, false>::pda_t,
                PdaSaxHandler<weight<uint32_t>, true>::pda_t,
                PdaSaxHandler<weight<uint32_t>, false>::pda_t,
                PdaSaxHandler<weight<int32_t>, true>::pda_t,
                PdaSaxHandler<weight<int32_t>, false>::pda_t>;
        template<TraceInfoType trace_info_type>
        using solver_instance_variant_t = typename SolverInstanceSaxHandler<trace_info_type>::solver_instance_variant_t;

        explicit Parsing(const std::string& caption = "Input Options") : input_options{caption} {
            input_options.add_options()
                    ("input", po::value<std::string>(&input_file), "Input file. To read from std input specify '--input -'.")
                    ("msgpack", po::bool_switch(&msgpack), "Use the binary MessagePack input format")
                    ("pda,p", po::value<std::string>(&pda_file), "PDA JSON file input.")
                    ("format", po::value<std::string>(&input_format), "Input format. pdaaal|moped (default=pdaaal).")
                    ("weight", po::value<std::string>(&weight_type), "Weight type. none|uint|int (default=none).")
                    ("state-names", po::bool_switch(&use_state_names), "Enable named states (instead of index).")
                    ("initial-automaton,i", po::value<std::string>(&initial_pa_file), "Initial PAutomaton file input.")
                    ("final-automaton,f", po::value<std::string>(&final_pa_file), "Final PAutomaton file input.")
                    ("json-automata", po::bool_switch(&json_automata), "Parse P-automata files using JSON format.")
                    ;
        }
        [[nodiscard]] const po::options_description& options() const { return input_options; }
        [[nodiscard]] double duration() const { return parsing_stopwatch.duration(); }

        pda_variant_t parse_pda(bool no_warnings = false);

        template<TraceInfoType trace_info_type = TraceInfoType::Single>
        solver_instance_variant_t<trace_info_type> parse_instance() {
            if (input_file.empty() && !pda_file.empty() && !initial_pa_file.empty() && !final_pa_file.empty()) {
                // Parse from separate PDA and P-automata files.
                parsing_stopwatch.start();
                auto pda = parse_pda();
                auto value = std::visit([this](auto&& pda){
                    auto initial_p_automaton = json_automata ?
                                               PAutomatonJsonParser::parse<trace_info_type>(initial_pa_file, pda) :
                                               PAutomatonParser::parse_file<trace_info_type>(initial_pa_file, pda);
                    auto final_p_automaton = json_automata ?
                                             PAutomatonJsonParser::parse<trace_info_type>(final_pa_file, pda) :
                                             PAutomatonParser::parse_file<trace_info_type>(final_pa_file, pda);
                    return solver_instance_variant_t<trace_info_type>(SolverInstance(std::forward<decltype(pda)>(pda), std::move(initial_p_automaton), std::move(final_p_automaton)));
                }, std::move(pda));
                parsing_stopwatch.stop();
                return value;
            } else {
                // Parse from single solver-instance file.
                if (input_file.empty() || input_file == "-") {
                    return parse_instance_stream<trace_info_type>(std::cin);
                } else {
                    std::ifstream input_stream(input_file);
                    if (!input_stream.is_open()) {
                        std::stringstream es;
                        es << "error: Could not open file: " << input_file << std::endl;
                        throw std::runtime_error(es.str());
                    }
                    return parse_instance_stream<trace_info_type>(input_stream);
                }
            }
        }

    private:
        template<TraceInfoType trace_info_type>
        auto parse_instance_stream(std::istream& input_stream) {
            parsing_stopwatch.start();
            auto value = SolverInstanceJsonParser::parse<trace_info_type>(input_stream, msgpack ? json::input_format_t::msgpack : json::input_format_t::json);
            parsing_stopwatch.stop();
            return value;
        }

        po::options_description input_options;
        std::string input_file;
        std::string input_format = "pdaaal";
        std::string weight_type = "none";
        bool use_state_names = false;
        bool msgpack = false;
        std::string pda_file, initial_pa_file, final_pa_file, solver_instance_input;
        bool json_automata = false;
        stopwatch parsing_stopwatch{false};
    };
}

#endif //PDAAAL_PARSING_H
