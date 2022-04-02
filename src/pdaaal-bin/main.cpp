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
 * File:   main.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 11-06-2021.
 */

#include "parsing/SolverInstanceJsonParser.h"
#include "parsing/Parsing.h"
#include "Verifier.h"
#include "utils/json_stream.h"

#include "version.h" // Generated at build time. Defines PDAAAL_GIT_HASH, PDAAAL_GIT_HASH_STR, PDAAAL_VERSION and PDAAAL_VERSION_STR

#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace pdaaal;

class main_output {
public:
    explicit main_output(const std::string& caption = "Output Options") : output_options(caption) {
        output_options.add_options()
                ("silent,s", po::bool_switch(&silent), "Disables non-essential output.")
                ("print-pda-json", po::value<std::string>(&pda_json_output), "Print PDA in JSON format to terminal.")
                ("print-solver-instance", po::value<std::string>(&solver_instance_output), "Print SolverInstance in JSON format to file.")
                ;
    }
    [[nodiscard]] const po::options_description& options() const { return output_options; }

    template<typename instance_t>
    void do_output(instance_t&& instance) {
        if (!pda_json_output.empty()) {
            std::ofstream out(pda_json_output);
            if (!out.is_open()) {
                std::cerr << "Could not open --print-pda-json \"" << pda_json_output << "\" for writing" << std::endl;
                exit(-1);
            }
            out << instance->pda().to_json().dump() << std::endl;
        }
        if (!solver_instance_output.empty()) {
            std::ofstream out(solver_instance_output);
            if (!out.is_open()) {
                std::cerr << "Could not open --print-solver-instance \"" << solver_instance_output << "\" for writing" << std::endl;
                exit(-1);
            }
            out << instance->to_json().dump() << std::endl;
        }
    }

    bool silent = false;
private:
    po::options_description output_options;
    std::string solver_instance_output, pda_json_output;
};

template<TraceInfoType traceInfoType>
void run(parsing::Parsing& parsing, Verifier& verifier, main_output& output) {
    auto instance_variant = parsing.parse_instance<traceInfoType>();
    json_stream json_out;
    if (!output.silent) { json_out.entry("parsing-duration", parsing.duration()); }
    std::visit([&verifier,&output,&json_out](auto&& instance) {
        output.do_output(instance);
        verifier.verify<traceInfoType>(*instance, json_out);
    }, instance_variant);
}

int main(int argc, const char** argv) {
    po::options_description opts;
    opts.add_options()
            ("help,h", "produce help message")
            ("version,v", "print version");

    parsing::Parsing parsing("Input Options");
    Verifier verifier("Verification Options");
    main_output output("Output Options");

    opts.add(parsing.options());
    opts.add(verifier.options());
    opts.add(output.options());

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, opts), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << opts << std::endl;
        return 1;
    }
    if (vm.count("version")) {
        std::cout << "PDAAAL v" PDAAAL_VERSION_STR " - git hash: " PDAAAL_GIT_HASH_STR << std::endl
                  << "Copyright (C) 2021  Morten K. Schou, Peter G. Jensen, Dan Kristiansen, Bernhard C. Schrenk" << std::endl
                  << "License GPLv3+: GNU GPL version 3 or later <https://gnu.org/licenses/gpl.html>." << std::endl
                  << "This is free software: you are free to change and redistribute it." << std::endl
                  << "There is NO WARRANTY, to the extent permitted by law." << std::endl;
        return 1;
    }

    if (verifier.needs_trace_info_pair()) { // Currently, we need to differentiate between these cases at top level.
        run<TraceInfoType::Pair>(parsing, verifier, output);
    } else {
        run<TraceInfoType::Single>(parsing, verifier, output);
    }

    return 0;
}
