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

#include "parsing/Parsing.h"
#include "Verifier.h"

#include "version.h" // Generated at build time. Defines PDAAAL_GIT_HASH, PDAAAL_GIT_HASH_STR, PDAAAL_VERSION and PDAAAL_VERSION_STR

#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace pdaaal;

int main(int argc, const char** argv) {
    po::options_description opts;
    opts.add_options()
            ("help,h", "produce help message")
            ("version,v", "print version");

    parsing::Parsing parsing("Input Options");
    Verifier verifier("Verification Options");
    po::options_description output("Output Options");

    bool no_parser_warnings = false;
    bool silent = false;
    bool print_pda_json = false;
    std::string solver_instance_file;
    output.add_options()
            ("disable-parser-warnings,W", po::bool_switch(&no_parser_warnings), "Disable warnings from parser.")
            ("silent,s", po::bool_switch(&silent), "Disables non-essential output (implies -W).")
            ("print-pda-json", po::bool_switch(&print_pda_json), "Print PDA in JSON format to terminal.")  // TODO: This is currently mostly a debug option. Make it useful!
            ("print-solver-instance", po::value<std::string>(&solver_instance_file), "Print SolverInstance in JSON format to file.")
            ;
    opts.add(parsing.options());
    opts.add(verifier.options());
    opts.add(output);

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

    if (silent) { no_parser_warnings = true; }

    auto pda_variant = parsing.parse(no_parser_warnings);
    if (!silent) {
        std::cout << "Parsing duration: " << parsing.duration() << std::endl;
    }
    if (print_pda_json) {
        std::visit([](auto&& pda){
            std::cout << pda.to_json().dump() << std::endl;
        }, pda_variant);
        return 0; // TODO: What else.?
    }
    if (!solver_instance_file.empty()) {
        std::ofstream out(solver_instance_file);
        if (out.is_open()) {
            std::visit([&out,&verifier](auto&& pda){
                auto instance = verifier.get_product(std::forward<decltype(pda)>(pda));
                out << instance.to_json().dump() << std::endl;
            }, pda_variant);
        } else {
            std::cerr << "Could not open --print-solver-instance\"" << solver_instance_file << "\" for writing" << std::endl;
            exit(-1);
        }
        return 0;
    }
    std::visit([](auto&& pda){
        std::cout << "States: " << pda.states().size() << ". Labels: " << pda.number_of_labels() << std::endl;
    }, pda_variant);

    std::visit([&verifier](auto&& pda) {
        verifier.verify(std::forward<decltype(pda)>(pda));
    }, pda_variant);

    return 0;
}