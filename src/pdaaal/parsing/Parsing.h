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

#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>

#include <pdaaal/utils/stopwatch.h>
#include <pdaaal/StateTypedPDA.h>

namespace po = boost::program_options;

namespace pdaaal {

    class Parsing {
    public:
        explicit Parsing(const std::string& caption = "Input Options") : input_options{caption} {
            input_options.add_options()
                    ("input", po::value<std::string>(&input_file), "Input file. To read from std input specify '--input -'.")
                    ("format", po::value<std::string>(&input_format), "Input format. pdaaal|moped (default=pdaaal).")
                    ;
        }
        [[nodiscard]] const po::options_description& options() const { return input_options; }
        [[nodiscard]] double duration() const { return parsing_stopwatch.duration(); }
        StateTypedPDA<std::string,std::string> parse(bool no_warnings = false);

    private:
        po::options_description input_options;
        std::string input_file;
        std::string input_format = "pdaaal";
        stopwatch parsing_stopwatch{false};
    };
}

#endif //PDAAAL_PARSING_H
