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
 * File:   Parsing.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-06-2021.
 */

#include <algorithm>

#include <pdaaal/parsing/Parsing.h>
#include <pdaaal/parsing/JsonParser.h>

namespace pdaaal {

    enum class input_format {PDAAAL, MOPED};

    struct parsing_options_t {
        explicit parsing_options_t(std::ostream& warnings, input_format format) : warnings(warnings), format(format) { };
        std::ostream& warnings;
        input_format format;
    };

    auto parse_stream(std::istream& stream, const parsing_options_t& parse_opts) {
        // TODO: More parsing options...
        return JsonParser::parse(stream, parse_opts.warnings);
    }

    auto parse_file(const std::string& input_file, const parsing_options_t& parse_opts) {
        std::ifstream input_stream(input_file);
        if (!input_stream.is_open()) {
            std::stringstream es;
            es << "error: Could not open file: " << input_file << std::endl;
            throw std::runtime_error(es.str());
        }
        return parse_stream(input_stream, parse_opts);
    }

    bool equals_case_insensitive_1(const std::string& any_case, const std::string& lower_case) {
        return any_case.size() == lower_case.size()
            && std::equal(any_case.begin(), any_case.end(), lower_case.begin(), lower_case.end(),
                          [](const char& c1, const char& c2){ return c1 == c2 || std::tolower(c1) == c2; });
    }
    input_format get_format(const std::string& format) {
        if (equals_case_insensitive_1(format, "pdaaal")) {
            return input_format::PDAAAL;
        } else if (equals_case_insensitive_1(format, "moped")) {
            return input_format::MOPED;
        } else {
            std::stringstream es;
            es << "error: Unrecognized input format: " << format << std::endl;
            throw std::runtime_error(es.str());
        }
    }

    StateTypedPDA<std::string,std::string> Parsing::parse(bool no_warnings) {
        auto format = get_format(input_format);
        std::stringstream dummy;
        parsing_options_t parse_opts(no_warnings ? dummy : std::cerr, format);

        parsing_stopwatch.start();
        auto value = (input_file.empty() || input_file == "-") ? parse_stream(std::cin, parse_opts) : parse_file(input_file, parse_opts);
        parsing_stopwatch.stop();
        return value;
    }



}