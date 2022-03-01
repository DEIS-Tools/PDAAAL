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

#include "Parsing.h"
#include <algorithm>

namespace pdaaal::parsing {

    enum class input_format {PDAAAL, MOPED};
    enum class weight_type {NONE, UINT, INT};

    struct parsing_options_t {
        parsing_options_t(std::ostream& warnings, input_format format, weight_type weight, bool use_state_names)
        : warnings(warnings), format(format), weight(weight), use_state_names(use_state_names) { };
        std::ostream& warnings;
        input_format format;
        weight_type weight;
        bool use_state_names;
    };

    template <typename W>
    Parsing::pda_variant_t parse_stream_json_w(std::istream& stream, const parsing_options_t& parse_opts) {
        if (parse_opts.use_state_names) {
            return PdaJsonParser::parse<W,true>(stream, parse_opts.warnings);
        } else {
            return PdaJsonParser::parse<W,false>(stream, parse_opts.warnings);
        }
    }
    Parsing::pda_variant_t parse_stream_json(std::istream& stream, const parsing_options_t& parse_opts) {
        switch (parse_opts.weight) {
            case weight_type::UINT:
                return parse_stream_json_w<weight<uint32_t>>(stream, parse_opts);
            case weight_type::INT:
                return parse_stream_json_w<weight<int32_t>>(stream, parse_opts);
            case weight_type::NONE:
                return parse_stream_json_w<weight<void>>(stream, parse_opts);
            default:
                throw std::logic_error("That weight type is not yet supported...");
        }
    }
    Parsing::pda_variant_t parse_stream(std::istream& stream, const parsing_options_t& parse_opts) {
        if (parse_opts.format == input_format::PDAAAL) {
            return parse_stream_json(stream, parse_opts);
        } else {
            // TODO: More parsing options...
            throw std::logic_error("That input format is not yet supported...");
        }
    }
    Parsing::pda_variant_t parse_file(const std::string& input_file, const parsing_options_t& parse_opts) {
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
    weight_type get_weight_type(const std::string& weight_type) {
        if (equals_case_insensitive_1(weight_type, "none")) {
            return weight_type::NONE;
        } else if (equals_case_insensitive_1(weight_type, "uint")) {
            return weight_type::UINT;
        } else if (equals_case_insensitive_1(weight_type, "int")) {
            return weight_type::INT;
        } else {
            std::stringstream es;
            es << "error: Unrecognized weight type: " << weight_type << std::endl;
            throw std::runtime_error(es.str());
        }
    }

    Parsing::pda_variant_t Parsing::parse_pda(bool no_warnings) {
        std::stringstream dummy;
        parsing_options_t parse_opts(no_warnings ? dummy : std::cerr, get_format(input_format),
                                     get_weight_type(weight_type), use_state_names);
        parsing_stopwatch.start();
        auto value = (pda_file.empty() || pda_file == "-")
                     ? parse_stream(std::cin, parse_opts)
                     : parse_file(pda_file, parse_opts);
        parsing_stopwatch.stop();
        return value;
    }

}