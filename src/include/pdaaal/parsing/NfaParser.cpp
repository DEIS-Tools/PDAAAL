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
 * File:   NfaParser.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 05-07-2021.
 */

#include "NfaParser.h"
#include "NfaParserGrammar.h"
#include <iomanip>
#include <filesystem>

namespace pdaaal {

    template<typename T, typename Input>
    NFA<T> parse(Input& in, const std::function<T(const std::string&)>& label_function) {
        NfaBuilder<T> nfa_builder(label_function);
        try {
            pegtl::parse<nfa_file<>,nfa_build_action>(in, nfa_builder);
        } catch (const pegtl::parse_error& e) {
            std::stringstream s;
            const auto p = e.positions().front();
            s << e.what() << std::endl
              << in.line_at(p) << std::endl
              << std::setw(p.column) << '^' << std::endl;
            throw std::runtime_error(s.str());
        }
        return nfa_builder.get_nfa();
    }
    NFA<NfaParser::T> NfaParser::parse_file(const std::string& file, const std::function<NfaParser::T(const std::string&)>& label_function) {
        std::filesystem::path file_path(file);
        pegtl::file_input in(file_path);
        return parse<NfaParser::T>(in, label_function);
    }

    NFA<NfaParser::T> NfaParser::parse_string(const std::string& content, const std::function<NfaParser::T(const std::string&)>& label_function) {
        pegtl::memory_input in(content, "");
        return parse<NfaParser::T>(in, label_function);
    }

}
