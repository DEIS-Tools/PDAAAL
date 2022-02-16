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
 * File:   NfaParser.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 04-07-2021.
 */

#ifndef PDAAAL_NFAPARSER_H
#define PDAAAL_NFAPARSER_H

#include <pdaaal/NFA.h>

namespace pdaaal {

    class NfaParser {
        using T = size_t;
    public:
        static NFA<T> parse_file(const std::string& file, const std::function<T(const std::string&)>& label_function);
        static NFA<T> parse_string(const std::string& content, const std::function<T(const std::string&)>& label_function);
    };

}

#endif //PDAAAL_NFAPARSER_H
