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
 * File:   vector_printer.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 22-04-2022.
 */

#ifndef PDAAAL_VECTOR_PRINTER_H
#define PDAAAL_VECTOR_PRINTER_H

#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace pdaaal {
    class vector_printer {
    public:
        vector_printer() = default;
        explicit vector_printer(std::string separator) : _separator(std::move(separator)) {};
        vector_printer(std::string start, std::string end) : _start(std::move(start)), _end(std::move(end)) {};
        vector_printer(std::string start, std::string separator, std::string end)
        : _start(std::move(start)), _separator(std::move(separator)), _end(std::move(end)) {};

    private:
        vector_printer(std::ostream& o, const vector_printer& vp)
        : _o(o), _start(vp._start), _separator(vp._separator), _end(vp._end) {};

        friend vector_printer operator<<(std::ostream& o, const vector_printer& vp) {
            return {o,vp};
        }
        template<typename T>
        friend std::ostream& operator<<(vector_printer vp, const std::vector<T>& v) {
            vp._o << vp._start;
            bool first = true;
            for (auto&& elem : v) {
                if (first) first = false;
                else vp._o << vp._separator;
                vp._o << elem;
            }
            vp._o << vp._end;
            return vp._o;
        }

    private:
        std::ostream& _o = std::cout;
        std::string _start = "[";
        std::string _separator = ",";
        std::string _end = "]";
    };

}

#endif //PDAAAL_VECTOR_PRINTER_H
