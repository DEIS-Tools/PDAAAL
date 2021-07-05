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
 * File:   Verifier.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 02-07-2021.
 */

#ifndef PDAAAL_VERIFIER_H
#define PDAAAL_VERIFIER_H

#include <pdaaal/Solver.h>
#include <pdaaal/parsing/NfaParser.h>

namespace pdaaal {
    class Verifier {
    public:
        explicit Verifier(const std::string& caption) : verification_options{caption} {
            verification_options.add_options()
                    ("engine,e", po::value<size_t>(&engine), "Engine. 0=no verification, 1=post*, 2=pre*, 3=dual*")
                    ("nfa", po::value<std::string>(&nfa_file), "Nfa file input.")
                    ;
        }
        [[nodiscard]] const po::options_description& options() const { return verification_options; }

        template <typename pda_t>
        void verify(const pda_t& pda) {

            auto nfa = NfaParser::parse_file(nfa_file, [&pda](const std::string& label) -> size_t {
                auto [found, id] = pda.exists_label(label);
                return found ? id : 0;
            });
            // TODO: Use nfa.

            switch (engine) {
                case 1: {
                    // TODO: Implement.
                    break;
                }
                case 2: {

                    break;
                }
                case 3: {

                    break;
                }
            }
        }

    private:
        po::options_description verification_options;
        size_t engine = 0;
        std::string nfa_file;
        //bool print_trace = false;
    };
}

#endif //PDAAAL_VERIFIER_H
