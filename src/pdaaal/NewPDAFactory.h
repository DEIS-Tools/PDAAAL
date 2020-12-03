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
 * File:   NewPDAFactory.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-11-2020.
 */

#ifndef PDAAAL_NEWPDAFACTORY_H
#define PDAAAL_NEWPDAFACTORY_H

#include <string>
#include <sstream>
#include <vector>
#include <ostream>
#include <unordered_set>

#include "NFA.h"
#include "TypedPDA.h"
#include "PAutomaton.h"
#include "SolverInstance.h"

namespace pdaaal {

    template<typename T, typename W = void, typename C = std::less<W>, typename A = add<W>>
    class NewPDAFactory {
    private:
        using Temp_PDA = TypedPDA<T,W,C,fut::type::hash>;
        using Result_PDA = TypedPDA<T,W,C,fut::type::vector>;
    public:
        using rule_t = typename Temp_PDA::rule_t;

        explicit NewPDAFactory(std::unordered_set<T>&& all_labels) : _all_labels(std::move(all_labels)) { };

        // NFAs must be already compiled before passing them to this function.
        SolverInstance<T,W,C,A> compile(const NFA<T>& initial_headers, const NFA<T>& final_headers) {
            Temp_PDA temp_pda(_all_labels);
            build_pda(temp_pda);
            return SolverInstance<T,W,C,A>{Result_PDA{std::move(temp_pda)}, initial_headers, initial(), final_headers, accepting_states};
        }

        void build_pda(Temp_PDA& pda) {
            // Build up PDA by searching through reachable states from initial states.
            // Derived class must define initial states, successor function (rules), and accepting state predicate.
            std::vector<size_t> waiting = initial();
            std::unordered_set<size_t> seen(waiting.begin(), waiting.end());
            while (!waiting.empty()) {
                auto from = waiting.back();
                waiting.pop_back();
                if (accepting(from)) {
                    accepting_states.push_back(from);
                }
                for (const auto &r : rules(from)) {
                    assert(_all_labels.count(r._pre) == 1);
                    assert(from == r._from);

                    pda.add_rule(r);

                    if (seen.emplace(r._to).second) {
                        waiting.push_back(r._to);
                    }
                }
            }
            std::sort(accepting_states.begin(), accepting_states.end());
        }

    protected:
        virtual const std::vector<size_t>& initial() = 0;
        virtual bool accepting(size_t) = 0;
        virtual std::vector<rule_t> rules(size_t) = 0;

        std::unordered_set<T> _all_labels;

    private:
        std::vector<size_t> accepting_states;
    };

}

#endif //PDAAAL_NEWPDAFACTORY_H
