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
 * File:   Reducer_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 17-03-2020.
 */

#define BOOST_TEST_MODULE Reducer

#include <boost/test/unit_test.hpp>
#include <pdaaal/internal/Reducer.h>
#include <pdaaal/PDA.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(ReducerTest1) {
    // This is pretty much the rules from the example in Figure 3.1 (Schwoon-php02)
    // However r_2 requires a swap and a push, which is done through auxiliary state 3.
    std::unordered_set<char> labels{'A', 'B', 'C'};
    PDA<char, weight<std::array<double, 3>>> pda(labels);
    std::array<double, 3> w{0.5, 1.2, 0.3};
    pda.add_rule(0, 1, PUSH, 'B', 'A', w);
    pda.add_rule(0, 0, POP , '*', 'B', w);
    pda.add_rule(1, 3, SWAP, 'A', 'B', w);
    pda.add_rule(2, 0, SWAP, 'B', 'C', w);
    pda.add_rule(3, 2, PUSH, 'C', 'A', w);

    auto res = internal::Reducer::reduce(pda, 3, 3, 0);

    BOOST_CHECK_LT(res.second, res.first);
}
