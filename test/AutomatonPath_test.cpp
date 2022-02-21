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
 * File:   AutomatonPath_test
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 18-02-2022.
 */

#define BOOST_TEST_MODULE AutomatonPath_test

#include <pdaaal/AutomatonPath.h>
#include <boost/test/unit_test.hpp>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(AutomatonPath_Test_1)
{
    AutomatonPath path(5);
    BOOST_CHECK(path.empty());
    path.emplace(3,42);
    BOOST_CHECK(!path.empty());
    path.emplace(1,100);
    BOOST_CHECK(!path.empty());
    auto [from, label, to] = path.front_edge();
    BOOST_CHECK_EQUAL(from, 1);
    BOOST_CHECK_EQUAL(label, 100);
    BOOST_CHECK_EQUAL(to, 3);
    path.pop();
    BOOST_CHECK(!path.empty());
    auto [from2, label2, to2] = path.front_edge();
    BOOST_CHECK_EQUAL(from2, 3);
    BOOST_CHECK_EQUAL(label2, 42);
    BOOST_CHECK_EQUAL(to2, 5);
    path.pop();
    BOOST_CHECK(path.empty());
    BOOST_CHECK(!path.is_null());
}