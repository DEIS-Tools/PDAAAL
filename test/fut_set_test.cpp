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
 * File:   fut_set_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-04-2020.
 */

#define BOOST_TEST_MODULE fut_set

#include <pdaaal/utils/fut_set.h>
#include <boost/test/unit_test.hpp>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(Test_fut_set_hash_vecter_map)
{
    fut::set<std::tuple<size_t, size_t, uint32_t>, fut::type::hash, fut::type::vector> set;

    auto res = set.emplace(4,7,10);
    BOOST_CHECK_EQUAL(res.second, true);

    BOOST_CHECK_EQUAL(set.contains(4,6), false);

    res = set.emplace(4,6,10);
    BOOST_CHECK_EQUAL(res.second, true);

    res = set.emplace(4,7,10);
    BOOST_CHECK_EQUAL(res.second, false);

    BOOST_CHECK_EQUAL(set.contains(4,7), true);

}

BOOST_AUTO_TEST_CASE(Test_fut_set_vector_hash_map)
{
    fut::set<std::tuple<size_t, size_t, uint32_t>, fut::type::vector, fut::type::hash> set;

    auto res = set.emplace(4,7,10);
    BOOST_CHECK_EQUAL(res.second, true);

    BOOST_CHECK_EQUAL(set.contains(4,6), false);

    res = set.emplace(4,6,10);
    BOOST_CHECK_EQUAL(res.second, true);

    res = set.emplace(4,7,10);
    BOOST_CHECK_EQUAL(res.second, false);

    BOOST_CHECK_EQUAL(set.contains(4,7), true);
}

BOOST_AUTO_TEST_CASE(Test_fut_set_hash_vector_set)
{
    fut::set<std::tuple<size_t, int>, fut::type::hash, fut::type::vector> set;

    auto res = set.emplace(4,7);
    BOOST_CHECK_EQUAL(res.second, true);

    BOOST_CHECK_EQUAL(set.contains(4,6), false);

    res = set.emplace(4,6);
    BOOST_CHECK_EQUAL(res.second, true);

    res = set.emplace(4,7);
    BOOST_CHECK_EQUAL(res.second, false);

    BOOST_CHECK_EQUAL(set.contains(4,7), true);
}

BOOST_AUTO_TEST_CASE(Test_fut_set_vector_hash_set)
{
    fut::set<std::tuple<size_t, size_t>, fut::type::vector, fut::type::hash> set;

    auto res = set.emplace(4,7);
    BOOST_CHECK_EQUAL(res.second, true);

    BOOST_CHECK_EQUAL(set.contains(4,6), false);

    res = set.emplace(4,6);
    BOOST_CHECK_EQUAL(res.second, true);

    res = set.emplace(4,7);
    BOOST_CHECK_EQUAL(res.second, false);

    BOOST_CHECK_EQUAL(set.contains(4,7), true);
}

BOOST_AUTO_TEST_CASE(Test_fut_set_hash_vecter_tuplemap)
{
    fut::set<std::tuple<size_t, std::string, uint32_t, std::vector<char>>, fut::type::hash, fut::type::vector> set;

    auto res = set.emplace(4,"1",10, std::vector<char>{'a', 'b'});
    BOOST_TEST(res.second);
    BOOST_CHECK_EQUAL(std::get<0>(res.first->second), 10);
    std::vector<char> v1{'a', 'b'};
    BOOST_CHECK_EQUAL_COLLECTIONS(std::get<1>(res.first->second).begin(), std::get<1>(res.first->second).end(), v1.begin(), v1.end());

    std::vector<char> v2{'c', 'd'};
    res = set.emplace(4,"678",10, v2);
    BOOST_TEST(res.second);
    BOOST_CHECK_EQUAL(std::get<0>(res.first->second), 10);
    BOOST_CHECK_EQUAL_COLLECTIONS(std::get<1>(res.first->second).begin(), std::get<1>(res.first->second).end(), v2.begin(), v2.end());

    res = set.emplace(4,"1",10, std::vector<char>{'a', 'b'});
    BOOST_TEST(!res.second);
    BOOST_CHECK_EQUAL(std::get<0>(res.first->second), 10);
    BOOST_CHECK_EQUAL_COLLECTIONS(std::get<1>(res.first->second).begin(), std::get<1>(res.first->second).end(), v1.begin(), v1.end());
}

BOOST_AUTO_TEST_CASE(Test_fut_set_hash_vecter_vector_set)
{
    fut::set<std::tuple<size_t, size_t, uint32_t>, fut::type::hash, fut::type::vector, fut::type::vector> set;

    uint32_t i = 10;
    auto res = set.emplace(4,7,i);
    BOOST_CHECK_EQUAL(res.second, true);

    BOOST_CHECK_EQUAL(set.contains(4,6,i), false);

    res = set.emplace(4,6,i);
    BOOST_CHECK_EQUAL(res.second, true);

    res = set.emplace(4,7,i);
    BOOST_CHECK_EQUAL(res.second, false);

    BOOST_CHECK_EQUAL(set.contains(4,7,i), true);
}