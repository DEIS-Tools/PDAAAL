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
 * File:   Weight_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 13-03-2020.
 */

#define BOOST_TEST_MODULE Weight

#include <pdaaal/Weight.h>
#include <boost/test/unit_test.hpp>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(VectorWeight) {
    std::vector<int> a{1,7,42};
    std::vector<int> b{3,1};
    std::vector<int> c{3,1,9};
    std::vector<int> d{3,0,9};
    std::vector<int> e;
    using W = min_weight<std::vector<int>>;
    auto result_1 = W::add(a,b);
    std::vector<int> expected_1{4,8,42};
    BOOST_CHECK_EQUAL_COLLECTIONS(result_1.begin(), result_1.end(), expected_1.begin(), expected_1.end());

    auto result_2 = W::add(e,a);
    auto expected_2 = a;
    BOOST_CHECK_EQUAL_COLLECTIONS(result_2.begin(), result_2.end(), expected_2.begin(), expected_2.end());

    BOOST_CHECK_EQUAL(W::less(a,b), true);
    BOOST_CHECK_EQUAL(W::less(b,a), false);
    BOOST_CHECK_EQUAL(W::less(b,c), true);
    BOOST_CHECK_EQUAL(W::less(c,d), false);
    BOOST_CHECK_EQUAL(W::less(d,a), false);
    BOOST_CHECK_EQUAL(W::less(e,a), true);
    BOOST_CHECK_EQUAL(W::less(a,a), false);
}


BOOST_AUTO_TEST_CASE(WeightFunctionCombinators) {
    // Setup some evaluation functions.
    linear_weight_function a(std::function([](const std::string& s, size_t i) -> long int {
        return s.size() - i;
    }));
    std::vector<int> v{3,5,0,1,7};
    linear_weight_function b(std::function([&v](const std::string& s, size_t i) -> long int {
        if (i < v.size() && i > 0) {
            return s.size() * v[i];
        } else {
            return s.size() / i;
        }
    }));

    std::vector<std::pair<long int, linear_weight_function<long int, const std::string &, size_t>>> fs;
    fs.emplace_back(2,a);
    fs.emplace_back(4,b);
    linear_weight_function c(fs);
    BOOST_CHECK_EQUAL(c("Hello", 3), (5-3)*2+5*1*4);

    std::vector<linear_weight_function<long int, const std::string &, size_t>> ls{a,b,c};
    ordered_weight_function d(ls);
    auto result = d("Hello", 3);
    std::vector<long int> expected{5-3, 5*1, (5-3)*2+5*1*4};
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), expected.begin(), expected.end());
}