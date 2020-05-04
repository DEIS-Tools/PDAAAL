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
 * File:   WPDA_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 19-02-2020.
 */

#define BOOST_TEST_MODULE WPDA

#include <boost/test/unit_test.hpp>
#include <pdaaal/PDA.h>
#include <pdaaal/TypedPDA.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(VoidWeight)
{
    std::unordered_set<char> labels{'a', 'b'};
    TypedPDA<char> pda(labels); // Check if it compiles
    BOOST_CHECK_EQUAL(true, true);
}

BOOST_AUTO_TEST_CASE(IntWeight)
{
    std::unordered_set<char> labels{'a', 'b'};
    TypedPDA<char, int> pda(labels); // Check if it compiles
    BOOST_CHECK_EQUAL(true, true);
}

BOOST_AUTO_TEST_CASE(LabelsMergeNegated)
{
    labels_t labels;
    std::vector<uint32_t> init_labels{1,5,6,9,11,15};
    size_t all_labels = 16;

    labels.merge(false, init_labels, all_labels);

    BOOST_CHECK_EQUAL_COLLECTIONS(labels.labels().begin(), labels.labels().end(), init_labels.begin(), init_labels.end());

    std::vector<uint32_t> neg_labels{2,5,7,9,12,13};
    labels.merge(true, neg_labels, all_labels);

    std::vector<uint32_t> res_labels{0,1,3,4,5,6,8,9,10,11,14,15};
    BOOST_CHECK_EQUAL_COLLECTIONS(labels.labels().begin(), labels.labels().end(), res_labels.begin(), res_labels.end());
}
BOOST_AUTO_TEST_CASE(LabelsMerge)
{
    labels_t labels;
    std::vector<uint32_t> init_labels{1,5,6,9,11,15};
    size_t all_labels = 20;

    labels.merge(false, init_labels, all_labels);

    BOOST_CHECK_EQUAL_COLLECTIONS(labels.labels().begin(), labels.labels().end(), init_labels.begin(), init_labels.end());

    std::vector<uint32_t> neg_labels{2,5,7,9,12,13};
    labels.merge(false, neg_labels, all_labels);

    std::vector<uint32_t> res_labels{1,2,5,6,7,9,11,12,13,15};
    BOOST_CHECK_EQUAL_COLLECTIONS(labels.labels().begin(), labels.labels().end(), res_labels.begin(), res_labels.end());
}

BOOST_AUTO_TEST_CASE(PDA_Container_Type) {
    std::unordered_set<char> labels{'A', 'B'};
    TypedPDA<char,int,std::less<int>,fut::type::hash> pda(labels);
    pda.add_rule(0, 1, PUSH, 'B', false, 'A');

    TypedPDA<char,int> pda2(std::move(pda));

    pda2.add_rule(1, 3, SWAP, 'A', false, 'B');

    BOOST_CHECK_EQUAL(true, true);
}
