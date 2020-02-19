//
// Created by Morten on 19-02-2020.
//
#define BOOST_TEST_MODULE WPDA

#include <boost/test/unit_test.hpp>
#include <pdaaal/WPDA.h>
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
