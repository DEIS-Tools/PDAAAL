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
 * File:   Refinement.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 19-01-2021.
 */

#ifndef PDAAAL_REFINEMENT_H
#define PDAAAL_REFINEMENT_H

#include <vector>
#include <algorithm>
#include <cassert>

namespace pdaaal {

    // Auxiliary function... Why is something like this not in <algorithm>??.
    template<typename T>
    inline bool is_disjoint(const std::vector<T>& a, const std::vector<T>& b) {
        // Inspired by: https://stackoverflow.com/a/29123390
        assert(std::is_sorted(a.begin(), a.end()));
        assert(std::is_sorted(b.begin(), b.end()));
        auto it_a = a.begin();
        auto it_b = b.begin();
        while (it_a != a.end() && it_b != b.end()) {
            if (*it_a < *it_b) {
                it_a = std::lower_bound(++it_a, a.end(), *it_b);
            } else if (*it_b < *it_a) {
                it_b = std::lower_bound(++it_b, b.end(), *it_a);
            } else {
                return false;
            }
        }
        return true;
    }
    template<typename T, typename U, typename Compare>
    inline bool is_disjoint(const std::vector<T>& a, const std::vector<U>& b, Compare comp) {
        // Inspired by: https://stackoverflow.com/a/29123390
        assert(std::is_sorted(a.begin(), a.end(), comp));
        assert(std::is_sorted(b.begin(), b.end(), comp));
        auto it_a = a.begin();
        auto it_b = b.begin();
        while (it_a != a.end() && it_b != b.end()) {
            if (comp(*it_a,*it_b)) {
                it_a = std::lower_bound(++it_a, a.end(), *it_b, comp);
            } else if (comp(*it_b, *it_a)) {
                it_b = std::lower_bound(++it_b, b.end(), *it_a, comp);
            } else {
                return false;
            }
        }
        return true;
    }

    template<typename T>
    class Refinement {
        std::vector<std::vector<T>> _partitions;
    public:
        const size_t abstract_id;

        explicit Refinement(size_t abstract_id) : abstract_id(abstract_id) {};

        Refinement(std::vector<T>&& first, std::vector<T>&& second, size_t abstract_id)
                : _partitions{std::move(first), std::move(second)}, abstract_id(abstract_id) {
            assert(_partitions.size() == 2);
            assert(!_partitions[0].empty());
            assert(!_partitions[1].empty());
            std::sort(_partitions[0].begin(), _partitions[0].end());
            std::sort(_partitions[1].begin(), _partitions[1].end());
            assert(is_disjoint(_partitions[0], _partitions[1]));
        };

        void combine(Refinement<T>&& other) {
            // TODO: This algorithm incrementally combines partitions.
            //  It might be possible for a clever algorithm to use fewer partitions given all (uncombined) sets of partitions at once.
            //  Investigate this...

            // TODO: Better move semantics...

            assert(abstract_id == other.abstract_id);

            if (_partitions.empty()) { // This should not actually happen...
                _partitions = std::move(other._partitions);
                return;
            }
            assert(_partitions.size() >= 2);
            assert(other._partitions.size() == 2); // This is the only use case, and it makes things a bit simpler.
            std::array<std::vector<T>, 2> already_in_partitions; // Not sorted.
            for (size_t other_i = 0; other_i < 2; ++other_i) {
                assert(!other._partitions[other_i].empty());
                assert(std::is_sorted(other._partitions[other_i].begin(), other._partitions[other_i].end()));
                already_in_partitions[other_i].reserve(other._partitions[other_i].size());
            }
            std::array<size_t, 2> ok_partition_id{std::numeric_limits<size_t>::max(),
                                                  std::numeric_limits<size_t>::max()};
            // Go through partitions and 1) check if they need to be split because of other._partitions, and
            // 2) record which elements of other._partitions are already in this _partitions, and where to put the rest.
            auto size = _partitions.size();
            for (size_t i = 0; i < size; ++i) {
                bool empty_intersection_found = false;
                std::array<size_t, 2> prev_size;
                for (size_t other_i = 0; other_i < 2; ++other_i) {
                    prev_size[other_i] = already_in_partitions[other_i].size();
                    std::set_intersection(_partitions[i].begin(), _partitions[i].end(),
                                          other._partitions[other_i].begin(), other._partitions[other_i].end(),
                                          std::back_inserter(already_in_partitions[other_i])); // We exploit that _partitions are disjoint, so temp and std::set_union is not needed.
                    if (already_in_partitions[other_i].size() == prev_size[other_i]) { // Empty intersection with this partition.
                        empty_intersection_found = true;
                        if (ok_partition_id[1 - other_i] ==
                            std::numeric_limits<size_t>::max() // Are we still looking for an available partition?
                            && ok_partition_id[other_i] != i) { // Check that we did not already reserve this partition.
                            ok_partition_id[1 -
                                            other_i] = i; // Empty intersection, so no violation in putting the rest of the opposite in here.
                        }
                    }
                }
                if (!empty_intersection_found) {
                    // Split _partitions[i]. Move the smallest intersection:
                    size_t move_i = already_in_partitions[0].size() - prev_size[0] <
                                    already_in_partitions[1].size() - prev_size[1] ? 0 : 1;
                    // Since we split, we have partitions available for the rest of other._partitions.
                    for (size_t other_i = 0; other_i < 2; ++other_i) {
                        if (ok_partition_id[other_i] != std::numeric_limits<size_t>::max()) {
                            ok_partition_id[other_i] = move_i == other_i ? _partitions.size() : i;
                        }
                    }
                    // Make new partition.
                    _partitions.emplace_back(already_in_partitions[move_i].begin() + prev_size[move_i],
                                             already_in_partitions[move_i].end());
                    // Remove elements from old partition.
                    std::vector<T> temp;
                    temp.reserve(_partitions[i].size() - _partitions.back().size());
                    std::set_difference(_partitions[i].begin(), _partitions[i].end(),
                                        _partitions.back().begin(), _partitions.back().end(),
                                        std::back_inserter(temp));
                    std::swap(_partitions[i], temp);
                }
            }
            // Insert parts of other._partitions that was not already in this _partitions
            for (size_t other_i = 0; other_i < 2; ++other_i) {
                if (already_in_partitions[other_i].size() == other._partitions[other_i].size())
                    continue; // Nothing left to add.
                std::sort(already_in_partitions[other_i].begin(), already_in_partitions[other_i].end());
                if (ok_partition_id[other_i] != std::numeric_limits<size_t>::max()) {
                    auto& partition = _partitions[ok_partition_id[other_i]];
                    auto old_size = partition.size();
                    partition.reserve(
                            old_size + other._partitions[other_i].size() - already_in_partitions[other_i].size());
                    // Insert into the partition those from other._partition that is not in any current partition.
                    std::set_difference(other._partitions[other_i].begin(), other._partitions[other_i].end(),
                                        already_in_partitions[other_i].begin(), already_in_partitions[other_i].end(),
                                        std::back_inserter(partition));
                    // Then merge inplace the two sorted ranges. We know they are disjoint, so this is essentially a set_union operation.
                    std::inplace_merge(partition.begin(), partition.begin() + old_size, partition.end());
                } else {
                    // The rest does not fit into any existing partition, so make a new one for it.
                    _partitions.emplace_back();
                    _partitions.back().reserve(
                            other._partitions[other_i].size() - already_in_partitions[other_i].size());
                    // Insert into the partition those from other._partition that is not in any current partition.
                    std::set_difference(other._partitions[other_i].begin(), other._partitions[other_i].end(),
                                        already_in_partitions[other_i].begin(), already_in_partitions[other_i].end(),
                                        std::back_inserter(_partitions.back()));
                }
            }
#ifndef NDEBUG
            // Assert that all partitions are pairwise disjoint.
            for (size_t i = 0; i < _partitions.size(); ++i) {
                for (size_t j = i + 1; j < _partitions.size(); ++j) {
                    assert(is_disjoint(_partitions[i], _partitions[j]));
                }
            }
#endif
        }

        [[nodiscard]] bool empty() const {
            return _partitions.empty();
        }

        std::vector<std::vector<T>>& partitions() {
            return _partitions;
        }

        const std::vector<std::vector<T>>& partitions() const {
            return _partitions;
        }
    };

    template<typename label_t>
    class HeaderRefinement {
        std::vector<Refinement<label_t>> _refinements;
    public:
        void combine(Refinement<label_t>&& refinement) {
            auto match = std::find_if(_refinements.begin(), _refinements.end(),
                                      [&refinement](const auto& r) { return r.abstract_id == refinement.abstract_id; });
            if (match != _refinements.end()) {
                match->combine(std::move(refinement));
            } else {
                _refinements.emplace_back(std::move(refinement));
            }
        }

        [[nodiscard]] bool empty() const {
            return _refinements.empty();
        }

        const std::vector<Refinement<label_t>>& refinements() const {
            return _refinements;
        }
    };

    template<typename A, typename B>
    struct CompFirst {
        bool operator()(const std::pair<A, B>& lhs, const std::pair<A, B>& rhs) const { return lhs.first < rhs.first; }
        bool operator()(const std::pair<A, B>& lhs, const A& rhs) const { return lhs.first < rhs; }
        bool operator()(const A& lhs, const std::pair<A, B>& rhs) const { return lhs < rhs.first; }
        bool operator()(const A& lhs, const A& rhs) const { return lhs < rhs; }
    };

    template<typename A, typename B>
    struct EqFirst {
        bool operator()(const std::pair<A, B>& lhs, const std::pair<A, B>& rhs) const { return lhs.first == rhs.first; }
        bool operator()(const std::pair<A, B>& lhs, const A& rhs) const { return lhs.first == rhs; }
        bool operator()(const A& lhs, const std::pair<A, B>& rhs) const { return lhs == rhs.first; }
        bool operator()(const A& lhs, const A& rhs) const { return lhs == rhs; }
    };

    template<typename A, typename B>
    static inline std::vector<B> BmatchA(const std::vector<std::pair<A, B>>& input, const A& a) {
        assert(std::is_sorted(input.begin(), input.end()));
        std::vector<B> result;
        auto[it, end] = std::equal_range(input.begin(), input.end(), a, CompFirst<A, B>{});
        for (; it != end; ++it) {
            if (result.empty() || result.back() != it->second) {
                result.emplace_back(it->second);
            }
        }
        return result;
    }

    template<typename A, typename B>
    static inline std::vector<size_t> AmatchB_bucket(const std::vector<std::pair<A, B>>& input, const B& b,
                                                     const std::vector<std::pair<A, size_t>>& bucket_map) {
        assert(std::is_sorted(input.begin(), input.end()));
        assert(std::is_sorted(bucket_map.begin(), bucket_map.end(), CompFirst<A, size_t>{}));
        std::vector<A> set;
        // Since input is sorted by first element, we need to iterate through it all, but at least the result is still sorted.
        for (const auto&[a, bb] : input) {
            if (bb == b && (set.empty() || set.back() != a)) {
                set.emplace_back(a);
            }
        }
        std::vector<size_t> result;
        auto lb = bucket_map.begin();
        for (const auto& a : set) {
            // Since set is sorted, we can start searching from the previous lower bound...
            lb = std::lower_bound(lb, bucket_map.end(), a, CompFirst<A, size_t>{});
            assert(lb != bucket_map.end() && lb->first == a);
            result.emplace_back(lb->second);
            ++lb; // ... plus one, since elements are unique.
        }
        std::sort(result.begin(), result.end());
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

    template<typename T, typename U>
    static inline size_t
    assign_to_bucket(Refinement<T>& refinement, std::vector<std::vector<U>>& Z_X, std::vector<std::vector<U>>& Z_Y,
                     std::vector<U>&& Xs, std::vector<U>&& Ys, const T& elem) {
        bool placed_in_bucket = false;
        size_t bucket_i = 0;
        for (auto& bucket : refinement.partitions()) {
            assert(bucket_i < Z_X.size() && bucket_i < Z_Y.size());
            std::vector<U> intersection;
            if (is_disjoint(Z_Y[bucket_i], Xs) && is_disjoint(Z_X[bucket_i], Ys)) {
                // Assign elem to bucket
                bucket.emplace_back(elem);
                placed_in_bucket = true;
                // Update Z_X and Z_Y for this bucket with Xs and Ys.
                std::vector<U> temp_union;
                temp_union.reserve(Z_X[bucket_i].size() + Xs.size());
                std::set_union(Z_X[bucket_i].begin(), Z_X[bucket_i].end(), Xs.begin(), Xs.end(),
                               std::back_inserter(temp_union));
                std::swap(temp_union, Z_X[bucket_i]);
                temp_union.clear();
                temp_union.reserve(Z_Y[bucket_i].size() + Ys.size());
                std::set_union(Z_Y[bucket_i].begin(), Z_Y[bucket_i].end(), Ys.begin(), Ys.end(),
                               std::back_inserter(temp_union));
                std::swap(temp_union, Z_Y[bucket_i]);
                break;
            }
            ++bucket_i;
        }
        if (!placed_in_bucket) { // New bucket with elem.
            refinement.partitions().emplace_back();
            refinement.partitions().back().emplace_back(elem);
            // Initialize Z_X and Z_Y for the new bucket.
            Z_X.emplace_back(std::move(Xs));
            Z_Y.emplace_back(std::move(Ys));
        }
        return bucket_i;
    }


    template<typename A, typename B>
    static inline std::pair<Refinement<A>, Refinement<B>>
    make_pair_refinement(std::vector<std::pair<A, B>>&& X, std::vector<std::pair<A, B>>&& Y, std::vector<A>&& Y_wildcard, size_t A_id, size_t B_id) {
        assert(!X.empty());
        assert(!Y.empty() || !Y_wildcard.empty());
        Refinement<A> a_partition(A_id);
        Refinement<B> b_partition(B_id);
        // Initialize first partitions
        a_partition.partitions().emplace_back();
        b_partition.partitions().emplace_back();

        std::sort(X.begin(), X.end());
        std::sort(Y.begin(), Y.end());
        assert(std::is_sorted(Y_wildcard.begin(), Y_wildcard.end()));
        assert(is_disjoint(Y, Y_wildcard, CompFirst<A,B>{}));
        assert(is_disjoint(X, Y_wildcard, CompFirst<A,B>{}));

        // Find the sets of As and Bs occuring in X and Y.
        // We can easily make As sorted and avoid duplicates, and we use that it is sorted in AmatchB_bucket.
        // Bs is only iterated through once, so an unordered_set is suitable here.
        std::vector<std::pair<A, size_t>> As; // Map from elements of A to the partition it is in
        std::unordered_set<B> Bs;
        for (const auto&[a, b] : X) {
            if (As.empty() || As.back().first != a) As.emplace_back(a, std::numeric_limits<size_t>::max());
            Bs.emplace(b);
        }
        size_t ax = As.size();
        for (const auto&[a, b] : Y) {
            if (As.back().first != a) As.emplace_back(a, std::numeric_limits<size_t>::max());
            Bs.emplace(b);
        }
        std::inplace_merge(As.begin(), As.begin() + ax, As.end(), CompFirst<A, size_t>{});
        As.erase(std::unique(As.begin(), As.end(), EqFirst<A, size_t>{}), As.end());

        {
            std::vector<std::vector<B>> Z_X, Z_Y; // Build up Z_X per bucket when adding to that bucket.
            Z_X.emplace_back();
            Z_Y.emplace_back(); // Initialize for first (empty) bucket.
            // First (greedily) find small partitioning for A that does not make B's partitioning impossible.
            for (auto&[a, a_bucket] : As) {
                a_bucket = assign_to_bucket(a_partition, Z_X, Z_Y, BmatchA(X, a), BmatchA(Y, a), a);
            }
        }
        {
            std::vector<std::vector<size_t>> Z_X, Z_Y;
            Z_X.emplace_back();
            Z_Y.emplace_back(); // Initialize for first (empty) bucket.
            // Find (greedily) a small partitioning of B respecting the partitioning of A.
            for (const auto& b : Bs) {
                assign_to_bucket(b_partition, Z_X, Z_Y, AmatchB_bucket(X, b, As), AmatchB_bucket(Y, b, As), b);
            }
        }

        if (!Y_wildcard.empty()) { // Wildcards is just put in its own bucket. Might not always be minimal, but it is good enough.
            a_partition.partitions().emplace_back();
            for (const auto& a : Y_wildcard) {
                a_partition.partitions().back().emplace_back(a);
            }
        }

        assert(a_partition.partitions().size() >= 2 || b_partition.partitions().size() >= 2);
        return std::make_pair(a_partition, b_partition);
    }


    template<typename A, typename B>
    static inline std::pair<Refinement<A>, Refinement<B>>
    make_simple_pair_refinement(std::vector<std::pair<A, B>>&& X, std::vector<std::pair<A, B>>&& Y, std::vector<A>&& Y_wildcard,
                                size_t A_id, size_t B_id) {
        assert(!X.empty());
        assert(!Y.empty() || !Y_wildcard.empty());
        Refinement<A> a_partition(A_id);
        Refinement<B> b_partition(B_id);

        // Just use first pairs in X and Y.
        const auto&[Xa, Xb] = X[0];
        if (!Y_wildcard.empty()) {
            const auto& Ya = Y_wildcard[0];
            assert(Xa != Ya);
            a_partition.partitions().emplace_back();
            a_partition.partitions().back().emplace_back(Xa);
            a_partition.partitions().emplace_back();
            a_partition.partitions().back().emplace_back(Ya);
        } else {
            assert(!Y.empty());
            const auto&[Ya, Yb] = Y[0];
            if (Xa != Ya) {
                a_partition.partitions().emplace_back();
                a_partition.partitions().back().emplace_back(Xa);
                a_partition.partitions().emplace_back();
                a_partition.partitions().back().emplace_back(Ya);
            }
            if (Xb != Yb) {
                b_partition.partitions().emplace_back();
                b_partition.partitions().back().emplace_back(Xb);
                b_partition.partitions().emplace_back();
                b_partition.partitions().back().emplace_back(Yb);
            }
        }
        assert(a_partition.partitions().size() >= 2 || b_partition.partitions().size() >= 2);
        return std::make_pair(a_partition, b_partition);
    }

    enum class refinement_option_t {
        fast_refinement, best_refinement
    };

    template<refinement_option_t refinement_option, typename A, typename B>
    static inline std::pair<Refinement<A>, Refinement<B>>
    make_refinement(std::vector<std::pair<A, B>>&& X, std::vector<std::pair<A, B>>&& Y, size_t A_id, size_t B_id, std::vector<A>&& Y_wildcard = std::vector<A>()) {
        if constexpr (refinement_option == refinement_option_t::fast_refinement) {
            return make_simple_pair_refinement(std::move(X), std::move(Y), std::move(Y_wildcard), A_id, B_id);
        } else if constexpr (refinement_option == refinement_option_t::best_refinement) {
            return make_pair_refinement(std::move(X), std::move(Y), std::move(Y_wildcard), A_id, B_id);
        } else {
            assert(false);
            //static_assert(false, "Invalid refinement option.");
        }
    }

}

#endif //PDAAAL_REFINEMENT_H
