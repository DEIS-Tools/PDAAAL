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


template <typename T>
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
#ifndef NDEBUG
        std::vector<T> intersection;
        std::set_intersection(_partitions[0].begin(), _partitions[0].end(),
                              _partitions[1].begin(), _partitions[1].end(),
                              std::back_inserter(intersection));
        assert(intersection.empty());
#endif
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
        std::array<std::vector<T>,2> already_in_partitions; // Not sorted.
        for (size_t other_i = 0; other_i < 2; ++other_i) {
            assert(!other._partitions[other_i].empty());
            assert(std::is_sorted(other._partitions[other_i].begin(), other._partitions[other_i].end()));
            already_in_partitions[other_i].reserve(other._partitions[other_i].size());
        }
        std::array<size_t,2> ok_partition_id{std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()};
        // Go through partitions and 1) check if they need to be split because of other._partitions, and
        // 2) record which elements of other._partitions are already in this _partitions, and where to put the rest.
        auto size = _partitions.size();
        for (size_t i = 0; i < size; ++i) {
            bool empty_intersection_found = false;
            std::array<size_t,2> prev_size;
            for (size_t other_i = 0; other_i < 2; ++other_i) {
                prev_size[other_i] = already_in_partitions[other_i].size();
                std::set_intersection(_partitions[i].begin(), _partitions[i].end(),
                                      other._partitions[other_i].begin(), other._partitions[other_i].end(),
                                      std::back_inserter(already_in_partitions[other_i])); // We exploit that _partitions are disjoint, so temp and std::set_union is not needed.
                if (already_in_partitions[other_i].size() == prev_size[other_i]) { // Empty intersection with this partition.
                    empty_intersection_found = true;
                    if (ok_partition_id[1-other_i] == std::numeric_limits<size_t>::max() // Are we still looking for an available partition?
                        && ok_partition_id[other_i] != i) { // Check that we did not already reserve this partition.
                        ok_partition_id[1-other_i] = i; // Empty intersection, so no violation in putting the rest of the opposite in here.
                    }
                }
            }
            if (!empty_intersection_found) {
                // Split _partitions[i]. Move the smallest intersection:
                size_t move_i = already_in_partitions[0].size() - prev_size[0] < already_in_partitions[1].size() - prev_size[1] ? 0 : 1;
                // Since we split, we have partitions available for the rest of other._partitions.
                for (size_t other_i = 0; other_i < 2; ++other_i) {
                    if (ok_partition_id[other_i] != std::numeric_limits<size_t>::max()) {
                        ok_partition_id[other_i] = move_i == other_i ? _partitions.size() : i;
                    }
                }
                // Make new partition.
                _partitions.emplace_back(already_in_partitions[move_i].begin() + prev_size[move_i], already_in_partitions[move_i].end());
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
            if (already_in_partitions[other_i].size() == other._partitions[other_i].size()) continue; // Nothing left to add.
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
                _partitions.back().reserve(other._partitions[other_i].size() - already_in_partitions[other_i].size());
                // Insert into the partition those from other._partition that is not in any current partition.
                std::set_difference(other._partitions[other_i].begin(), other._partitions[other_i].end(),
                                    already_in_partitions[other_i].begin(), already_in_partitions[other_i].end(),
                                    std::back_inserter(_partitions.back()));
            }
        }
#ifndef NDEBUG
        // Assert that all partitions are pairwise disjoint.
        for (size_t i = 0; i < _partitions.size(); ++i) {
            for (size_t j = i+1; j < _partitions.size(); ++j) {
                std::vector<T> intersection;
                std::set_intersection(_partitions[i].begin(), _partitions[i].end(),
                                      _partitions[j].begin(), _partitions[j].end(),
                                      std::back_inserter(intersection));
                assert(intersection.empty());
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

template <typename label_t>
class HeaderRefinement {
    std::vector<Refinement<label_t>> _refinements;
public:
    void combine(Refinement<label_t>&& refinement) {
        auto match = std::find_if(_refinements.begin(), _refinements.end(), [&refinement](const auto& r){ return r.abstract_id == refinement.abstract_id; });
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

template <typename A, typename B>
struct CompFirst {
    bool operator() ( const std::pair<A,B>& a, const std::pair<A,B>& b ) const { return a.first < b.first; }
    bool operator() ( const std::pair<A,B>& p, const A& a ) const { return p.first < a; }
    bool operator() ( const A& a, const std::pair<A,B>& p ) const { return a < p.first; }
};
template <typename A, typename B>
struct EqFirst {
    bool operator() ( const std::pair<A,B>& a, const std::pair<A,B>& b ) const { return a.first == b.first; }
    bool operator() ( const std::pair<A,B>& p, const A& a ) const { return p.first == a; }
    bool operator() ( const A& a, const std::pair<A,B>& p ) const { return a == p.first; }
};

template <typename A, typename B>
static inline std::vector<B> BmatchA(const std::vector<std::pair<A,B>> input, const A& a) {
    assert(std::is_sorted(input.begin(), input.end()));
    std::vector<B> result;
    auto [it,end] = std::equal_range(input.begin(), input.end(), a, CompFirst<A,B>{});
    for (; it != end; ++it) {
        if (result.empty() || result.back() != it->second) {
            result.emplace_back(it->second);
        }
    }
    return result;
}

template <typename A, typename B>
static inline std::vector<size_t> AmatchB_bucket(const std::vector<std::pair<A,B>> input, const B& b, const std::vector<std::pair<A,size_t>>& bucket_map) {
    assert(std::is_sorted(input.begin(), input.end()));
    assert(std::is_sorted(bucket_map.begin(), bucket_map.end(), CompFirst<A,size_t>{}));
    std::vector<A> set;
    // Since input is sorted by first element, we need to iterate through it all, but at least the result is still sorted.
    for (const auto& [a,bb] : input) {
        if (bb == b && (set.empty() || set.back() != a)) {
            set.emplace_back(a);
        }
    }
    std::vector<size_t> result;
    auto lb = bucket_map.begin();
    for (const auto& a : set) {
        // Since set is sorted, we can start searching from the previous lower bound...
        lb = std::lower_bound(lb, bucket_map.end(), a, CompFirst<A,size_t>{});
        assert(lb != bucket_map.end() && lb->first == a);
        result.emplace_back(lb->second);
        ++lb; // ... plus one, since elements are unique.
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

template <typename A, typename B>
static inline std::pair<Refinement<A>, Refinement<B>>
make_pair_refinement(std::vector<std::pair<A,B>>&& X, std::vector<std::pair<A,B>>&& Y, size_t A_id, size_t B_id) {
    assert(!X.empty());
    assert(!Y.empty());
    Refinement<A> a_partition(A_id);
    Refinement<B> b_partition(B_id);
    // Initialize first partitions
    a_partition.partitions().emplace_back();
    b_partition.partitions().emplace_back();

    std::sort(X.begin(), X.end()); std::sort(Y.begin(), Y.end());

    // Find A's and B's and end up with a sorted vector without duplicates.
    std::vector<std::pair<A,size_t>> As; // Map from elements of A to the partition it is in
    std::vector<B> Bs;
    for (const auto& [a,b] : X) {
        if (As.empty() || As.back().first != a) As.emplace_back(a, std::numeric_limits<size_t>::max());
        if (Bs.empty() || Bs.back() != b) Bs.emplace_back(b);
    }
    size_t ax = As.size(), bx = Bs.size();
    for (const auto& [a,b] : Y) {
        if (As.empty() || As.back().first != a) As.emplace_back(a, std::numeric_limits<size_t>::max());
        if (Bs.empty() || Bs.back() != b) Bs.emplace_back(b);
    }
    std::inplace_merge(As.begin(), As.begin() + ax, As.end(), CompFirst<A,size_t>{});
    std::inplace_merge(Bs.begin(), Bs.begin() + bx, Bs.end());
    As.erase(std::unique(As.begin(), As.end(), EqFirst<A,size_t>{}), As.end());
    Bs.erase(std::unique(Bs.begin(), Bs.end()), Bs.end());

    // First (greedily) find small partitioning for A that does not make B's partitioning impossible.
    for (auto& [a,a_bucket] : As) {
        bool placed_in_bucket = false;
        size_t bucket_i = 0;
        for (auto& bucket : a_partition.partitions()) {
            bool bucket_ok = true;
            std::vector<B> X_b = BmatchA(X,a), Y_b = BmatchA(Y,a);
            for (const auto& a_prime : bucket) {
                std::vector<B> temp, Z_X = BmatchA(X, a_prime);
                std::set_intersection(Z_X.begin(), Z_X.end(), Y_b.begin(), Y_b.end(), std::back_inserter(temp));
                if (!temp.empty()) { bucket_ok = false; break; }
                auto Z_Y = BmatchA(Y, a_prime);
                std::set_intersection(Z_Y.begin(), Z_Y.end(), X_b.begin(), X_b.end(), std::back_inserter(temp));
                if (!temp.empty()) { bucket_ok = false; break; }
            }
            if (bucket_ok) {
                bucket.emplace_back(a);
                a_bucket = bucket_i;
                placed_in_bucket = true;
                break;
            }
            ++bucket_i;
        }
        if (!placed_in_bucket) { // New bucket with a.
            a_partition.partitions().emplace_back();
            a_partition.partitions().back().emplace_back(a);
            a_bucket = bucket_i;
        }
    }

    // Find (greedily) a small partitioning of B respecting the partitioning of A.
    for (const auto& b : Bs) {
        bool placed_in_bucket = false;
        for (auto& bucket : b_partition.partitions()) {
            bool bucket_ok = true;
            auto X_a = AmatchB_bucket(X, b, As), Y_a = AmatchB_bucket(Y, b, As);
            for (const auto& b_prime : bucket) {
                std::vector<size_t> temp, Z_X = AmatchB_bucket(X, b_prime, As);
                std::set_intersection(Z_X.begin(), Z_X.end(), Y_a.begin(), Y_a.end(), std::back_inserter(temp));
                if (!temp.empty()) { bucket_ok = false; break; }
                auto Z_Y = AmatchB_bucket(Y, b_prime, As);
                std::set_intersection(Z_Y.begin(), Z_Y.end(), X_a.begin(), X_a.end(), std::back_inserter(temp));
                if (!temp.empty()) { bucket_ok = false; break; }
            }
            if (bucket_ok) {
                bucket.emplace_back(b);
                placed_in_bucket = true;
                break;
            }
        }
        if (!placed_in_bucket) { // New bucket with b.
            b_partition.partitions().emplace_back();
            b_partition.partitions().back().emplace_back(b);
        }
    }

    assert(a_partition.partitions().size() >= 2 || b_partition.partitions().size() >= 2);
    return std::make_pair(a_partition, b_partition);
}



template <typename A, typename B>
static inline std::pair<Refinement<A>, Refinement<B>>
make_simple_pair_refinement(std::vector<std::pair<A,B>>&& X, std::vector<std::pair<A,B>>&& Y, size_t A_id, size_t B_id) {
    assert(!X.empty());
    assert(!Y.empty());
    Refinement<A> a_partition(A_id);
    Refinement<B> b_partition(B_id);

    // Just use first pairs in X and Y.
    const auto& [Xa,Xb] = X[0];
    const auto& [Ya,Yb] = Y[0];
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
    assert(a_partition.partitions().size() >= 2 || b_partition.partitions().size() >= 2);
    return std::make_pair(a_partition, b_partition);
}

#endif //PDAAAL_REFINEMENT_H
