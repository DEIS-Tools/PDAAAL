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
static inline std::pair<Refinement<A>, Refinement<B>>
make_pair_refinement(const std::vector<std::pair<A,B>>& X, const std::vector<std::pair<A,B>>& Y, const std::vector<A>& X_wildcard, size_t A_id, size_t B_id) {
    Refinement<A> a_partition(A_id);
    Refinement<B> b_partition(B_id);
    // TODO: Implement!!

    // TODO: Assign X_wildcard to a different bucket than all As in Y.

    return std::make_pair(a_partition, b_partition);
}

#endif //PDAAAL_REFINEMENT_H
