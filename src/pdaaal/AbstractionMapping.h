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
 * File:   AbstractionMapping.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 03-12-2020.
 */

#ifndef PDAAAL_ABSTRACTIONMAPPING_H
#define PDAAAL_ABSTRACTIONMAPPING_H

#include "ptrie_interface.h"
#include "Refinement.h"
#include <iterator>
#include <memory>
#include <vector>


namespace pdaaal {
    using namespace pdaaal::utils;

    template <typename ConcreteType, typename AbstractType> class AbstractionMapping;

    // This mapping can be build from an AbstractionMapping.
    // A RefinementMapping does not allow adding new elements, only refining the existing mapping.
    template <typename ConcreteType>
    class RefinementMapping {
    public:
        RefinementMapping() = default;
        template <typename AbstractType>
        explicit RefinementMapping(AbstractionMapping<ConcreteType,AbstractType>&& abstraction_mapping)
        : _many_to_one_map(std::move(abstraction_mapping._many_to_one_map)),
          _one_to_many_ids(std::move(abstraction_mapping._one_to_many_ids)) {
              assert(std::all_of(_one_to_many_ids.begin(), _one_to_many_ids.end(), [](const auto& x){ return !x.empty(); }));
          }

        void refine(const Refinement<ConcreteType>& refinement) {
            if (refinement.partitions().size() <= 1) return; // We might have empty refinement (if other component of a pair is refined), and only one partition is no refinement.
            // We don't move the largest partition
            auto max_partition = std::max_element(refinement.partitions().begin(), refinement.partitions().end(),
                                                  [](const auto& a, const auto& b){ return a.size() < b.size(); });
            std::vector<size_t> moved_values;
            for (auto partition = refinement.partitions().begin(); partition != refinement.partitions().end(); ++partition) {
                assert(!partition->empty());
                assert(std::all_of(partition->begin(), partition->end(), [this, id=refinement.abstract_id](const auto& x){ auto [found, xid] = exists(x); return found && xid == id; }));
                if (partition == max_partition) continue;
                // Create a new 'abstract' id for each new partition
                auto new_id = _one_to_many_ids.size();
                _one_to_many_ids.emplace_back();
                // Change the id of each concrete value
                for (const auto& concrete_value : *partition) {
                    auto [found, key_id] = _many_to_one_map.exists(concrete_value);
                    _many_to_one_map.get_data(key_id) = new_id;
                    _one_to_many_ids[new_id].emplace_back(key_id);
                    moved_values.emplace_back(key_id);
                }
            }
            // Remove all moved key ids from the original partition.
            // Sorting and set_difference is maybe a bit overkill... Depends on the sizes...
            std::sort(moved_values.begin(), moved_values.end());
            std::sort(_one_to_many_ids[refinement.abstract_id].begin(), _one_to_many_ids[refinement.abstract_id].end());
            std::vector<size_t> temp;
            std::set_difference(_one_to_many_ids[refinement.abstract_id].begin(), _one_to_many_ids[refinement.abstract_id].end(),
                                moved_values.begin(), moved_values.end(),
                                std::back_inserter(temp));
            std::swap(_one_to_many_ids[refinement.abstract_id], temp);
        }

        std::pair<bool,size_t> exists(const ConcreteType& key) const {
            auto [exists, key_id] = _many_to_one_map.exists(key);
            return {exists, exists ? _many_to_one_map.get_data(key_id) : key_id};
        }

        bool maps_to(const ConcreteType& key, size_t id) const {
            auto [exist, res_id] = exists(key);
            return exist && res_id == id;
        }

        std::vector<size_t> encode_many(const std::vector<ConcreteType>& concrete_values) const {
            std::unordered_set<size_t> res_set;
            for (const auto& concrete_value : concrete_values) {
                auto [exists, id] = _many_to_one_map.exists(concrete_value);
                assert(exists);
                res_set.emplace(_many_to_one_map.get_data(id));
            }
            std::vector<size_t> result(res_set.begin(), res_set.end());
            std::sort(result.begin(), result.end());
            return result;
        }

        // Construct vector
        std::vector<ConcreteType> get_concrete_values(size_t abstract_value) const {
            std::vector<ConcreteType> result;
            if (abstract_value < _one_to_many_ids.size()) {
                result.reserve(_one_to_many_ids[abstract_value].size());
                for (const auto& id : _one_to_many_ids[abstract_value]) {
                    result.emplace_back(_many_to_one_map.at(id));
                }
            }
            return result;
        }

        // TODO: Determine design and relevance later...
        // TODO: Use C++20 ranges::view when available.
        // Return range structure with begin and end defined.
        struct concrete_value_range {
            explicit concrete_value_range(const ptrie_map<ConcreteType, size_t>* map, const std::vector<size_t>* range = nullptr)
                    : _map(map), _range(range) { };
            using iterator = ptrie_access_iterator<ConcreteType>;
            iterator begin() const noexcept {
                return _range != nullptr ? iterator(_range->begin(), _map) : iterator(_map);
            }
            iterator end() const noexcept {
                return _range != nullptr ? iterator(_range->end(), _map) : iterator(_map);
            }
        private:
            const ptrie_map<ConcreteType, size_t>* _map;
            const std::vector<size_t>* _range;
        };

        concrete_value_range get_concrete_values_range(size_t abstract_value) const {
            if (abstract_value < _one_to_many_ids.size()) {
                return concrete_value_range(&_many_to_one_map, &_one_to_many_ids[abstract_value]);
            }
            return concrete_value_range(&_many_to_one_map);
        }

        [[nodiscard]] size_t size() const {
            return _one_to_many_ids.size();
        }

    protected:
        ptrie_map<ConcreteType, size_t> _many_to_one_map;
        std::vector<std::vector<size_t>> _one_to_many_ids;
    };


    // Given a mapping function from ConcreteType to AbstractType,
    // the AbstractionMapping gives a consecutive id to each distinct (by byte representation) AbstractType value,
    // and provides efficient concrete to abstract (many-to-one) and abstract to concrete (one-to-many) mapping.
    template <typename ConcreteType, typename AbstractType>
    class AbstractionMapping : private RefinementMapping<ConcreteType> {
        friend class RefinementMapping<ConcreteType>;
        using rm = RefinementMapping<ConcreteType>;
    public:
        // map_fn should always produce the same output on the same input.
        explicit AbstractionMapping(std::function<AbstractType(const ConcreteType&)>&& map_fn) : _map_fn(std::move(map_fn)) {
            static_assert(std::is_default_constructible_v<ConcreteType>, "ConcreteType must be default constructible");
            static_assert(std::is_default_constructible_v<AbstractType>, "AbstractType must be default constructible");
            static_assert(has_ptrie_interface_v<ConcreteType>, "ConcreteType must satisfy has_ptrie_interface_v<ConcreteType> e.g. by satisfying std::has_unique_object_representations_v<ConcreteType> or specializing ptrie::byte_iterator<ConcreteType> or ptrie_interface<ConcreteType>");
            static_assert(has_ptrie_interface_v<AbstractType>, "AbstractType must satisfy has_ptrie_interface_v<AbstractType> e.g. by satisfying std::has_unique_object_representations_v<AbstractType> or specializing ptrie::byte_iterator<AbstractType> or ptrie_interface<AbstractType>");
        };
        AbstractionMapping(std::function<AbstractType(const ConcreteType&)>&& map_fn, const std::unordered_set<ConcreteType>& initial_values)
                : AbstractionMapping(std::move(map_fn)) {
            for (const auto& value : initial_values) {
                insert(value);
            }
        };

        // returns whether a new abstract_value was added, and the id of the abstract value (new or old) corresponding to the concrete key.
        // optionally don't insert the concrete value in the backward map.
        std::pair<bool,size_t> insert(const ConcreteType& key, bool ignore_concrete = false) {
            size_t key_id;
            if (!ignore_concrete) {
                bool fresh_key;
                std::tie(fresh_key, key_id) = this->_many_to_one_map.insert(key);
                if (!fresh_key) return {false, this->_many_to_one_map.get_data(key_id)};
            }
            auto [fresh_value, value_id] = _abstract_values.insert(_map_fn(key));
            if (value_id >= this->_one_to_many_ids.size()) {
                this->_one_to_many_ids.resize(value_id + 1);
            }
            if (!ignore_concrete) {
                this->_many_to_one_map.get_data(key_id) = value_id;
                this->_one_to_many_ids[value_id].push_back(key_id);
            }
            return {fresh_value, value_id};
        }

        std::pair<bool,size_t> exists(const ConcreteType& key, bool ignore_concrete = false) const {
            if (!ignore_concrete) {
                auto [exists, key_id] = this->_many_to_one_map.exists(key);
                if (!exists) return {exists, key_id};
            }
            return _abstract_values.exists(_map_fn(key));
        }

        AbstractType map(const ConcreteType& concrete_value) const {
            return _map_fn(concrete_value);
        }
        AbstractType get_abstract_value(size_t id) const {
            return _abstract_values.at(id);
        }

        [[nodiscard]] size_t size() const {
            assert(rm::size() == _abstract_values.size());
            return rm::size();
        }

        std::vector<ConcreteType> get_concrete_values(const AbstractType& abstract_value) const {
            auto [found, id] = _abstract_values.exists(abstract_value);
            return rm::get_concrete_values(found ? id : this->_one_to_many_ids.size());
        }
        auto get_concrete_values_range(const AbstractType& abstract_value) const {
            auto [found, id] = _abstract_values.exists(abstract_value);
            return rm::get_concrete_values_range(found ? id : this->_one_to_many_ids.size());
        }

    private:
        std::function<AbstractType(const ConcreteType&)> _map_fn;
        ptrie_set<AbstractType> _abstract_values;
    };
    template <typename ConcreteType, typename AbstractType>
    AbstractionMapping(std::function<AbstractType(const ConcreteType&)>&& map_fn) -> AbstractionMapping<ConcreteType,AbstractType>;
    template <typename ConcreteType, typename AbstractType>
    AbstractionMapping(std::function<AbstractType(const ConcreteType&)>&& map_fn, const std::unordered_set<ConcreteType>& initial_values) -> AbstractionMapping<ConcreteType,AbstractType>;


}

#endif //PDAAAL_ABSTRACTIONMAPPING_H
