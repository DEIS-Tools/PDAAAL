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
#include <iterator>
#include <memory>
#include <vector>

namespace pdaaal::details {
    using namespace pdaaal::utils;
/*
    // Iterator used by AbstractionMapping to iterate through the concrete values that correspond to a given abstract value.
    // It is maybe a bit overkill to define an (almost) complete random access iterator, but here goes.
    template<typename T, typename _inner_iterator = std::vector<size_t>::const_iterator>
    struct concrete_value_iterator {
    private:
        static_assert(std::is_same_v<std::remove_const_t<typename _inner_iterator::value_type>, size_t>,
                      "_inner_iterator::value_type is not size_t");
        using _iterator_type = std::iterator<std::random_access_iterator_tag, T>;
        _inner_iterator _inner;
        const ptrie_map<T, size_t>* _map;
    public:
        using iterator_category = typename _iterator_type::iterator_category;
        using value_type = typename _iterator_type::value_type;
        static_assert(std::is_same_v<value_type, T>, "value_type and T not matching");
        using difference_type = typename _iterator_type::difference_type;
        // pointer and reference are not used, as we have to return by value what is stored in the ptrie_map.
        //using pointer           = typename _iterator_type::pointer;
        //using reference         = typename _iterator_type::reference;

        explicit constexpr concrete_value_iterator(const ptrie_map<T, size_t>* map) noexcept: _inner(
                _inner_iterator()), _map(map) {}

        concrete_value_iterator(_inner_iterator&& i, const ptrie_map<T, size_t>* map) noexcept: _inner(std::move(i)),
                                                                                                 _map(map) {}

        concrete_value_iterator(const _inner_iterator& i, const ptrie_map<T, size_t>* map) noexcept: _inner(i),
                                                                                                      _map(map) {}

        value_type operator*() const { // Note this gives a value_type not a reference.
            return _map->at(*_inner);
        }

        // Forward iterator requirements
        concrete_value_iterator& operator++() noexcept {
            ++_inner;
            return *this;
        }

        concrete_value_iterator operator++(int) noexcept { return concrete_value_iterator(_inner++, _map); }

        template<typename U, typename _iteratorR>
        bool operator==(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept {
            return base() == rhs.base();
        }

        template<typename U, typename _iteratorR>
        bool operator!=(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept {
            return base() != rhs.base();
        }

        // Bidirectional iterator requirements
        concrete_value_iterator& operator--() noexcept {
            --_inner;
            return *this;
        }

        concrete_value_iterator operator--(int) noexcept { return concrete_value_iterator(_inner--, _map); }

        // Random access iterator requirements
        value_type operator[](difference_type n) const { // Note this gives a value_type not a reference.
            return _map->at(_inner[n]);
        }

        concrete_value_iterator& operator+=(difference_type n) noexcept {
            _inner += n;
            return *this;
        }

        concrete_value_iterator operator+(difference_type n) const noexcept {
            return concrete_value_iterator(_inner + n, _map);
        }

        concrete_value_iterator& operator-=(difference_type n) noexcept {
            _inner -= n;
            return *this;
        }

        concrete_value_iterator operator-(difference_type n) const noexcept {
            return concrete_value_iterator(_inner - n, _map);
        }

        template<typename U, typename _iteratorR>
        difference_type operator-(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept {
            return base() - rhs.base();
        }

        template<typename U, typename _iteratorR>
        bool operator<(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept { return base() < rhs.base(); }

        template<typename U, typename _iteratorR>
        bool operator>(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept { return base() > rhs.base(); }

        template<typename U, typename _iteratorR>
        bool operator<=(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept {
            return base() <= rhs.base();
        }

        template<typename U, typename _iteratorR>
        bool operator>=(const concrete_value_iterator<U, _iteratorR>& rhs) const noexcept {
            return base() >= rhs.base();
        }

        const _inner_iterator& base() const noexcept { return _inner; }

        const ptrie_map<T, size_t>* map() const noexcept { return _map; }
    };

    template<typename T, typename _iterator>
    inline concrete_value_iterator<T, _iterator>
    operator+(typename concrete_value_iterator<T, _iterator>::difference_type n,
              const concrete_value_iterator<T, _iterator>& i) noexcept {
        return concrete_value_iterator<T, _iterator>(i.base() + n, i.map());
    }
*/
}

namespace pdaaal {
    using namespace pdaaal::utils;

    // Given a mapping function from ConcreteType to AbstractType,
    // the AbstractionMapping gives a consecutive id to each distinct (by byte representation) AbstractType value,
    // and provides efficient concrete to abstract (many-to-one) and abstract to concrete (one-to-many) mapping.
    template <typename ConcreteType, typename AbstractType>
    class AbstractionMapping {
    public:
        // map_fn should always produce the same output on the same input.
        explicit AbstractionMapping(std::function<AbstractType(const ConcreteType&)>&& map_fn) : _map_fn(std::move(map_fn)) {
            static_assert(std::is_default_constructible_v<ConcreteType>, "ConcreteType must be default constructible");
            static_assert(std::is_default_constructible_v<AbstractType>, "AbstractType must be default constructible");
            static_assert(has_ptrie_interface_v<ConcreteType>, "ConcreteType must satisfy has_ptrie_interface_v<ConcreteType> e.g. by satisfying std::has_unique_object_representations_v<ConcreteType> or specializing ptrie::byte_iterator<ConcreteType> or ptrie_interface<ConcreteType>");
            static_assert(has_ptrie_interface_v<AbstractType>, "AbstractType must satisfy has_ptrie_interface_v<AbstractType> e.g. by satisfying std::has_unique_object_representations_v<AbstractType> or specializing ptrie::byte_iterator<AbstractType> or ptrie_interface<AbstractType>");
        };
        AbstractionMapping(std::function<AbstractType(const ConcreteType&)>&& map_fn, std::unordered_set<ConcreteType>&& initial_values)
        : AbstractionMapping(std::move(map_fn)) {
            for (auto&& value : initial_values) {
                insert(value);
            }
        };

        // returns whether a new abstract_value was added, and the id of the abstract value (new or old) corresponding to the concrete key.
        // optionally don't insert the concrete value in the backward map.
        std::pair<bool,size_t> insert(const ConcreteType& key, bool ignore_concrete = false) {
            size_t key_id;
            if (!ignore_concrete) {
                bool fresh_key;
                std::tie(fresh_key, key_id) = _many_to_one_map.insert(key);
                if (!fresh_key) return {false, _many_to_one_map.get_data(key_id)};
            }
            auto [fresh_value, value_id] = _abstract_values.insert(_map_fn(key));
            if (value_id >= _one_to_many_ids.size()) {
                _one_to_many_ids.resize(value_id + 1);
            }
            if (!ignore_concrete) {
                _many_to_one_map.get_data(key_id) = value_id;
                _one_to_many_ids[value_id].push_back(key_id);
            }
            return {fresh_value, value_id};
        }

        std::pair<bool,size_t> exists(const ConcreteType& key, bool ignore_concrete = false) {
            if (!ignore_concrete) {
                auto [exists, key_id] = _many_to_one_map.exists(key);
                if (!exists) return {exists, key_id};
            }
            return _abstract_values.exists(_map_fn(key));
        }

        AbstractType map(const ConcreteType& concrete_value) {
            return _map_fn(concrete_value);
        }

        std::vector<size_t> encode_many(const std::vector<ConcreteType>& concrete_values) const {
            std::unordered_set<size_t> res_set;
            for (const auto& concrete_value : concrete_values) {
                auto [exists, value] = _many_to_one_map.exists(concrete_value);
                assert(exists);
                res_set.emplace(value);
            }
            std::vector<size_t> result(res_set.begin(), res_set.end());
            std::sort(result.begin(), result.end());
            return result;
        }

        AbstractType get_abstract_value(size_t id) const {
            return _abstract_values.at(id);
        }

        // Construct vector
        std::vector<ConcreteType> get_concrete_values_from_abstract(const AbstractType& value) const {
            auto [exists, id] = _abstract_values.exists(value);
            return get_concrete_values(exists ? id : _one_to_many_ids.size());
        }
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
            explicit concrete_value_range(const ptrie_map<ConcreteType, size_t>& map, const std::vector<size_t>* range = nullptr)
            : _map(map), _range(range) { };
            using iterator = details::ptrie_access_iterator<ConcreteType>;
            iterator begin() const noexcept {
                return _range != nullptr ? iterator(_range->begin(), _map) : iterator(_map);
            }
            iterator end() const noexcept {
                return _range != nullptr ? iterator(_range->end(), _map) : iterator(_map);
            }
        private:
            const ptrie_map<ConcreteType, size_t>& _map;
            const std::vector<size_t>* _range;
        };

        concrete_value_range get_concrete_values_range_from_abstract(const AbstractType& value) const {
            auto [exists, id] = _abstract_values.exists(value);
            return get_concrete_values_range(exists ? id : _one_to_many_ids.size());
        }
        concrete_value_range get_concrete_values_range(size_t abstract_value) const {
            if (abstract_value < _one_to_many_ids.size()) {
                return concrete_value_range(_many_to_one_map, &_one_to_many_ids[abstract_value]);
            }
            return concrete_value_range(_many_to_one_map);
        }

        [[nodiscard]] size_t size() const {
            return _abstract_values.size();
        }

    private:
        std::function<AbstractType(const ConcreteType&)> _map_fn;
        ptrie_set<AbstractType> _abstract_values;
        ptrie_map<ConcreteType, size_t> _many_to_one_map;
        std::vector<std::vector<size_t>> _one_to_many_ids;
    };

}

#endif //PDAAAL_ABSTRACTIONMAPPING_H
