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
 * File:   vector_set.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-04-2020.
 */

#ifndef PDAAAL_VECTOR_SET_H
#define PDAAAL_VECTOR_SET_H

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

namespace pdaaal::fut {

    template<typename Key, typename Value>
    struct vector_map {
        struct elem_t {
            elem_t() = default;
            template <typename... Args>
            explicit elem_t(const Key& key, Args&&... args) : first(key), second(std::forward<Args>(args)...) {}
            template <typename... Args>
            explicit elem_t(Key&& key, Args&&... args) : first(std::move(key)), second(std::forward<Args>(args)...) {}
            explicit elem_t(const std::pair<Key,Value>& arg) : first(arg.first), second(arg.second) {};
            explicit elem_t(std::pair<Key,Value>&& arg) : first(std::move(arg.first)), second(std::move(arg.second)) {};
            Key first;
            Value second;
            bool operator<(const elem_t &other) const { return first < other.first; }
            bool operator==(const elem_t &other) const { return first == other.first; }
            bool operator!=(const elem_t &other) const { return !(*this == other); }
        };

        vector_map() = default;
        explicit vector_map(const std::unordered_map<Key,Value>& other) : elems(other.begin(), other.end()) {
            std::sort(elems.begin(), elems.end());
        };
        explicit vector_map(std::unordered_map<Key,Value>&& other) : elems(std::make_move_iterator(other.begin()), std::make_move_iterator(other.end())) {
            std::sort(elems.begin(), elems.end());
        };

        using value_type = typename std::vector<elem_t>::value_type;
        using iterator = typename std::vector<elem_t>::iterator;
        using const_iterator = typename std::vector<elem_t>::const_iterator;

        iterator begin() noexcept  { return elems.begin(); }
        iterator end() noexcept { return elems.end(); }
        const_iterator begin() const noexcept { return elems.begin(); }
        const_iterator end() const noexcept { return elems.end(); }
        const_iterator cbegin() const noexcept { return elems.cbegin(); }
        const_iterator cend() const noexcept { return elems.cend(); }

        [[nodiscard]] size_t size() const noexcept { return elems.size(); }
        [[nodiscard]] bool empty() const noexcept { return elems.empty(); }

        template <typename... Args>
        auto emplace(const Key& key, Args&&... args) {
            elem_t elem(key, std::forward<Args>(args)...);
            auto lb = std::lower_bound(elems.begin(), elems.end(), elem);
            if (lb == elems.end() || *lb != elem) {
                lb = elems.insert(lb, elem);
                return std::make_pair(lb, true);
            }
            return std::make_pair(lb, false);
        }
        template <typename... Args>
        auto emplace(Key&& key, Args&&... args) {
            elem_t elem(std::move(key), std::forward<Args>(args)...);
            auto lb = std::lower_bound(elems.begin(), elems.end(), elem);
            if (lb == elems.end() || *lb != elem) {
                lb = elems.insert(lb, elem);
                return std::make_pair(lb, true);
            }
            return std::make_pair(lb, false);
        }
        // Provide interface similar to std::unordered_map
        template <typename... Args> auto try_emplace(const Key& key, Args&&... args) { return emplace(key, args...); }
        template <typename... Args> auto try_emplace(Key&& key, Args&&... args) { return emplace(key, args...); }

        bool contains(const Key& key) const {
            elem_t elem(key);
            auto lb = std::lower_bound(elems.begin(), elems.end(), elem);
            return lb != elems.end() && *lb == elem;
        }

        iterator find(const Key& key) {
            elem_t elem(key);
            auto lb = std::lower_bound(elems.begin(), elems.end(), elem);
            if (lb == elems.end() || *lb != elem) {
                return elems.end();
            }
            return lb;
        }
        const_iterator find(const Key& key) const {
            elem_t elem(key);
            auto lb = std::lower_bound(elems.begin(), elems.end(), elem);
            if (lb == elems.end() || *lb != elem) {
                return elems.end();
            }
            return lb;
        }
        value_type& operator[](std::size_t index) { return elems[index]; }
        const value_type& operator[](std::size_t index) const { return elems[index]; }
        void resize(size_t count) {
            assert(count <= size());
            elems.resize(count);
        };
        void clear() noexcept { elems.clear(); };

        auto lower_bound(const Key& key) const {
            elem_t elem(key);
            return std::lower_bound(elems.begin(), elems.end(), elem);
        }

    private:
        std::vector<elem_t> elems;
    };

    template<typename Key>
    struct vector_set {

        vector_set() = default;
        explicit vector_set(const std::unordered_set<Key>& other) : elems(other.begin(), other.end()) {
            std::sort(elems.begin(), elems.end());
        };
        explicit vector_set(std::unordered_set<Key>&& other) : elems(std::make_move_iterator(other.begin()), std::make_move_iterator(other.end())) {
            std::sort(elems.begin(), elems.end());
        };

        using value_type = typename std::vector<Key>::value_type;
        using iterator = typename std::vector<Key>::iterator;
        using const_iterator = typename std::vector<Key>::const_iterator;

        iterator begin() noexcept  { return elems.begin(); }
        iterator end() noexcept { return elems.end(); }
        const_iterator begin() const noexcept { return elems.begin(); }
        const_iterator end() const noexcept { return elems.end(); }
        const_iterator cbegin() const noexcept { return elems.cbegin(); }
        const_iterator cend() const noexcept { return elems.cend(); }

        [[nodiscard]] size_t size() const noexcept { return elems.size(); }
        [[nodiscard]] bool empty() const noexcept { return elems.empty(); }

        template <typename... Args>
        auto emplace(Args&&... args) {
            Key elem{std::forward<Args>(args)...};
            auto lb = std::lower_bound(elems.begin(), elems.end(), elem);
            if (lb == elems.end() || *lb != elem) {
                lb = elems.insert(lb, elem);
                return std::make_pair(lb, true);
            }
            return std::make_pair(lb, false);
        }

        bool contains(const Key& key) const {
            auto lb = std::lower_bound(elems.begin(), elems.end(), key);
            return lb != elems.end() && *lb == key;
        }

        iterator find(const Key& key) {
            auto lb = std::lower_bound(elems.begin(), elems.end(), key);
            if (lb == elems.end() || *lb != key) {
                return elems.end();
            }
            return lb;
        }
        const_iterator find(const Key& key) const {
            auto lb = std::lower_bound(elems.begin(), elems.end(), key);
            if (lb == elems.end() || *lb != key) {
                return elems.end();
            }
            return lb;
        }
        value_type& operator[](std::size_t index) { return elems[index]; }
        const value_type& operator[](std::size_t index) const { return elems[index]; }
        void resize(size_t count) {
            assert(count <= size());
            elems.resize(count);
        };
        void clear() noexcept { elems.clear(); };

        auto lower_bound(const Key& key) const {
            return std::lower_bound(elems.begin(), elems.end(), key);
        }

    private:
        std::vector<Key> elems;
    };

}

#endif //PDAAAL_VECTOR_SET_H
