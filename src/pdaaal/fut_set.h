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
 * File:   fut_set.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 30-04-2020.
 */

#ifndef PDAAAL_FUT_SET_H
#define PDAAAL_FUT_SET_H

#include "vector_set.h"
#include <unordered_set>
#include <unordered_map>

namespace std20 { // TODO: When C++20 arrives: Delete this.
    // Add contains method to unordered containers, until we can use the ones in C++20.
    template<typename Key, typename Tp,
            typename Hash = std::hash<Key>,
            typename Pred = std::equal_to<Key>,
            typename Alloc = std::allocator<std::pair<const Key, Tp>>>
    class unordered_map : public std::unordered_map<Key,Tp,Hash,Pred,Alloc> {
    public:
        bool contains(const Key &key) const {
            return this->find(key) != this->end();
        }
    };
    template<typename Value,
            typename Hash = std::hash<Value>,
            typename Pred = std::equal_to<Value>,
            typename Alloc = std::allocator<Value>>
    class unordered_set : public std::unordered_set<Value,Hash,Pred,Alloc> {
    public:
        bool contains(const Value &value) const {
            return this->find(value) != this->end();
        }
    };
}

namespace pdaaal::fut {

    enum class type {
        hash,
        vector
    };

    namespace detail {
        // FUT-set is a Fast Unordered Tuple-set. Hopefully so fast that executing it says FUT.
        template<class T, type... C>
        class fut_set { };

        template<typename Key, typename Value, type C>
        using map_container = std::conditional_t<C == type::hash, std20::unordered_map<Key, Value>, vector_map<Key, Value>>;
        template<typename Key, type C>
        using set_container = std::conditional_t<C == type::hash, std20::unordered_set<Key>, vector_set<Key>>;

        // TODO: Add 'compare_by' functionality somehow.

        // recursive case - hash
        template<typename Head, typename... Tail, type CHead, type CNeck, type... CTail>
        class fut_set<std::tuple<Head, Tail...>, CHead, CNeck, CTail...> {
        private:
            using Inner = fut_set<std::tuple<Tail...>, CNeck, CTail...>;
            using container_type = map_container<Head, Inner, CHead>;
        public:
            using inner_value_type = typename Inner::inner_value_type;

            using value_type = typename container_type::value_type;
            using iterator = typename container_type::iterator;
            using const_iterator = typename container_type::const_iterator;

            iterator begin() noexcept  { return elems.begin(); }
            iterator end() noexcept { return elems.end(); }
            const_iterator begin() const noexcept { return elems.begin(); }
            const_iterator end() const noexcept { return elems.end(); }
            const_iterator cbegin() const noexcept { return elems.cbegin(); }
            const_iterator cend() const noexcept { return elems.cend(); }

            [[nodiscard]] size_t size() const noexcept { return elems.size(); }
            [[nodiscard]] bool empty() const noexcept { return elems.empty(); }

            template<typename... Args>
            auto emplace(const Head &head, Args &&... args) {
                return elems.try_emplace(head).first->second.emplace(std::forward<Args>(args)...);
            }

            template<typename... Args>
            auto emplace(Head &&head, Args &&... args) {
                return elems.try_emplace(std::move(head)).first->second.emplace(std::forward<Args>(args)...);
            }

            template<typename... Args>
            bool contains(const Head &head, const Args &... tail) const {
                auto it = elems.find(head);
                return it != elems.end() && it->second.contains(tail...);
            }

            template<typename... Args>
            const inner_value_type* get(const Head &head, const Args &... tail) const {
                auto it = elems.find(head);
                return it == elems.end() ? nullptr : it->second.get(tail...);
            }
            template<typename... Args>
            inner_value_type* get(const Head &head, const Args &... tail) {
                auto it = elems.find(head);
                return it == elems.end() ? nullptr : it->second.get(tail...);
            }

        private:
            container_type elems;
        };


        // Singleton tuples decay to their inner type.
        template<typename Head, typename... Tail>
        using weak_tuple = std::conditional_t<std::tuple_size_v<std::tuple<Head,Tail...>> == 1, Head, std::tuple<Head,Tail...>>;
        // base case - map
        template<typename Head, typename Neck, typename... Tail, type C>
        class fut_set<std::tuple<Head, Neck, Tail...>, C> : public map_container<Head, weak_tuple<Neck, Tail...>, C> {
        public:
            using inner_value_type = weak_tuple<Neck, Tail...>;

            const inner_value_type* get(const Head &head) const {
                auto it = this->find(head);
                return it == this->end() ? nullptr : &it->second;
            }
            inner_value_type* get(const Head &head) {
                auto it = this->find(head);
                return it == this->end() ? nullptr : &it->second;
            }
        };

        // base case - set
        template<typename Head, type C>
        class fut_set<std::tuple<Head>, C> : public set_container<Head,C> {
        public:
            using inner_value_type = std::tuple<>;
        };

    }

    // This separation of detail is less important now, without use of std::enable_if, but still a good pattern.
    template<class T, type... C>
    using set = typename detail::fut_set<T, C...>;

}

#endif //PDAAAL_FUT_SET_H
