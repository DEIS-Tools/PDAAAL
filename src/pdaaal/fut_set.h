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
#include <boost/functional/hash.hpp>
#include "std20.h"

namespace pdaaal::fut {

    enum class type {
        hash,
        vector
    };

    namespace detail {

        // Use std::hash by default, but for tuples use custom implementation based on boost::hash_combine, but using std::hash on inner types.
        template <typename T>
        struct hash {
            size_t operator()(const T& t) const {
                return std::hash<T>()(t);
            }
        };
        template <typename... Args>
        struct hash<std::tuple<Args...>> {
            size_t operator()(const std::tuple<Args...>& tuple) const {
                size_t seed = 0;
                std::apply([&seed](auto&&... args){ (boost::hash_detail::hash_combine_impl(seed, std::hash<std20::remove_cvref_t<decltype(args)>>()(args)), ...); }, tuple);
                return seed;
            }
        };

        // FUT-set is a Fast Unordered Tuple-set. Hopefully so fast that executing it says FUT.
        template<class T, type... C>
        class fut_set { };

        template<typename Key, typename Value, type C>
        using map_container = std::conditional_t<C == type::hash, std20::unordered_map<Key, Value, hash<Key>>, vector_map<Key, Value>>;
        template<typename Key, type C>
        using set_container = std::conditional_t<C == type::hash, std20::unordered_set<Key, hash<Key>>, vector_set<Key>>;

        // TODO: Add 'compare_by' functionality somehow.

        // recursive case
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
            void clear() noexcept { elems.clear(); };

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

            fut_set() = default;
            template<type OtherC>
            fut_set(const fut_set<std::tuple<Head, Neck, Tail...>, OtherC>& other) : map_container<Head, inner_value_type, C>(other) {}
            template<type OtherC>
            fut_set(fut_set<std::tuple<Head, Neck, Tail...>, OtherC>&& other) : map_container<Head, inner_value_type, C>(std::move(other)) {}

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
