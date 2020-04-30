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

namespace pdaaal {

    enum class fut_container_type {
        hash,
        vector
    };

    // FUT-set is a Fast Unordered Tuple-set. Hopefully so fast that executing it says FUT.
    template<class T, typename = void, fut_container_type... C>
    class fut_set {};

    // recursive case - hash
    template<typename Head, typename... Tail, fut_container_type CHead, fut_container_type... CTail>
    struct fut_set<std::tuple<Head, Tail...>, std::enable_if_t<CHead == fut_container_type::hash>, CHead, CTail...> {
        using Inner = fut_set<std::tuple<Tail...>, void, CTail...>;
        std::unordered_map<Head,Inner> elems;

        template <typename... Args>
        auto emplace(const Head& head, Args&&... args) {
            return elems.try_emplace(head).first->second.emplace(std::forward<Args>(args)...);
        }
        template <typename... Args>
        auto emplace(Head&& head, Args&&... args) {
            return elems.try_emplace(std::move(head)).first->second.emplace(std::forward<Args>(args)...);
        }
        template <typename... Args>
        bool contains(const Head& head, const Args&... tail) const {
            auto it = elems.find(head);
            return it != elems.end() && it->second.contains(tail...);
        }
    };

    // recursive case - vector
    template<typename Head, typename... Tail, fut_container_type CHead, fut_container_type... CTail>
    struct fut_set<std::tuple<Head, Tail...>, std::enable_if_t<CHead == fut_container_type::vector>, CHead, CTail...> {
        using Inner = fut_set<std::tuple<Tail...>, void, CTail...>;
        vector_map<Head,Inner> elems;

        template <typename... Args>
        auto emplace(const Head& head, Args&&... args) {
            return elems.emplace(head).first->value.emplace(std::forward<Args>(args)...);
        }
        template <typename... Args>
        auto emplace(Head&& head, Args&&... args) {
            return elems.emplace(std::move(head)).first->value.emplace(std::forward<Args>(args)...);
        }
        template <typename... Args>
        bool contains(const Head& head, const Args&... tail) const {
            auto it = elems.find(head);
            return it != elems.end() && it->value.contains(tail...);
        }
    };

    // base case - map - hash
    template< class Head, class... Tail, fut_container_type C>
    struct fut_set<std::tuple<Head, Tail...>, std::enable_if_t<C == fut_container_type::hash>, C> : std::unordered_map<Head,std::tuple<Tail...>> {
        bool contains(const Head& head) const { // TODO: When C++20 arrives: Delete this.
            return this->find(head) != this->end();
        }
    };
    // base case - set - hash
    template< class Head, fut_container_type C>
    struct fut_set<std::tuple<Head>, std::enable_if_t<C == fut_container_type::hash>, C> : std::unordered_set<Head> {
        bool contains(const Head& head) const { // TODO: When C++20 arrives: Delete this.
            return this->find(head) != this->end();
        }
    };
    // base case - map - vector
    template< class Head, class... Tail, fut_container_type C>
    struct fut_set<std::tuple<Head, Tail...>, std::enable_if_t<C == fut_container_type::vector>, C> : vector_map<Head, std::tuple<Tail...>> { };
    // base case - set - vector
    template< class Head, fut_container_type C>
    struct fut_set<std::tuple<Head>, std::enable_if_t<C == fut_container_type::vector>, C> : vector_set<Head> { };

}

#endif //PDAAAL_FUT_SET_H
