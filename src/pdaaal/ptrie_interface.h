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
 * File:   ptrie_interface.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 18-12-2020.
 */

#ifndef PDAAAL_PTRIE_INTERFACE_H
#define PDAAAL_PTRIE_INTERFACE_H

#include <ptrie/ptrie_map.h>
#include <vector>

namespace pdaaal::utils {
    // This file defines interfacing functionality for using more types with ptrie.
    // ptrie_interface provides a unified interface for already supported types and types that get
    // support from its specialization of ptrie_interface by conversion to/from std::vector<std::byte>.
    //
    // This file provides specializations for KEY where:
    //  a) std::has_unique_object_representations_v<KEY> is true or ptrie::byte_iterator<KEY> is explicitly defined
    //  b) std::vector<T> where T satisfies a).
    //  c) std::tuple<Args...> where each of Args satisfies a).
    //  d) std::pair<std::tuple<Args...>, std::vector<T>> where T and each of Args satisfies a).
    //
    // Lastly, for convenience, the file provides ptrie_set and ptrie_map that uses ptrie_interface directly.

    template <typename KEY, typename = void>
    struct ptrie_interface;
    template <typename KEY, typename = void>
    struct has_ptrie_interface : std::false_type {};
    template <typename KEY>
    struct has_ptrie_interface<KEY, std::void_t<ptrie_interface<KEY>>> : std::true_type {};
    template <typename KEY>
    constexpr bool has_ptrie_interface_v = has_ptrie_interface<KEY>::value;
    template <typename KEY>
    constexpr bool has_byte_iterator_v = std::conjunction_v<
            std::is_same<decltype(ptrie::byte_iterator<KEY>::access(std::declval<KEY*>(), std::declval<size_t>())),unsigned char&>,
            std::is_same<decltype(ptrie::byte_iterator<KEY>::const_access(std::declval<const KEY*>(), std::declval<size_t>())),const unsigned char&>,
            std::is_same<decltype(ptrie::byte_iterator<KEY>::element_size()), size_t>,
            std::is_same<decltype(ptrie::byte_iterator<KEY>::continious()), bool>>;

    // First define instances for resp. value and vector of values that trivially fits into a ptrie.
    template <typename KEY>
    struct ptrie_interface<KEY, std::enable_if_t<has_byte_iterator_v<KEY>>> {
        using elem_type = KEY;
        using insert_type = KEY;
        using external_type = KEY;
        static constexpr insert_type to_ptrie(external_type&& key) {
            return std::move(key);
        }
        template<uint16_t H, uint16_t S, size_t A, typename T, typename I>
        static constexpr external_type unpack(const ptrie::set_stable<elem_type,H,S,A,T,I>& p, size_t id) {
            external_type res;
            p.unpack(id, &res);
            return res;
        }
    };
    template <typename KEY>
    struct ptrie_interface<std::vector<KEY>, std::enable_if_t<has_byte_iterator_v<KEY>>> {
        using elem_type = KEY;
        using insert_type = std::vector<KEY>;
        using external_type = std::vector<KEY>;
        static insert_type to_ptrie(external_type&& key) {
            return std::move(key);
        }
        template<uint16_t H, uint16_t S, size_t A, typename T, typename I>
        static external_type unpack(const ptrie::set_stable<elem_type,H,S,A,T,I>& p, size_t id) {
            return p.unpack(id);
        }
    };

    // Next define interface for tuple of suitable types. This allows for tuples that are not necessarily packed, to used by converting to and from bytes vectors.
    template <typename... Args>
    struct ptrie_interface<std::tuple<Args...>, std::enable_if_t<((has_byte_iterator_v<Args>) && ...)>> {
        using elem_type = std::byte;
        using insert_type = std::vector<std::byte>;
        using external_type = std::tuple<Args...>;
        static insert_type to_ptrie(external_type&& key) {
            std::vector<std::byte> result;
            result.reserve((ptrie::byte_iterator<Args>::element_size() + ...));
            std::apply([&result](auto&&... args){(push_back_bytes(result, &args), ...);}, key);
            return result;
        }
        template<uint16_t H, uint16_t S, size_t A, typename T, typename I>
        static external_type unpack(const ptrie::set_stable<elem_type,H,S,A,T,I>& p, size_t id) {
            auto bytes = p.unpack(id);
            external_type result;
            std::apply([&bytes](auto&&... args){
                size_t bytes_id = 0;
                (from_bytes(bytes, &args, &bytes_id), ...);
            }, result);
            return result;
        }
    private:
        template <typename KEY>
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const KEY* data){
            for (size_t i = 0; i < ptrie::byte_iterator<KEY>::element_size(); ++i){
                result.push_back(ptrie::byte_iterator<KEY>::const_access(data, i));
            }
        }
        template <typename KEY>
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, KEY* data, size_t& bytes_id){
            for (size_t i = 0; i < ptrie::byte_iterator<KEY>::element_size(); ++i, ++bytes_id){
                ptrie::byte_iterator<KEY>::access(data, i) = bytes[bytes_id];
            }
        }
    };

    // Lastly define for a (tuple, vector) pair, where all elements has a byte iterator.
    // This allows for a combination of single fields and a vector of values.
    // This is about as general as it gets without storing additional information e.g. about sizes of each vector.
    // (okay, we could try nested tuples, but who wants that really...)
    template <typename VElem, typename... Args>
    struct ptrie_interface<std::pair<std::tuple<Args...>,std::vector<VElem>>,
    std::enable_if_t<has_byte_iterator_v<VElem> && ((has_byte_iterator_v<Args>) && ...)>> {
        using elem_type = std::byte;
        using insert_type = std::vector<std::byte>;
        using external_type = std::pair<std::tuple<Args...>,std::vector<VElem>>;
        static insert_type to_ptrie(external_type&& key) {
            std::vector<std::byte> result;
            result.reserve((ptrie::byte_iterator<Args>::element_size() + ...) + key.second.size() * ptrie::byte_iterator<VElem>::element_size());
            std::apply([&result](auto&&... args){(push_back_bytes(result, &args), ...);}, key.first);
            for (auto&& elem : key.second) {
                push_back_bytes(result, &elem);
            }
            return result;
        }
        template<uint16_t H, uint16_t S, size_t A, typename T, typename I>
        static external_type unpack(const ptrie::set_stable<elem_type,H,S,A,T,I>& p, size_t id) {
            auto bytes = p.unpack(id);
            external_type result;
            size_t bytes_id = 0;
            std::apply([&bytes, &bytes_id](auto&&... args){
                (from_bytes(bytes, &args, &bytes_id), ...);
            }, result.first);
            while (bytes_id < bytes.size()) {
                result.second.emplace_back();
                from_bytes(bytes, bytes_id, &result.second.back());
            }
            assert(bytes_id == bytes.size()); // Otherwise some misalignment happened. Not good.
            return result;
        }
    private:
        template <typename KEY>
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const KEY* data){
            for (size_t i = 0; i < ptrie::byte_iterator<KEY>::element_size(); ++i){
                result.push_back(ptrie::byte_iterator<KEY>::const_access(data, i));
            }
        }
        template <typename KEY>
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, KEY* data, size_t& bytes_id){
            for (size_t i = 0; i < ptrie::byte_iterator<KEY>::element_size(); ++i, ++bytes_id){
                ptrie::byte_iterator<KEY>::access(data, i) = bytes[bytes_id];
            }
        }
    };

    template <typename KEY>
    using ptrie_interface_elem = typename ptrie_interface<KEY>::elem_type;


    // Next we define ptrie_set and ptrie_map which makes using ptrie_interface seamless (except now you should use 'at' instead of 'unpack').
    template <typename KEY>
    class ptrie_set : public ptrie::set_stable<ptrie_interface_elem<KEY>> {
        static_assert(has_ptrie_interface_v<KEY>, "KEY does not provide a specialization for ptrie_interface.");
        static_assert(std::is_same_v<KEY, typename ptrie_interface<KEY>::external_type>, "KEY not matching ptrie_interface<KEY>::external_type.");
        using pt = ptrie::set_stable<ptrie_interface_elem<KEY>>;
    public:
        std::pair<bool, size_t> insert(const KEY& key) {
            return pt::insert(ptrie_interface<KEY>::to_ptrie(key));
        }
        [[nodiscard]] std::pair<bool, size_t> exists(const KEY& key) const {
            return pt::exists(ptrie_interface<KEY>::to_ptrie(key));
        }
        bool erase (const KEY& key) {
            return pt::erase(ptrie_interface<KEY>::to_ptrie(key));
        }
        KEY at(size_t index) const {
            return ptrie_interface<KEY>::unpack(*this, index);
        }
    };

    template <typename KEY, typename T>
    class ptrie_map : public ptrie::map<ptrie_interface_elem<KEY>, T>, public ptrie_set<KEY> {
        static_assert(has_ptrie_interface_v<KEY>, "KEY does not provide a specialization for ptrie_interface.");
        static_assert(std::is_same_v<KEY, typename ptrie_interface<KEY>::external_type>, "KEY not matching ptrie_interface<KEY>::external_type.");
        using pt = ptrie::map<ptrie_interface_elem<KEY>, T>;
    public:
        T& operator[](const KEY& key) {
            return pt::operator[](ptrie_interface<KEY>::to_ptrie(key));
        }
    };


}

#endif //PDAAAL_PTRIE_INTERFACE_H
