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
    //  b) Nested combinations of std::tuple and std::vector, but without vectors nested (somewhere) inside other vectors,
    //  where the lowest element types satisfies a). The size of std::vector is stored to make parsing consistent.
    //  c) A single std::vector that does not need to store its size.
    //  d) A std::pair where first is a) or b), and second is c) or d).
    //
    // Lastly, for convenience, the file provides ptrie_set and ptrie_map that uses ptrie_interface directly.

    template <typename KEY, typename = void> struct ptrie_interface;
    template <typename KEY, typename = void> struct has_ptrie_interface : std::false_type {};
    template <typename KEY> struct has_ptrie_interface<KEY, std::void_t<ptrie_interface<KEY>>> : std::true_type {};
    template <typename KEY> constexpr bool has_ptrie_interface_v = has_ptrie_interface<KEY>::value;

    template <typename KEY> constexpr bool has_byte_iterator_v = std::conjunction_v<
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

    // For vectors, we need to know if elements are fixed size
    template <typename KEY, typename = void> struct fixed_byte_size;
    template <typename KEY, typename = void> struct has_fixed_byte_size : std::false_type {};
    template <typename KEY> struct has_fixed_byte_size<KEY, std::void_t<fixed_byte_size<KEY>>> : std::true_type {};
    template <typename KEY> constexpr bool has_fixed_byte_size_v = has_fixed_byte_size<KEY>::value;
    // For tuples, we need to know that whether size information is knowable from the byte vector (to ensure correct parsing).
    template <typename KEY, bool knowable_size, typename = void> struct byte_vector_converter;
    template <typename KEY, bool knowable_size, typename = void> struct has_byte_vector_converter : std::false_type {};
    template <typename KEY, bool knowable_size> struct has_byte_vector_converter<KEY, knowable_size, std::void_t<byte_vector_converter<KEY, knowable_size>>> : std::true_type {};
    template <typename KEY, bool knowable_size> constexpr bool has_byte_vector_converter_v = has_byte_vector_converter<KEY, knowable_size>::value;
    // Convenience definition
    template <typename KEY> constexpr bool has_fixed_size_converter = has_fixed_byte_size_v<KEY> && has_byte_vector_converter_v<KEY, true>;

    // If KEY satisfies has_byte_iterator_v<KEY> it has a fixed size.
    template <typename KEY>
    struct fixed_byte_size<KEY, std::enable_if_t<has_byte_iterator_v<KEY>>> {
        using T = KEY;
        static constexpr size_t size() {
            return ptrie::byte_iterator<KEY>::element_size();
        }
    };
    // A tuple of fixed-size Args has a fixed size.
    template <typename... Args>
    struct fixed_byte_size<std::tuple<Args...>, std::enable_if_t<((has_fixed_byte_size_v<Args>) && ...)>> {
        using T = std::tuple<Args...>;
        static constexpr size_t size() {
            return (fixed_byte_size<Args>::size() + ...);
        }
    };

    // Byte converter by using byte_iterator<KEY> directly.
    template <typename KEY>
    struct byte_vector_converter<KEY, true, std::enable_if_t<has_byte_iterator_v<KEY>>> {
        using T = KEY;
        static constexpr size_t size(const T& data) {
            return ptrie::byte_iterator<KEY>::element_size();
        }
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const T& data){
            for (size_t i = 0; i < ptrie::byte_iterator<KEY>::element_size(); ++i){
                result.push_back(ptrie::byte_iterator<KEY>::const_access(&data, i));
            }
        }
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, size_t& bytes_id, T& data){
            for (size_t i = 0; i < ptrie::byte_iterator<KEY>::element_size(); ++i, ++bytes_id){
                ptrie::byte_iterator<KEY>::access(&data, i) = bytes[bytes_id];
            }
        }
    };
    // Byte converter for tuples of fixed-size elements that has a byte converter.
    // This allows nested tuples.
    template <typename... Args>
    struct byte_vector_converter<std::tuple<Args...>, true, std::enable_if_t<((has_fixed_size_converter<Args>) && ...)>> {
        using T = std::tuple<Args...>;
        static constexpr size_t size(const T& data) {
            return fixed_byte_size<std::tuple<Args...>>::size();
        }
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const T& data){
            std::apply([&result](auto&&... args){(byte_vector_converter<decltype(args),true>::push_back_bytes(result, args), ...);}, data);
        }
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, size_t& bytes_id, T& data){
            std::apply([&bytes, &bytes_id](auto&&... args){(byte_vector_converter<decltype(args),true>::from_bytes(bytes, bytes_id, args), ...);}, data);
        }
    };
    // Byte converter for tuples of elements that has a byte converter that knows its size (i.e. vector stores size, so it can parse it back correctly).
    // This generalizes the one above, but its ::size(method) is potentially slower (i.e. not compile-time evaluated - maybe, I don't know..)
    template <typename... Args>
    struct byte_vector_converter<std::tuple<Args...>, true, std::enable_if_t<((has_byte_vector_converter_v<Args, true>) && ...)>> {
        using T = std::tuple<Args...>;
        static constexpr size_t size(const T& data) {
            return std::apply([](auto&&... args){return (byte_vector_converter<decltype(args),true>::size(args) + ...);}, data);
        }
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const T& data){
            std::apply([&result](auto&&... args){(byte_vector_converter<decltype(args),true>::push_back_bytes(result, args), ...);}, data);
        }
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, size_t& bytes_id, T& data){
            std::apply([&bytes, &bytes_id](auto&&... args){(byte_vector_converter<decltype(args),true>::from_bytes(bytes, bytes_id, args), ...);}, data);
        }
    };
    // Vector of fixed size elements. Is not it self fixed_size. It stores extra info of its size, so it only uses the bytes corresponding to it.
    // This means multiple vectors can be in a tuple using this approach.
    template <typename Elem>
    struct byte_vector_converter<std::vector<Elem>, true, std::enable_if_t<has_fixed_size_converter<Elem> && has_fixed_size_converter<typename std::vector<Elem>::size_type>>> {
        using T = std::vector<Elem>;
        using size_type = typename std::vector<Elem>::size_type;
        static constexpr size_t size(const T& data) {
            return fixed_byte_size<size_type>::size() + fixed_byte_size<Elem>::size() * data.size();
        }
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const T& data){
            byte_vector_converter<size_type, true>::push_back_bytes(result, data.size());
            for (const auto& elem : data) {
                byte_vector_converter<Elem, true>::push_back_bytes(result, elem);
            }
        }
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, size_t& bytes_id, T& data){
            size_type size;
            byte_vector_converter<Elem, true>::from_bytes(bytes, bytes_id, size);
            for (size_type i = 0; i < size; ++i) {
                data.emplace_back();
                byte_vector_converter<Elem, true>::from_bytes(bytes, bytes_id, data.back());
            }
        }
    };
    // Vector of fixed size elements. Is not it self fixed_size. It uses all bytes left.
    template <typename Elem>
    struct byte_vector_converter<std::vector<Elem>, false, std::enable_if_t<has_fixed_size_converter<Elem>>> {
        using T = std::vector<Elem>;
        static constexpr size_t size(const T& data) {
            return fixed_byte_size<Elem>::size() * data.size();
        }
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const T& data){
            for (const auto& elem : data) {
                byte_vector_converter<Elem, true>::push_back_bytes(result, elem);
            }
        }
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, size_t& bytes_id, T& data){
            while (bytes_id < bytes.size()) {
                data.emplace_back();
                byte_vector_converter<Elem, true>::from_bytes(bytes, bytes_id, data.back());
            }
            assert(bytes_id == bytes.size()); // Otherwise some misalignment happened. Not good.
        }
    };
    // Byte converter for a pair, where the first element has a fixed size, and the second can use the rest of the bytes. (no need for extra size info).
    template <typename First, typename Second>
    struct byte_vector_converter<std::pair<First,Second>, false, std::enable_if_t<has_byte_vector_converter_v<First,true> && has_byte_vector_converter_v<Second,false>>> {
        using T = std::pair<First,Second>;
        static constexpr size_t size(const T& data) {
            return byte_vector_converter<First, true>::size(data.first); + byte_vector_converter<Second, false>::size(data.second);
        }
        static constexpr void push_back_bytes(std::vector<std::byte>& result, const T& data){
            byte_vector_converter<First, true>::push_back_bytes(result, data.first);
            byte_vector_converter<Second, false>::push_back_bytes(result, data.second);
        }
        static constexpr void from_bytes(const std::vector<std::byte>& bytes, size_t& bytes_id, T& data){
            byte_vector_converter<First, true>::from_bytes(bytes, bytes_id, data.first);
            byte_vector_converter<Second, false>::from_bytes(bytes, bytes_id, data.second);
        }
    };

    // Special ptrie_interface for any KEY was a byte vector converter.
    template <typename KEY>
    struct ptrie_interface<KEY, std::enable_if_t<(has_byte_vector_converter_v<KEY,true> || has_byte_vector_converter_v<KEY,false>) && !has_byte_iterator_v<KEY>>> {
        using elem_type = std::byte;
        using insert_type = std::vector<std::byte>;
        using external_type = KEY;
        static constexpr bool use_knowable_size = !has_byte_vector_converter_v<KEY,false>; // Prefer not adding unnecesary size info.
        static insert_type to_ptrie(const external_type& key) {
            std::vector<std::byte> result;
            result.reserve(byte_vector_converter<KEY,use_knowable_size>::size(key));
            byte_vector_converter<KEY,use_knowable_size>::push_back_bytes(result, key);
            return result;
        }
        template<uint16_t H, uint16_t S, size_t A, typename T, typename I>
        static external_type unpack(const ptrie::set_stable<elem_type,H,S,A,T,I>& p, size_t id) {
            auto bytes = p.unpack(id);
            external_type result;
            size_t bytes_id = 0;
            byte_vector_converter<KEY,use_knowable_size>::from_bytes(bytes, bytes_id, result);
            return result;
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
