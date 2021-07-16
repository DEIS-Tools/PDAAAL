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
 * File:   Weight.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 03-03-2020.
 */

#ifndef PDAAAL_WEIGHT_H
#define PDAAAL_WEIGHT_H

#include <limits>
#include <type_traits>
#include <array>
#include <algorithm>
#include <numeric>
#include <vector>
#include <optional>
#include <functional>
#include <boost/container_hash/hash.hpp>

namespace pdaaal {

    struct weight_base {
        static constexpr bool is_weight = false;
        static constexpr bool is_signed = false;
        static constexpr bool has_mult = false;
    };
    template<typename W, typename = void> struct weight : public weight_base {
        using type = W;
    };
    template<typename W>
    struct weight<W, std::enable_if_t<std::is_arithmetic_v<W> && std::numeric_limits<W>::is_specialized>> {
        using type = W;
        static constexpr bool is_weight = true;
        static constexpr bool is_signed = std::numeric_limits<W>::is_signed;
        static constexpr bool has_mult = true;
        static constexpr auto zero = []() -> type { return static_cast<type>(0); };
        static constexpr auto bottom = []() -> type { return std::numeric_limits<type>::min(); };
        static constexpr auto max = []() -> type { return std::numeric_limits<type>::max(); };
        static constexpr bool less(type lhs, type rhs) {
            return lhs < rhs;
        }
        static constexpr type add(type lhs, type rhs) {
            if (lhs == max_val || rhs == max_val) return max_val;
            if constexpr (is_signed) {
                if (lhs == bottom_val || rhs == bottom_val) return bottom_val;
            }
            return lhs + rhs;
        };
        static constexpr type multiply(type lhs, type rhs) {
            if (lhs == max_val || rhs == max_val) return max_val;
            if constexpr (is_signed) {
                if (lhs == bottom_val || rhs == bottom_val) return bottom_val; // We don't expect to multiply with bottom...
            }
            return lhs * rhs;
        };
    private:
        static constexpr type bottom_val = std::numeric_limits<type>::min();
        static constexpr type max_val = std::numeric_limits<type>::max();
    };
    template<typename Inner, std::size_t N>
    struct weight<std::array<Inner, N>, std::enable_if_t<std::is_arithmetic_v<Inner> && std::numeric_limits<Inner>::is_specialized>> {
        using type = std::array<Inner, N>;
        static constexpr bool is_weight = true;
        static constexpr bool is_signed = weight<Inner>::is_signed;
        static constexpr bool has_mult = false;
        static constexpr auto zero = []() -> type {std::array<Inner, N> arr{}; arr.fill(weight<Inner>::zero()); return arr;};
        static constexpr auto bottom = []() -> type {std::array<Inner, N> arr{}; arr.fill(weight<Inner>::bottom()); return arr;};
        static constexpr auto max = []() -> type {std::array<Inner, N> arr{}; arr.fill(weight<Inner>::max()); return arr;};
        static constexpr bool less(const type& lhs, const type& rhs) {
            return lhs < rhs;
        }
        static constexpr type add(const type& lhs, const type& rhs) {
            std::array<Inner, N> res{};
            for (size_t i = 0; i < N; ++i) {
                res[i] = weight<Inner>::add(lhs[i], rhs[i]);
            }
            return res;
        };
    };
    template<typename Inner>
    struct weight<std::vector<Inner>, std::enable_if_t<std::is_arithmetic_v<Inner> && std::numeric_limits<Inner>::is_specialized>> {
        using type = std::vector<Inner>;
        static constexpr bool is_weight = true;
        static constexpr bool is_signed = weight<Inner>::is_signed;
        static constexpr bool has_mult = false;
        static constexpr auto zero = []() -> type { return type{}; };
        static constexpr auto bottom = []() -> type { return type{weight<Inner>::bottom()}; };
        static constexpr auto max = []() -> type { return type{weight<Inner>::max()}; }; // TODO: When C++20 arrives, use a constexpr vector instead
        static constexpr bool less(const type& lhs, const type& rhs) {
            if constexpr (is_signed) {
                if (rhs.size() == 1 && rhs[0] == weight<Inner>::bottom()) return false;
                if (lhs.size() == 1 && lhs[0] == weight<Inner>::bottom()) return true;
            }
            return lhs < rhs;
        }
        static constexpr type add(const type& lhs, const type& rhs) {
            const auto& [small, large] = std::minmax(lhs, rhs, [](const type& lhs, const type& rhs){ return lhs.size() < rhs.size(); });
            std::vector<Inner> result = large;
            std::transform(small.begin(), small.end(), large.begin(),
                           result.begin(), weight<Inner>::add);
            return result;
        };
    };

//    template <typename T, typename = void> struct has_type : std::false_type {};
//    template <typename T> struct has_type<T, std::void_t<decltype(std::declval<typename T::type>())>> : std::true_type {};
//    template <typename T> inline constexpr auto has_type_v = has_type<T>::value;
//
//    template <typename T, typename = void> struct has_zero : std::false_type {};
//    template <typename T> struct has_zero<T, std::void_t<decltype(T::zero())>> : std::true_type {};
//    template <typename T> inline constexpr auto has_zero_v = has_zero<T>::value;
//
//    template <typename T, typename = void> struct has_max : std::false_type {};
//    template <typename T> struct has_max<T, std::void_t<decltype(T::max())>> : std::true_type {};
//    template <typename T> inline constexpr auto has_max_v = has_max<T>::value;
//
//    template <typename T, typename = void> struct has_less : std::false_type {};
//    template <typename T> struct has_less<T, std::void_t<decltype(T::less(std::declval<typename T::type>(), std::declval<typename T::type>()))>> : std::true_type {};
//    template <typename T> inline constexpr auto has_less_v = has_less<T>::value;
//
//    template <typename T, typename = void> struct has_add : std::false_type {};
//    template <typename T> struct has_add<T, std::void_t<decltype(T::add(std::declval<typename T::type>(), std::declval<typename T::type>()))>> : std::true_type {};
//    template <typename T> inline constexpr auto has_add_v = has_add<T>::value;

    template <typename T, typename = void> struct has_boost_hash : std::false_type {};
    template <typename T> struct has_boost_hash<T, std::void_t<decltype(std::declval<boost::hash<typename T::type>>())>> : std::true_type {};
    template <typename T> inline constexpr auto has_boost_hash_v = has_boost_hash<T>::value;

    template<typename W> inline constexpr auto is_weighted = W::is_weight && has_boost_hash_v<W>;
//            !std::is_void_v<W> && has_type_v<W> && !std::is_void_v<typename W::type> &&
//            has_zero_v<W> && has_max_v<W> && has_less_v<W> && has_add_v<W> && has_boost_hash_v<W>;

//    template <typename T, typename = void> struct has_mult : std::false_type {};
//    template <typename T> struct has_mult<T, std::void_t<decltype(T::multiply(std::declval<typename T::type>(), std::declval<typename T::type>()))>> : std::true_type {};
//    template <typename T> constexpr auto has_mult_v = has_mult<T>::value;

    template <typename W, typename... Args>
    class linear_weight_function {
    private:
        const std::vector<std::pair<W, linear_weight_function<W, Args...>>> _functions;
        const std::optional<std::function<W(Args...)>> _function;
    public:
        static_assert(is_weighted<weight<W>>);
        using result_type = W;

        // A single function
        explicit linear_weight_function(std::function<W(Args...)> function) : _function(function) {}

        // Linear combination of functions
        explicit linear_weight_function(std::vector<std::pair<W, linear_weight_function<W, Args...>>> functions) : _functions(functions) {
            static_assert(weight<W>::has_mult, "For a linear combination, the weight type W needs to implement W::multiply.");
        }

        constexpr result_type operator()(Args... args) const {
            if (_function) {
                return _function.value()(args...);
            }
            return std::accumulate(_functions.begin(), _functions.end(), weight<W>::zero(),
                    [&args...](const W& lhs, const std::pair<W, linear_weight_function<W, Args...>>& rhs) -> W {
                return weight<W>::add(lhs, weight<W>::multiply(rhs.first, rhs.second(args...)));
            });
        }
    };
    template <typename W, typename... Args> linear_weight_function(std::function<W(Args...)>) -> linear_weight_function<W, Args...>;
    template <typename W, typename... Args> linear_weight_function(std::vector<std::pair<W, linear_weight_function<W, Args...>>>) -> linear_weight_function<W, Args...>;

    template <typename W, typename... Args>
    class ordered_weight_function {
    private:
        const std::vector<linear_weight_function<W, Args...>> _functions;
    public:
        static_assert(is_weighted<weight<W>>);
        using result_type = std::vector<W>;

        explicit ordered_weight_function(std::vector<linear_weight_function<W, Args...>> functions) : _functions(functions) {}

        constexpr result_type operator()(Args... args) const {
            std::vector<W> result;
            std::transform(_functions.begin(), _functions.end(), std::back_inserter(result),
                    [&args...](const linear_weight_function<W, Args...>& f) -> W { return f(args...); });
            return result;
        }
    };
    template <typename W, typename... Args> ordered_weight_function(std::vector<linear_weight_function<W, Args...>>) -> ordered_weight_function<W, Args...>;

}

#endif //PDAAAL_WEIGHT_H
