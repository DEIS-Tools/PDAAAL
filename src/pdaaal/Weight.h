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

namespace pdaaal {

    template<typename W, typename = void> struct zero;
    template<typename W, typename = void> struct max;
    template<typename W, typename = void> struct add;
    template <typename T, typename = void> struct has_zero : std::false_type {};
    template <typename T> struct has_zero<T, std::void_t<decltype(std::declval<zero<T>>()())>> : std::true_type {};
    template <typename T> inline constexpr auto has_zero_v = has_zero<T>::value;
    template <typename T, typename = void> struct has_max : std::false_type {};
    template <typename T> struct has_max<T, std::void_t<decltype(std::declval<max<T>>()())>> : std::true_type {};
    template <typename T> inline constexpr auto has_max_v = has_max<T>::value;
    template <typename T, typename = void> struct has_add : std::false_type {};
    template <typename T> struct has_add<T, std::void_t<decltype(std::declval<add<T>>()(std::declval<T>(), std::declval<T>()))>> : std::true_type {};
    template <typename T> inline constexpr auto has_add_v = has_add<T>::value;
    template<typename W> inline constexpr auto is_weighted = !std::is_void_v<W> && has_zero_v<W> && has_max_v<W> && has_add_v<W>;
    // TODO is_weighted<W> should also require that boost::hash<W> is defined.

    template<typename W>
    struct zero<W, std::enable_if_t<std::is_arithmetic_v<W>>> {
        constexpr W operator()() const {
            return (W) 0;
        };
    };

    template<typename W>
    struct max<W, std::enable_if_t<std::numeric_limits<W>::is_specialized>> {
        constexpr W operator()() const {
            return std::numeric_limits<W>::max();
        };
    };

    template<typename W>
    struct add<W, std::enable_if_t<std::is_arithmetic_v<W>>> {
        constexpr W operator()(W lhs, W rhs) const {
            return lhs + rhs;
        };
    };

    template<typename Inner, std::size_t N>
    struct zero<std::array<Inner, N>, std::enable_if_t<has_zero_v<Inner>>> {
        constexpr std::array<Inner, N> operator()() const {
            std::array<Inner, N> arr{};
            arr.fill(zero<Inner>()());
            return arr;
        };
    };

    template<typename Inner, std::size_t N>
    struct max<std::array<Inner, N>, std::enable_if_t<has_max_v<Inner>>> {
        constexpr std::array<Inner, N> operator()() const {
            std::array<Inner, N> arr{};
            arr.fill(max<Inner>()());
            return arr;
        };
    };

    template<typename Inner, std::size_t N>
    struct add<std::array<Inner, N>, std::enable_if_t<has_add_v<Inner>>> {
        constexpr std::array<Inner, N> operator()(std::array<Inner, N> lhs, std::array<Inner, N> rhs) const {
            std::array<Inner, N> res{};
            for (size_t i = 0; i < N; ++i) {
                res[i] = lhs[i] + rhs[i];
            }
            return res;
        };
    };


    template<typename Inner>
    struct zero<std::vector<Inner>, std::enable_if_t<has_zero_v<Inner>>> {
        constexpr std::vector<Inner> operator()() const {
            std::vector<Inner> vec;
            return vec;
        };
    };

    template<typename Inner>
    struct max<std::vector<Inner>, std::enable_if_t<has_max_v<Inner>>> {
        constexpr std::vector<Inner> operator()() const {
            std::vector<Inner> vec{max<Inner>()()};
            return vec;
        };
    };

    template<typename Inner>
    struct add<std::vector<Inner>, std::enable_if_t<has_add_v<Inner>>> {
        constexpr std::vector<Inner> operator()(const std::vector<Inner>& lhs, const std::vector<Inner>& rhs) const {
            auto l_size = lhs.size();
            auto r_size = rhs.size();
            auto min = l_size < r_size ? l_size : r_size;
            auto max = l_size < r_size ? r_size : l_size;
            auto &large = max == l_size ? lhs : rhs;
            std::vector<Inner> res(max);
            add<Inner> add_i;
            size_t i = 0;
            for (; i < min; ++i) {
                res[i] = add_i(lhs[i], rhs[i]);
            }
            for (; i < max; ++i) {
                res[i] = large[i];
            }
            return res;
        };
    };


    template<typename W, typename = void> struct mult;
    template <typename T, typename = void> struct has_mult : std::false_type {};
    template <typename T> struct has_mult<T, std::void_t<decltype(std::declval<mult<T>>()(std::declval<T>(), std::declval<T>()))>> : std::true_type {};
    template <typename T> constexpr auto has_mult_v = has_mult<T>::value;
    template<typename W>
    struct mult<W, std::enable_if_t<std::is_arithmetic_v<W>>> {
        constexpr W operator()(W lhs, W rhs) const {
            return lhs * rhs;
        };
    };

    template <typename W, typename... Args>
    class linear_weight_function {
    private:
        const std::vector<std::pair<W, linear_weight_function<W, Args...>>> _functions;
        const std::optional<std::function<W(Args...)>> _function;
    public:
        static_assert(is_weighted<W>);
        using result_type = W;

        // A single function
        explicit linear_weight_function(std::function<W(Args...)> function) : _function(function) {}

        // Linear combination of functions
        explicit linear_weight_function(std::vector<std::pair<W, linear_weight_function<W, Args...>>> functions) : _functions(functions) {
            static_assert(has_mult_v<W>, "For a linear combination, he weight type needs to specialize mult<W>.");
        }

        constexpr result_type operator()(Args... args) const {
            if (_function) {
                return _function.value()(args...);
            }
            return std::accumulate(_functions.begin(), _functions.end(), zero<W>()(),
                    [&args...](const W& lhs, const std::pair<W, linear_weight_function<W, Args...>>& rhs) -> W {
                return add<W>()(lhs, mult<W>()(rhs.first, rhs.second(args...)));
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
        static_assert(is_weighted<W>);
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
