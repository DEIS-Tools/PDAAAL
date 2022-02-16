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
    namespace details {
        struct weight_base {
            static constexpr bool is_weight = false;
            static constexpr bool is_signed = false;
        };
    }
    template<typename W, typename = void> struct weight : public details::weight_base {
        using type = W;
    };
    template<typename W>
    struct weight<W, std::enable_if_t<std::is_arithmetic_v<W> && std::numeric_limits<W>::is_specialized>> {
        using type = W;
        static constexpr bool is_weight = true;
        static constexpr bool is_signed = std::numeric_limits<W>::is_signed;
        static constexpr auto zero = []() -> type { return static_cast<type>(0); };
    };
    template<typename Inner, std::size_t N>
    struct weight<std::array<Inner, N>, std::enable_if_t<std::is_arithmetic_v<Inner> && std::numeric_limits<Inner>::is_specialized>> {
        using type = std::array<Inner, N>;
        static constexpr bool is_weight = true;
        static constexpr bool is_signed = weight<Inner>::is_signed;
        static constexpr auto zero = []() -> type {
            std::array<Inner, N> arr{};
            arr.fill(weight<Inner>::zero());
            return arr;
        };
    };
    template<typename Inner>
    struct weight<std::vector<Inner>, std::enable_if_t<std::is_arithmetic_v<Inner> && std::numeric_limits<Inner>::is_specialized>> {
        using type = std::vector<Inner>;
        static constexpr bool is_weight = true;
        static constexpr bool is_signed = weight<Inner>::is_signed;
        static constexpr auto zero = []() -> type { return type{}; }; // TODO: When C++20 arrives, use a constexpr vector instead
    };

    namespace details {
        template<typename W, bool maximize, typename = void> struct weight_impl : public weight<W> {};

        template<typename W, bool maximize>
        struct weight_impl<W, maximize, std::enable_if_t<std::is_arithmetic_v<W> && std::numeric_limits<W>::is_specialized>>
                : public weight<W> {
            using parent_t = weight<W>;
            using typename parent_t::type;
            static constexpr bool is_signed = parent_t::is_signed;
        private:
            static constexpr type bottom_val = maximize ? std::numeric_limits<type>::max() - (is_signed ? 0 : 1)
                                                        : std::numeric_limits<type>::min();
            static constexpr type max_val = (maximize && is_signed) ? std::numeric_limits<type>::min()
                                                                    : std::numeric_limits<type>::max();
        public:
            static constexpr auto bottom = []() -> type { return bottom_val; };
            static constexpr auto max = []() -> type { return max_val; };
            static constexpr bool less(type lhs, type rhs) {
                if constexpr (maximize) {
                    if constexpr (!is_signed) {
                        if (lhs == max_val) return false;
                        if (rhs == max_val) return true;
                    }
                    return rhs < lhs;
                } else {
                    return lhs < rhs;
                }
            }
            static constexpr type add(type lhs, type rhs) {
                if (lhs == max_val || rhs == max_val) return max_val;
                if constexpr (bottom_val != 0) {
                    if (lhs == bottom_val || rhs == bottom_val) return bottom_val;
                }
                return lhs + rhs;
            };
        };
        template<typename Inner, std::size_t N, bool maximize>
        struct weight_impl<std::array<Inner, N>, maximize, std::enable_if_t<std::is_arithmetic_v<Inner> && std::numeric_limits<Inner>::is_specialized>>
                : public weight<std::array<Inner, N>> {
            using parent_t = weight<std::array<Inner, N>>;
            using typename parent_t::type;

            static constexpr auto bottom = []() -> type {std::array<Inner, N> arr{}; arr.fill(weight_impl<Inner, maximize>::bottom()); return arr;};
            static constexpr auto max = []() -> type {std::array<Inner, N> arr{}; arr.fill(weight_impl<Inner, maximize>::max()); return arr;};
            static constexpr bool less(const type& lhs, const type& rhs) {
                return std::lexicographical_compare(
                        lhs.begin(), lhs.end(),
                        rhs.begin(), rhs.end(),
                        weight_impl<Inner, maximize>::less);
            }
            static constexpr type add(const type& lhs, const type& rhs) {
                std::array<Inner, N> res{};
                for (size_t i = 0; i < N; ++i) {
                    res[i] = weight_impl<Inner, maximize>::add(lhs[i], rhs[i]);
                }
                return res;
            };
        };
        template<typename Inner, bool maximize>
        struct weight_impl<std::vector<Inner>, maximize, std::enable_if_t<std::is_arithmetic_v<Inner> && std::numeric_limits<Inner>::is_specialized>>
                : public weight<std::vector<Inner>> {
            using parent_t = weight<std::vector<Inner>>;
            using typename parent_t::type;
            static constexpr bool is_signed = parent_t::is_signed;

            static constexpr auto bottom = []() -> type { return type{weight_impl<Inner, maximize>::bottom()}; };
            static constexpr auto max = []() -> type { return type{weight_impl<Inner, maximize>::max()}; }; // TODO: When C++20 arrives, use a constexpr vector instead
            static constexpr bool less(const type& lhs, const type& rhs) {
                if constexpr (!maximize && !is_signed) {
                    return lhs < rhs;
                } else {
                    auto f1 = lhs.begin(), f2 = rhs.begin();
                    auto l1 = lhs.end(), l2 = rhs.end();
                    bool exhaust1 = (f1 == l1);
                    bool exhaust2 = (f2 == l2);
                    for (; !exhaust1 || !exhaust2; exhaust1 = (++f1 == l1), exhaust2 = (++f2 == l2)) {
                        auto v1 = exhaust1 ? weight<Inner>::zero() : *f1; // Missing elements are implicitly zero.
                        auto v2 = exhaust2 ? weight<Inner>::zero() : *f2;
                        if (weight_impl<Inner, maximize>::less(v1, v2)) return true;
                        if (weight_impl<Inner, maximize>::less(v2, v1)) return false;
                        // if (auto c = weight<Inner, maximize>::compare(v1, v2); c != 0) return c;
                    }
                    return false; // return std::strong_ordering::equal;
                }
            }
            static constexpr type add(const type& lhs, const type& rhs) {
                const auto& [small, large] = std::minmax(lhs, rhs, [](const type& l, const type& r){ return l.size() < r.size(); });
                std::vector<Inner> result = large;
                std::transform(small.begin(), small.end(), large.begin(),
                               result.begin(), weight_impl<Inner, maximize>::add);
                return result;
            };
        };
    }

    template<typename W> using min_weight = details::weight_impl<W,false>;
    template<typename W> using max_weight = details::weight_impl<W,true>;

    template<typename W> inline constexpr auto is_weighted = W::is_weight; // TODO: Remove usage of is_weighted<W>. Just use W::is_weight directly instead.

    template <typename W, typename... Args>
    class linear_weight_function {
    private:
        const std::vector<std::pair<W, linear_weight_function<W, Args...>>> _functions;
        const std::optional<std::function<W(Args...)>> _function;
    public:
        static_assert(std::is_arithmetic_v<W> && std::numeric_limits<W>::is_specialized);
        using result_type = W;

        // A single function
        explicit linear_weight_function(std::function<W(Args...)> function) : _function(function) {}

        // Linear combination of functions
        explicit linear_weight_function(std::vector<std::pair<W, linear_weight_function<W, Args...>>> functions) : _functions(functions) {}

        constexpr result_type operator()(Args... args) const {
            if (_function) {
                return _function.value()(args...);
            }
            return std::accumulate(_functions.begin(), _functions.end(), 0,
                    [&args...](const W& lhs, const std::pair<W, linear_weight_function<W, Args...>>& rhs) -> W {
                return lhs + (rhs.first * rhs.second(args...));
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
        static_assert(std::is_arithmetic_v<W> && std::numeric_limits<W>::is_specialized);
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
