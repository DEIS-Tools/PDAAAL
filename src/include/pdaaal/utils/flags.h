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
 * File:   flags.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 24-02-2022.
 */

#ifndef PDAAAL_FLAGS_H
#define PDAAAL_FLAGS_H

#include <type_traits>
#include <vector>
#include <cassert>
#include <cstdint>

namespace pdaaal::utils {

    template <std::size_t N> struct flags {
        static_assert(N <= 64, "At most 64 flags supported.");
        using flag_t = std::conditional_t<N<=8, uint8_t,
                        std::conditional_t<N<=16, uint16_t,
                         std::conditional_t<N<=32, uint32_t, uint64_t>>>;
        static constexpr void add(flag_t& mask, flag_t flag) { mask |= flag; }
        static constexpr void remove(flag_t& mask, flag_t flag) { mask &= ~flag; }
        static constexpr flag_t select(flag_t mask, flag_t flag) { return mask & flag; }
        static constexpr bool contains(flag_t mask, flag_t flag) { return select(mask, flag) == flag; }
        static constexpr bool is_single_flag(flag_t x) { return x != 0 && (x & (x - 1)) == 0; }

        template<std::size_t I>
        static constexpr flag_t flag() {
            static_assert(I <= N);
            if constexpr(I==0) {
                return 0;
            } else {
                return (flag_t)1<<(I-1);
            }
        }
        static constexpr flag_t no_flags = flag<0>();
        template<std::size_t I>
        static constexpr flag_t fill() {
            static_assert(I <= N);
            if constexpr(I==0) {
                return no_flags;
            } else if constexpr(I==N) {
                return flag<I>() | fill<I-1>();
            } else {
                return ((flag_t)1<<I)-(flag_t)1;
            }
        }

        static constexpr flag_t flag(std::size_t i) {
            assert(i <= N);
            return i == 0 ? 0 : (flag_t)1 << (i-1);
        }
        static constexpr flag_t fill(std::size_t n) {
            assert(n <= N);
            if (n == 0) return 0;
            if (n == N) {
                flag_t f = (flag_t)1 << (n-1);
                return f | (f - (flag_t)1);
            }
            return ((flag_t)1 << n) - (flag_t)1;
        }
        static std::vector<flag_t> split_to_single_flags(flag_t mask) {
            std::vector<flag_t> flags;
            for (std::size_t i = 0; i < N; ++i) {
                flag_t f = (flag_t)1<<i;
                if (contains(mask,f)) {
                    flags.emplace_back(f);
                }
            }
            return flags;
        }
    };

    template <std::size_t N>
    class flag_mask : flags<N> {
    public:
        using flag_t = typename flags<N>::flag_t;
        using flags<N>::flag;
        using flags<N>::fill;
        using flags<N>::is_single_flag;

        explicit constexpr flag_mask(flag_t mask) noexcept : _mask(mask) {};

        constexpr void got_flag(flag_t value) {
            flags<N>::remove(_mask, value);
        }
        [[nodiscard]] constexpr bool needs_flag(flag_t value) const {
            return flags<N>::contains(_mask, value);
        }
        [[nodiscard]] constexpr bool has_missing_flags() const {
            return _mask != flags<N>::no_flags;
        }
        std::vector<flag_t> get_missing_flags() const {
            return flags<N>::split_to_single_flags(_mask);
        }
    private:
        flag_t _mask;
    };

}

#endif //PDAAAL_FLAGS_H
