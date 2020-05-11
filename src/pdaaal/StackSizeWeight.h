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
 * File:   StackSizeWeight.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 05-05-2020.
 */

#ifndef PDAAAL_STACKSIZEWEIGHT_H
#define PDAAAL_STACKSIZEWEIGHT_H

#include <cassert>
#include <limits>
#include <type_traits>
#include <array>
#include <algorithm>
#include <numeric>
#include <iostream>

namespace pdaaal {

    struct StackSizeWeight {

        struct elem_t {
            elem_t() = default;
            constexpr elem_t(uint32_t max, int32_t diff) : max(max), diff(diff) {};

            uint32_t max;
            int32_t diff;

            static elem_t append(const elem_t& lhs, const elem_t& rhs);

            bool operator<(const elem_t& other) const {
                return max < other.max || (max == other.max && diff < other.diff); // Lexicographical.
            }
            bool operator==(const elem_t& other) const {
                return max == other.max && diff == other.diff;
            }
            bool operator!=(const elem_t& other) const {
                return !(*this == other);
            }
            friend std::ostream& operator<<(std::ostream& os, const elem_t& elem);

        };

        StackSizeWeight() = default;
        explicit StackSizeWeight(elem_t&& elem) : elems{elem} {};

        [[nodiscard]] StackSizeWeight add(const StackSizeWeight& other) const {
            return StackSizeWeight::combine(*this, other);
        }
        [[nodiscard]] StackSizeWeight min(const StackSizeWeight& other) const {
            return StackSizeWeight::extend(*this, other);
        }
        bool operator<(const StackSizeWeight& other) const {
            return std::lexicographical_compare(elems.begin(), elems.end(), other.elems.begin(), other.elems.end());
        }
        bool operator==(const StackSizeWeight& other) const {
            return std::equal(elems.begin(), elems.end(), other.elems.begin(), other.elems.end());
        }
        bool operator!=(const StackSizeWeight& other) const {
            return !(*this == other);
        }

        [[nodiscard]] uint32_t final_value() const {
            assert(holds_invariant());
            return elems[0].max;
        }

        static StackSizeWeight extend(const StackSizeWeight& lhs, const StackSizeWeight& rhs);
        static StackSizeWeight combine(const StackSizeWeight& lhs, const StackSizeWeight& rhs);

        void make_minimal();
        [[nodiscard]] bool holds_invariant() const;

        static StackSizeWeight max() { return StackSizeWeight{elem_t{std::numeric_limits<uint32_t>::max(), std::numeric_limits<int32_t>::max()}}; }
        static StackSizeWeight zero() { return StackSizeWeight{elem_t{0,0}}; }
        static StackSizeWeight pop() { return StackSizeWeight{elem_t{0,-1}}; }
        static StackSizeWeight push() { return StackSizeWeight{elem_t{1,1}}; }
        static StackSizeWeight swap() { return zero(); }

        friend std::ostream& operator<<(std::ostream& os, const StackSizeWeight& s);
    private:
        std::vector<elem_t> elems;
    };
}

#endif //PDAAAL_STACKSIZEWEIGHT_H
