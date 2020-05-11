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
 * File:   StackSizeWeight.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 07-05-2020.
 */

#include "StackSizeWeight.h"

namespace pdaaal {

    StackSizeWeight::elem_t StackSizeWeight::elem_t::append(const elem_t &lhs, const elem_t &rhs) {
        // Basically: elem_t{std::max(lhs.max, lhs.diff + rhs.max), lhs.diff + rhs.diff}
        // But without underflow.
        if (lhs.diff >= 0) {
            return elem_t{std::max(lhs.max, lhs.diff + rhs.max), lhs.diff + rhs.diff};
        } else {
            int32_t temp = lhs.diff + (int32_t)rhs.max;
            if (temp < 0) {
                return elem_t{lhs.max, lhs.diff + rhs.diff};
            } else {
                return elem_t{std::max(lhs.max, (uint32_t)temp), lhs.diff + rhs.diff};
            }
        }
    }

    std::ostream& operator<<(std::ostream& os, const StackSizeWeight& s) {
        os << '{';
        bool first = true;
        for (auto elem : s.elems) {
            if (!first) {
                os << ',';
            }
            os << elem;
            first = false;
        }
        os << '}';
        return os;
    }
    std::ostream& operator<<(std::ostream& os, const StackSizeWeight::elem_t& elem) {
        os << '(' << elem.max << ';' << elem.diff << ')';
        return os;
    }

    StackSizeWeight StackSizeWeight::extend(const StackSizeWeight &lhs, const StackSizeWeight &rhs) {
        assert(lhs.holds_invariant());
        assert(rhs.holds_invariant());
        StackSizeWeight result;
        auto it1 = lhs.elems.begin();
        auto it2 = rhs.elems.begin();
        auto last1 = lhs.elems.end();
        auto last2 = rhs.elems.end();

        int32_t min_diff = std::numeric_limits<int32_t>::max();

        while(it1 != last1) {
            if (it2 == last2) {
                for (; it1 != last1; ++it1) {
                    if (it1->diff < min_diff) {
                        min_diff = it1->diff;
                        result.elems.push_back(*it1);
                    }
                }
                assert(result.holds_invariant());
                return result;
            }
            if (it1->max < it2->max) {
                if (it1->diff < min_diff) {
                    min_diff = it1->diff;
                    result.elems.push_back(*it1);
                }
                it1++;
            } else if (it2->max < it1->max) {
                if (it2->diff < min_diff) {
                    min_diff = it2->diff;
                    result.elems.push_back(*it2);
                }
                it2++;
            } else { // it1->max == it2->max
                if (it1->diff < it2->diff) {
                    if (it1->diff < min_diff) {
                        min_diff = it1->diff;
                        result.elems.push_back(*it1);
                    }
                } else if (it2->diff < min_diff) {
                    min_diff = it2->diff;
                    result.elems.push_back(*it2);
                }
                it1++;
                it2++;
            }
        }
        for (; it2 != last2; ++it2) {
            if (it2->diff < min_diff) {
                min_diff = it2->diff;
                result.elems.push_back(*it2);
            }
        }
        assert(result.holds_invariant());
        return result;
    }

    StackSizeWeight StackSizeWeight::combine(const StackSizeWeight &lhs, const StackSizeWeight &rhs) {
        assert(lhs.holds_invariant());
        assert(rhs.holds_invariant());
        StackSizeWeight result;

        // TODO: This should be possible to do faster.
        for (auto elem1 : lhs.elems) {
            for (auto elem2 : rhs.elems) {
                result.elems.push_back(elem_t::append(elem1, elem2));
            }
        }
        result.make_minimal();

        assert(result.holds_invariant());
        return result;
    }

    void StackSizeWeight::make_minimal() {
        std::sort(elems.begin(), elems.end());
        auto it = elems.begin();
        auto result = it;
        while (++it != elems.end()) {
            if ((result->max < it->max && result->diff > it->diff) && ++result != it) {
                *result = *it;
            }
        }
        elems.erase(++result, elems.end());
    }

    bool StackSizeWeight::holds_invariant() const {
        // Nonempty.
        // max >= diff
        // Sorted by max
        // Minimal representation
        if (elems.empty()) return false;
        for (auto elem : elems) {
            if (elem.diff >= 0 && elem.max < (uint32_t)elem.diff) return false;
        }
        auto last = elems[0];
        for (size_t i = 1; i < elems.size(); ++i) {
            if (last.max >= elems[i].max || last.diff <= elems[i].diff) return false;
            last = elems[i];
        }
        return true;
    }


}
