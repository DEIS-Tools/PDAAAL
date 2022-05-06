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
 * File:   more_algorithms.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 06-05-2022.
 */

#ifndef PDAAAL_MORE_ALGORITHMS_H
#define PDAAAL_MORE_ALGORITHMS_H

#include <utility>
#include <vector>
#include <algorithm>

namespace pdaaal {
    // Auxiliary function... Why is something like this not in <algorithm>??.
    template<typename T>
    inline bool is_disjoint(const std::vector<T>& a, const std::vector<T>& b) {
        // Inspired by: https://stackoverflow.com/a/29123390
        assert(std::is_sorted(a.begin(), a.end()));
        assert(std::is_sorted(b.begin(), b.end()));
        auto it_a = a.begin();
        auto it_b = b.begin();
        while (it_a != a.end() && it_b != b.end()) {
            if (*it_a < *it_b) {
                it_a = std::lower_bound(++it_a, a.end(), *it_b);
            } else if (*it_b < *it_a) {
                it_b = std::lower_bound(++it_b, b.end(), *it_a);
            } else {
                return false;
            }
        }
        return true;
    }

    template<typename T, typename U, typename Compare>
    inline bool is_disjoint(const std::vector<T>& a, const std::vector<U>& b, Compare comp) {
        // Inspired by: https://stackoverflow.com/a/29123390
        assert(std::is_sorted(a.begin(), a.end(), comp));
        assert(std::is_sorted(b.begin(), b.end(), comp));
        auto it_a = a.begin();
        auto it_b = b.begin();
        while (it_a != a.end() && it_b != b.end()) {
            if (comp(*it_a, *it_b)) {
                it_a = std::lower_bound(++it_a, a.end(), *it_b, comp);
            } else if (comp(*it_b, *it_a)) {
                it_b = std::lower_bound(++it_b, b.end(), *it_a, comp);
            } else {
                return false;
            }
        }
        return true;
    }

    template<typename A, typename B, typename Comp = std::less<A>>
    struct CompFirst {
        Comp comp{};
        constexpr bool operator()(const std::pair<A, B>& lhs, const std::pair<A, B>& rhs) const { return comp(lhs.first,rhs.first); }
        constexpr bool operator()(const std::pair<A, B>& lhs, const A& rhs) const { return comp(lhs.first,rhs); }
        constexpr bool operator()(const A& lhs, const std::pair<A, B>& rhs) const { return comp(lhs,rhs.first); }
        constexpr bool operator()(const A& lhs, const A& rhs) const { return comp(lhs,rhs); }
    };
    template<typename A, typename B>
    struct EqFirst : CompFirst<A,B,std::equal_to<A>> {};

    template<typename A, typename Comp = std::less<A>>
    struct CompSwapArgs {
        Comp comp{};
        constexpr bool operator()(const A& lhs, const A& rhs) const { return comp(rhs,lhs); }
    };

    /**
     * A priority queue that doesn't pop the same element twice.
     */
    template<typename Weight, typename Elem, typename Comp = std::less<Weight>> // Default to max-heap, use CompSwapArgs<Weight,std::less<Weight>> to make it a min-heap.
    class priority_set {
        std::priority_queue<std::pair<Weight,Elem>, std::vector<std::pair<Weight,Elem>>, CompFirst<Weight,Elem,Comp>> _queue{};
        std::unordered_set<Elem> _seen{};
    public:
        template<typename... Args>
        void emplace(const Weight& weight, Args... args) {
            _queue.emplace(weight, Elem(std::forward<Args>(args)...));
        }
        template<typename... Args>
        void emplace(Weight&& weight, Args... args) {
            _queue.emplace(std::move(weight), Elem(std::forward<Args>(args)...));
        }
        void pop() {
            assert(!_seen.contains(_queue.top().second)); // This should be the class invariant.
            _seen.emplace(_queue.top().second);
            _queue.pop_back();
            while(_seen.contains(_queue.top().second)) {
                _queue.pop_back(); // Remove elems already processed.
            }
        }
        [[nodiscard]] auto top() const {
            return _queue.top();
        }
        [[nodiscard]] bool empty() const {
            return _queue.empty();
        }

        // Make it look like a vector<Elem> queue.
        [[nodiscard]] auto back() const {
            return top().second;
        }
        void pop_back() {
            return pop();
        }
    };
}

#endif //PDAAAL_MORE_ALGORITHMS_H
