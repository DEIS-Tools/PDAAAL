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
 * File:   workset.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-08-2021.
 */

#ifndef PDAAAL_WORKSET_H
#define PDAAAL_WORKSET_H

#include <cassert>
#include <stack>
#include <queue>

namespace pdaaal {

    template<typename Derived, typename Elem>
    class fixed_point_workset {
        static constexpr Elem next_round_elem = Derived::next_round_elem;
        bool _done = false;
        const size_t _round_limit;
        size_t _rounds = 0;
        std::deque<Elem> _workset;
    public:
        explicit fixed_point_workset(size_t round_limit) : _round_limit(round_limit) {
            _workset.emplace_back(next_round_elem);
        };

        template<bool change_is_bottom = false>
        bool step() {
            auto current = _workset.front();
            _workset.pop_front();

            if (current == next_round_elem) {
                ++_rounds;
                if (_workset.empty()) {
                    _done = true;
                    return false;
                }
                _workset.emplace_back(next_round_elem);
                current = _workset.front();
                _workset.pop_front();
                assert(current != next_round_elem);
            }
            if (!static_cast<Derived*>(this)->template step_with<change_is_bottom>(std::move(current))) {
                _done = true;
                return false;
            }
            return true;
        }
        void finalize() {
            _rounds = 0; // Reset _round count and run again, this time changes gives -inf weight.
            while (!done()) {
                step<true>();
            }
        }
        [[nodiscard]] bool done() const {
            return _done || _rounds == _round_limit;
        }
        void run() {
            while(!done()) {
                step();
            }
            finalize();
        }
    protected:
        template<typename... Args>
        auto emplace(Args&&... args){
            return _workset.emplace_back(std::forward<Args>(args)...);
        }
    };
}

#endif //PDAAAL_WORKSET_H
