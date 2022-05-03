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

    template<typename Derived, typename Elem, bool enforce_set = true>
    class fixed_point_workset {
        bool _done = false;
        size_t _round_limit;
        size_t _rounds = 0;
        std::vector<Elem> _workset;
        std::vector<Elem> _next_workset;
    public:
        explicit fixed_point_workset(size_t round_limit = 0) : _round_limit(round_limit) { };

        template<bool change_is_bottom = false>
        bool step() {
            if (_workset.empty()) {
                ++_rounds;
                if (_next_workset.empty()) {
                    _done = true;
                    return false;
                }
                if constexpr (enforce_set) { // Remove duplicates, if this is not ensured somewhere else.
                    std::sort(_next_workset.begin(), _next_workset.end());
                    _next_workset.erase(std::unique(_next_workset.begin(), _next_workset.end()), _next_workset.end());
                }
                std::swap(_workset, _next_workset);
            }

            auto current = _workset.back();
            _workset.pop_back();

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
            return _next_workset.emplace_back(std::forward<Args>(args)...);
        }
        void set_round_limit(size_t new_round_limit) { _round_limit = new_round_limit; }
    };
}

#endif //PDAAAL_WORKSET_H
