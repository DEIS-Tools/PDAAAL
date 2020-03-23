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
 *  Copyright Peter G. Jensen
 *  Modified by Morten K. Schou
 */

/*
 * File:   Reducer.cpp
 * Author: Peter G. Jensen <root@petergjoel.dk>
 *
 * Created on August 21, 2019, 2:47 PM
 */

#include "Reducer.h"
#include <cassert>

namespace pdaaal {

    bool Reducer::tos_t::update_state(const std::pair<bool, bool>& new_state) {
        auto pre = _in_waiting;
        switch (_in_waiting) {
            case TOS:
            case BOS:
            case NOT_IN_STACK:
                if (new_state.first)
                    _in_waiting = (waiting_t) (_in_waiting | TOS);
                if (new_state.second)
                    _in_waiting = (waiting_t) (_in_waiting | BOS);
                return (_in_waiting != pre);
                break;
            case BOTH:
                return false;
                break;
        }
        assert(false);
        return false;
    }

    bool Reducer::tos_t::empty_tos() const {
        return _tos.empty();
    }

    bool Reducer::tos_t::forward_stack(const tos_t& prev, size_t all_labels)
    {
        auto os = _stack.size();
        if (_stack.size() != all_labels) {
            if (prev._stack.size() == all_labels) {
                _stack = prev._stack;
            }
            else {
                auto lid = _stack.begin();
                auto pid = prev._stack.begin();
                while (pid != std::end(prev._stack)) {
                    while (lid != _stack.end() && *lid < *pid) ++lid;
                    if (lid != _stack.end() && *lid == *pid) {
                        ++pid;
                        continue;
                    }
                    if (lid == _stack.end()) break;
                    auto oid = pid;
                    while (pid != std::end(prev._stack) && *pid < *lid) ++pid;
                    lid = _stack.insert(lid, oid, pid);
                }
                _stack.insert(lid, pid, std::end(prev._stack));
            }
        }
        return os != _stack.size();
    }

}