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

namespace pdaaal::internal {

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

    bool Reducer::tos_t::active(const Reducer::tos_t &prev, const labels_t &labels) {
        if (labels.empty()) {
            return false;
        }
        if (labels.wildcard()) {
            return true;
        }
        auto rit = labels.labels().begin();
        bool match = false;
        for (auto& s : prev._tos) {
            while (rit != std::end(labels.labels()) && *rit < s) ++rit;
            if (rit == std::end(labels.labels())) {
                break;
            }
            if (*rit == s) {
                match = true;
                break;
            }
        }
        return match;
    }

    std::pair<bool, bool>
    Reducer::tos_t::merge_pop(const Reducer::tos_t &prev, const labels_t &labels, bool dual_stack, size_t all_labels) {
        if (!active(prev, labels)) {
            return std::make_pair(false, false);
        }
        bool changed = false;
        bool stack_changed = false;
        if (!dual_stack) {
            if (_tos.size() != all_labels) {
                _tos.resize(all_labels);
                for (uint32_t i = 0; i < all_labels; ++i) _tos[i] = i;
                changed = true;
            }
        }
        else {
            // move stack->stack
            stack_changed |= forward_stack(prev, all_labels);
            // move stack -> TOS
            if (_tos.size() != all_labels) {
                if (prev._stack.size() == all_labels) {
                    changed = true;
                    _tos = prev._stack;
                }
                else {
                    auto it = _tos.begin();
                    for (auto s : prev._stack) {
                        while (it != std::end(_tos) && *it < s) ++it;
                        if (it != std::end(_tos) && *it == s) continue;
                        it = _tos.insert(it, s);
                        changed = true;
                    }
                }
            }
        }
        return std::make_pair(changed, stack_changed);
    }

    std::pair<bool, bool>
    Reducer::tos_t::merge_noop(const Reducer::tos_t &prev, const labels_t &labels, bool dual_stack, size_t all_labels) {
        if (labels.empty()) return std::make_pair(false, false);
        bool changed = false;
        {
            auto iit = _tos.begin();
            for (auto& symbol : labels.wildcard() ? prev._tos : labels.labels()) {
                while (iit != _tos.end() && *iit < symbol) ++iit;
                if (iit != _tos.end() && *iit == symbol) {
                    ++iit;
                    continue;
                }
                changed = true;
                iit = _tos.insert(iit, symbol);
                ++iit;
            }
        }
        bool stack_changed = false;
        if (dual_stack) {
            stack_changed |= forward_stack(prev, all_labels);
        }
        return std::make_pair(changed, stack_changed);
    }

    std::pair<bool, bool>
    Reducer::tos_t::merge_swap(const Reducer::tos_t &prev, uint32_t op_label, const labels_t &labels, bool dual_stack,
                               size_t all_labels) {
        if (!active(prev, labels))
            return std::make_pair(false, false); // we know that there is a match!
        bool changed = false;
        {
            auto lb = std::lower_bound(_tos.begin(), _tos.end(), op_label);
            if (lb == std::end(_tos) || *lb != op_label) {
                changed = true;
                _tos.insert(lb, op_label);
            }
        }
        bool stack_changed = false;
        if (dual_stack) {
            stack_changed |= forward_stack(prev, all_labels);
        }
        return std::make_pair(changed, stack_changed);
    }

    std::pair<bool, bool>
    Reducer::tos_t::merge_push(const Reducer::tos_t &prev, uint32_t op_label, const labels_t &labels, bool dual_stack,
                               size_t all_labels) {
        // similar to swap!
        auto changed = merge_swap(prev, op_label, labels, dual_stack, all_labels);
        if (dual_stack) {
            // but we also push all TOS labels down
            if (_stack.size() != all_labels) {
                if (prev._tos.size() == all_labels) {
                    changed.second = true;
                    _stack = prev._tos;
                }
                else {
                    auto it = _stack.begin();
                    for (auto& s : prev._tos) {
                        while (it != _stack.end() && *it < s) ++it;
                        if (it != std::end(_stack) && *it == s) continue;
                        it = _stack.insert(it, s);
                        changed.second = true;
                    }
                }
            }
        }
        return changed;
    }

}