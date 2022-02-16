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
 * File:   AutomatonPath.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 24-08-2021.
 */

#ifndef PDAAAL_AUTOMATONPATH_H
#define PDAAAL_AUTOMATONPATH_H

#include <cassert>
#include <vector>

namespace pdaaal {

    class AutomatonPath {
        size_t _end{};
        std::vector<std::pair<uint32_t,size_t>> _edges{};
    public:
        explicit AutomatonPath(size_t end) : _end(end) {};
        AutomatonPath(const std::vector<size_t>& path, const std::vector<uint32_t>& stack)
        : _end(path.back()) {
            assert(path.size() == stack.size() + 1);
            _edges.reserve(stack.size());
            auto path_it = path.crbegin()+1;
            for (auto stack_it = stack.crbegin(); stack_it != stack.crend(); ++stack_it, ++path_it) {
                _edges.emplace_back(*stack_it, *path_it);
            }
        }

        [[nodiscard]] bool empty() const {
            return _edges.empty();
        }
        [[nodiscard]] size_t front_state() const {
            return _edges.empty() ? _end : _edges.back().second;
        }
        [[nodiscard]] std::tuple<size_t,uint32_t,size_t> front_edge() const {
            assert(!_edges.empty());
            return {_edges.back().second, _edges.back().first, _edges.size() > 1 ? _edges[_edges.size() - 2].second : _end};
        }
        void pop() {
            assert(!_edges.empty());
            _edges.pop_back();
        }
        void emplace(size_t from, uint32_t label) {
            _edges.emplace_back(label, from);
        }
        [[nodiscard]] std::vector<uint32_t> stack() const {
            std::vector<uint32_t> stack;
            stack.reserve(_edges.size());
            for (auto it = _edges.crbegin(); it != _edges.crend(); ++it) {
                stack.emplace_back(it->first);
            }
            return stack;
        }
        [[nodiscard]] std::pair<std::vector<size_t>, std::vector<uint32_t>> get_path_and_stack() const {
            std::vector<size_t>   path;  path.reserve(_edges.size() + 1);
            std::vector<uint32_t> stack; stack.reserve(_edges.size());
            for (auto it = _edges.crbegin(); it != _edges.crend(); ++it) {
                if (it->first != std::numeric_limits<uint32_t>::max()) {
                    path.push_back(it->second);
                    stack.push_back(it->first);
                }
            }
            path.push_back(_end);
            return std::make_pair(path, stack);
        }
    };

}

#endif //PDAAAL_AUTOMATONPATH_H
