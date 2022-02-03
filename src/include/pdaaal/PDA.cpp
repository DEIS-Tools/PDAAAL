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
 */

/*
 * File:   PDA.cpp
 * Author: Peter G. Jensen <root@petergjoel.dk>
 * Modified by: Morten K. Schou
 *
 * Created on 19-02-2020.
 */

#include <pdaaal/PDA.h>
#include <cassert>

namespace pdaaal {

    void labels_t::merge(bool wildcard, const std::vector<uint32_t>& other) {
        if (_wildcard) return;
        if (wildcard) {
            _wildcard = true;
            _labels.clear();
            return;
        }

        assert(std::is_sorted(_labels.begin(), _labels.end()));
        assert(std::is_sorted(other.begin(), other.end()));
        std::vector<uint32_t> temp_labels;
        temp_labels.swap(_labels);
        std::set_union(temp_labels.begin(), temp_labels.end(),
                       other.begin(), other.end(),
                       std::back_inserter(_labels));
        assert(std::is_sorted(_labels.begin(), _labels.end()));
    }

    bool labels_t::intersect(const std::vector<uint32_t>& other, size_t all_labels) {
        if (other.size() == all_labels) {
            //            std::cerr << "EMPY 1" << std::endl;
            return !empty();
        }
        if (_wildcard) {
            if (other.size() == all_labels)
                return empty();
            else {
                _labels = other;
                _wildcard = false;
            }
        }
        else {
            auto fit = other.begin();
            size_t bit = 0;
            for (size_t nl = 0; nl < _labels.size(); ++nl) {
                while (fit != std::end(other) && *fit < _labels[nl]) ++fit;
                if (fit == std::end(other)) break;
                if (*fit == _labels[nl]) {
                    _labels[bit] = _labels[nl];
                    ++bit;
                }
            }
            _labels.resize(bit);
            assert(_labels.size() != all_labels);
        }
        return !empty();
    }

    bool labels_t::noop_pre_filter(const std::set<uint32_t>& usefull) { // TODO: Remove this. post* (and to some extent pre*) is optimized to handle wildcard labels well. This ruins that.
        if (_wildcard) {
            _labels.insert(_labels.begin(), usefull.begin(), usefull.end());
            _wildcard = false;
            return true;
        }
        else {
            auto it = _labels.begin();
            auto wit = it;
            auto uit = usefull.begin();
            while (it != _labels.end()) {
                while (uit != std::end(usefull) && *uit < *it) ++uit;
                if (uit != std::end(usefull) && *uit == *it) {
                    *wit = *it;
                    ++wit;
                    ++it;
                }
                else {
                    ++it;
                }
            }
            if (wit != it) {
                _labels.resize(wit - _labels.begin());
                return true;
            }
        }
        return false;
    }

}