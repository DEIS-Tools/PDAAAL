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
 * File:   TypedPDA.h
 * Author: Peter G. Jensen <root@petergjoel.dk>
 *
 * Created on July 23, 2019, 1:34 PM
 */

#ifndef TPDA_H
#define TPDA_H

#include "WPDA.h"

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <cassert>
#include <iostream>
#include <ptrie/ptrie_map.h>

namespace pdaaal {

    template<typename T, typename W = void, typename C = std::less<W>>
    class TypedPDA : public WPDA<W, C> {
    public:
        struct tracestate_t {
            size_t _pdastate = 0;
            std::vector<T> _stack;
        };

    public:
        explicit TypedPDA(const std::unordered_set<T> &all_labels) {
            std::set<T> sorted(all_labels.begin(), all_labels.end());
            for (auto &l : sorted) {
#ifndef NDEBUG
                auto r =
#endif
                _label_map.insert(l);
#ifndef NDEBUG
                assert(r.first);
#endif
            }
        }

        [[nodiscard]] virtual size_t number_of_labels() const {
            return _label_map.size();
        }

        T get_symbol(size_t i) const {
            T res;
            _label_map.unpack(i, &res);
            return res;
        }

        template<typename... Args>
        void add_rules(Args &&... args) {
            add_rules_<W>(std::forward<Args>(args)...);
        }

        template<typename... Args>
        void add_rule(Args &&... args) {
            add_rule_<W>(std::forward<Args>(args)...);
        }

        std::vector<uint32_t> encode_pre(const std::vector<T> &pre) const {
            std::vector<uint32_t> tpre(pre.size());
            for (size_t i = 0; i < pre.size(); ++i) {
                auto &p = pre[i];
                auto res = _label_map.exists(p);
                if (!res.first) {
                    //std::cerr << (int) p.type() << ", " << (int) p.mask() << ", " << (int) p.value() << std::endl;
                    //std::cerr << "SIZE " << _label_map.size() << std::endl;
                    assert(false);
                }
                tpre[i] = res.second;
            }
            return tpre;
        }

    protected:
        uint32_t find_labelid(op_t op, T label) const {
            if (op != POP && op != NOOP) {
                auto res = _label_map.exists(label);
                if (res.first) {
                    return res.second;
                } else {
                    throw std::logic_error("Couldnt find label during construction");
                }
            }
            return std::numeric_limits<uint32_t>::max();
        }

        void
        add_rules_impl(size_t from, rule_t <W, C> rule, bool negated, const std::vector<T> &labels, bool negated_pre,
                       const std::vector<T> &pre) {
            auto tpre = encode_pre(pre);
            if (negated) {
                size_t last = 0;
                for (auto &l : labels) {
                    auto res = _label_map.exists(l);
                    assert(res.first);
                    for (; last < res.second; ++last) {
                        rule._op_label = last;
                        _add_rule(from, rule, negated_pre, tpre);
                    }
                    ++last;
                }
                for (; last < _label_map.size(); ++last) {
                    rule._op_label = last;
                    _add_rule(from, rule, negated_pre, tpre);
                }
            } else {
                for (auto &s : labels) {
                    auto lid = find_labelid(rule._operation, s);
                    rule._op_label = lid;
                    _add_rule(from, rule, negated_pre, tpre);
                }
            }
        }

    private:
        template<typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_rules_(size_t from, size_t to, op_t op, bool negated, const std::vector<T> &labels, bool negated_pre,
                        const std::vector<T> &pre) {
            add_rules_impl(from, {to, op}, negated, labels, negated_pre, pre);
        }

        template<typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_rules_(size_t from, size_t to, op_t op, WT weight, bool negated, const std::vector<T> &labels,
                        bool negated_pre, const std::vector<T> &pre) {
            add_rules_impl(from, {to, op, weight}, negated, labels, negated_pre, pre);
        }

        template<typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, T label, bool negated, const std::vector<T> &pre) {
            auto lid = find_labelid(op, label);
            auto tpre = encode_pre(pre);
            this->add_untyped_rule(from, to, op, lid, negated, tpre);
        }

        template<typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, T label, WT weight, bool negated, const std::vector<T> &pre) {
            auto lid = find_labelid(op, label);
            auto tpre = encode_pre(pre);
            this->add_untyped_rule(from, to, op, lid, weight, negated, tpre);
        }

        template<typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, T label, bool negated, T pre) {
            std::vector<T> _pre{pre};
            add_rule(from, to, op, label, negated, _pre);
        }

        template<typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, T label, WT weight, bool negated, T pre) {
            std::vector<T> _pre{pre};
            add_rule(from, to, op, label, weight, negated, _pre);
        }

        ptrie::set_stable<T> _label_map;

    };
}
#endif /* TPDA_H */

