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

#include <pdaaal/PDA.h>
#include <pdaaal/utils/ptrie_interface.h>

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <cassert>
#include <iostream>

namespace pdaaal {

    // Allow conditionally enabling state mapping. (E.g. string -> size_t map vs. just directly use size_t states.)
    template<typename state_t>
    struct state_mapping {
        state_t get_state(size_t id) {
            assert(id < _state_map.size());
            return _state_map.at(id);
        }
    protected:
        utils::ptrie_set<state_t> _state_map;
    };
    struct no_state_mapping {};

    template<typename label_t, typename W = weight<void>, fut::type Container = fut::type::vector, typename state_t = size_t, bool skip_state_mapping = std::is_same_v<state_t,size_t>>
    class TypedPDA : public PDA<W, Container>, public std::conditional_t<skip_state_mapping, no_state_mapping, state_mapping<state_t>> {
    protected:
        using impl_rule_t = typename PDA<W, Container>::rule_t; // This rule type is used internally.
        static_assert(!skip_state_mapping || std::is_same_v<state_t,size_t>, "When skip_state_mapping==true, you must use state_t=size_t");
        using StateMapOrEmpty = std::conditional_t<skip_state_mapping, no_state_mapping, state_mapping<state_t>>;
    private:
        template <typename WT, typename = void> struct rule_t_;
        template <typename WT>
        struct rule_t_<WT, std::enable_if_t<!is_weighted<WT>>> {
            size_t _from = std::numeric_limits<size_t>::max();
            label_t _pre;
            size_t _to = std::numeric_limits<size_t>::max();
            op_t _op = POP;
            label_t _op_label;
        };
        template <typename WT>
        struct rule_t_<WT, std::enable_if_t<is_weighted<WT>>> {
            size_t _from = std::numeric_limits<size_t>::max();
            label_t _pre;
            size_t _to = std::numeric_limits<size_t>::max();
            op_t _op = POP;
            label_t _op_label;
            typename WT::type _weight;
        };
    public:
        using rule_t = rule_t_<W>; // This rule type can be used by users of the library.

    public:
        struct tracestate_t {
            size_t _pdastate = 0;
            std::vector<label_t> _stack;
        };

    public:
        template<fut::type OtherContainer>
        explicit TypedPDA(TypedPDA<label_t,W,OtherContainer,state_t,skip_state_mapping>&& other_pda)
        : PDA<W,Container>(std::move(static_cast<PDA<W,OtherContainer>&>(other_pda))),
          StateMapOrEmpty(std::move(static_cast<StateMapOrEmpty&>(other_pda))),
          _label_map(other_pda.move_label_map()) {}

        explicit TypedPDA(const std::unordered_set<label_t>& all_labels) {
            std::set<label_t> sorted(all_labels.begin(), all_labels.end());
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
        TypedPDA() = default;

        auto move_label_map() { return std::move(_label_map); }

        [[nodiscard]] virtual size_t number_of_labels() const {
            return _label_map.size();
        }

        label_t get_symbol(size_t id) {
            assert(id < _label_map.size());
            return _label_map.at(id);
        }

        template<typename... Args>
        void add_rules(Args &&... args) {
            add_rules_<W>(std::forward<Args>(args)...);
        }

        void add_rule(const rule_t& r) {
            if constexpr (is_weighted<W>) {
                add_rule(r._from, r._to, r._op, r._op_label, r._pre, r._weight);
            } else {
                add_rule(r._from, r._to, r._op, r._op_label, r._pre);
            }
        }
        void add_wildcard_rule(const rule_t& r) {
            auto lid = find_labelid(r._op, r._op_label);
            if constexpr (is_weighted<W>) {
                this->add_untyped_rule(r._from, r._to, r._op, lid, r._weight, true, std::vector<uint32_t>());
            } else {
                this->add_untyped_rule(r._from, r._to, r._op, lid, true, std::vector<uint32_t>());
            }
        }

        template<typename... Args>
        void add_rule(Args &&... args) {
            add_rule_<W>(std::forward<Args>(args)...);
        }

        std::vector<uint32_t> encode_pre(const std::vector<label_t> &pre) const {
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

        // Enable incremental construction of label (and state) set.
        std::pair<bool,size_t> exists_label(const label_t& label) const {
            return _label_map.exists(label);
        }
        [[nodiscard]] std::pair<bool,size_t> exists_state(const state_t& state) const {
            if constexpr (skip_state_mapping) {
                return std::make_pair(state < this->states().size(), state);
            } else {
                return this->_state_map.exists(state);
            }
        }
        uint32_t insert_label(const label_t& label) {
            return _label_map.insert(label).second;
        }
        size_t insert_state(const state_t& state) {
            if constexpr (skip_state_mapping) {
                return state;
            } else {
                return this->_state_map.insert(state).second;
            }
        }
        void add_rule_detail(size_t from, typename PDA<W>::rule_t r, bool negated, const std::vector<uint32_t>& pre) {
            this->add_untyped_rule_impl(from, r, negated, pre);
        }

    protected:
        uint32_t find_labelid(op_t op, label_t label) const {
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

        void add_rules_impl(size_t from, impl_rule_t rule, bool negated, const std::vector<label_t> &labels, bool negated_pre, const std::vector<label_t> &pre) {
            auto tpre = encode_pre(pre);
            if (negated) {
                size_t last = 0;
                for (auto &l : labels) {
                    auto res = _label_map.exists(l);
                    assert(res.first);
                    for (; last < res.second; ++last) {
                        rule._op_label = last;
                        this->add_untyped_rule_impl(from, rule, negated_pre, tpre);
                    }
                    ++last;
                }
                for (; last < _label_map.size(); ++last) {
                    rule._op_label = last;
                    this->add_untyped_rule_impl(from, rule, negated_pre, tpre);
                }
            } else {
                for (auto &s : labels) {
                    auto lid = find_labelid(rule._operation, s);
                    rule._op_label = lid;
                    this->add_untyped_rule_impl(from, rule, negated_pre, tpre);
                }
            }
        }

    private:
        template<typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_rules_(size_t from, size_t to, op_t op, bool negated, const std::vector<label_t> &labels, bool negated_pre,
                        const std::vector<label_t> &pre) {
            add_rules_impl(from, {to, op}, negated, labels, negated_pre, pre);
        }

        template<typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_rules_(size_t from, size_t to, op_t op, bool negated, const std::vector<label_t> &labels, bool negated_pre,
                        const std::vector<label_t> &pre, typename WT::type weight = WT::zero()) {
            add_rules_impl(from, {to, op, weight}, negated, labels, negated_pre, pre);
        }

        template<typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, label_t label, bool negated, const std::vector<label_t> &pre) {
            auto lid = find_labelid(op, label);
            auto tpre = encode_pre(pre);
            this->add_untyped_rule(from, to, op, lid, negated, tpre);
        }

        template<typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, label_t label, bool negated, const std::vector<label_t> &pre, typename WT::type weight = WT::zero()) {
            auto lid = find_labelid(op, label);
            auto tpre = encode_pre(pre);
            this->add_untyped_rule(from, to, op, lid, weight, negated, tpre);
        }

        template<typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, label_t label, label_t pre) {
            std::vector<label_t> _pre{pre};
            add_rule(from, to, op, label, false, _pre);
        }

        template<typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_rule_(size_t from, size_t to, op_t op, label_t label, label_t pre, typename WT::type weight = WT::zero()) {
            std::vector<label_t> _pre{pre};
            add_rule(from, to, op, label, false, _pre, weight);
        }

        utils::ptrie_set<label_t> _label_map;

    };
}
#endif /* TPDA_H */

