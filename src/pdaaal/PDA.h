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
 * File:   PDA.h
 * Author: Peter G. Jensen <root@petergjoel.dk>
 * Modified by: Morten K. Schou
 *
 * Created on 19-02-2020.
 */

#ifndef PDAAAL_PDA_H
#define PDAAAL_PDA_H

#include "Weight.h"

#include <cinttypes>
#include <vector>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <functional>
#include <type_traits>
#include <boost/container_hash/hash.hpp>

namespace pdaaal {

    struct labels_t {
    private:
        bool _wildcard = false;
        std::vector<uint32_t> _labels;

    public:

        [[nodiscard]] bool wildcard() const {
            return _wildcard;
        }

        [[nodiscard]] const std::vector<uint32_t> &labels() const {
            return _labels;
        }

        [[nodiscard]] bool empty() const {
            return !_wildcard && _labels.empty();
        }

        [[nodiscard]] bool contains(uint32_t label) const {
            if (_wildcard) return true;
            auto lb = std::lower_bound(_labels.begin(), _labels.end(), label);
            return lb != std::end(_labels) && *lb == label;
        }

        void clear() {
            _wildcard = false;
            _labels.clear();
        }

        void merge(bool negated, const std::vector<uint32_t> &other, size_t all_labels);

        bool intersect(const std::vector<uint32_t> &tos, size_t all_labels);

        bool noop_pre_filter(const std::set<uint32_t> &usefull);
    };

    enum op_t {
        PUSH = 1,
        POP = 2,
        SWAP = 4,
        NOOP = 8
    };


    // Define rules with and without weights.
    template<typename W, typename C, typename = void>
    struct rule_t;

    template<typename W, typename C>
    struct rule_t<W, C, std::enable_if_t<!is_weighted<W>>> {
        size_t _to = 0;
        op_t _operation = PUSH;
        uint32_t _op_label = 0;
        labels_t _labels;

        bool operator<(const rule_t<W, C> &other) const {
            if (_to != other._to)
                return _to < other._to;
            if (_op_label != other._op_label)
                return _op_label < other._op_label;
            return _operation < other._operation;
        }

        bool operator==(const rule_t<W, C> &other) const {
            return _to == other._to && _op_label == other._op_label && _operation == other._operation;
        }

        bool operator!=(const rule_t<W, C> &other) const {
            return !(*this == other);
        }
        struct hasher {
            size_t operator()(const pdaaal::rule_t<W,C>& rule) const {
                size_t seed = 0;
                boost::hash_combine(seed, rule._to);
                boost::hash_combine(seed, rule._op_label);
                boost::hash_combine(seed, rule._operation);
                return seed;
            }
        };
    };

    template<typename W, typename C>
    struct rule_t<W, C, std::enable_if_t<is_weighted<W>>> {
        size_t _to = 0;
        op_t _operation = PUSH;
        W _weight = zero<W>()();
        uint32_t _op_label = 0;
        labels_t _labels;

        bool operator<(const rule_t<W, C> &other) const {
            if (_to != other._to)
                return _to < other._to;
            if (_op_label != other._op_label)
                return _op_label < other._op_label;
            if (_operation != other._operation)
                return _operation < other._operation;
            C comp;
            return comp(_weight, other._weight);
        }

        bool operator==(const rule_t<W, C> &other) const {
            return _to == other._to && _op_label == other._op_label && _operation == other._operation &&
                   _weight == other._weight;
        }

        bool operator!=(const rule_t<W, C> &other) const {
            return !(*this == other);
        }
        struct hasher {
            size_t operator()(const pdaaal::rule_t<W,C>& rule) const {
                size_t seed = 0;
                boost::hash_combine(seed, rule._to);
                boost::hash_combine(seed, rule._op_label);
                boost::hash_combine(seed, rule._operation);
                boost::hash_combine(seed, rule._weight);
                return seed;
            }
        };
    };

    template <typename W, typename C>
    class TempPDA {
    public:
        struct state_t {
            std::unordered_set<rule_t<W,C>, typename rule_t<W,C>::hasher> _rules;
            std::vector<size_t> _pre_states;
        };

    public:
        [[nodiscard]] virtual size_t number_of_labels() const = 0;

    protected:
        // Handle both weighted and unweighted rules appropriately.
        template <typename... Args>
        void add_untyped_rule(Args&&... args) {
            add_untyped_rule_<W>(std::forward<Args>(args)...);
        }
        void add_untyped_rule_impl(size_t from, rule_t<W,C> r, bool negated, const std::vector<uint32_t>& pre) {
            auto mm = std::max(from, r._to);
            if (mm >= _states.size()) {
                _states.resize(mm + 1);
            }

            auto rule = _states[from]._rules.emplace(r);
            rule->_labels.merge(negated, pre, number_of_labels());
            auto& prestates = _states[r._to]._pre_states;
            auto lpre = std::lower_bound(prestates.begin(), prestates.end(), from);
            if (lpre == std::end(prestates) || *lpre != from) {
                prestates.insert(lpre, from);
            }
        }

    private:
        template <typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_untyped_rule_(size_t from, size_t to, op_t op, uint32_t label, bool negated, const std::vector<uint32_t>& pre) {
            add_untyped_rule_impl(from, {to, op, label}, negated, pre);
        }
        template <typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_untyped_rule_(size_t from, size_t to, op_t op, uint32_t label, WT weight, bool negated, const std::vector<uint32_t>& pre) {
            add_untyped_rule_impl(from, {to, op, weight, label}, negated, pre);
        }

        std::vector<state_t> _states;
    };

    template <typename W, typename C>
    class PDA {
    public:
        struct state_t {
            std::vector<rule_t<W,C>> _rules;
            std::vector<size_t> _pre_states;
            explicit state_t(typename TempPDA<W,C>::state_t&& temp_state)
                    : _rules(std::make_move_iterator(temp_state._rules.begin()), std::make_move_iterator(temp_state._rules.end())),
                      _pre_states(std::move(temp_state._pre_states)) {
                std::sort(_rules.begin(), _rules.end());
            }
            state_t() = default;
        };

    public:
        explicit PDA(TempPDA<W,C>&& temp_pda)
        : _states(std::make_move_iterator(temp_pda._states.begin()), std::make_move_iterator(temp_pda._states.end())) {};
        PDA() = default;

        [[nodiscard]] virtual size_t number_of_labels() const = 0;
        const std::vector<state_t>& states() const {
            return _states;
        }
        std::vector<state_t>& states_mutable() {
            return _states;
        }
        void clear_state(size_t s) {
            _states[s]._rules.clear();
            for (auto& p : _states[s]._pre_states) {
                auto rit = _states[p]._rules.begin();
                auto wit = rit;
                for (; rit != std::end(_states[p]._rules); ++rit) {
                    if (rit->_to != s) {
                        if (wit != rit)
                            *wit = *rit;
                        ++wit;
                    }
                }
                _states[p]._rules.resize(wit - std::begin(_states[p]._rules));
            }
            _states[s]._pre_states.clear();
        }

    protected:

        // Handle both weighted and unweighted rules appropriately.
        template <typename... Args>
        void add_untyped_rule(Args&&... args) {
            add_untyped_rule_<W>(std::forward<Args>(args)...);
        }
        void add_untyped_rule_impl(size_t from, rule_t<W,C> r, bool negated, const std::vector<uint32_t>& pre) {
            auto mm = std::max(from, r._to);
            if (mm >= _states.size()) {
                _states.resize(mm + 1);
            }

            auto& rules = _states[from]._rules;
            auto lb = std::lower_bound(rules.begin(), rules.end(), r);
            if (lb == std::end(rules) || *lb != r) {
                lb = rules.insert(lb, r);
            }
            lb->_labels.merge(negated, pre, number_of_labels());
            auto& prestates = _states[r._to]._pre_states;
            auto lpre = std::lower_bound(prestates.begin(), prestates.end(), from);
            if (lpre == std::end(prestates) || *lpre != from) {
                prestates.insert(lpre, from);
            }
        }

    private:
        template <typename WT, typename = std::enable_if_t<!is_weighted<WT>>>
        void add_untyped_rule_(size_t from, size_t to, op_t op, uint32_t label, bool negated, const std::vector<uint32_t>& pre) {
            add_untyped_rule_impl(from, {to, op, label}, negated, pre);
        }
        template <typename WT, typename = std::enable_if_t<is_weighted<WT>>>
        void add_untyped_rule_(size_t from, size_t to, op_t op, uint32_t label, WT weight, bool negated, const std::vector<uint32_t>& pre) {
            add_untyped_rule_impl(from, {to, op, weight, label}, negated, pre);
        }

        std::vector<state_t> _states;
    };

}

#endif //PDAAAL_PDA_H
