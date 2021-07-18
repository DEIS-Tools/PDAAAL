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

#include <pdaaal/Weight.h>
#include <pdaaal/utils/fut_set.h>

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
}

namespace pdaaal::details {
    // Implementation details of PDA structure. Should not be accessed by user.
    // TypedPDA defines a rule_t to be used by users.

    // Define rules with and without weights.
    template<typename W, typename = void>
    struct rule_t;

    template<typename W>
    struct rule_t<W, std::enable_if_t<!is_weighted<W>>> {
        size_t _to = 0;
        op_t _operation = PUSH;
        uint32_t _op_label = 0;

        bool operator<(const rule_t<W> &other) const {
            if (_to != other._to)
                return _to < other._to;
            if (_op_label != other._op_label)
                return _op_label < other._op_label;
            return _operation < other._operation;
        }

        bool operator==(const rule_t<W> &other) const {
            return _to == other._to && _op_label == other._op_label && _operation == other._operation;
        }

        bool operator!=(const rule_t<W> &other) const {
            return !(*this == other);
        }

        template <typename H>
        friend H AbslHashValue(H h, const rule_t<W>& rule) {
            return H::combine(std::move(h), rule._to, rule._operation, rule._op_label);
        }

        struct hasher {
            size_t operator()(const pdaaal::details::rule_t<W> &rule) const noexcept {
                size_t seed = 0;
                boost::hash_combine(seed, rule._to);
                boost::hash_combine(seed, rule._op_label);
                boost::hash_combine(seed, rule._operation);
                return seed;
            }
        };
    };

    template<typename W>
    struct rule_t<W, std::enable_if_t<is_weighted<W>>> {
        size_t _to = 0;
        op_t _operation = PUSH;
        typename W::type _weight = W::zero();
        uint32_t _op_label = 0;

        bool operator<(const rule_t<W> &other) const {
            if (_to != other._to)
                return _to < other._to;
            if (_op_label != other._op_label)
                return _op_label < other._op_label;
            if (_operation != other._operation)
                return _operation < other._operation;
            return W::less(_weight, other._weight);
        }

        bool operator==(const rule_t<W> &other) const {
            return _to == other._to && _op_label == other._op_label && _operation == other._operation &&
                   _weight == other._weight;
        }

        bool operator!=(const rule_t<W> &other) const {
            return !(*this == other);
        }

        template <typename H>
        friend H AbslHashValue(H h, const rule_t<W>& rule) {
            return H::combine(std::move(h), rule._to, rule._operation, rule._weight, rule._op_label);
        }
    };
}

namespace pdaaal {
    template<typename W, typename = void>
    struct user_rule_t;
    template<typename W>
    struct user_rule_t<W, std::enable_if_t<!is_weighted<W>>> {
        size_t _from = std::numeric_limits<size_t>::max();
        size_t _to = std::numeric_limits<size_t>::max();
        uint32_t _pre = std::numeric_limits<uint32_t>::max();
        uint32_t _op_label = std::numeric_limits<uint32_t>::max();
        op_t _op = POP;
        // Use max as default value, so we will notice if it has not been set.
        user_rule_t() = default;
        user_rule_t(size_t from, uint32_t pre, size_t to, op_t op, uint32_t op_label)
        : _from(from), _to(to), _pre(pre), _op_label(op_label), _op(op) {};
        user_rule_t(size_t from, uint32_t pre, const details::rule_t<W>& rule)
        : _from(from), _to(rule._to), _pre(pre),
          _op_label((rule._operation == PUSH || rule._operation == SWAP) ? rule._op_label : std::numeric_limits<uint32_t>::max()),
          _op(rule._operation) {};

        [[nodiscard]] size_t from() const {
            return _from;
        }

        details::rule_t<W> to_impl_rule() const {
            return details::rule_t<W>{_to, _op, _op_label};
        }
    } __attribute__((packed)); // packed is used to make this work fast with ptries
    template<typename W>
    struct user_rule_t<W, std::enable_if_t<is_weighted<W>>> {
        size_t _from = std::numeric_limits<size_t>::max();
        size_t _to = std::numeric_limits<size_t>::max();
        uint32_t _pre = std::numeric_limits<uint32_t>::max();
        uint32_t _op_label = std::numeric_limits<uint32_t>::max();
        op_t _op = POP;
        typename W::type _weight = W::zero();
        // Use max as default value for most values, so we will notice if it has not been set.
        user_rule_t() = default;
        user_rule_t(size_t from, uint32_t pre, size_t to, op_t op, uint32_t op_label, typename W::type weight)
                : _from(from), _to(to), _pre(pre), _op_label(op_label), _op(op), _weight(weight) {};
        user_rule_t(size_t from, uint32_t pre, const details::rule_t<W>& rule)
                : _from(from), _to(rule._to), _pre(pre),
                  _op_label((rule._operation == PUSH || rule._operation == SWAP) ? rule._op_label : std::numeric_limits<uint32_t>::max()),
                  _op(rule._operation), _weight(rule._weight) {};

        details::rule_t<W> to_impl_rule() const {
            return details::rule_t<W>{_to, _op, _weight, _op_label};
        }
    };


    template <typename W, fut::type Container = fut::type::vector>
    class PDA {
    public:
        static constexpr bool has_weight = is_weighted<W>;
        using weight_type = std::conditional_t<has_weight, typename W::type, void>;

        using rule_t = typename details::rule_t<W>;

        struct state_t {
            fut::set<std::tuple<rule_t,labels_t>,Container> _rules;
            std::vector<size_t> _pre_states;
            explicit state_t(typename PDA<W,fut::type::hash>::state_t&& other_state)
                    : _rules(std::move(other_state._rules)), _pre_states(std::move(other_state._pre_states)) {}
            state_t() = default;
        };

    public:
        template<fut::type OtherContainer>
        explicit PDA(PDA<W,OtherContainer>&& other_pda)
                : _states(std::make_move_iterator(other_pda.states_begin()), std::make_move_iterator(other_pda.states_end())) {}
        PDA() = default;

        auto states_begin() noexcept { return _states.begin(); }
        auto states_end() noexcept { return _states.end(); }

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
                    if (rit->first._to != s) {
                        if (wit != rit)
                            *wit = *rit;
                        ++wit;
                    }
                }
                _states[p]._rules.resize(wit - std::begin(_states[p]._rules));
            }
            _states[s]._pre_states.clear();
        }

        void add_rule(user_rule_t<W> rule) {
            add_untyped_rule_impl(rule._from, rule.to_impl_rule(), false, std::vector<uint32_t>{rule._pre});
        }
        void add_wildcard_rule(user_rule_t<W> rule) {
            // Ignore rule._pre
            add_untyped_rule_impl(rule._from, rule.to_impl_rule(), true, std::vector<uint32_t>());
        }

    protected:

        // Handle both weighted and unweighted rules appropriately.
        template <typename... Args>
        void add_untyped_rule(Args&&... args) {
            add_untyped_rule_<W>(std::forward<Args>(args)...);
        }
        void add_untyped_rule_impl(size_t from, rule_t r, bool negated, const std::vector<uint32_t>& pre) {
            auto mm = std::max(from, r._to);
            if (mm >= _states.size()) {
                _states.resize(mm + 1);
            }

            auto [it,succeed] = _states[from]._rules.emplace(r, labels_t{});
            it->second.merge(negated, pre, number_of_labels());

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
        void add_untyped_rule_(size_t from, size_t to, op_t op, uint32_t label, typename WT::type weight, bool negated, const std::vector<uint32_t>& pre) {
            add_untyped_rule_impl(from, {to, op, weight, label}, negated, pre);
        }

        std::vector<state_t> _states;
    };

}

#endif //PDAAAL_PDA_H
