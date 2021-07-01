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
 * File:   StateTypedPDA.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 01-07-2021.
 */

#ifndef PDAAAL_STATETYPEDPDA_H
#define PDAAAL_STATETYPEDPDA_H

#include <pdaaal/PDA.h>
#include <pdaaal/utils/ptrie_interface.h>

namespace pdaaal {

    template<typename label_t, typename state_t, typename W = weight<void>, fut::type Container = fut::type::vector, bool skip_state_mapping = false>
    class StateTypedPDA : public PDA<W, Container> {
        static_assert(!skip_state_mapping || std::is_same_v<state_t,size_t>, "When skip_state_mapping==true, you must use state_t=size_t");
    public:
        template<fut::type OtherContainer>
        explicit StateTypedPDA(StateTypedPDA<label_t,state_t,W,OtherContainer,skip_state_mapping>&& other_pda)
        : PDA<W,Container>(std::move(other_pda)), _label_map(other_pda.move_label_map()), _state_map(other_pda.move_state_map()) {}
        StateTypedPDA() = default;

        auto move_label_map() { return std::move(_label_map); }
        auto move_state_map() { return std::move(_state_map); }

        [[nodiscard]] virtual size_t number_of_labels() const {
            return _label_map.size();
        }

        label_t get_label(size_t id) {
            assert(id < _label_map.size());
            return _label_map.at(id);
        }
        state_t get_state(size_t id) {
            assert(id < _state_map.size());
            return _state_map.at(id);
        }
        std::pair<bool,size_t> exists_label(const label_t& label) const {
            return _label_map.exists(label);
        }
        std::pair<bool,size_t> exists_state(const state_t& state) const {
            if constexpr (skip_state_mapping) {
                return std::make_pair(state < this->states(), state);
            } else {
                return _state_map.exists(state);
            }
        }
        uint32_t insert_label(const label_t& label) {
            return _label_map.insert(label).second;
        }
        size_t insert_state(const state_t& state) {
            if constexpr (skip_state_mapping) {
                return state;
            } else {
                return _state_map.insert(state).second;
            }
        }
        void add_rule_detail(size_t from, typename PDA<W>::rule_t r, bool negated, const std::vector<uint32_t>& pre) {
            this->add_untyped_rule_impl(from, r, negated, pre);
        }

    private:
        utils::ptrie_set<label_t> _label_map;
        utils::ptrie_set<state_t> _state_map;
    };

}

#endif //PDAAAL_STATETYPEDPDA_H
