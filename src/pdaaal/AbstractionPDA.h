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
 * File:   AbstractionPDA.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 11-12-2020.
 */

#ifndef PDAAAL_ABSTRACTIONPDA_H
#define PDAAAL_ABSTRACTIONPDA_H

#include "AbstractionMapping.h"
#include "PDA.h"

namespace pdaaal {

    template <typename label_t, typename abstract_label_t, typename W, typename C, fut::type Container = fut::type::vector>
    class AbstractionPDA : public PDA<W, C, Container> {
    protected:
        using impl_rule_t = typename PDA<W, C, Container>::rule_t; // This rule type is used internally.
    public:
        template<fut::type OtherContainer>
        explicit AbstractionPDA(AbstractionPDA<label_t,abstract_label_t,W,C,OtherContainer>&& other_pda)
        : PDA<W,C,Container>(std::move(other_pda)), _label_abstraction(other_pda.move_label_map()) {}

        explicit AbstractionPDA(std::unordered_set<label_t>&& all_labels, std::function<abstract_label_t(const label_t&)>&& label_abstraction_fn)
        : _label_abstraction(std::move(label_abstraction_fn), std::move(all_labels)) { };

        auto move_label_map() { return std::move(_label_abstraction); }

        [[nodiscard]] virtual size_t number_of_labels() const {
            return _label_abstraction.size();
        }

        std::pair<bool,size_t> insert_label(const label_t& label){
            return _label_abstraction.insert(label);
        }

        std::vector<uint32_t> encode_labels(const std::vector<label_t>& labels, bool negated) const {
            assert(std::is_sorted(labels.begin(), labels.end()));
            auto abstract_labels = _label_abstraction.encode_many(labels);
            if (negated) {
                // Create negation, but only include abstract labels, where all concrete labels, mapping to it, is in 'labels'.
                abstract_labels.erase(std::remove_if(abstract_labels.begin(), abstract_labels.end(), [this, &labels](const auto& abstract_label){
                    auto concrete = _label_abstraction.get_concrete_values(abstract_label);
                    std::sort(concrete.begin(), concrete.end());
                    return std::includes(labels.begin(), labels.end(), concrete.begin(), concrete.end());
                }), abstract_labels.end());
            }
            return std::vector<uint32_t>(abstract_labels.begin(), abstract_labels.end()); // TODO: This is not optimal. Label type in PDA should be size_t instead of uint32_t. This is a change many places...
        }

        std::vector<label_t> get_concrete_labels(size_t label) const {
            return _label_abstraction.get_concrete_values(label);
        }
        auto get_concrete_labels_range(size_t label) const {
            return _label_abstraction.get_concrete_values_range(label);
        }
        bool maps_to(label_t label, size_t id) const {
            auto [exists, res_id] = _label_abstraction.exists(label, true);
            return exists && res_id == id;
        }

    private:
        AbstractionMapping<label_t,abstract_label_t> _label_abstraction;
    };

}

#endif //PDAAAL_ABSTRACTIONPDA_H
