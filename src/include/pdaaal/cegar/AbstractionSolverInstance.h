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
 * File:   AbstractionSolverInstance.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 21-02-2022.
 */

#ifndef PDAAAL_ABSTRACTIONSOLVERINSTANCE_H
#define PDAAAL_ABSTRACTIONSOLVERINSTANCE_H

#include "AbstractionPDA.h"
#include "AbstractionPAutomaton.h"
#include "pdaaal/PAutomatonProduct.h"

namespace pdaaal {

    template <typename T, typename W>
    class AbstractionSolverInstance {
    public:
        using pda_t = AbstractionPDA<T,W>;
        using pautomaton_t = AbstractionPAutomaton<T,W>;
        using product_t = PAutomatonProduct<pda_t, pautomaton_t, W, TraceInfoType::Single>;
        AbstractionSolverInstance(pda_t&& pda,
                                  const NFA<T>& initial_nfa, const std::vector<size_t>& initial_states,
                                  const NFA<T>& final_nfa,   const std::vector<size_t>& final_states)
                : _pda(std::move(pda)), _product(_pda, initial_nfa, initial_states, final_nfa, final_states) {};

        auto move_pda_refinement_mapping() {
            return _pda.move_label_map();
        }
        auto move_pda_refinement_mapping(const Refinement<T>& refinement) {
            auto map = _pda.move_label_map();
            map.refine(refinement);
            return map;
        }
        auto move_pda_refinement_mapping(const HeaderRefinement<T>& header_refinement) {
            auto map = _pda.move_label_map();
            for (const auto& refinement : header_refinement.refinements()) {
                map.refine(refinement);
            }
            return map;
        }

        product_t* operator->() {
            return &_product;
        }
        const product_t* operator->() const {
            return &_product;
        }
        product_t& operator*() {
            return _product;
        }
        const product_t& operator*() const {
            return _product;
        }
        product_t& get() {
            return _product;
        }
        const product_t& get() const {
            return _product;
        }

    private:
        pda_t _pda;
        product_t _product;
    };
}

#endif //PDAAAL_ABSTRACTIONSOLVERINSTANCE_H
