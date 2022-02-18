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
 * File:   SolverInstance.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 27-11-2020.
 */

#ifndef PDAAAL_SOLVERINSTANCE_H
#define PDAAAL_SOLVERINSTANCE_H

#include "PAutomaton.h"
#include "PAutomatonProduct.h"
#include "pdaaal/cegar/AbstractionPDA.h"
#include "pdaaal/cegar/AbstractionPAutomaton.h"
#include <limits>

namespace pdaaal {

    // Wrapper for PAutomatonProduct with an owning PDA member (as opposed to the reference in PAutomatonProduct).
    // SolverInstance is for normal PDA, AbstractionSolverInstance is for when using CEGAR.

    template <typename T, typename W, typename state_t = size_t, bool skip_state_mapping = std::is_same_v<state_t,size_t>, TraceInfoType trace_info_type = TraceInfoType::Single>
    class SolverInstance {
    public:
        using pda_t = PDA<T,W,fut::type::vector,state_t,skip_state_mapping>;
        using pautomaton_t = PAutomaton<T,W,state_t,skip_state_mapping, trace_info_type>;
        using product_t = PAutomatonProduct<pda_t, pautomaton_t, W, trace_info_type>;
        SolverInstance(pda_t&& pda,
                       const NFA<T>& initial_nfa, const std::vector<size_t>& initial_states,
                       const NFA<T>& final_nfa,   const std::vector<size_t>& final_states)
        : _pda(std::move(pda)), _product(_pda, initial_nfa, initial_states, final_nfa, final_states) {};

        SolverInstance(pda_t&& pda, pautomaton_t&& initial, pautomaton_t&& final)
        : _pda(std::move(pda)), _product(_pda, std::move(initial), std::move(final)) {};

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
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type = TraceInfoType::Single>
    SolverInstance(PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>&& pda,
                   const NFA<label_t>& initial_nfa, const std::vector<size_t>& initial_states,
                   const NFA<label_t>& final_nfa, const std::vector<size_t>& final_states) -> SolverInstance<label_t,W,state_t,skip_state_mapping,trace_info_type>;
    template<typename label_t, typename W, typename state_t, bool skip_state_mapping, TraceInfoType trace_info_type>
    SolverInstance(PDA<label_t,W,fut::type::vector,state_t,skip_state_mapping>&& pda,
                   PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type> initial,
                   PAutomaton<label_t,W,state_t,skip_state_mapping,trace_info_type> final) -> SolverInstance<label_t,W,state_t,skip_state_mapping,trace_info_type>;

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

#endif //PDAAAL_SOLVERINSTANCE_H
