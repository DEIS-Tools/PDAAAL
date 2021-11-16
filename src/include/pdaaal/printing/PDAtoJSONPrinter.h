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
 * File:   PDAtoJSONPrinter.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 15-11-2021.
 */

#ifndef PDAAAL_PDATOJSONPRINTER_H
#define PDAAAL_PDATOJSONPRINTER_H

#include <nlohmann/json.hpp>
#include <pdaaal/TypedPDA.h>

using json = nlohmann::json;

namespace pdaaal {
    namespace details {
        template<typename label_t>
        std::string label_to_string(const label_t& label) {
            if constexpr (std::is_same_v<label_t, std::string>) {
                return label;
            } else {
                std::stringstream ss;
                ss << label;
                return ss.str();
            }
        }

        template<typename label_t, typename W, fut::type Container, typename state_t, bool ssm>
        void pda_rule_to_json(json& j_state, json& j_rule, const label_t& pre_label,
                          const typename PDA<W, Container>::rule_t& rule,
                          const TypedPDA<label_t, W, Container, state_t, ssm>& pda) {
            auto label_s = label_to_string(pre_label);
            switch (rule._operation) {
                case POP:
                    j_rule["pop"] = "";
                    break;
                case SWAP:
                    j_rule["swap"] = label_to_string(pda.get_symbol(rule._op_label));
                    break;
                case NOOP:
                    j_rule["swap"] = label_s;
                    break;
                case PUSH:
                    j_rule["push"] = label_to_string(pda.get_symbol(rule._op_label));
                    break;
                default:
                    assert(false);
            }
            if constexpr (W::is_weight) {
                j_rule["weight"] = rule._weight;
            }
            if (j_state.contains(label_s)) {
                if (!j_state[label_s].is_array()) {
                    auto temp_rule = j_state[label_s];
                    j_state[label_s] = json::array();
                    j_state[label_s].emplace_back(temp_rule);
                }
                j_state[label_s].emplace_back(j_rule);
            } else {
                j_state[label_s] = j_rule;
            }
        }
    }

    template<typename label_t, typename W, fut::type Container>
    void to_json(json& j, const TypedPDA<label_t,W,Container,size_t,true>& pda) {
        j = json::object();
        auto j_states = json::array();
        for (const auto& state : pda.states()) {
            auto j_state = json::object();
            for (const auto& [rule, labels] : state._rules) {
                for (const auto label : pda.get_labels(labels)) {
                    auto j_rule = json::object();
                    j_rule["to"] = rule._to;
                    details::pda_rule_to_json(j_state, j_rule, label, rule, pda);
                }
            }
            j_states.emplace_back(j_state);
        }
        j["states"] = j_states;
    }
    template<typename label_t, typename W, fut::type Container, typename state_t>
    void to_json(json& j, const TypedPDA<label_t,W,Container,state_t,false>& pda) {
        j = json::object();
        auto j_states = json::object();
        size_t state_i = 0;
        for (const auto& state : pda.states()) {
            auto j_state = json::object();
            for (const auto& [rule, labels] : state._rules) {
                for (const auto label : pda.get_labels(labels)) {
                    auto j_rule = json::object();
                    std::stringstream ss;
                    ss << pda.get_state(rule._to);
                    j_rule["to"] = ss.str();
                    details::pda_rule_to_json(j_state, j_rule, label, rule, pda);
                }
            }
            std::stringstream ss;
            ss << pda.get_state(state_i);
            j_states[ss.str()] = j_state;
            ++state_i;
        }
        j["states"] = j_states;
    }
}



#endif //PDAAAL_PDATOJSONPRINTER_H
