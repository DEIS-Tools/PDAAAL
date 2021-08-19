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
 * File:   PDAFactory.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-11-2020.
 */

#ifndef PDAAAL_PDAFACTORY_H
#define PDAAAL_PDAFACTORY_H

#include <string>
#include <sstream>
#include <vector>
#include <ostream>
#include <unordered_set>

#include <pdaaal/NFA.h>
#include <pdaaal/TypedPDA.h>
#include <pdaaal/PAutomaton.h>
#include <pdaaal/SolverInstance.h>

namespace pdaaal {

    template<typename Label, typename BuilderPDA, typename ResultPDA, typename Rule, typename SolverInstance>
    class PDAFactory {
        // Expose template parameters for convenience in deriving/consuming classes.
    protected:
        using builder_pda_t = BuilderPDA;
        using pda_t = ResultPDA;
    public:
        using label_t = Label;
        using rule_t = Rule;
        using solver_instance_t = SolverInstance;

        template<typename... Args>
        explicit PDAFactory(Args&&... args) : _temp_pda(std::forward<Args>(args)...) { }

        // NFAs must be already compiled before passing them to this function.
        solver_instance_t compile(const NFA<label_t>& initial_headers, const NFA<label_t>& final_headers) {
            build_pda();
            return solver_instance_t{pda_t{std::move(_temp_pda)}, initial_headers, initial(), final_headers, accepting()};
        }
    protected:
        virtual void build_pda() = 0;
        virtual const std::vector<size_t>& initial() = 0;
        virtual const std::vector<size_t>& accepting() = 0;

        void add_rule(const rule_t& rule) {
            _temp_pda.add_rule(rule);
        }
        void add_wildcard_rule(const rule_t& rule) {
            // Ignores rule._pre
            _temp_pda.add_wildcard_rule(rule);
        }

        builder_pda_t _temp_pda;
    };

    template<typename T, typename W = weight<void>>
    class TypedPDAFactory : public PDAFactory<T, TypedPDA<T,W,fut::type::hash>, TypedPDA<T,W,fut::type::vector>,
                                       typename TypedPDA<T,W,fut::type::hash>::rule_t, SolverInstance<T,W>> {
    private:
        using parent_t = PDAFactory<T, TypedPDA<T,W,fut::type::hash>, TypedPDA<T,W,fut::type::vector>,
                                    typename TypedPDA<T,W,fut::type::hash>::rule_t, SolverInstance<T,W>>;
    public:
        using rule_t = typename parent_t::rule_t;
        explicit TypedPDAFactory(const std::unordered_set<T>& all_labels) : parent_t(all_labels) { };
    };

    // This is the 'old' PDAFactory.
    template<typename T, typename W = weight<void>>
    class DFS_PDAFactory : public TypedPDAFactory<T,W> {
    private:
        using parent_t = TypedPDAFactory<T,W>;
    public:
        using rule_t = typename parent_t::rule_t;
        DFS_PDAFactory(const std::unordered_set<T>& all_labels, T wildcard_label)
        : parent_t(all_labels), _wildcard_label(wildcard_label) { };

    protected:
        void build_pda() override {
            // Build up PDA by searching through reachable states from initial states.
            // Derived class must define initial states, successor function (rules), and accepting state predicate.
            std::vector<size_t> waiting = this->initial();
            std::unordered_set<size_t> seen(waiting.begin(), waiting.end());
            while (!waiting.empty()) {
                auto from = waiting.back();
                waiting.pop_back();
                if (accepting(from)) {
                    _accepting_states.push_back(from);
                }
                for (const auto &r : rules(from)) {
                    assert(from == r._from);
                    if (r._pre == _wildcard_label) {
                        this->add_wildcard_rule(r);
                    } else {
                        this->add_rule(r);
                    }

                    if (seen.emplace(r._to).second) {
                        waiting.push_back(r._to);
                    }
                }
            }
            std::sort(_accepting_states.begin(), _accepting_states.end());
        }

        const std::vector<size_t>& accepting() override {
            return _accepting_states;
        }
        virtual bool accepting(size_t) = 0;
        virtual std::vector<rule_t> rules(size_t) = 0;

        T _wildcard_label;
    private:
        std::vector<size_t> _accepting_states;
    };

}

#endif //PDAAAL_PDAFACTORY_H
