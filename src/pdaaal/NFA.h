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
 * File:   NFA.h
 * Author: Peter G. Jensen <root@petergjoel.dk>
 *
 * Created on July 16, 2019, 5:39 PM
 */

#ifndef NFA_H
#define NFA_H

#include <vector>
#include <unordered_set>
#include <memory>
#include <unordered_map>
#include <ostream>
#include <functional>
#include <iostream>

namespace pdaaal {

    template<typename T = char>
    class NFA {
        // TODO: Some trivial optimizations to be had here.
    public:

        struct state_t;

        struct edge_t {
            // we already know that we will have many more symbols than destinations
            bool _epsilon = false;
            bool _negated = false;
            std::vector<T> _symbols;
            state_t* _destination;
            edge_t(bool negated, std::unordered_set<T>& symbols, state_t* dest, state_t* source, size_t id)
                    : _negated(negated), _symbols(symbols.begin(), symbols.end()), _destination(dest) 
            {
                _destination->_backedges.emplace_back(source, id);
                std::sort(_symbols.begin(), _symbols.end());
            };
            edge_t(state_t* dest, state_t* source, size_t id, bool epsilon = false)
                    : _epsilon(epsilon), _negated(!epsilon), _destination(dest) 
            {
                _destination->_backedges.emplace_back(source, id);
            };
            edge_t(const edge_t& other) = default;
            [[nodiscard]] bool empty(size_t n) const {
                if(!_negated)
                    return _symbols.empty();
                return _symbols.size() == n;
            }
            [[nodiscard]] bool wildcard(size_t n) const {
                if(_negated)
                    return _symbols.empty();
                else //if(!_negated)
                    return _symbols.size() == n;
            }
            bool contains(const T& symbol) const {
                auto lb = std::lower_bound(_symbols.begin(), _symbols.end(), symbol);
                bool found = lb != std::end(_symbols) && *lb == symbol;
                return _negated ? !found : found;
            }

            std::vector<state_t*> follow_epsilon() const {
                std::vector<state_t*> next{_destination};
                NFA<T>::follow_epsilon(next);
                return next;
            }

        };
        
        struct state_t {
            std::vector<edge_t> _edges;
            std::vector<std::pair<state_t*,size_t>> _backedges;
            bool _accepting = false;
            explicit state_t(bool accepting) : _accepting(accepting) {};
            state_t(const state_t& other) = default;
            [[nodiscard]] bool has_non_epsilon() const {
                for(auto& e : _edges) {
                    if(e._negated || !e._symbols.empty()) {
                        return true;
                    }
                }
                return false;
            }

            void add_epsilon_edge(state_t* to) {
                auto id = _edges.size();
                _edges.emplace_back(to, this, id, true);
            }

            std::vector<T> prelabels(const std::unordered_set<T>& all_labels) const
            {
                std::unordered_set<const state_t*> seen{this};
                std::vector<const state_t*> waiting{this};
                std::vector<T> labels;
                while(!waiting.empty())
                {
                    auto s = waiting.back();
                    waiting.pop_back();
                    
                    for(auto& e : s->_backedges)
                    {
                        if(seen.count(e.first) == 0)
                        {
                            seen.insert(e.first);
                            waiting.push_back(e.first);
                        }
                        auto& edge = e.first->_edges[e.second];
                        if(edge._negated)
                        {
                            if(edge._symbols.empty())
                            {
                                labels.clear();
                                labels.insert(labels.end(), all_labels.begin(), all_labels.end());
                                std::sort(labels.begin(), labels.end());
                                return labels;
                            }
                            else
                            {
                                for(auto& l : all_labels)
                                {
                                    auto lb = std::lower_bound(edge._symbols.begin(), edge._symbols.end(), l);
                                    if(lb == std::end(edge._symbols) || *lb != l)
                                    {
                                        auto lb2 = std::lower_bound(labels.begin(), labels.end(), l);
                                        if(lb2 == std::end(labels) || *lb2 != l)
                                            labels.insert(lb2, l);
                                    }
                                }
                            }
                        }
                        else
                        {
                            for(auto l : edge._symbols)
                            { // sorted, so we can optimize here of we want to 
                                auto lb2 = std::lower_bound(labels.begin(), labels.end(), l);
                                if(lb2 == std::end(labels) || *lb2 != l)
                                    labels.insert(lb2, l);
                            }
                        }
                    }
                }
                return labels;
            }
        };
        
    public:

        explicit NFA(bool initially_accepting = true) {
            _states.emplace_back(std::make_unique<state_t>(initially_accepting));
            _accepting.push_back(_states.back().get());
            _initial.push_back(_states.back().get());
        }

        explicit NFA(std::unordered_set<T>&& initial_accepting, bool negated = false) {
            if (initial_accepting.size() > 0 || negated) {
                _states.emplace_back(std::make_unique<state_t>(false));
                _initial.push_back(_states[0].get());
                _states.emplace_back(std::make_unique<state_t>(true));
                _accepting.push_back(_states[1].get());
                _initial.back()->_edges.emplace_back(negated,initial_accepting,_accepting.back(),_initial.back(), 0);
            }
        }
        
        void compile()
        {
            std::sort(_accepting.begin(), _accepting.end());
            std::sort(_initial.begin(), _initial.end());
            follow_epsilon(_initial);
            std::sort(_states.begin(), _states.end());
        }
        
        NFA(NFA&&) noexcept = default;
        NFA& operator=(NFA&&) noexcept = default;
        NFA(const NFA& other) {
            (*this) = other;
        }

        [[nodiscard]] bool empty_accept() const {
            assert(std::is_sorted(_initial.begin(), _initial.end()));
            // Also assume that follow_epsilon(_initial) has been executed (e.g. in compile()).
            return std::any_of(_initial.begin(), _initial.end(), [](const state_t* s){ return s->_accepting; });
        }

        template<typename C>
        static std::enable_if_t<std::is_same_v<state_t*, C> || std::is_same_v<const state_t*, C>, void>
        follow_epsilon(std::vector<C>& states)
        {
            std::vector<C> waiting = states;
            while(!waiting.empty())
            {
                auto s = waiting.back();
                waiting.pop_back();
                for(auto& e : s->_edges)
                {
                    if(e._epsilon)
                    {
                        auto lb = std::lower_bound(states.begin(), states.end(), e._destination);
                        if(lb == std::end(states) || *lb != e._destination)
                        {   // FIXME: Vector.insert is usually slow for large vectors.
                            states.insert(lb, e._destination);
                            waiting.push_back(e._destination);
                        }
                    }
                }
            }
            auto res = std::remove_if(states.begin(), states.end(), [](const state_t* s){ return !(s->_accepting || s->has_non_epsilon());});
            states.erase(res, states.end());
        }

        template<typename C>
        static std::enable_if_t<std::is_same_v<state_t*, C> || std::is_same_v<const state_t*, C>, std::vector<C>>
        successor(const std::vector<C>& states, const T& label) {
            std::vector<C> next;
            for(const auto& s : states) {
                for(const auto& e : s->_edges) {
                    if(e.contains(label)) {
                        auto slb = std::lower_bound(next.begin(), next.end(), e._destination);
                        if(slb == std::end(next) || *slb != e._destination) {
                            next.insert(slb, e._destination);
                        }
                    }
                }
            }
            follow_epsilon(next);
            return next;
        }

        template<typename C>
        static std::enable_if_t<std::is_same_v<state_t*, C> || std::is_same_v<const state_t*, C>, bool>
        has_as_successor(const std::vector<C>& states, const T& label, const state_t* goal_successor_state) {
            std::vector<C> successor_states = successor(states, label);
            auto lb = std::lower_bound(successor_states.begin(), successor_states.end(), goal_successor_state); // Since successor_states vector is sorted, we can use binary search.
            return lb != successor_states.end() && *lb == goal_successor_state;
        }
        static bool has_as_successor(const state_t* state, const T& label, const state_t* goal_successor_state) {
            return has_as_successor(std::vector<const state_t*>{state}, label, goal_successor_state);
        }
        static bool leads_to_by_epsilon(const state_t* from, const state_t* to) { // More efficient (specialized) than using follow_epsilon naively
            if (from == to) return true;
            std::vector<const state_t*> waiting{from};
            std::vector<const state_t*> seen{from};
            while(!waiting.empty()) {
                auto s = waiting.back();
                waiting.pop_back();
                for(auto& e : s->_edges) {
                    if(e._epsilon) {
                        if (e._destination == to) return true;
                        auto lb = std::lower_bound(seen.begin(), seen.end(), e._destination);
                        if(lb == std::end(seen) || *lb != e._destination) {
                            seen.insert(lb, e._destination); // FIXME: Vector.insert is usually slow for large vectors.
                            waiting.push_back(e._destination);
                        }
                    }
                }
            }
            return false;
        }
        template<typename C>
        static std::enable_if_t<std::is_same_v<state_t*, C> || std::is_same_v<const state_t*, C>, std::vector<T>>
        intersect_edge_labels(const std::vector<C>& from_states, const state_t* to_state, const std::vector<T>& labels) {
            // Find all the labels l in 'labels', such that has_as_successor(from_states, l, to_state) is true.
            assert(std::is_sorted(labels.begin(), labels.end()));
            std::vector<T> result;
            std::vector<T> temp_union;
            std::vector<T> temp;
            for (const auto& from_state : from_states) {
                for (const auto& e: from_state->_edges) {
                    if (!leads_to_by_epsilon(e._destination, to_state)) continue;

                    if (e._negated && e._symbols.empty()) { // Wildcard. Note: we cannot use e.wildcard(labels.size()) since labels is only a subset of all labels.
                        return labels;
                    }
                    // Only reserve if needed (result and temp_union actually used). These reserve calls in subsequent iterations does nothing (cheap).
                    result.reserve(labels.size());
                    temp_union.reserve(labels.size());
                    temp.reserve(labels.size());
                    if (e._negated) {
                        std::set_difference(labels.begin(), labels.end(), e._symbols.begin(), e._symbols.end(), std::back_inserter(temp));
                    } else {
                        std::set_intersection(labels.begin(), labels.end(), e._symbols.begin(), e._symbols.end(), std::back_inserter(temp));
                    }
                    temp_union.clear();
                    std::set_union(result.begin(), result.end(), temp.begin(), temp.end(), std::back_inserter(temp_union));
                    std::swap(result, temp_union);
                    if (result.size() == labels.size()) {
                        return result; // Possibly early terminate.
                    }
                }
            }
            return result;
        }
        static std::vector<T> intersect_edge_labels(const state_t* from_state, const state_t* to_state, const std::vector<T>& labels) {
            return intersect_edge_labels(std::vector<const state_t*>{from_state}, to_state, labels);
        }

        
        NFA& operator=(const NFA& other) {
            std::unordered_map<state_t*, state_t*> indir;
            for(auto& s : other._states) {
                _states.emplace_back(std::make_unique<state_t>(*s));
                indir[s.get()] = _states.back().get();
            }     
            // fix links
            for(auto& s : _states) {
                for(auto& e : s->_edges) {
                    e._destination = indir[e._destination];
                }
                for(auto& e : s->_backedges) {
                    e.first = indir[e.first];
                }
            }
            for(auto& s : other._accepting) {
                _accepting.push_back(indir[s]);
            }
            for(auto& s : other._initial) {
                _initial.push_back(indir[s]);
            }
            return *this;
        }


        // construction from regex
        void concat(NFA&& other) {
            for(auto& s : other._states) {
                _states.emplace_back(s.release());
            }
            for(auto& sa : _accepting) {
                for(auto& s : other._initial) {
                    sa->add_epsilon_edge(s);
                }
                sa->_accepting = false;
            }
            _accepting = std::move(other._accepting);
        }
        
        void question_extend() {
            for(auto s : _initial) {
                if(!s->_accepting) {
                    s->_accepting = true;
                    _accepting.push_back(s);
                }
            }
        }
        
        void plus_extend() {
            for(auto s : _accepting) {
                for(auto si : _initial) {
                    bool found = false;
                    for(auto& e : s->_edges) {
                        if(e._destination == si) {
                            e._epsilon = true;
                            found = true;
                        }
                    }
                    if(!found) {
                        s->add_epsilon_edge(si);
                    }
                }
            }
        }

        void star_extend() {
            plus_extend();
            question_extend();
        }

        void and_extend(NFA&& other) {
            // prune? Powerset?
            throw std::logic_error("conjunction for NFAs are not yet implemented");
        }

        void or_extend(NFA&& other) {
            for(auto& s : other._states) {
                _states.emplace_back(s.release());
            }
            _states.emplace_back(std::make_unique<state_t>(false));
            auto initial = _states.back().get();
            for(auto& i : _initial) {
                initial->add_epsilon_edge(i);
            }
            for(auto& i : other._initial) {
                initial->add_epsilon_edge(i);
            }            
            _initial = {initial};
            _accepting.insert(_accepting.end(), other._accepting.begin(), other._accepting.end());
        }       
        
        void to_dot(std::ostream& out, std::function<void(std::ostream&, const T&)> printer = [](auto& s, auto& e){ s << e ;}) const
        {
            out << "digraph NFA {\n";
            for(const auto& s : _states)
            {
                out << "\"" << s.get() << "\" [shape=";
                if(s->_accepting)
                    out << "double";
                out << "circle];\n";
                for(const edge_t& e : s->_edges)
                {
                    out << "\"" << s.get() << "\" -> \"" << e._destination << "\" [ label=\"";
                    if(e._negated && !e._symbols.empty())
                    {
                        out << "^";
                    }
                    if(!e._symbols.empty())
                    {
                        out << "\\[";
                        bool first = true;
                        for(auto& s : e._symbols)
                        {
                            if(!first)
                                out << ", ";
                            first = false;
                            printer(out, s);
                        }
                        out << "\\]";
                    }
                    if(e._symbols.empty() && e._negated)
                        out << "*";
                    if(e._epsilon)
                    {
                        if(!e._symbols.empty() || e._negated) out << " ";
                        out << "𝜀";
                    }
                    
                    out << "\"];\n";
                }
            }
            for(const auto& i : _initial)
            {
                out << "\"I" << i << "\" -> \"" << i << "\";\n";
                out << "\"I" << i << "\" [style=invisible];\n";
            }
            
            out << "}\n";
        }

        const std::vector<state_t*>& initial() const { return _initial; }
        const std::vector<state_t*>& accepting() const { return _accepting; }
        const std::vector<std::unique_ptr<state_t>>& states() { return _states; }
        
    private:
        const static std::vector<state_t*> empty;
        std::vector<std::unique_ptr<state_t>> _states;
        std::vector<state_t*> _initial;
        std::vector<state_t*> _accepting;
    };


}

#endif /* NFA_H */

