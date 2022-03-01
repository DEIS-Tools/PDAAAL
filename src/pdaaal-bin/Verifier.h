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
 * File:   Verifier.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 02-07-2021.
 */

#ifndef PDAAAL_VERIFIER_H
#define PDAAAL_VERIFIER_H

#include "parsing/PAutomatonParser.h"
#include "parsing/PAutomatonJsonParser.h"
#include <pdaaal/Solver.h>

namespace pdaaal {

    std::istream& operator>>(std::istream& in, Trace_Type& trace_type) {
        std::string token;
        in >> token;
        if (token == "0") {
            trace_type = Trace_Type::None;
        } else if (token == "1") {
            trace_type = Trace_Type::Any;
        } else if (token == "2") {
            trace_type = Trace_Type::Shortest;
        } else if (token == "3") {
            trace_type = Trace_Type::Longest;
        } else if (token == "4") {
            trace_type = Trace_Type::ShortestFixedPoint;
        } else {
            in.setstate(std::ios_base::failbit);
        }
        return in;
    }
    constexpr std::ostream& operator<<(std::ostream& s, const Trace_Type& trace_type) {
        switch (trace_type) {
            case Trace_Type::None:
                s << "0";
                break;
            case Trace_Type::Any:
                s << "1";
                break;
            case Trace_Type::Shortest:
                s << "2";
                break;
            case Trace_Type::Longest:
                s << "3";
                break;
            case Trace_Type::ShortestFixedPoint:
                s << "4";
                break;
        }
        return s;
    }

    class Verifier {
    public:
        explicit Verifier(const std::string& caption) : verification_options{caption} {
            verification_options.add_options()
                    ("engine,e", po::value<size_t>(&engine), "Engine. 0=no verification, 1=post*, 2=pre*, 3=dual*")
                    ("trace,t", po::value<Trace_Type>(&trace_type)->default_value(Trace_Type::None), "Trace type. 0=no trace, 1=any trace, 2=shortest trace, 3=longest trace, 4=fixed-point shortest trace")
                    ;
        }
        [[nodiscard]] const po::options_description& options() const { return verification_options; }

        [[nodiscard]] bool needs_trace_info_pair() const {
            return trace_type == Trace_Type::Longest || trace_type == Trace_Type::ShortestFixedPoint;
        }

        template <TraceInfoType trace_info_type = TraceInfoType::Single, typename instance_t>
        void verify(instance_t& instance) {
            using pda_t = std20::remove_cvref_t<decltype(instance.pda())>;

            if (engine == 0) return; // No verification if not specified.

            bool result = false;
            std::vector<typename pda_t::tracestate_t> trace;
            if constexpr (trace_info_type == TraceInfoType::Single) {
                switch (engine) {
                    case 1: {
                        std::cout << "Using post*" << std::endl;
                        switch (trace_type) {
                            case Trace_Type::None:
                                result = Solver::post_star_accepts<Trace_Type::None>(instance);
                                break;
                            case Trace_Type::Any:
                                result = Solver::post_star_accepts<Trace_Type::Any>(instance);
                                if (result) {
                                    trace = Solver::get_trace<Trace_Type::Any>(instance);
                                }
                                break;
                            case Trace_Type::Shortest:
                                if constexpr(pda_t::has_weight) {
                                    result = Solver::post_star_accepts<Trace_Type::Shortest>(instance);
                                    if (result) {
                                        typename pda_t::weight_type weight;
                                        using W = typename pda_t::weight;
                                        std::tie(trace, weight) = Solver::get_trace<Trace_Type::Shortest>(instance);
                                        std::cout << "Weight: "; W::print(std::cout, weight) << std::endl;
                                    }
                                } else {
                                    assert(false);
                                    throw std::runtime_error("Cannot use shortest trace option for unweighted PDA.");
                                }
                                break;
                            case Trace_Type::Longest:
                            case Trace_Type::ShortestFixedPoint:
                                assert(false);
                                throw std::logic_error("Impossible control flow in switches");
                                break;
                        }
                        break;
                    }
                    case 2: {
                        std::cout << "Using pre*" << std::endl;
                        switch (trace_type) {
                            case Trace_Type::None:
                                result = Solver::pre_star_accepts(instance);
                                break;
                            case Trace_Type::Any:
                                result = Solver::pre_star_accepts(instance);
                                if (result) {
                                    trace = Solver::get_trace(instance);
                                }
                                break;
                            case Trace_Type::Shortest:
                                assert(false);
                                throw std::runtime_error("Cannot use shortest trace, not implemented for pre* engine.");
                                break;
                            case Trace_Type::Longest:
                            case Trace_Type::ShortestFixedPoint:
                                assert(false);
                                throw std::logic_error("Impossible control flow in switches");
                                break;
                        }
                        break;
                    }
                    case 3: {
                        std::cout << "Using dual*" << std::endl;
                        switch (trace_type) {
                            case Trace_Type::None:
                                result = Solver::dual_search_accepts(instance);
                                break;
                            case Trace_Type::Any:
                                result = Solver::dual_search_accepts(instance);
                                if (result) {
                                    trace = Solver::get_trace_dual_search(instance);
                                }
                                break;
                            case Trace_Type::Shortest:
                                assert(false);
                                throw std::runtime_error("Cannot use shortest trace, not implemented for dual* engine.");
                                break;
                            case Trace_Type::Longest:
                            case Trace_Type::ShortestFixedPoint:
                                assert(false);
                                throw std::logic_error("Impossible control flow in switches");
                                break;
                        }
                        break;
                    }
                    default: {
                        std::stringstream error;
                        error << "error: Unsupported option value --engine " << engine << std::endl;
                        throw std::runtime_error(error.str());
                    }
                }
            } else {
                switch (engine) {
                    case 1:
                    case 3: {
                        assert(false);
                        throw std::runtime_error("Cannot use fixed-point (longest or shortest) trace, not implemented for post* and dual* engine.");
                    }
                    case 2: {
                        std::cout << "Using pre*" << std::endl;
                        if constexpr(pda_t::has_weight) {
                            if (trace_type == Trace_Type::Longest) {
                                result = Solver::pre_star_fixed_point_accepts<Trace_Type::Longest>(instance);
                                if (result) {
                                    typename pda_t::weight_type weight;
                                    std::tie(trace, weight) = Solver::get_trace<Trace_Type::Longest>(instance);
                                    using W = typename pda_t::weight;
                                    if (weight == internal::solver_weight<W,Trace_Type::Longest>::bottom()) {
                                        std::cout << "Weight: infinity" << std::endl;
                                    } else {
                                        std::cout << "Weight: "; W::print(std::cout, weight) << std::endl;
                                    }
                                }
                            } else { // (trace_type == Trace_Type::ShortestFixedPoint)
                                result = Solver::pre_star_fixed_point_accepts<Trace_Type::ShortestFixedPoint>(instance);
                                if (result) {
                                    typename pda_t::weight_type weight;
                                    std::tie(trace, weight) = Solver::get_trace<Trace_Type::ShortestFixedPoint>(instance);
                                    using W = typename pda_t::weight;
                                    if (weight == internal::solver_weight<W,Trace_Type::ShortestFixedPoint>::bottom()) {
                                        std::cout << "Weight: negative infinity" << std::endl;
                                    } else {
                                        std::cout << "Weight: "; W::print(std::cout, weight) << std::endl;
                                    }
                                }
                            }
                        } else {
                            assert(false);
                            throw std::runtime_error("Cannot use fixed-point (longest or shortest) trace option for unweighted PDA.");
                        }
                    }
                    default: {
                        std::stringstream error;
                        error << "error: Unsupported option value --engine " << engine << std::endl;
                        throw std::runtime_error(error.str());
                    }
                }
            }

            std::cout << ((result) ? "Reachable" : "Not reachable") << std::endl;
            for (const auto& trace_state : trace) {
                std::cout << "< " << trace_state._pdastate << ", [";
                bool first = true;
                for (const auto& label : trace_state._stack) {
                    if (first) {
                        first = false;
                    } else {
                        std::cout << ", ";
                    }
                    std::cout << label;
                }
                std::cout << "] >" << std::endl;
            }

        }

    private:
        po::options_description verification_options;
        size_t engine = 0;
        Trace_Type trace_type = Trace_Type::None;
    };
}

#endif //PDAAAL_VERIFIER_H
