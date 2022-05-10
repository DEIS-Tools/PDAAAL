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
 * File:   SolverInstanceJsonParser.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-02-2022.
 */

#ifndef PDAAAL_SOLVERINSTANCEJSONPARSER_H
#define PDAAAL_SOLVERINSTANCEJSONPARSER_H

#include "PdaJsonParser.h"
#include "PAutomatonJsonParser.h"
#include "SaxHandlerDispatch.h"
#include <pdaaal/PDA.h>
#include <pdaaal/SolverInstance.h>

using json = nlohmann::json;

namespace pdaaal::parsing {

    namespace detail {
        // Get index of a type in a variant. Only works if the variant has unique types.
        // Based on https://stackoverflow.com/questions/52303316/get-index-by-type-in-stdvariant
        // and https://stackoverflow.com/questions/53651609/why-does-this-get-index-implementation-fail-on-vs2017
        template <typename> struct tag { };
        template <typename T, typename... Ts> constexpr std::size_t variant_index() { return std::variant<tag<Ts>...>(tag<T>()).index(); }
        template <typename T, typename V> struct get_index;
        template <typename T, typename... Ts> struct get_index<T, std::variant<Ts...>> : std::integral_constant<std::size_t, variant_index<T, Ts...>()> { };
        template <typename T, typename V> constexpr std::size_t get_index_v = get_index<T,V>::value;
    }

    struct SolverInstanceSaxHelper {
        enum class keys : uint32_t { none, instance };
        friend constexpr std::ostream& operator<<(std::ostream& s, keys key) {
            switch (key) {
                case keys::none:
                    s << "<none>";
                    break;
                case keys::instance:
                    s << "instance";
                    break;
            }
            return s;
        }
        enum class context_type : uint8_t { initial, instance };
        friend constexpr std::ostream& operator<<(std::ostream& s, context_type t) {
            switch (t) {
                case context_type::initial:
                    s << "<initial>";
                    break;
                case context_type::instance:
                    s << "instance array";
                    break;
            }
            return s;
        }
        static constexpr std::size_t N = 1;
        static constexpr auto FLAG_1 = utils::flag_mask<N>::template flag<1>();
        static constexpr keys get_key(context_type type, typename utils::flag_mask<N>::flag_t flag) {
            if (type == context_type::initial && flag == FLAG_1) {
                return keys::instance;
            }
            assert(false);
            return keys::none;
        }
    };

    template <TraceInfoType trace_info_type = TraceInfoType::Single>
    class SolverInstanceSaxHandler : public SAXHandlerContextStack<SolverInstanceSaxHelper> {
        using parent_t = SAXHandlerContextStack<SolverInstanceSaxHelper>;

        constexpr static context_t initial_context = context_object<context_type::initial, 1>();
        constexpr static context_t instance_context = context_array<context_type::instance>();

        using pda_sax_variant_t = PdaTypeSaxHandler::pda_sax_variant_t;
        using pda_variant_t = PdaTypeSaxHandler::pda_variant_t;

        template<typename T> struct pda_to_automaton_sax;
        template<typename... Args> struct pda_to_automaton_sax<std::variant<Args...>> {
            using type = std::variant<PAutomatonSaxHandler<Args, trace_info_type, true>...>;
        };
        template<typename T> struct deduce_solver_instance;
        template<typename... Args> struct deduce_solver_instance<std::variant<Args...>> {
            using type = std::variant<decltype(SolverInstance(std::declval<Args&&>(),
                            std::declval<PAutomatonSaxHandler<Args, trace_info_type, true>>().get_automaton(),
                           std::declval<PAutomatonSaxHandler<Args, trace_info_type, true>>().get_automaton()))...>;
        };

        using automaton_sax_variant_t = typename pda_to_automaton_sax<pda_variant_t>::type;
    public:
        using solver_instance_variant_t = typename deduce_solver_instance<pda_variant_t>::type;
    private:
        PdaTypeSaxHandler pda_type_sax_handler;
        std::optional<pda_sax_variant_t> pda_sax_handler;
        std::optional<pda_variant_t> pda_variant;
        std::optional<automaton_sax_variant_t> automaton_sax_handler_1;
        std::optional<automaton_sax_variant_t> automaton_sax_handler_2;

    public:
        using number_integer_t = typename nlohmann::json::number_integer_t;
        using number_unsigned_t = typename nlohmann::json::number_unsigned_t;
        using number_float_t = typename nlohmann::json::number_float_t;
        using string_t = typename nlohmann::json::string_t;
        using binary_t = typename nlohmann::json::binary_t;

        explicit SolverInstanceSaxHandler(std::ostream& errors) : parent_t(errors), pda_type_sax_handler(*this) {};
        explicit SolverInstanceSaxHandler(const SAXHandlerBase& base) : parent_t(base), pda_type_sax_handler(*this) {};

        solver_instance_variant_t get_solver_instance() {
            assert(pda_variant);
            assert(automaton_sax_handler_1);
            assert(automaton_sax_handler_2);
            // Use std::visit to find type of pda_variant, and on compile-time find corresponding index
            // and use it to std::get value of automaton_sax_handler_1 and automaton_sax_handler_2.
            return std::visit([this](auto&& pda) {
                using pda_t = std::decay_t<decltype(pda)>;
                constexpr auto i = detail::get_index_v<pda_t,pda_variant_t>;
                auto& sax_1 = std::get<i>(automaton_sax_handler_1.value());
                auto& sax_2 = std::get<i>(automaton_sax_handler_2.value());
                return solver_instance_variant_t(SolverInstance(std::forward<decltype(pda)>(pda), sax_1.get_automaton(), sax_2.get_automaton()));
            }, std::move(pda_variant).value());
        }

        bool callback(SAXHandlerDispatch& dispatch_handler) {
            switch (current_context().get_index()) {
                case 0:
                    dispatch_handler.set_dispatch(pda_type_sax_handler);
                    break;
                case 1:
                    pda_sax_handler.emplace(pda_type_sax_handler.get_pda_sax_handler());
                    std::visit([&dispatch_handler](auto&& sax_handler){ dispatch_handler.set_dispatch(sax_handler); }, pda_sax_handler.value());
                    break;
                case 2:
                    pda_variant.emplace(std::visit([](auto&& sax_handler){ return pda_variant_t(sax_handler.get_pda()); }, pda_sax_handler.value()));
                    automaton_sax_handler_1.emplace(std::visit([this](auto&& pda){
                        return automaton_sax_variant_t(PAutomatonSaxHandler<std::remove_reference_t<decltype(pda)>,trace_info_type,true>(pda,*this));
                    }, pda_variant.value()));
                    std::visit([&dispatch_handler](auto&& sax_handler){ dispatch_handler.set_dispatch(sax_handler); }, automaton_sax_handler_1.value());
                    break;
                case 3:
                    automaton_sax_handler_2.emplace(std::visit([this](auto&& pda){
                        return automaton_sax_variant_t(PAutomatonSaxHandler<std::remove_reference_t<decltype(pda)>,trace_info_type,true>(pda,*this));
                    }, pda_variant.value()));
                    std::visit([&dispatch_handler](auto&& sax_handler){ dispatch_handler.set_dispatch(sax_handler); }, automaton_sax_handler_2.value());
                    break;
                case 4:
                    dispatch_handler.set_dispatch(*this);
                    break;
                default:
                    errors() << "error: Solver instance should only contain four elements." << std::endl;
                    return false;
            }
            return element_done();
        }

        bool null() { return error_unexpected("null value"); }
        bool boolean(bool value) { return error_unexpected("boolean", value); }
        bool number_integer(number_integer_t value) { return error_unexpected("integer", value); }
        bool number_unsigned(number_unsigned_t value) { return error_unexpected("unsigned", value); }
        bool number_float(number_float_t value, const string_t& /*unused*/) { return error_unexpected("float", value); }
        bool string(string_t& value) { return error_unexpected("string", value); }
        bool binary(binary_t& /*val*/) { return error_unexpected("binary value"); }
        bool start_object(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                push_context(initial_context);
                return true;
            }
            return error_unexpected("start of object");
        }
        bool key(string_t& key) {
            if (no_context()) {
                return error_unexpected_key(key);
            }
            if (current_context_type() == context_type::initial) {
                if (key == "instance") {
                    return handle_key<context_type::initial,FLAG_1,keys::instance>();
                }
            }
            return error_unexpected_key(key);
        }
        bool end_object() {
            if (no_context()) {
                errors() << "error: Unexpected end of object." << std::endl;
                return false;
            }
            if (current_context().has_missing_flags()) {
                return error_missing_keys();
            }
            pop_context();
            return element_done();
        }
        bool start_array(std::size_t /*unused*/ = std::size_t(-1)) {
            if (no_context()) {
                errors() << "error: Encountered start of array, but must start with an object." << std::endl;
                return false;
            }
            if (last_key == keys::instance) {
                push_context(instance_context);
                return false; // Trigger callback.
            }
            return error_unexpected("start of array");
        }
        bool end_array() {
            if (no_context()) {
                errors() << "error: Unexpected end of array." << std::endl;
                return false;
            }
            pop_context();
            return element_done();
        }

    };

    class SolverInstanceJsonParser {
    public:
        template <TraceInfoType trace_info_type = TraceInfoType::Single>
        static auto parse(std::istream& stream, nlohmann::json::input_format_t format = nlohmann::json::input_format_t::json) {
            std::stringstream error_stream;
            SolverInstanceSaxHandler<trace_info_type> solver_instance_sax(error_stream);
            SAXHandlerDispatch my_sax(solver_instance_sax, [&solver_instance_sax](SAXHandlerDispatch& dispatch_handler){
                return solver_instance_sax.callback(dispatch_handler);
            });
            if (!json::sax_parse(stream, &my_sax, format)) {
                throw std::runtime_error(error_stream.str());
            }
            return solver_instance_sax.get_solver_instance();
        }
    };
}

#endif //PDAAAL_SOLVERINSTANCEJSONPARSER_H
