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
 * File:   SaxHandlerHelpers.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 23-02-2022.
 */

#ifndef PDAAAL_SAXHANDLERHELPERS_H
#define PDAAAL_SAXHANDLERHELPERS_H

#include <pdaaal/utils/flags.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <cassert>

//using json = nlohmann::json;

namespace pdaaal::parsing {

    // We wish to be able to stop parsing before the end of input, but still handle errors correctly.
    // Keep track of whether the error stream was used, and this way distinguish between error and early return.
    class SAXHandlerBase {
    public:
        explicit SAXHandlerBase(std::ostream& errors) : _errors(errors) {};

        bool parse_error(std::size_t location, const std::string& last_token, const nlohmann::detail::exception& e) {
            errors() << "error at line " << location << " with last token " << last_token << ". " << std::endl;
            errors() << "\terror message: " << e.what() << std::endl;
            return false;
        }
        [[nodiscard]] bool is_errored() const { return _errored; }

    protected:
        std::ostream& errors() {
            _errored = true;
            return _errors;
        }
    private:
        std::ostream& _errors;
        bool _errored = false;
    };

    template<typename context_type, std::size_t N>
    struct parser_object_context : public utils::flag_mask<N> {
        using type_t = context_type; // Expose template parameter.
        using parent_t = utils::flag_mask<N>;
        using key_flag = typename parent_t::flag_t;
        using parent_t::flag;
        using parent_t::fill;
        using parent_t::is_single_flag;
        constexpr parser_object_context(context_type type, key_flag flags) noexcept : parent_t(flags), type(type) {};
        const context_type type;
    };
    template<typename context, typename context::type_t type, size_t n_flags>
    static constexpr context make_object_context() {
        return context(type, context::template fill<n_flags>());
    }

}

#endif //PDAAAL_SAXHANDLERHELPERS_H
