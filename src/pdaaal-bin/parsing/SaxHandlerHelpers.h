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

namespace pdaaal {

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

    template <size_t N>
    class SAXHandlerContext : utils::flags<N> {
        using parent_t = utils::flags<N>;
    public:
        using flag_t = typename parent_t::flag_t;
        using parent_t::flag;
        using parent_t::fill;
        using parent_t::is_single_flag;

        explicit constexpr SAXHandlerContext(flag_t flags) noexcept : values_left(flags) {};

        flag_t values_left;
        constexpr void got_value(flag_t value) {
            parent_t::remove(values_left, value);
        }
        [[nodiscard]] constexpr bool needs_value(flag_t value) const {
            return parent_t::contains(values_left, value);
        }
        [[nodiscard]] constexpr bool missing_keys() const {
            return values_left != parent_t::no_flags;
        }
        std::vector<flag_t> get_missing_flags() const {
            return parent_t::split_to_single_flags(values_left);
        }

    };


}

#endif //PDAAAL_SAXHANDLERHELPERS_H
