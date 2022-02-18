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
 * File:   PDAFactory_test.cpp
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 15-04-2020.
 */

#define BOOST_TEST_MODULE PDAFactory

#include <boost/test/unit_test.hpp>
#include <pdaaal/PDAFactory.h>
#include <pdaaal/cegar/CegarPdaFactory.h>

using namespace pdaaal;

BOOST_AUTO_TEST_CASE(PDAFactoryTest1) {
    // We don't really have any good tests here, but #including PDAFactory.h makes it part of code analysis in CLion.
}

