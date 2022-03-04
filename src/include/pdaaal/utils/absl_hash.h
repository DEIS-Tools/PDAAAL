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
 * File:   absl_hash.h
 * Author: Morten K. Schou <morten@h-schou.dk>
 *
 * Created on 03-03-2022.
 */

#ifndef PDAAAL_ABSL_HASH_H
#define PDAAAL_ABSL_HASH_H

// All code that needs absl::hash should include this file instead of <absl/hash/hash.h>
// This allows us to compile with -Wpedantic without being spammed by warnings from Abseil.
// In the case of Abseil, these warnings are false positives, and the Abseil team has decided to not support -Wpedantic.
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <absl/hash/hash.h>
#pragma GCC diagnostic pop
#else
#include <absl/hash/hash.h>
#endif

#endif //PDAAAL_ABSL_HASH_H
