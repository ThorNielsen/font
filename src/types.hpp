#ifndef MATH_TYPES_HPP_INCLUDED
#define MATH_TYPES_HPP_INCLUDED

#include <cstddef>
#include <cstdint>

// Exactly specified size.
using U64 = uint64_t;
using U32 = uint32_t;
using U16 = uint16_t;
using U8 = uint8_t;

using S64 = int64_t;
using S32 = int32_t;
using S16 = int16_t;
using S8 = int8_t;

// At least specified size.
using UL64 = uint_fast64_t;
using UL32 = uint_fast32_t;
using UL16 = uint_fast16_t;
using UL8 = uint_fast8_t;

using SL64 = int_fast64_t;
using SL32 = int_fast32_t;
using SL16 = int_fast16_t;
using SL8 = int_fast8_t;

using F64 = double;
using F32 = float;

// Tests only needed for floats as u?int[0-9]+_t is guaranteed to be the correct
// size (or not be defined) and tests would therefore be redundant. No such
// typedefs exists for floats, however.
static_assert(sizeof(F64) == 8, "F64 size.");
static_assert(sizeof(F32) == 4, "F32 size.");

#endif // MATH_TYPES_HPP_INCLUDED

