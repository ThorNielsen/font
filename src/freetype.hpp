#ifndef GRAPHICS_FONT_FREETYPE_HPP_INCLUDED
#define GRAPHICS_FONT_FREETYPE_HPP_INCLUDED

#include "types.hpp"
#include <stdexcept>
#include <string>

#include <ft2build.h>
#include FT_FREETYPE_H

inline void printFTError(FT_Error fterr, std::string func, U32 line)
{
    if (fterr)
    {
        throw std::runtime_error("Freetype error (in "
                                 + func
                                 + ", line "
                                 + std::to_string(line)
                                 + "): "
                                 + std::to_string(fterr));
    }
}

#define checkFTError(fterr)\
    do{ printFTError(fterr, __func__, __LINE__); } while(false)

#endif // GRAPHICS_FONT_FREETYPE_HPP_INCLUDED

