#include "image.hpp"
#include "freetype.hpp"
#include "common.hpp"
#include "glyph.hpp"

#include <iostream>
#include <sstream>
#include <vector>

int main()
{
    FT_Library ftLib;
    checkFTError(FT_Init_FreeType(&ftLib));

    FT_Face face;

    checkFTError(FT_New_Face(ftLib,
                             "font.ttf",
                             0,
                             &face));

    checkFTError(FT_Set_Pixel_Sizes(face, 0, 32));
    // For complicated glyph, use U+2593.
    // Slightly simpler: U+E5.
    // Even simpler: 'A'.
    // Simplest: 'H'.
    auto idx = FT_Get_Char_Index(face, 'o');
    checkFTError(FT_Load_Glyph(face, idx, FT_LOAD_NO_SCALE));
    checkFTError(FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL));

    FT_GlyphSlot slot = face->glyph;

    FT_Bitmap bitmap = slot->bitmap;
    Image img(bitmap.width, bitmap.rows, "Character");

    const U8* bmData = static_cast<const U8*>(bitmap.buffer);
    auto pitch = bitmap.pitch;

    for (size_t y = 0; y < img.height; ++y)
    {
        for (size_t x = 0; x < img.width; ++x)
        {
            U8 val = bmData[y*pitch+x];
            img.setPixel(x, y, Colour(val, val, val));
        }
    }
    writeImage(img);

    Glyph glyph(slot->outline, slot->metrics);

    glyph.dumpInfo();

    checkFTError(FT_Done_FreeType(ftLib));
}
