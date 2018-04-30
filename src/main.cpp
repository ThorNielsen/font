#include "image.hpp"
#include "freetype.hpp"
#include "common.hpp"
#include "glyph.hpp"

#include <iostream>
#include <sstream>
#include <vector>
#include "primitives.hpp"
int intersectionCount(Ray ray, QuadraticBezier qb);

int main()
{
    QuadraticBezier qb;
    qb.p0 = {0,-10};
    qb.p1 = {10, 0};
    qb.p2 = {0, 10};
    //std::cout << "1 == " << intersectionCount(Ray{{0,0},{0,1}}, qb) << std::endl;
    //std::cout << "0 == " << intersectionCount(Ray{{0,0},{0,-1}}, qb) << std::endl;
    //std::cout << "2 == " << intersectionCount(Ray{{-10,2},{1,0}}, qb) << std::endl;
    std::cout << "0 == " << intersectionCount(Ray{{-10,2},{-1,0}}, qb) << std::endl;
    std::cout << "0 == " << intersectionCount(Ray{{-10,2},{-4,0}}, qb) << std::endl;

    //return 0;

    FT_Library ftLib;
    checkFTError(FT_Init_FreeType(&ftLib));

    FT_Face face;

    checkFTError(FT_New_Face(ftLib,
                             "font.ttf",
                             0,
                             &face));

    checkFTError(FT_Set_Pixel_Sizes(face, 0, 24));
    // For complicated glyph, use U+2593.
    // Slightly simpler: U+E5.
    // Even simpler: 'A'.
    // Simplest: 'H'.
    auto idx = FT_Get_Char_Index(face, 'X');
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
    FontInfo info(face);

    glyph.dumpInfo();

    img = render(info, glyph, 0, 24);
    img.name = "Custom";
    writeImage(img);

    checkFTError(FT_Done_FreeType(ftLib));
}
