#include "image.hpp"
#include "freetype.hpp"
#include "common.hpp"

#include <iostream>
#include <sstream>
#include <vector>

std::ostream& operator<<(std::ostream& ost, FT_Vector vec)
{
    return ost << "FT_Vector{" << vec.x << ", " << vec.y << "}";
}


std::string decodeTag(U8 tag)
{
    std::stringstream ss;
    ss << "Tag{";
    ss << "OnCurve: " << bit<0>(tag);
    if (!bit<0>(tag))
    {
        ss << " [" << (bit<1>(tag) ? "Third" : "Second") << " order]";
    }
    ss << "}";
    return ss.str();
}

void dumpInfo(FT_Outline outline)
{
    std::cout << "=== Glyph data === \n";
    std::cout << "Contour count: " << outline.n_contours << "\n";
    std::cout << "Point count: " << outline.n_points << "\n";
    short p = 0;
    for (short i = 0; i < outline.n_contours; ++i)
    {
        std::cout << "Contour #" << i << ":\n";
        for (short j = p; j <= outline.contours[i]; ++j)
        {
            std::cout << "Point[" << j << "]: "
                      << outline.points[j] << " "
                      << decodeTag(outline.tags[j])
                      << "\n";
        }
        p = outline.contours[i]+1;
    }
}


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
    auto idx = FT_Get_Char_Index(face, 'H');
    checkFTError(FT_Load_Glyph(face, idx, FT_LOAD_DEFAULT));
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

    dumpInfo(slot->outline);

    checkFTError(FT_Done_FreeType(ftLib));
}
