#include "image.hpp"
#include "freetype.hpp"
#include "common.hpp"
#include "glyph.hpp"

#include <iostream>
#include <sstream>
#include <vector>
#include "primitives.hpp"

int main()
{
    //QuadraticBezier curve{{1126, 588}, {1083, 618}, {1027, 615}};
    //std::cerr << intersectCount({1000, 615}, curve); // Expect 2
    //QuadraticBezier curve{{988, 705}, {1049, 709}, {1092, 735}};
    //std::cerr << intersectCount({980, 706}, curve); // Expect 1
    //QuadraticBezier curve{{988, 1155}, {988, 705}, {988, 705}};
    //std::cerr << intersectCount({980, 705}, curve); // Expect 1
    //QuadraticBezier curve{{1320, 270}, {1296, 333}, {1268, 396}};
    //std::cerr << intersectCount({1300, 270}, curve); // Expect 1
    //QuadraticBezier curve{{1041, 645}, {1058, 645}, {1080, 648}};
    //std::cerr << intersectCount({1000, 645}, curve); // Expect 1
    //return 0;
    FT_Library ftLib;
    checkFTError(FT_Init_FreeType(&ftLib));

    FT_Face face;

    checkFTError(FT_New_Face(ftLib,
                             "seriffont.ttf",
                             0,
                             &face));

    checkFTError(FT_Set_Pixel_Sizes(face, 0, 64));
    // For complicated glyph, use U+2593.
    // Slightly simpler: U+E5.
    // More simple: U+416.
    // Even simpler: 'A'.
    // Simplest: 'H'.
    bool drawSimple = true;
    auto idx = FT_Get_Char_Index(face, 0x416*drawSimple+!drawSimple*0x2593);
    checkFTError(FT_Load_Glyph(face, idx, FT_LOAD_NO_SCALE));
    checkFTError(FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL));

    FT_GlyphSlot slot = face->glyph;
    Glyph glyph(slot->outline, slot->metrics);
    FontInfo info(face);

    glyph.dumpInfo();

    Image img = render(info, glyph, 0, info.emSize);
    img.name = "Custom";
    writeImage(img);

    checkFTError(FT_Done_FreeType(ftLib));
}
