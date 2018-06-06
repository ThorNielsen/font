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
    FT_Library ftLib;
    checkFTError(FT_Init_FreeType(&ftLib));

    std::vector<std::string> faces
    {
        "decorative", "special", "sans", "serif", "complex"
    };

    for (auto& fontname : faces)
    {
    FT_Face face;

    checkFTError(FT_New_Face(ftLib,
                             ("fonts/" + fontname + ".ttf").c_str(),
                             0,
                             &face));

    checkFTError(FT_Set_Pixel_Sizes(face, 0, 64));

    std::cerr << "Beginning font '" << fontname << "'.\n";
    std::cerr << "Glyph count: " << face->num_glyphs << "\n";
    for (int idx = 0; idx < face->num_glyphs; ++idx)
    {
        //if (idx != 33 && idx != 34) continue;
        if (idx != 32) continue;
        std::stringstream name;
        name << idx;
        std::cerr << "Rendering glyph #" << name.str() << "...";
        checkFTError(FT_Load_Glyph(face, idx, FT_LOAD_NO_SCALE));
        FT_GlyphSlot slot = face->glyph;
        try
        {
            Glyph glyph(slot->outline, slot->metrics);
            FontInfo info(face);
            Image img = render(info, glyph, 0, info.emSize);
            img.name = "output/" + fontname + "_" + name.str() + ".pnm";
            //std::cerr << " and reverse ...";
            //glyph.reverseCurves();
            //Image rev = render(info, glyph, 0, info.emSize);
            //rev.name = "output/" + fontname + "_" + name.str() + "_r.pnm";
            std::cerr << " done!\n";
            /*if (img != rev)
                std::cerr << "\033[1;31mWARNING\033[0m: glyph "
                          << name.str() << " is rendered inconsistently.\n";*/
            writeImage(img);
            /*if (img != rev)
                writeImage(rev);*/
        }
        catch (const std::runtime_error& err)
        {
            std::cerr << " FAILED: " << err.what() << "\n";
        }
    }

    checkFTError(FT_Done_Face(face));
    break;

    }

    checkFTError(FT_Done_FreeType(ftLib));
}
