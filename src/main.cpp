#include "common.hpp"
#include "freetype.hpp"
#include "glyph.hpp"
#include "image.hpp"
#include "primitives.hpp"
#include "timer.hpp"

#include <iostream>
#include <sstream>
#include <vector>

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

    std::vector<Image> images;

    std::cerr << "Rendering font '" << fontname << "' [";
    std::cerr << face->num_glyphs << " glyphs].\n";

    Timer timer;
    timer.start();
    for (int idx = 0; idx < face->num_glyphs; ++idx)
    {
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
            images.emplace_back(std::move(img));
            std::cerr << " done!\n";
        }
        catch (const std::runtime_error& err)
        {
            std::cerr << " FAILED: " << err.what() << "\n";
        }
    }
    timer.stop();
    std::cerr << "Total render time: " << timer.duration() << "\n";

    std::cerr << "Writing images...";
    for (auto& image : images)
    {
        writeImage(image);
    }
    std::cerr << " DONE!\n";

    checkFTError(FT_Done_Face(face));
    break;

    }


    checkFTError(FT_Done_FreeType(ftLib));
}
