#include "common.hpp"
#include "crc.hpp"
#include "freetype.hpp"
#include "glyph.hpp"
#include "image.hpp"
#include "primitives.hpp"
#include "timer.hpp"

#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

std::map<int, U32> readChecksums(std::string fontname)
{
    std::map<int, U32> checksums;
    std::ifstream input((fontname + ".crc32").c_str());
    if (!input.is_open()) return checksums;
    while (input.good())
    {
        int glyph;
        U32 sum;
        input >> glyph >> sum;
        checksums[glyph] = sum;
    }
    return checksums;
}

bool writeChecksums(std::string fontname, std::map<int, U32> checksums)
{
    std::ofstream output((fontname + ".crc32").c_str());
    if (!output.is_open()) return false;
    for (auto& sum : checksums)
    {
        output << sum.first << ' ' << sum.second << '\n';
    }
    return true;
}

int main()
{
    FT_Library ftLib;
    checkFTError(FT_Init_FreeType(&ftLib));

    std::vector<std::string> faces
    {
        "decorative", "special", "sans", "serif", "complex"
    };


    bool validate = true;
    bool writeImages = true;
    bool updateChecksums = false;

    for (auto& fontname : faces)
    {
    FT_Face face;
    checkFTError(FT_New_Face(ftLib,
                             ("fonts/" + fontname + ".ttf").c_str(),
                             0,
                             &face));
    checkFTError(FT_Set_Pixel_Sizes(face, 0, 64));

    std::map<int, U32> checksums = readChecksums(fontname);

    std::cerr << "Rendering font '" << fontname << "' [";
    std::cerr << face->num_glyphs << " glyphs].\n";

    Timer timer;
    timer.start();
    for (int idx = 0; idx < face->num_glyphs; ++idx)
    {
        //if (idx > 50) break;
        std::stringstream name;
        name << idx;
        std::cerr << "Rendering glyph #" << name.str() << "...";
        checkFTError(FT_Load_Glyph(face, idx, FT_LOAD_NO_SCALE));
        FT_GlyphSlot slot = face->glyph;
        try
        {
            U32 checksum;
            Glyph glyph(slot->outline, slot->metrics);
            FontInfo info(face);
            Image img = render(info, glyph, 0, info.emSize);
            img.name = "output/" + fontname + "_" + name.str() + ".pnm";

            if (writeImages)
            {
                writeImage(img);
            }

            if (validate)
            {
                checksum = crc(img.p.data(), img.p.size());
                if (checksums.find(idx) == checksums.end())
                {
                    std::cerr << " done, but unvalidated!\n";
                }
                else
                {
                    std::cerr << ((checksum == checksums[idx])
                                  ? " good.\n"
                                  : " \033[1;31mBAD!\033[0m\n");
                }
            }
            else
            {
                std::cerr << " done!\n";
            }
            if (updateChecksums)
            {
                if (!validate) checksum = crc(img.p.data(), img.p.size());
                checksums[idx] = checksum;
            }

        }
        catch (const std::runtime_error& err)
        {
            std::cerr << " FAILED: " << err.what() << "\n";
        }
    }
    timer.stop();
    std::cerr << "Total time: " << timer.duration() << "\n";

    checkFTError(FT_Done_Face(face));

    if (updateChecksums)
    {
        writeChecksums(fontname, checksums);
    }
    break;

    }


    checkFTError(FT_Done_FreeType(ftLib));
}
