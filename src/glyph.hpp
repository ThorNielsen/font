#ifndef GLYPH_HPP_INCLUDED
#define GLYPH_HPP_INCLUDED

#include "compressedbitmap.hpp"
#include "freetype.hpp"
#include "image.hpp"
#include "primitives.hpp"
#include "vector2.hpp"

#include <vector>

class Glyph
{
public:
    struct GlyphInfo
    {
        int width; // Bounding box width in grid units
        int height; // Bounding box height in grid units
        // For horizontal text layouts:
        int hCursorX; // Horizontal distance from cursor position to leftmost
                      // border of the bounding box.
        int hCursorY; // Vertical distance from cursor position (on baseline)
                      // to topmost border of the bounding box.
        int xAdvance; // Distance to advance the cursor position (horizontally)
                      // after drawing the current glyph.
        // For vertical text layouts:
        int vCursorX; // Horizontal(!) distance from cursor position to left-
                      // most border of bounding box.
        int vCursorY; // Vertical distance from baseline to topmost border of
                      // bounding box.
        int yAdvance; // Distance to advance the cursor position (vertically)
                      // after this glyph has been drawn.
    };

    Glyph(FT_Outline, FT_Glyph_Metrics);

    void dumpInfo() const;

    const GlyphInfo& info() const { return m_info; }

    bool isInside(vec2 pos) const noexcept;

    const CompressedBitmap& getMap() const { return m_bitmap; }
private:

    void extractOutlines(const std::vector<size_t>& contourEnd,
                         const std::vector<ivec2>& position,
                         const std::vector<bool>& control);
    void processCurves(const std::vector<PackedBezier>& curves);
    void createLookup(size_t logLength,
                      const std::vector<PackedBezier>& curves);

    void sortByY(std::vector<PackedBezier>& curves);

    std::vector<PackedBezier> m_curves;

    CompressedBitmap m_bitmap;
    std::vector<size_t> m_rowindices;
    size_t m_boxLength;

    GlyphInfo m_info;
};

int intersect(vec2 pos, PackedBezier bezier, float& minusX, float& plusX) noexcept;

struct FontInfo
{
    FontInfo(FT_Face);
    // Bounding box large enough to contain all glyphs in the font (not at once,
    // of course).
    ivec2 bboxMin; // Bottom left.
    ivec2 bboxMax; // Top right.
    int emSize; // Size of EM square (EM units).
    int ascender; // Vertical distance from horizontal baseline to highest
                  // character coordinate in font. May not be reliable.
    int descender;// Vertical distance from horizontal baseline to lowest
                  // character coordinate in font. Negative if below baseline.
                  // May not be reliable.
    int lineHeight; // Default spacing between two lines.
    int maxAdvanceWidth;
    int maxAdvanceHeight;
    int underlinePosition; // Underline position relative to baseline. Negative
                           // if below. Note that this position is the CENTER of
                           // the underline - i.e. the underline's vertical
                           // extents is [pos-thickness/2, pos+thickness/2]
                           // (account for truncation, of course).
    int underlineThickness;
};


Image render(const FontInfo& info, const Glyph& glyph, int width, int height);

#endif // GLYPH_HPP_INCLUDED

