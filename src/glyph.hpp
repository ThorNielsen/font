#ifndef GLYPH_HPP_INCLUDED
#define GLYPH_HPP_INCLUDED

#include "freetype.hpp"
#include "image.hpp"
#include "vector2.hpp"

#include <vector>

class Glyph
{
public:
    struct GlyphInfo
    {
        int width; // Bounding box width
        int height; // Bounding box height
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


    bool isInside(ivec2 pos, ivec2 dir) const;

    size_t contours() const { return m_contourEnd.size(); }
    size_t positions() const { return m_position.size(); }

    size_t contourEnd(size_t c) const { return m_contourEnd[c]; }
    ivec2 position(size_t i) const { return m_position[i]; }
    bool isControl(size_t i) const { return m_isControlPoint[i]; }
    bool isThirdOrder(size_t i) const { return m_isThirdOrder[i]; }

private:
    std::vector<size_t> m_contourEnd;
    std::vector<ivec2> m_position;
    std::vector<bool> m_isControlPoint;
    std::vector<bool> m_isThirdOrder;
    GlyphInfo m_info;
};


Image render(const Glyph& glyph, int width, int height);

#endif // GLYPH_HPP_INCLUDED

