#ifndef GLYPH_HPP_INCLUDED
#define GLYPH_HPP_INCLUDED

#include "freetype.hpp"
#include "vector2.hpp"

#include <vector>

class Glyph
{
public:
    Glyph(FT_Outline);

    void dumpInfo() const;

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
};

#endif // GLYPH_HPP_INCLUDED

