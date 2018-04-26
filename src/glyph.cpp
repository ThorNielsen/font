#include "glyph.hpp"
#include "common.hpp"

#include <iostream>

Glyph::Glyph(FT_Outline outline)
    : m_contourEnds(outline.n_contours),
      m_position(outline.n_points),
      m_isControlPoint(outline.n_points),
      m_isThirdOrder(outline.n_points)
{
    for (short i = 0; i < outline.n_points; ++i)
    {
        m_position[i] = ivec2{static_cast<S32>(outline.points[i].x),
                              static_cast<S32>(outline.points[i].y)};
        m_isControlPoint[i] = bit<0>(outline.tags[i]);
        m_isThirdOrder[i] = bit<1>(outline.tags[i]);
    }
    for (short i = 0; i < outline.n_contours; ++i)
    {
        m_contourEnds[i] = outline.contours[i]+1;
    }
}
