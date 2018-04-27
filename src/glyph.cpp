#include "glyph.hpp"
#include "common.hpp"
#include "matrix2.hpp"
#include "primitives.hpp"

#include <iostream>
#include <stdexcept>

Glyph::Glyph(FT_Outline outline, FT_Glyph_Metrics metrics)
    : m_contourEnd(outline.n_contours),
      m_position(outline.n_points),
      m_isControlPoint(outline.n_points),
      m_isThirdOrder(outline.n_points)
{
    for (short i = 0; i < outline.n_points; ++i)
    {
        m_position[i] = ivec2{static_cast<S32>(outline.points[i].x),
                              static_cast<S32>(outline.points[i].y)};
        m_isControlPoint[i] = !bit<0>(outline.tags[i]);
        m_isThirdOrder[i] = bit<1>(outline.tags[i]);
    }
    for (short i = 0; i < outline.n_contours; ++i)
    {
        m_contourEnd[i] = outline.contours[i]+1;
    }
    m_info.width = static_cast<int>(metrics.width);
    m_info.height = static_cast<int>(metrics.height);
    m_info.hCursorX = static_cast<int>(metrics.horiBearingX);
    m_info.hCursorY = static_cast<int>(metrics.horiBearingY);
    m_info.xAdvance = static_cast<int>(metrics.horiAdvance);
    m_info.vCursorX = static_cast<int>(metrics.vertBearingX);
    m_info.vCursorY = static_cast<int>(metrics.vertBearingY);
    m_info.yAdvance = static_cast<int>(metrics.vertAdvance);
}

void Glyph::dumpInfo() const
{
    std::cout << "=== Glyph outline ===\n";
    std::cout << "BBox: " << m_info.width << "x" << m_info.height << "\n";
    std::cout << "Horizontal mode offset: (" << m_info.hCursorX << ", "
              << m_info.hCursorY << ")\n";
    std::cout << "Horizontal mode advance: " << m_info.xAdvance << "\n";
    std::cout << "Vertical mode offset: (" << m_info.vCursorX << ", "
              << m_info.vCursorY << ")\n";
    std::cout << "Vertical mode advance: " << m_info.yAdvance << "\n";
    std::cout << "Contour count: " << m_contourEnd.size() << "\n";
    std::cout << "Point count: " << m_position.size() << "\n";
    size_t p = 0;
    for (size_t i = 0; i < m_contourEnd.size(); ++i)
    {
        std::cout << "Contour #" << i << ":\n";
        for (size_t j = p; j < m_contourEnd[i]; ++j)
        {
            std::cout << "Point[" << j << "]: "
                      << m_position[j] << " "
                      << "[Control: " << m_isControlPoint[j];
            if (m_isControlPoint[j])
            {
                std::cout << ", ThirdOrder: " << m_isThirdOrder[j];
            }
            std::cout << "]\n";
        }
        std::cout << "\n";
        p = m_contourEnd[i];
    }
    std::cout << "\n";
}

bool intersects(Ray r, LineSegment l, bool& rayBeginOnSegment)
{
    auto ad = l.pos - r.pos;
    // Test whether ray beginning is on line segment.
    if (ad.x * l.dir.y == ad.y * l.dir.x)
    {
        // Now we know that the ray starts on the line which l lies on. Now we
        // need to know whether the ray start lies on the segment. We have the
        // following possible configurations (L is a line, numeric is ray start
        // possibilities, note that line segment is directed):
        //         B       E
        // L:      [->--->-]
        // 0:  R
        // 1:      R
        // 2:          R
        // 3:              R
        // 4:                  R

        auto lBegin = dot(l.pos, l.dir);
        auto rPos = dot(r.pos, l.dir);
        auto lEnd = dot(l.pos + l.dir, l.dir);
        if (lBegin <= rPos)
        {
            // Case 1&2&3:
            if (rPos <= lEnd)
            {
                rayBeginOnSegment = true;
                return 1;
            }
            // Case 4, do nothing.
        }
        // else: Case 0

        // Test if ray has same direction as line segment:
        if (ad.x * r.dir.y == ad.y * r.dir.x)
        {
            // We either have case 0 or case 4. RB and the ray have same
            // direction then we have an intersection (actually infinitely
            // many), otherwise not. Note that RB = l.pos - r.pos the ray's
            // direction is simply r.dir.
            return dot(l.pos - r.pos, r.dir) > 0;
        }
    }

    auto invDet = l.dir.x * r.dir.y - l.dir.y * r.dir.x;
    if (!invDet)
    {
        // They are parallel, but cannot be intersecting (tested before).
        return 0;
    }
    imat2 m( r.dir.y, -r.dir.x,
            -l.dir.y,  l.dir.x);
    ivec2 coeffs = m * (r.pos - l.pos);
    if (invDet < 0)
    {
        coeffs.x = -coeffs.x;
        invDet = -invDet;
    }
    else
    {
        coeffs.y = -coeffs.y;
    }
    if (coeffs.y < 0)
    {
        // Line is behind ray.
        return 0;
    }
    if (coeffs.x < 0)
    {
        // Ray is below line segment.
        return 0;
    }
    if (invDet <= coeffs.x)
    {
        // Ray is above line segment.
        return 0;
    }
    // Ray intersects line segment non-degenerately.
    return 1;
}

// Currently only handles line segments.
bool Glyph::isInside(ivec2 pos, ivec2 dir) const
{
    size_t intersectCount = 0;
    Ray testRay;
    testRay.dir = dir; //{1, 0};
    testRay.pos = pos;
    size_t contourBegin = 0;
    for (size_t i = 0; i < contours(); ++i)
    {
        size_t contourLength = contourEnd(i) - contourBegin;
        if (contourLength < 3) continue; // Degenerate contour.
        LineSegment cLine;
        cLine.pos = position(contourBegin + contourLength - 1);
        for (size_t j = contourBegin; j < contourEnd(i); ++j)
        {
            cLine.dir = position(j) - cLine.pos;
            bool rayBeginOnSegment = false;
            intersectCount += intersects(testRay, cLine, rayBeginOnSegment);
            if (rayBeginOnSegment) return true;
            cLine.pos = position(j);
        }
    }
    return intersectCount&1;
}


Image render(const Glyph& glyph, int width, int height)
{
    if (width <= 0)
    {
        if (height <= 0)
        {
            throw std::runtime_error("Bad render size.");
        }
        width = (glyph.info().width*height + glyph.info().height-1)
                / glyph.info().height;
    }
    else
    {
        height = (glyph.info().height*width + glyph.info().width-1)
                 / glyph.info().width;
    }
    Image img(width, height);
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            ivec2 glyphPos;
            glyphPos.x = glyph.info().hCursorX + x*glyph.info().width/(width-1);
            glyphPos.y = glyph.info().hCursorY - y*glyph.info().height/(height-1);
            if (glyph.isInside(glyphPos, ivec2{1, 0}))
            {
                img.setPixel(x, y, 0xffffff);
            }
            else
            {
                img.setPixel(x, y, 0x000000);
            }
        }
    }
    return img;
}
