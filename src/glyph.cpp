#include "glyph.hpp"
#include "common.hpp"
#include "matrix2.hpp"
#include "primitives.hpp"

#include <iostream>

Glyph::Glyph(FT_Outline outline)
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
}

void Glyph::dumpInfo() const
{
    std::cout << "=== Glyph outline ===\n";
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

int intersectionCount(Ray r, LineSegment l)
{
    auto invDet = l.dir.x * r.dir.y - l.dir.y * r.dir.x;
    if (!invDet) // Degenerate case (parallel)
    {
        auto ad = l.pos - r.pos;
        if (ad.x * r.dir.y == ad.y * r.dir.x) // Same line
        {
            auto adrd = dot(ad, r.dir);
            auto adldrd = adrd + dot(l.dir, r.dir);
            if (dot(r.dir, l.dir) >= 0) // Same direction
            {
                int ret = 0;
                if (adrd > 0) ++ret;
                if (adldrd > 0) ++ret;
                std::cout << ret;
                return ret;
            }
            else // Opposite direction
            {
                int ret = 0;
                if (adrd <= 0) ++ret;
                if (adldrd <= 0) ++ret;
                std::cout << 8-ret;
                return ret;
            }
        }
        // Parallel but non-intersecting.
        std::cout << 'a';
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
        std::cout << 'b';
        return 0;
    }
    if (coeffs.x < 0)
    {
        // Ray is below line segment.
        std::cout << 'c';
        return 0;
    }
    if (invDet < coeffs.x)
    {
        // Ray is above line segment.
        std::cout << 'd';
        return 0;
    }
    // Ray intersects line segment non-degenerately.
    std::cout << 'e';
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
            intersectCount += intersectionCount(testRay, cLine);

            cLine.pos = position(j);
        }
    }
    return intersectCount & 1;
}
