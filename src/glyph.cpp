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

// This finds the number of intersections between a ray and a bezier curve using
// only integer operations (and comparisons and control flow and variables).
// The derivations are quite complicated (not very difficult, just numerous and
// time-consuming) so I have probably made a mistake somewhere. The derivations
// are included below to assist in fixing any potential problems and to better
// describe what is going on where.
int intersectionCount(Ray ray, QuadraticBezier qb)
{
    // Extract coefficients from ray and bezier curve:
    auto a = ray.pos.x; auto c = ray.dir.x;
    auto b = ray.pos.y; auto d = ray.dir.y;
    // The ray can now be written parametrically as:
    // x(s) = a+s*c
    // y(s) = b+s*d
    // For s >= 0.
    auto e = qb.p0.x; auto g = qb.p1.x; auto j = qb.p2.x;
    auto f = qb.p0.y; auto h = qb.p1.y; auto k = qb.p2.y;
    // Note that the bezier curve can now be written parametrically as:
    // x(t) = (e-2g+j)t^2+2(g-e)t+e
    // y(t) = (f-2h+k)t^2+2(h-f)t+f
    // For 0 <= t < 1. Note that the last inequality must be sharp since an
    // intersection at an endpoint would also be an intersection at the
    // beginning of the next curve leading to two intersections in a single
    // point.
    // To find the intersection count we simply need to find the number of
    // solutions to the following equations:
    // a+sc=(e-2g+j)t^2+2(g-e)t+e
    // b+sd=(f-2h+k)t^2+2(h-f)t+f
    // Of course, we must make sure that the parameters are in range, so a
    // solution (s, t) to the above is valid iff s >= 0 and 0 <= t < 1.
    // First we isolate s:
    // s = c^{-1}((e-2g+j)t^2+2(g-e)t+e-a)
    // Note that we have assumed that c is invertible. However if it is not, we
    // can simply flip the coefficients since the equations are symmetric.
    if (c == 0)
    {
        // However what if d is zero as well? Then we have a degenerate ray, so
        // we simply return zero intersections (we could of course test if the
        // ray's origin lies on the curve, but giving a degenerate ray is
        // probably a bug).
        if (d == 0) return 0;
        std::swap(a, b); std::swap(c, d);
        std::swap(e, f); std::swap(g, h); std::swap(j, k);
    }
    // Now we can proceed. We insert s in the second equation and re-arranging,
    // we get  At^2+Bt+C=0  where
    auto A = d*(e-2*g+j)-c*(f-2*h+k);
    auto B = 2*(d*(g-e)-c*(h-f));
    auto C = c*(b-f)+d*(e-a);
    // are the ugly coefficients. Now we calculate the discriminant:
    auto D = B*B-4*A*C;
    // Of course, if the discriminant is negative, we have no solutions. The
    // situation will look like this:
    //    \     \
    //      \    |
    //       |    \
    //      /      |
    //    /         \
    // Bezier      Line, which contains ray
    if (D < 0) return 0;

    // Depending on whether D = 0 or D > 0, the line and infinitely extended
    // bezier curve intersects somewhere in one or two points, respectively.
    // However, we need 0 <= t < 1 and s >= 0. Since we have the coefficients
    // for At^2+Bt+C=0, we start with t. Note that we do not want to bring
    // floating-point arithmetic into this so we rearrange until we get integer
    // inequalities.

    // We need to remember which of our solutions satisfies the parameter
    // constraints.
    // If D = 0, sol holds the single solution. Otherwise:
    bool sol = false;    // Is  -B/2A + sqrt(D)/2A  a valid t? [Plus solution]
    bool solSub = false; // Is  -B/2A - sqrt(D)/2A  a valid t? [Minus solution]

    // We look at the case where A > 0. The case where A < 0 will be handled
    // later. Please note that A != 0 since the bezier curve would otherwise be
    // a line segment, a case which is caught in pre-processing.
    if (A > 0)
    {
        // If D=0 we only have one solution and therefore we get t=-B/2A. This
        // can be rearranged as follows:
        // 0 <= t < 1      <=>
        // 0 <= -B/2A < 1  <=>
        // 0 <= -B < 2A
        if (D == 0)
        {
            if (0 <= -B && -B < 2*A) sol = true;
        }
        else // D > 0
        {
            // We have two potential solutions. Let us look at plus:
            // 0 <= -B/2A + sqrt(D)/2A < 1  <=>
            // 0 <= -B+sqrt(D) < 2A         <=>
            // B <= sqrt(D) < 2A+B

            // First we look at B <= sqrt(D):
            //   if B >= 0 then
            //     B   <= sqrt(D)  <=>
            //     B^2 <= D        <=>
            //     0   <= -4AC     <=>
            //     0   >= C
            //   if B < 0 then
            //     B <= sqrt(D)
            //     is simply true.
            // To sum up, the  B <= sqrt(D)  is satisfied iff:
            // [(B >= 0) and (C <= 0)] or (B <= 0)
            // This reduces further to  (B <= 0) or (C <= 0)  .

            // Onwards to the second equation,  sqrt(D) < 2A+B  .
            // If  2A+B <= 0  , this cannot be true. Otherwise we can simply
            // square both sides and obtain:
            // D    < 4A^2+4AB+B^2  <=>
            // -4AC < 4A^2+4AB      <=>
            // 0    < 4A^2+4AB+4AC  <=>
            // 0    < A+B+C
            // Giving that  sqrt(D) < 2A+B  is satisfied iff:
            // (2A+B > 0) and (A+B+C > 0)
            if (((B <= 0) || (C <= 0)) && (2*A+B > 0) && (A+B+C > 0))
            {
                // The plus root satisfies the constraints on t.
                sol = true;
            }
        }
    }
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

FontInfo::FontInfo(FT_Face face)
{
    bboxMin.x = static_cast<int>(face->bbox.xMin);
    bboxMin.y = static_cast<int>(face->bbox.yMin);
    bboxMax.x = static_cast<int>(face->bbox.xMax);
    bboxMax.y = static_cast<int>(face->bbox.yMax);
    emSize = static_cast<int>(face->units_per_EM);
    ascender = static_cast<int>(face->ascender);
    descender = static_cast<int>(face->descender);
    lineHeight = static_cast<int>(face->height);
    maxAdvanceWidth = static_cast<int>(face->max_advance_width);
    maxAdvanceHeight = static_cast<int>(face->max_advance_height);
    underlinePosition = static_cast<int>(face->underline_position);
    underlineThickness = static_cast<int>(face->underline_thickness);
}

Image render(const FontInfo& info, const Glyph& glyph, int width, int height)
{
    int pixelWidth, pixelHeight;
    (void)(info);
    if (width <= 0)
    {
        if (height <= 0)
        {
            throw std::runtime_error("Bad render size.");
        }
        pixelHeight = (height * glyph.height) / info.emSize;
        pixelWidth = (pixelHeight * width) / height;
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
