#include "glyph.hpp"
#include "common.hpp"
#include "matrix2.hpp"
#include "primitives.hpp"

#include <algorithm>
#include <iostream>
#include <map>
#include <stdexcept>

Glyph::Glyph(FT_Outline outline, FT_Glyph_Metrics metrics)
{
    std::vector<size_t> contourEnd(outline.n_contours);
    std::vector<ivec2> position;
    std::vector<bool> isControl;
    std::vector<bool> zeroAcceptable;

    for (short i = 0; i < outline.n_contours; ++i)
    {
        contourEnd[i] = outline.contours[i]+1;
    }

    short contour = 0;
    bool prevControl = false;
    for (short i = 0; i < outline.n_points; ++i)
    {
        while (static_cast<size_t>(i) == contourEnd[contour]) ++contour;

        bool thirdOrder = bit<1>(outline.tags[i]);
        if (thirdOrder)
        {
            throw std::runtime_error("Third order Bézier curves unsupported.");
        }

        auto currPos = ivec2{static_cast<S32>(outline.points[i].x),
                             static_cast<S32>(outline.points[i].y)};
        bool currControl = !bit<0>(outline.tags[i]);
        if (currControl && prevControl)
        {
            position.push_back((currPos + position.back())/2);
            isControl.push_back(false);
            ++contourEnd[contour];
        }
        prevControl = currControl;
        position.push_back(currPos);
        isControl.push_back(currControl);
    }

    zeroAcceptable.resize(position.size());

    extractOutlines(contourEnd, position, isControl, zeroAcceptable);

    m_info.width = static_cast<int>(metrics.width);
    m_info.height = static_cast<int>(metrics.height);
    m_info.hCursorX = static_cast<int>(metrics.horiBearingX);
    m_info.hCursorY = static_cast<int>(metrics.horiBearingY);
    m_info.xAdvance = static_cast<int>(metrics.horiAdvance);
    m_info.vCursorX = static_cast<int>(metrics.vertBearingX);
    m_info.vCursorY = static_cast<int>(metrics.vertBearingY);
    m_info.yAdvance = static_cast<int>(metrics.vertAdvance);
}

void Glyph::extractOutlines(const std::vector<size_t>& contourEnd,
                            const std::vector<ivec2>& position,
                            const std::vector<bool>& control,
                            std::vector<bool>& zeroAcceptable)
{
    computeUsableZeroes(contourEnd, position, zeroAcceptable);

    std::multimap<int, ivec2> horLines;

    size_t contourBegin = 0;
    for (size_t contour = 0; contour < contourEnd.size(); ++contour)
    {
        auto prevIdx = contourEnd[contour]-1;
        auto prevPos = position[prevIdx];
        bool prevControl = control[prevIdx];
        for (size_t i = contourBegin; i < contourEnd[contour]; prevIdx = i++)
        {
            auto cPos = position[i];
            if (zeroAcceptable[i] && !control[i])
            {
                m_critPoints.emplace_back(ivec2{cPos.x, cPos.y});
            }
            if (control[i])
            {
                auto nextIdx = i+1-contourBegin;
                nextIdx %= (contourEnd[contour]-contourBegin);
                nextIdx += contourBegin;
                m_bezier.push_back({prevPos, cPos, position[nextIdx]});
                prevControl = true;
            }
            else if (!prevControl)
            {
                if (prevPos.y == cPos.y)
                {
                    horLines.emplace(cPos.y,
                                     ivec2{std::min(prevPos.x, cPos.x),
                                           std::max(prevPos.x, cPos.x)});
                }
                else
                {
                    if (cPos.y > prevPos.y)
                    {
                        m_segments.push_back({prevPos, cPos});
                    }
                    else
                    {
                        m_segments.push_back({cPos, prevPos});
                    }
                }
            }
            else
            {
                prevControl = false;
            }
            prevPos = cPos;
        }
        contourBegin = contourEnd[contour];
    }

    for (auto line = horLines.begin(), elem = line;
         line != horLines.end();
         ++line)
    {
        std::vector<ivec2> hseg;
        for (; elem != horLines.end() && elem->first == line->first; ++elem)
        {
            hseg.push_back(elem->second);
        }
        std::sort(hseg.begin(), hseg.end(),
                  [](const ivec2& a, const ivec2& b)
                  {
                      if (a.x == b.x) return a.y < b.y;
                      return a.x < b.x;
                  });
        int prevEnd = std::numeric_limits<int>::min();
        for (auto& seg : hseg)
        {
            if (seg.x <= prevEnd)
            {
                m_horSegments.back().xmax = seg.y;
            }
            else
            {
                m_horSegments.push_back({line->first, seg.x, seg.y});
            }
            prevEnd = seg.y;
        }
    }

    std::sort(m_critPoints.begin(), m_critPoints.end(),
              [](const ivec2& a, const ivec2 b)
              {
                  return a.y < b.y;
              });

    std::sort(m_segments.begin(), m_segments.end(),
              [](const LineSegment& a, const LineSegment& b)
              {
                  if (a.pos.y == b.pos.y) return a.pos.x < b.pos.x;
                  return a.pos.y < b.pos.y;
              });

}

inline size_t increment(size_t val, size_t outlineBegin, size_t outlineLength)
{
    return (val+1-outlineBegin)%outlineLength+outlineBegin;
}

inline size_t decrement(size_t val, size_t outlineBegin, size_t outlineLength)
{
    return ((val+outlineLength)-1-outlineBegin)%outlineLength+outlineBegin;
}

int leftSign(size_t i, size_t beg, size_t len, const std::vector<ivec2>& pos)
{
    return (pos[decrement(i, beg, len)].y - pos[i].y);
    return -(pos[i].y - pos[decrement(i, beg, len)].y);
}

int rightSign(size_t i, size_t beg, size_t len, const std::vector<ivec2>& pos)
{
    return (pos[increment(i, beg, len)].y - pos[i].y);
}

void Glyph::computeCriticalPoints(const std::vector<size_t>& contourEnd,
                                  const std::vector<ivec2>& pos,
                                  std::vector<bool>& critical,
                                  size_t beg, size_t outlineID)
{
    size_t len = contourEnd[outlineID]-beg;
    size_t j = increment(beg, beg, len);
    while (!leftSign(j, beg, len, pos) && j != beg)
    {
        j = increment(j, beg, len);
    }
    if (j == beg)
    {
        throw std::runtime_error("Degenerate outline.");
    }

    size_t iter = len;
    while (iter)
    {
        size_t k = j;
        while (!rightSign(k, beg, len, pos))
        {
            k = increment(k, beg, len);
            --iter;
        }
        auto left = leftSign(j, beg, len, pos);
        auto right = rightSign(k, beg, len, pos);
        if (left*right < 0)
        {
            if (left > 0)
            {
                if (critical[j]) break;
                critical[j] = true;
            }
            if (right > 0)
            {
                if (critical[k]) break;
                critical[k] = true;
            }
        }
        j = increment(k, beg, len);
        --iter;
    }
}


void Glyph::computeUsableZeroes(const std::vector<size_t>& contourEnd,
                                const std::vector<ivec2>& position,
                                std::vector<bool>& zeroAcceptable)
{
    zeroAcceptable.resize(position.size(), false);
    size_t cBegin = 0;
    for (size_t i = 0; i < contourEnd.size(); ++i)
    {
        computeCriticalPoints(contourEnd, position, zeroAcceptable, cBegin, i);
        cBegin = contourEnd[i];
    }
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
    std::cout << "Segment count: " << m_segments.size() << std::endl;
    for (size_t i = 0; i < m_segments.size(); ++i)
    {
        std::cout << "Segment #" << i << ": ";
        std::cout << m_segments[i].pos << "+t*" << m_segments[i].dir << "\n";
    }
    std::cout << "Horizontal segments: " << m_horSegments.size() << std::endl;
    for (size_t i = 0; i < m_horSegments.size(); ++i)
    {
        std::cout << "Horizontal segment #" << i << ": ";
        std::cout << "[" << m_horSegments[i].xmin << ", "
                         << m_horSegments[i].xmax << "]x{"
                         << m_horSegments[i].y << "}" << std::endl;
    }
    std::cout << "Bezier count: " << m_bezier.size() << std::endl;
    for (size_t i = 0; i < m_bezier.size(); ++i)
    {
        std::cout << "Bezier #" << i << ": ";
        std::cout << "[" << m_bezier[i].p0 << ", "
                  << m_bezier[i].p1 << ", "
                  << m_bezier[i].p2 << "]" << std::endl;

    }
    std::cout << "\n";
}

size_t intersectCount(vec2 pos, QuadraticBezier bezier) noexcept
{
    auto e = bezier.p0.y;
    auto g = bezier.p1.y;
    auto k = bezier.p2.y;
    auto b = pos.y;

    auto A = e-2*g+k;
    auto B = 2*(g-e);
    auto C = e-b;

    auto d = bezier.p0.x;
    auto f = bezier.p1.x;
    auto h = bezier.p2.x;
    auto a = pos.x;

    auto E = d-2*f+h;
    auto F = 2*(f-d);
    auto G = d-a;

    if (A == 0)
    {
        if (!B) return 0;
        if (B > 0 && !(b > e && C > -B)) return 0;
        if (B < 0 && !(b < e && C < -B)) return 0;
        float t = -C / (float)B;
        return t * (E * t + F) + G >= 0;
    }

    // Note: This expression may look prone to losing precision, but note that
    // the only non-integer (and thereby non-exact) variable in the expression
    // is b.
    if (e*k-g*g > b*A) return 0;
    bool minusGood = false;
    bool plusGood = false;
    if (std::abs((e*k-g*g) - (b*A)) <= 0.f) // Equality comparison
    {
        if (A > 0) plusGood = B < 0 && (-B < 2*A);
        else plusGood = B > 0 && (-B > 2*A);
    }
    else
    {
        // Check that t > 0 for minus solution.
        if (e>=g) minusGood = e>b;
        else minusGood = A < 0;

        // Same for plus solution.
        if (e<g) plusGood = e<b;
        else plusGood = A > 0;

        // Check that it still holds when reversing the curve (in effect testing
        // whether t < 1).
        if (plusGood)
        {
            if (k >= g) plusGood = k > b;
            else plusGood = A < 0;
        }
        if (minusGood)
        {
            if (k < g) minusGood = k < b;
            else minusGood = A > 0;
        }
    }

    // Just check x using floats since doing it exactly means computing a
    // complicated expression which likely needs 64-bit integers even if all
    // absolute values of the coordinates are less than 2^12. Also, precision is
    // probably not an issue since we only use this to get the x-coordinate of
    // our intersections relative to the ray origin. Yes, it might mean that an
    // intersection very close to the ray origin may be missed (or a non-
    // intersection is counted) but since it is very close to the ray origin, it
    // probably isn't visible.

    float tMinus = (-B - std::sqrt((float)(B*B-4*A*C))) / (float)(2*A);
    float tPlus  = (-B + std::sqrt((float)(B*B-4*A*C))) / (float)(2*A);

    return (tMinus * (E * tMinus + F) + G >= 0) * minusGood
           + (tPlus * (E * tPlus + F) + G >= 0) * plusGood;
}

size_t intersects(ivec2 p,
                  LineSegment l,
                  bool& rayBeginOnSegment) noexcept
{
    auto subfac1 = p.y - l.pos.y;
    auto subfac2 = std::abs(2*subfac1-l.dir.y);
    if (subfac2 <= l.dir.y)
    {
        auto subfac4 = p.x - l.pos.x;
        if (l.dir.y * subfac4 == l.dir.x * subfac1 &&
            std::abs(2*subfac4-l.dir.x) <= std::abs(l.dir.x))
        {
            rayBeginOnSegment = true;
            return 2;
        }
    }


    if (dot(perp(l.dir), p-l.pos) > 0)
    {
        return subfac2 < l.dir.y;
    }
    return 0;
}

size_t Glyph::isInside(ivec2 pos) const noexcept
{
    size_t intersections = 0;
    for (auto& cp : m_critPoints)
    {
        if (cp.y < pos.y) continue;
        if (cp.y > pos.y) break;
        intersections += pos.x <= cp.x;
        if (pos.x == cp.x) return 251;
    }

    for (size_t i = 0; i < m_horSegments.size(); ++i)
    {
        if (m_horSegments[i].y < pos.y) continue;
        if (m_horSegments[i].y > pos.y) break;
        if (m_horSegments[i].xmin <= pos.x &&
            pos.x <= m_horSegments[i].xmax)
        {
            return 253;
        }
    }

    for (size_t i = 0; i < m_segments.size(); ++i)
    {
        if (m_segments[i].pos.y+m_segments[i].dir.y < pos.y)
        {
            continue;
        }
        if (m_segments[i].pos.y > pos.y)
        {
            break;
        }
        bool rayOnSegment = false;

        intersections += intersects(pos,
                                    m_segments[i],
                                    rayOnSegment);
        if (rayOnSegment) return 255;
    }
    for (size_t i = 0; i < m_bezier.size(); ++i)
    {
        intersections += intersectCount(vec2{(float)pos.x, (float)pos.y},
                                        m_bezier[i]);
    }
    return intersections;

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
    height = height;
    int pixelWidth, pixelHeight;

    if (width <= 0)
    {
        if (height <= 0)
        {
            throw std::runtime_error("Bad render size.");
        }
        pixelHeight = (height * glyph.info().height) / info.emSize;
        if (pixelHeight < 2) pixelHeight = 2;
        pixelWidth = pixelHeight * glyph.info().width / glyph.info().height;
    }
    else
    {
        pixelWidth = (width * glyph.info().width) / info.emSize;
        if (pixelWidth < 2) pixelWidth = 2;
        pixelHeight = pixelWidth * glyph.info().height / glyph.info().width;
    }

    Image img(pixelWidth, pixelHeight);

    for (int y = 0; y < pixelHeight; ++y)
    {
        for (int x = 0; x < pixelWidth; ++x)
        {
            ivec2 glyphPos;
            glyphPos.x = glyph.info().hCursorX + x*glyph.info().width/(pixelWidth-1);
            glyphPos.y = glyph.info().hCursorY - y*glyph.info().height/(pixelHeight-1);
            //std::cerr << "Pixel " << ivec2{x, y} << " to " << glyphPos << " -- ";
            auto c = glyph.isInside(glyphPos);
            //std::cerr << c << "\n";
            img.setPixel(x, y, (c&1)*0xffffff);
            //img.setPixel(x, y, (c<<16) + ((c&1)<<7));
        }
    }
    return img;
}
