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

    if (!outline.n_contours || !outline.n_points)
    {
        throw std::runtime_error("Glyph is empty.");
    }

    for (short i = 0; i < outline.n_contours; ++i)
    {
        contourEnd[i] = outline.contours[i]+1;
        std::cerr << "Contour " << i << " ends at " << contourEnd[i] << "!\n";
    }

    short contour = 0;
    short contourPos = 0;
    bool prevControl = false;
    std::cerr << "n_points: " << outline.n_points << "\n";
    for (short i = 0; i < outline.n_points; ++i)
    {
        std::cerr << contourPos << "/" << outline.contours[contour] << " (contour #" << contour << ")\n";
        if (i == outline.contours[contour]+1)
        {
            contourEnd[contour] = contourPos;
            ++contour;
            prevControl = false;
        }

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
            ++contourPos;
        }
        prevControl = currControl;
        position.push_back(currPos);
        isControl.push_back(currControl);
        ++contourPos;
    }
    contourEnd[contour] = position.size();
    std::cerr << "Final data:\n";
    for (size_t i = 0; i < contourEnd.size(); ++i)
    {
        std::cerr << "Outline #" << i << ": " << contourEnd[i] << "\n";
    }
    std::cerr << contour << "\n";
    std::cerr << position.size() << "\n";
    /*
    while (contourEnd.size() > 1) contourEnd.pop_back();
    while (position.size() > contourEnd.back()) position.pop_back();*/

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
                if (prevPos.y > cPos.y)
                {
                    m_bezier.push_back({prevPos, prevPos, cPos});
                }
                else if (prevPos.y < cPos.y)
                {
                    m_bezier.push_back({prevPos, cPos, cPos});
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

    std::sort(m_bezier.begin(), m_bezier.end(),
              [](const QuadraticBezier& a, const QuadraticBezier& b)
              {
                  if (a.minY() == b.minY()) return a.minY() < b.minY();
                  return a.minY() < b.minY();
              });
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
    auto d = bezier.p0.x;
    auto f = bezier.p1.x;
    auto h = bezier.p2.x;
    auto a = pos.x;

    bool eHit = std::abs(e-b) <= 0.f;
    bool kHit = std::abs(k-b) <= 0.f;

    bool eLow = e < g || (e == g && e < k);
    bool kLow = k < g || (k == g && k < e);

    size_t extraSols = 0;

    if (eHit && kHit && e < g)
    {
        return (a <= d) + (a <= h);
    }

    if (eHit && eLow && a <= d)
    {
        ++extraSols;
    }
    if (kHit && kLow && a <= h)
    {
        ++extraSols;
    }


    auto A = e-2*g+k;
    auto B = 2*(g-e);
    auto C = e-b;
    auto K = k-b;


    auto E = d-2*f+h;
    auto F = 2*(f-d);
    auto G = d-a;

    if (A == 0)
    {
        if (B > 0 && !(b > e && C > -B)) return extraSols;
        if (B < 0 && !(b < e && C < -B)) return extraSols;
        float t = -C / (float)B;
        return t * (E * t + F) + G >= 0 + extraSols;
    }

    // Note: This expression may look prone to losing precision, but note that
    // the only non-integer (and thereby non-exact) variable in the expression
    // is b.
    if (e*k-g*g > b*A) return extraSols;
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
        if (e>=g) minusGood = C>0;
        else minusGood = A < 0;

        // Same for plus solution.
        if (e<g) plusGood = C<0;
        else plusGood = A > 0;

        // Check that it still holds when reversing the curve (in effect testing
        // whether t < 1).
        if (plusGood)
        {
            if (k >= g) plusGood = K>0;
            else plusGood = A < 0;
        }
        if (minusGood)
        {
            if (k < g) minusGood = K<0;
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
           + (tPlus * (E * tPlus + F) + G >= 0) * plusGood
           + extraSols;
}

size_t Glyph::isInside(ivec2 pos, size_t& beginAt) const noexcept
{
    size_t intersections = 0;
    while (beginAt < m_bezier.size() && m_bezier[beginAt].maxY() < pos.y)
    {
        ++beginAt;
    }
    for (size_t i = beginAt; i < m_bezier.size(); ++i)
    {
        if (m_bezier[i].minY() > pos.y) return intersections;
        if (m_bezier[i].maxX() < pos.x) continue;
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

    size_t beginAt = 0;
    for (int y = pixelHeight-1; y >= 0; --y)
    {
        for (int x = 0; x < pixelWidth; ++x)
        {
            ivec2 glyphPos;
            glyphPos.x = glyph.info().hCursorX + x*glyph.info().width/(pixelWidth-1);
            glyphPos.y = glyph.info().hCursorY - y*glyph.info().height/(pixelHeight-1);
            auto c = glyph.isInside(glyphPos, beginAt);
            U32 col = ((c&1)*0xffff) << 8;
            img.setPixel(x, y, col+c*16);
        }
    }
    return img;
}
