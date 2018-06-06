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

    if (!outline.n_contours || !outline.n_points)
    {
        throw std::runtime_error("Glyph is empty.");
    }

    for (short i = 0; i < outline.n_contours; ++i)
    {
        contourEnd[i] = outline.contours[i]+1;
    }

    short contour = 0;
    short contourPos = 0;
    bool prevControl = false;
    for (short i = 0; i < outline.n_points; ++i)
    {
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

    extractOutlines(contourEnd, position, isControl);

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
                            const std::vector<bool>& control)
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
            if (control[i])
            {
                auto nextIdx = i+1-contourBegin;
                nextIdx %= (contourEnd[contour]-contourBegin);
                nextIdx += contourBegin;
                m_curves.push_back({prevPos, cPos, position[nextIdx]});
                prevControl = true;
            }
            else if (!prevControl)
            {
                m_curves.push_back({prevPos, prevPos, cPos});
            }
            else
            {
                prevControl = false;
            }
            prevPos = cPos;
            if (!m_curves.empty())
            {
                if (m_curves.back().p0x == m_curves.back().p1x
                    && m_curves.back().p1x == m_curves.back().p2x)
                {
                    m_curves.back().p1y = m_curves.back().p0y;
                }
                if (m_curves.back().p0x == m_curves.back().p1x
                    && m_curves.back().p0y == m_curves.back().p1y
                    && m_curves.back().p1y < m_curves.back().p2y)
                {
                    m_curves.back().p1x = m_curves.back().p2x;
                    m_curves.back().p1y = m_curves.back().p2y;
                }
                if (m_curves.back().p2x == m_curves.back().p2x
                    && m_curves.back().p2y == m_curves.back().p2y
                    && m_curves.back().p1y < m_curves.back().p0y)
                {
                    m_curves.back().p1x = m_curves.back().p0x;
                    m_curves.back().p1y = m_curves.back().p0y;
                }
                if (m_curves.back().p0y == m_curves.back().p1y
                    && m_curves.back().p1y == m_curves.back().p2y)
                {
                    m_curves.pop_back();
                }
            }
        }
        contourBegin = contourEnd[contour];
    }
    std::sort(m_curves.begin(), m_curves.end(),
              [](const PackedBezier& a, const PackedBezier& b)
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
    std::cout << "Bezier count: " << m_curves.size() << std::endl;
    for (size_t i = 0; i < m_curves.size(); ++i)
    {
        std::cout << "Bezier #" << i << ": ";
        std::cout << "[(" << m_curves[i].p0x << ", " << m_curves[i].p0y << "), "
                  <<  "(" << m_curves[i].p1x << ", " << m_curves[i].p1y << "), "
                  <<  "(" << m_curves[i].p2x << ", " << m_curves[i].p2y << ")]"
                  << std::endl;

    }
    std::cout << "\n";
}

// Note that to handle self-intersecting glyphs (for example where two different
// outlines intersect (which sometimes happens for e.g. ffl-ligatures)) this
// computes not how many intersections there are but instead a number signifying
// in which direction a curve is intersected. This means that a point is
// considered outside the glyph if and only if a ray from this point will
// intersect the same number of curves having opposite direction (e.g. if it
// intersects two curves where the point is to the right of these, then it must
// intersect two curves which it is to the left of in order to be considered
// outside all outlines).
int intersect(vec2 pos, PackedBezier bezier) noexcept
{
    /*
    std::cerr << "NOTE: Curve has lowest point ";
    if (bezier.minY() == bezier.p0y) std::cerr << bezier.p0x << ", " << bezier.p0y << "\n";
    else if (bezier.minY() == bezier.p1y) std::cerr << bezier.p1x << ", " << bezier.p1y << "\n";
    else if (bezier.minY() == bezier.p2y) std::cerr << bezier.p2x << ", " << bezier.p2y << "\n";
    //*/
    const auto& e = bezier.p0y;
    const auto& g = bezier.p1y;
    const auto& k = bezier.p2y;
    const auto& b = pos.y;
    const auto& d = bezier.p0x;
    const auto& f = bezier.p1x;
    const auto& h = bezier.p2x;
    const auto& a = pos.x;

    // Todo: get rid of this.
    if (e < g && std::abs(e-b) <= 0.f && std::abs(k-b) <= 0.f)
    {
        return (a <= d) - (a <= h);
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
        int mul = 1;
        if (e <= k) mul = -1;
        if (!B) return 0;
        if (B > 0 && !(b > e && C > -B)) return 0;
        if (B < 0 && !(b < e && C < -B)) return 0;
        float t = -C / (float)B;
        return (t * (E * t + F) + G >= 0) * mul;
    }
    // Note: This expression may look prone to losing precision, but note that
    // the only non-integer (and thereby non-exact) variable in the expression
    // is b.
    if (e*k-g*g >= b*A)
    {
        return 0;
    }

    bool minusGood = false;
    bool plusGood = false;

    /// [-,+]:
    /// (e < g)&(k < g)  =>  [-,+] = [(A < 0)&(K < 0), (C < 0)&(A < 0)]
    /// !(e < g)&(k < g)  =>  [-,+] = [(C > 0)&(K < 0), (A > 0)&(A < 0)]
    /// (e < g)&!(k < g)  =>  [-,+] = [(A < 0)&(A > 0), (C < 0)&(K > 0)]
    /// !(e < g)&!(k < g)  =>  [-,+] = [(C > 0)&(A > 0), (A > 0)&(K < 0)]

    minusGood = (e >= g ? C > 0 : A < 0) && (k < g ? K < 0 : A > 0);
    plusGood = (e < g ? C < 0 : A > 0) && (k >= g ? K > 0 : A < 0);

    if (e <= g && std::abs(e-b) <= 0.f)
    {
        /*std::cerr << "Hit point 'e' which says ";
        if (B < 0) std::cerr << "minus";
        else std::cerr << "plus";
        std::cerr << "!\n";*/
        if (B < 0) minusGood = true;
        else plusGood = true;
    }
    if (k <= g && std::abs(k-b) <= 0.f)
    {
        /*std::cerr << "Hit point 'k' which says ";
        if (B < 0) std::cerr << "minus";
        else std::cerr << "plus";
        std::cerr << "!\n";*/
        if (B < 0) minusGood = true;
        else if (B > 0) plusGood = true;
        else
        {
            if (A < 0) minusGood = true;
            else plusGood = true;
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

    auto tmX = tMinus * (E * tMinus + F) + G;
    auto tpX = tPlus * (E * tPlus + F) + G;

/*
    std::cerr << "\033[1;35m-t: " << tMinus << "\n";
    std::cerr << "\033[1;35m+t: " << tPlus << "\n\033[0m";
    std::cerr << "\033[1;32mx[-t]? " << tmX << "\n";
    std::cerr << "\033[1;32mx[+t]? " << tpX << "\n\033[0m";
    //*/

    auto cnt = (tmX >= 0) * minusGood
               - (tpX >= 0) * plusGood;

    return cnt;
}

#define SHOWG LYPHDEBUG

bool Glyph::isInside(vec2 pos, size_t& beginAt) const noexcept
{
#ifdef SHOWGLYPHDEBUG
    std::cerr << "\n\nAt " << pos << " we get:\n";
#endif // SHOWGLYPHDEBUG
    int intersections = 0;
    while (beginAt < m_curves.size() && m_curves[beginAt].maxY() < pos.y)
    {
        ++beginAt;
    }
    for (size_t i = beginAt; i < m_curves.size(); ++i)
    {
#ifdef SHOWGLYPHDEBUG
        if (m_curves[i].minY() > pos.y)
    std::cerr << "\033[1;33;41mRetrunning " << intersections << "\033[0m\n\n";
#endif // SHOWGLYPHDEBUG
        if (m_curves[i].minY() > pos.y) return intersections;
        if (m_curves[i].maxX() < pos.x) continue;
        int ic = intersect(pos, m_curves[i]);
#ifdef SHOWGLYPHDEBUG
        std::cerr << "\033[31mAdding " << ic << " to intersections [curr " << intersections << "]\033[0m\n\n";
#endif // SHOWGLYPHDEBUG
        intersections += ic;
    }
#ifdef SHOWGLYPHDEBUG
    std::cerr << "Retrunning " << intersections << "\n\n";
#endif // SHOWGLYPHDEBUG
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
        //if (y != 120) continue;
        for (int x = 0; x < pixelWidth; ++x)
        {
            //if (x < 250 || x > 300) continue;
            //if (x > pixelWidth / 10) continue;
            //if (x&0x7) continue;
            vec2 glyphPos;
            glyphPos.x = glyph.info().hCursorX + x*glyph.info().width/float(pixelWidth);
            glyphPos.y = glyph.info().hCursorY - y*glyph.info().height/float(pixelHeight);
            auto inside = glyph.isInside(glyphPos, beginAt);
            img.setPixel(x, y, inside ? 0xffffff : 0);
        }
        ///if (y == pixelHeight-1) break;
    }
    return img;
}
