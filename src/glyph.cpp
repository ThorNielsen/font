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

    m_info.width = static_cast<int>(metrics.width);
    m_info.height = static_cast<int>(metrics.height);
    m_info.hCursorX = static_cast<int>(metrics.horiBearingX);
    m_info.hCursorY = static_cast<int>(metrics.horiBearingY);
    m_info.xAdvance = static_cast<int>(metrics.horiAdvance);
    m_info.vCursorX = static_cast<int>(metrics.vertBearingX);
    m_info.vCursorY = static_cast<int>(metrics.vertBearingY);
    m_info.yAdvance = static_cast<int>(metrics.vertAdvance);

    extractOutlines(contourEnd, position, isControl);
}


void Glyph::extractOutlines(const std::vector<size_t>& contourEnd,
                            const std::vector<ivec2>& position,
                            const std::vector<bool>& control)
{
    ivec2 offset{32767, 32767};
    size_t contourBegin = 0;
    std::vector<PackedBezier> curves;
    for (size_t contour = 0; contour < contourEnd.size(); ++contour)
    {
        auto prevIdx = contourEnd[contour]-1;
        auto prevPos = position[prevIdx];
        bool prevControl = control[prevIdx];
        for (size_t i = contourBegin; i < contourEnd[contour]; prevIdx = i++)
        {
            ivec2 p{0, 0}, q{0, 0}, r{0, 0};
            auto cPos = position[i];
            bool addedCurve = false;
            if (control[i])
            {
                auto nextIdx = i+1-contourBegin;
                nextIdx %= (contourEnd[contour]-contourBegin);
                nextIdx += contourBegin;
                p = prevPos;
                q = cPos;
                r = position[nextIdx];
                prevControl = true;
                addedCurve = true;
            }
            else if (!prevControl)
            {
                p = prevPos;
                q = prevPos;
                r = cPos;
                addedCurve = true;
            }
            else
            {
                prevControl = false;
            }
            prevPos = cPos;
            if (addedCurve)
            {
                curves.emplace_back(p, q, r);
                offset.x = std::min(std::min(offset.x, p.x), std::min(q.x, r.x));
                offset.y = std::min(std::min(offset.y, p.y), std::min(q.y, r.y));
            }
        }
        contourBegin = contourEnd[contour];
    }

    --offset.x;
    --offset.y;

    offset = -offset;

    for (auto& curve : curves)
    {
        curve.p0x += offset.x; curve.p1x += offset.x; curve.p2x += offset.x;
        curve.p0y += offset.y; curve.p1y += offset.y; curve.p2y += offset.y;
    }

    m_info.hCursorX += offset.x;
    m_info.hCursorY += offset.y;
    m_info.vCursorX += offset.x;
    m_info.vCursorY += offset.y;

    processCurves(curves);
    //createBitmap(1, curves);
}

void Glyph::processCurves(const std::vector<PackedBezier>& curves)
{
    for (const auto& curve : curves)
    {
        auto p = ivec2{curve.p0x, curve.p0y};
        auto q = ivec2{curve.p1x, curve.p1y};
        auto r = ivec2{curve.p2x, curve.p2y};
        if (p.y != q.y || q.y != r.y)
        {
            if (p.x == q.x && p.y == q.y && q.y < r.y)
            {
                q = r;
            }
            if (r.x == q.x && r.y == q.y && q.y < p.y)
            {
                q = p;
            }
            m_curves.emplace_back(p, q, r);
        }
    }

    sortByY(m_curves);
}
void Glyph::sortByY(std::vector<PackedBezier>& curves)
{
    std::sort(curves.begin(), curves.end(),
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

void Glyph::createBitmap(size_t logLength,
                         const std::vector<PackedBezier>& curves)
{
    m_bitmap.setResolution(logLength);
    size_t length = 1 << logLength;
    // We add two to maximum dimension since boxes are half-open and zero
    // coordinates are reserved.
    size_t maxDim = std::max(m_info.width, m_info.height)+1;
    m_boxLength = maxDim / length + (maxDim % length ? 1 : 0);

    for (size_t y = 0; y < length; ++y)
    {
        for (size_t x = 0; x < length; ++x)
        {
            auto p0 = ivec2{int(x*m_boxLength),
                            int(y*m_boxLength)};

            if (p0.x > m_info.width + 1 || p0.y > m_info.height + 1)
            {
                m_bitmap.setValue(x, y, 0);
                continue;
            }

            auto p1 = p0+ivec2{int(m_boxLength), int(m_boxLength)};
            if (p0.x <= m_info.width && p1.x > m_info.width)
            {
                p1.x = m_info.width+1;
            }
            if (p0.y <= m_info.height && p1.y > m_info.height)
            {
                p1.y = m_info.height+1;
            }
            bool intersections = boxIntersects(p0, p1, curves);
            int status = 0;
            if (!intersections)
            {
                // Trick yIsInside into thinking that the current pixel should
                // be calculated instead of looked up.
                m_bitmap.setValue(x, y, 2);
                status = isInside({(float)(p0.x+p1.x)/2, (float)(p0.y+p1.y)/2});
            }
            else
            {
                status = 2;
            }
            m_bitmap.setValue(x, y, status);
        }
    }
}

bool Glyph::boxIntersects(ivec2 p0, ivec2 p1,
                          const std::vector<PackedBezier>& curves)
{
    float p0x = p0.x; float p0y = p0.y;
    float p1x = p1.x; float p1y = p1.y;
    for (const PackedBezier& yCurve : curves)
    {
        bool xDegenerate = yCurve.p0x == yCurve.p1x && yCurve.p1x == yCurve.p2x;
        bool yDegenerate = yCurve.p0y == yCurve.p1y && yCurve.p1y == yCurve.p2y;
        if (xDegenerate && yCurve.p0x == 1) continue;
        if (xDegenerate && yCurve.p0x == m_info.width+1) continue;
        if (yDegenerate && yCurve.p0y == 1) continue;
        if (yDegenerate && yCurve.p0y == m_info.height+1) continue;
        float t0 = 0.f, t1 = 0.f;
        if (yCurve.minY() <= p0y && yCurve.maxY() > p0y)
        {
            intersect({p0x, p0y}, yCurve, t0, t1);
            if ((t0 > 0 && t0 <= m_info.width+1 && p0x <= t0 && t0 < p1x)
                || (t1 > 0 && t1 <= m_info.width+1 && p0x <= t1 && t1 < p1x))
            {
                return true;
            }
        }
        if (yCurve.minY() <= p1y && yCurve.maxY() > p1y)
        {
            intersect({p0x, p1y}, yCurve, t0, t1);
            if ((t0 > 0 && t0 <= m_info.width+1 && p0x <= t0 && t0 < p1x)
                || (t1 > 0 && t1 <= m_info.width+1 && p0x <= t1 && t1 < p1x))
            {
                return true;
            }
        }

        auto xCurve = yCurve.swapCoordinates();
        if (xCurve.minY() <= p0x && xCurve.maxY() > p0x)
        {
            intersect({p0y, p0x}, xCurve, t0, t1);
            if ((t0 > 0 && t0 <= m_info.height+1 && p0y <= t0 && t0 < p1y)
                || (t1 > 0 && t1 <= m_info.height+1 && p0y <= t1 && t1 < p1y))
            {
                return true;
            }
        }
        if (xCurve.minY() <= p1x && xCurve.maxY() > p1x)
        {
            intersect({p0y, p1x}, xCurve, t0, t1);
            if ((t0 > 0 && t0 <= m_info.height+1 && p0y <= t0 && t0 < p1y)
                || (t1 > 0 && t1 <= m_info.height+1 && p0y <= t1 && t1 < p1y))
            {
                return true;
            }
        }
    }
    return false;
}

int intersect(vec2 pos, PackedBezier bezier, float& minusX, float& plusX) noexcept
{
    float C = bezier.p0y-pos.y;
    float K = bezier.p2y-pos.y;
    S16 B = bezier.p1y-bezier.p0y;
    S16 A = B+bezier.p1y-bezier.p2y;

    auto lookup = (bezier.lookup>>(2*(C>=0)+4*(K>=0)))&3;

    float tMinus, tPlus;
    if (A == 0)
    {
        tMinus = C / (float)((-2)*B);
        tPlus = tMinus;
    }
    else
    {
        float comp1 = std::sqrt((float)(B*B+A*C));
        tMinus = (B + comp1)/(float)(A); // Add temp variable invA = 1/A, and
        tPlus  = (B - comp1)/(float)(A); // multiply by that instead.
    }

    S16 E = bezier.p0x-2*bezier.p1x+bezier.p2x;
    S16 F = 2*(bezier.p1x-bezier.p0x);
    float G = bezier.p0x-pos.x;
    auto tmX = tMinus * (E * tMinus + F);
    auto tpX = tPlus * (E * tPlus + F);

    minusX = (tmX+bezier.p0x) * (lookup&1);
    plusX = (tpX+bezier.p0x) * ((lookup&2)>>1);

    int cnt = (tmX + G <= 0) * (lookup&1)
            - (tpX + G <= 0) * ((lookup&2)>>1);

    return cnt;
}

// Todo: Add some form of anti-aliasing. One option is the method specified in
// https://wdobbie.com/post/gpu-text-rendering-with-vector-textures/
// but this may not work if we do not know what curves are on the outline. So we
// need a way to distinguish between curves on the outline and curves inside the
// glyph (for self-intersecting glyphs). However, this is not as simple as it
// seems, since we could have a situation where two curves are partially on the
// outline - take a look at the following:
// 6             |  /
// 5   Inside    | /
// 4             |/
// 3             |    Outside
// 2            /|
// 1           / |
// 0          /  |
// At y=0, '|' is on the outline, but at y=3 this changes to them both being
// on the outline and finally at y=6, '/' is on the outline but '|' is not.
// One option is to mark all such locations, but this will probably be too
// expensive.

// Another way of anti-aliasing could be to simply calculate the distance to the
// nearest curve in a few directions (say {+-1, 0} and {0, +-1}, since we have
// the code to find horizontal distances and this can easily be adapted to work
// with vertical distances), compare these distances to the pixel size to get an
// estimate of the coverage and hope that this curve is indeed on the outline -
// it will work fine if the font is well-behaved and there are no self-
// intersections (or they only overlap very little). Unfortunately overlapping
// is somewhat common (depending on the font) and if we happen to hit a point
// very near an interior line, we could end up with a pixel value of about one
// half meaning that there would potentially be a gray line inside the glyph.

bool Glyph::isInside(vec2 pos) const noexcept
{
    /*
    int x = (pos.x - m_info.hCursorX) / m_boxLength;
    int y = pos.y / m_boxLength;
    int v = 2;
    if (pos.x > 1 && pos.x <= m_info.width &&
        pos.y > 1 && pos.y <= m_info.height)
    {
        v = m_bitmap(x, y);
    }
    if (v != 2) return v;*/
    int intersections = 0;
    for (size_t i = 0; i < m_curves.size(); ++i)
    {
        const auto& curve = m_curves[i];
        if (curve.minY() > pos.y) break;
        if (curve.maxY() < pos.y) continue;
        if (curve.minX() > pos.x) continue;
        float dummy;
        intersections += intersect(pos, curve, dummy, dummy);
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

    for (int y = pixelHeight-1; y >= 0; --y)
    {
        for (int x = 0; x < pixelWidth; ++x)
        {
            vec2 glyphPos;
            glyphPos.x = glyph.info().hCursorX + x*glyph.info().width/float(pixelWidth);
            glyphPos.y = glyph.info().hCursorY - y*glyph.info().height/float(pixelHeight);
            /*
            auto inside = glyph.xIsInside(glyphPos);
            U32 col = inside == 2 ? 0xff : inside * 0x7f;
            U32 c2 = glyph.yIsInside(glyphPos) * 0xff;
            img.setPixel(x, y, col | (col << 8) | (c2 << 16));/*/
            auto inside = glyph.isInside(glyphPos);
            img.setPixel(x, y, inside * 0xffffff);//*/
        }
    }
    return img;
}
