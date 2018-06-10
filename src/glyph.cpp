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
    createBitmap(5);
}


void Glyph::extractOutlines(const std::vector<size_t>& contourEnd,
                            const std::vector<ivec2>& position,
                            const std::vector<bool>& control)
{
    ivec2 offset{0, 0};
    size_t contourBegin = 0;
    std::vector<ivec2> curves;
    for (size_t contour = 0; contour < contourEnd.size(); ++contour)
    {
        auto prevIdx = contourEnd[contour]-1;
        auto prevPos = position[prevIdx];
        bool prevControl = control[prevIdx];
        for (size_t i = contourBegin; i < contourEnd[contour]; prevIdx = i++)
        {
            ivec2 p{0, 0}, q{0, 0}, r{0, 0};
            auto cPos = position[i];
            if (control[i])
            {
                auto nextIdx = i+1-contourBegin;
                nextIdx %= (contourEnd[contour]-contourBegin);
                nextIdx += contourBegin;
                p = prevPos;
                q = cPos;
                r = position[nextIdx];
                prevControl = true;
            }
            else if (!prevControl)
            {
                p = prevPos;
                q = prevPos;
                r = cPos;
            }
            else
            {
                prevControl = false;
            }
            prevPos = cPos;
            curves.push_back(p);
            curves.push_back(q);
            curves.push_back(r);
            offset.x = std::min(std::min(offset.x, p.x), std::min(q.x, r.x));
            offset.y = std::min(std::min(offset.y, p.y), std::min(q.y, r.y));
        }
        contourBegin = contourEnd[contour];
    }

    --offset.x;
    --offset.y;

    offset = -offset;

    for (size_t i = 0; i < curves.size(); i += 3)
    {
        auto p = curves[i]+offset;
        auto q = curves[i+1]+offset;
        auto r = curves[i+2]+offset;
        if (p.x != q.x || q.x != r.x)
        {
            if (p.y == q.y && p.x == q.x && q.x < r.x)
            {
                q = r;
            }
            if (r.y == q.y && r.x == q.x && q.x < p.x)
            {
                q = p;
            }
            m_xcurves.emplace_back(ivec2{p.y, p.x},
                                   ivec2{q.y, q.x},
                                   ivec2{r.y, r.x});
        }
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
            m_ycurves.emplace_back(p, q, r);
        }
    }

    m_info.hCursorX += offset.x;
    m_info.hCursorY += offset.y;
    m_info.vCursorX += offset.x;
    m_info.vCursorY += offset.y;

    std::sort(m_xcurves.begin(), m_xcurves.end(),
              [](const PackedBezier& a, const PackedBezier& b)
              {
                  if (a.minY() == b.minY()) return a.minY() < b.minY();
                  return a.minY() < b.minY();
              });
    std::sort(m_ycurves.begin(), m_ycurves.end(),
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
    std::cout << "Bezier count: " << m_ycurves.size() << std::endl;
    for (size_t i = 0; i < m_ycurves.size(); ++i)
    {
        std::cout << "Bezier #" << i << ": ";
        std::cout << "[(" << m_ycurves[i].p0x << ", " << m_ycurves[i].p0y << "), "
                  <<  "(" << m_ycurves[i].p1x << ", " << m_ycurves[i].p1y << "), "
                  <<  "(" << m_ycurves[i].p2x << ", " << m_ycurves[i].p2y << ")]"
                  << std::endl;

    }
    std::cout << "\n";
}

void Glyph::createBitmap(size_t logLength)
{
    m_bitmap.setResolution(logLength);

    size_t length = 1 << logLength;
    size_t maxDim = std::max(m_info.width, m_info.height)+1; // +1, since boxes
                                                             // are half-open.
    m_boxLength = maxDim / length + (maxDim % length ? 1 : 0);

    for (size_t x = 0; x < length; ++x)
    {
        for (size_t y = 0; y < length; ++y)
        {
            vec2 p0 = vec2{float(m_info.hCursorX+x*m_boxLength),
                           float(m_info.hCursorY-y*m_boxLength)};
            bool inside0 = yIsInside(p0);
            m_bitmap.setValue(x, y, inside0);
        }
    }
}

int intersect(vec2 pos, PackedBezier bezier, float& minusX, float& plusX) noexcept
{
    float C = bezier.p0y-pos.y;
    float K = bezier.p2y-pos.y;

    bool cgz = C>0;
    bool kgz = K>0;
    bool cez = std::abs(C) <= 0.f;
    bool kez = std::abs(K) <= 0.f;
    auto lookup = ((bezier.lookup>>(2*cgz+4*kgz))
                   | (bezier.lookup>>(8+2*cez+4*kez)))&3;

    S16 B = bezier.p1y-bezier.p0y;
    S16 A = B+bezier.p1y-bezier.p2y;

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
    auto tmX = tMinus * (E * tMinus + F) + G;
    auto tpX = tPlus * (E * tPlus + F) + G;

    minusX = tmX * (lookup&1);
    plusX = tpX * ((lookup&2)>>1);

    auto cnt = (tmX >= 0) * (lookup&1)
               - (tpX >= 0) * ((lookup&2)>>1);

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
bool Glyph::xIsInside(vec2 pos) const noexcept
{
    int x = (pos.x - m_info.hCursorX) / m_boxLength;
    int y = (m_info.hCursorY - pos.y) / m_boxLength;
    if (y >= 128) y = 127;
    if (y < 0) y = 0;
    return m_bitmap(x, y);
    std::swap(pos.x, pos.y);
    int intersections = 0;
    for (size_t i = 0; i < m_xcurves.size(); ++i)
    {
        const auto& curve = m_xcurves[i];
        if (curve.minY() > pos.y) break;
        if (curve.maxY() <= pos.y) continue;
        if (curve.maxX() < pos.x) continue;
        float dummy;
        intersections += intersect(pos, curve, dummy, dummy);
    }
    return intersections;
}

bool Glyph::yIsInside(vec2 pos) const noexcept
{
    int intersections = 0;
    for (size_t i = 0; i < m_ycurves.size(); ++i)
    {
        const auto& curve = m_ycurves[i];
        if (curve.minY() > pos.y) break;
        if (curve.maxY() <= pos.y) continue;
        if (curve.maxX() < pos.x) continue;
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
{/*
    const auto& map = glyph.getMap();
    Image msf(map.width(), map.rows());
    for (size_t i = 0; i < map.width(); ++i)
    {
        for (size_t j = 0; j < map.rows(); ++j)
        {
            msf.setPixel(i, j, map(i, j) * 0xffffff);
        }
    }
    return msf;*/
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
            auto inside = glyph.xIsInside(glyphPos);
            img.setPixel(x, y, inside ? 0xffffff : 0);
        }
    }
    return img;
}
