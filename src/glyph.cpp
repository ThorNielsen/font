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
      m_isThirdOrder(outline.n_points),
      m_zeroAcceptable(outline.n_points)
{
    // Todo:
    //     Replace degenerate bezier curves with segments.
    //     Remove degenerate outlines (if any).
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
    computeUsableZeroes();
}

bool Glyph::isInitialPositive(size_t cBegin, size_t contourID)
{
    auto prevY = position(contourEnd(contourID) - 1).y;
    for (size_t j = contourEnd(contourID)-1; j > cBegin; --j)
    {
        auto currY = position(j).y;

        // Note that we iterate backwards, so index(prevY) > index(currY).
        if (auto yDiff = prevY - currY)
        {
            return yDiff > 0;
        }
        prevY = currY;
    }
    throw std::runtime_error("Bad contour");
}

void Glyph::computeUsableZeroes()
{
    size_t cBegin = 0;
    for (size_t i = 0; i < contours(); ++i)
    {
        auto cLength = contourEnd(i) - cBegin;
        LineSegment cLine;
        cLine.pos = position(cBegin + cLength - 1);
        auto prevPositive = isInitialPositive(cBegin, i);
        for (size_t j = cBegin; j < contourEnd(i); ++j)
        {
            cLine.dir = position(j) - cLine.pos;
            if (!cLine.dir.y) m_zeroAcceptable[j] = false;
            else
            {
                m_zeroAcceptable[j] = (cLine.dir.y > 0) == prevPositive;
                prevPositive = cLine.dir.y > 0;
            }
            cLine.pos = position(j);
        }
        cBegin = contourEnd(i);
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
            std::cout << ", ZeroOK: " << m_zeroAcceptable[j];
            std::cout << "]\n";
        }
        std::cout << "\n";
        p = m_contourEnd[i];
    }
    std::cout << "\n";
}

bool intersects(ivec2 p,
                LineSegment l,
                bool& rayBeginOnSegment,
                bool zeroHitAcceptable)
{
    // Ray and segment are parallel:
    if (!l.dir.y)
    {
        // No intersections.
        if (l.pos.y != p.y) return 0;
        // Line begin on segment <=> p.x in {l.pos.x+t*l.dir.x | 0<=t<=1}
        // <=> |p.x-l.pos.x-l.dir.x/2| <= |l.dir.x/2|
        // <=> |2(p.x-l.pos.x)-l.dir.x| <= |l.dir.x|
        if (std::abs(2*(p.x-l.pos.x)-l.dir.x) <= std::abs(l.dir.x))
        {
            rayBeginOnSegment = true;
        }
        return 0;
    }
    if (l.pos.x - p.x >= 0 && l.pos.y == p.y)
    {
        return zeroHitAcceptable;
    }

    if (dot(perp(l.dir), p-l.pos)*sign(l.dir.y) >= 0)
    {
        return std::abs(2*(p.y-l.pos.y)-l.dir.y) < std::abs(l.dir.y);
    }
    return 0;
}

/*
#define RETURN(x) return (x)


// This finds the number of intersections between a ray and a bezier curve using
// only integer operations (and comparisons and control flow and variables).
// The derivations are quite complicated (not very difficult, just numerous and
// time-consuming) so I have probably made a mistake somewhere. The derivations
// are included below to assist in fixing any potential problems and to better
// describe what is going on where.
int intersectionCount(Ray ray, QuadraticBezier qb)
{
    std::cout << "Utilised!\n";
    using Prec = S32;
    // Extract coefficients from ray and bezier curve:
    Prec a = ray.pos.x; Prec c = ray.dir.x;
    Prec b = ray.pos.y; Prec d = ray.dir.y;
    // The ray can now be written parametrically as
    // x(s) = a+s
    // y(s) = b
    // for s >= 0.
    Prec e = qb.p0.x; Prec g = qb.p1.x; Prec j = qb.p2.x;
    Prec f = qb.p0.y; Prec h = qb.p1.y; Prec k = qb.p2.y;
    // Note that the bezier curve can now be written parametrically as
    // x(t) = (e-2g+j)t^2+2(g-e)t+e
    // y(t) = (f-2h+k)t^2+2(h-f)t+f
    // for 0 <= t < 1. Note that the last inequality must be sharp since an
    // intersection at an endpoint would also be an intersection at the
    // beginning of the next curve leading to two intersections in a single
    // point.
    // To find the intersection count we simply need to find the number of
    // solutions to the following equations:
    // a+s=(e-2g+j)t^2+2(g-e)t+e
    // b=(f-2h+k)t^2+2(h-f)t+f
    // By setting
    {auto A = (f-2*h+k);
    auto B = 2*(h-f);
    auto C = f-b;
    auto D = B*B-4*A*C;
    // we get the equation At^2+Bt+C=0 with no solutions if D < 0, possibly one
    // if D = 0 and possibly two if D > 0.
    if (D < 0) return 0;

    if (D == 0)
    {
        // At most one solution.
        if (A == 0)
        {
            // Since  A=D=0, B = 0 and therefore we have a solution only if
            // C=0.
            if (C) return 0;
        }
        // We have t = -B/2A. If A != 0 then  0 <= t < 1 gives:
        // If  A > 0  then  0 <= B < -2A  <=>  0 <= t < 1
        // If  A < 0  then  -2A < B <= 0  <=>  0 <= t < 1.
        // In total, |B| < |2A| and A*B <= 0  <=>  0 <= t < 1.
        if (std::abs(B) >= std::abs(2*A) || A*B > 0) return false;

        // We have a solution in the bezier curve. Now we just need to check
        // that it is indeed a solution in the ray.
        // We get that  (e-2g+j)t^2+2(g-e)t+(e-a) >= 0. Setting
        auto E = e-2*g+j;
        //auto F = 2*(g-e);
        auto G = e-a;
        // the solution is simply (see derivation externally):
        if (C*E-A*G <= sign(A)*B*(g-e)) return 1;
        return 0;
    }


    }
    return 0;




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
    //    *     '
    //      *    '
    //       *    '
    //      *      '
    //    *         '
    // Bezier      Line, which contains ray
    if (D < 0) RETURN(0);///return 0;

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

    // We look at the case where A > 0. The cases where A <= 0 will be handled
    // later.
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


            // Now we look at the minus root. We get that it satisfies the t-
            // constraints iff:
            // 0 <= -B/2A - sqrt(D)/2A < 1     <=>
            // 0 <= -B - sqrt(D)       < 2A    <=>
            // B <= -sqrt(D)           < 2A+B
            // Again we split this up in two.

            // Looking at  B <= -sqrt(D)  we get that B < 0 and:
            // B   <= -sqrt(D)  <=>
            // B^2 >= D         <=>
            // B^2 >= B^2-4AC   <=>
            // 0   >= -4AC      <=>
            // 0   <= C
            // Therefore  B <= -sqrt(D) iff (B < 0) and (C >= 0)  .

            // Looking at  -sqrt(D) < 2A+B  we again get two cases:
            // 2A+B >= 0  in which case it is true and
            // 2A+B < 0  in which case
            //   -sqrt(D) < 2A+B          <=>
            //   D        > 4A^2+4AB+B^2  <=>
            //   B^2-4AC  > 4A^2+4AB+B^2  <=>
            //   0        > 4A^2+4AB+4AC  <=>
            //   0        > A+B+C
            // Therefore  -sqrt(D) < 2A+B iff (2A+B >= 0) or (A+B+C < 0)  .
            if ((B < 0) && (C >= 0) && ((2*A+B >= 0) || (A+B+C < 0)))
            {
                solSub = true;
            }
        }
    }
    if (A < 0)
    {
        // Again we have two potential solutions. This time we need to remember
        // to flip the inequality whenever we divide or multiply by A.

        // Starting with the plus solution:
        // 0 <= -B/2A + sqrt(D)/2A < 1     <=>
        // 0 >=    -B + sqrt(D)    > 2A    <=>
        // B >=         sqrt(D)    > 2A-B

        // Taking the first one:
        // B >= sqrt(D)  is true iff B > 0 and
        // B^2 >= D          <=>
        // B^2 >= B^2 - 4AC  <=>
        // 0   >= -4AC       <=>
        // 0   >= C
        // That is,  B >= sqrt(D) iff (B > 0) and (C <= 0)  .

        // The second inequality:
        // sqrt(D) > 2A-B  is true iff 2A-B <= 0 or
        // D    > 4A^2+B^2-4AB  <=>
        // -4AC > 4A^2-4AB      <=>
        // 0    > 4A^2-4AB+4AC  <=>
        // 0    < A-B+C
        // That is,  sqrt(D) > 2A-B iff (2A-B <= 0) or (0 < A-B+C)  .
        if ((B > 0) && (C <= 0) && ((2*A-B <= 0) || (0 < A-B+C)))
        {
            sol = true;
        }


        // The minus solution now gives us:
        // 0  <= -B/2A - sqrt(D)/2A < 1     <=>
        // 0  >=    -B - sqrt(D)    > 2A    <=>
        // 0  <=     B + sqrt(D)    < 2A    <=>
        // -B <=         sqrt(D)    < 2A-B

        // The first one:
        // -B <= sqrt(D)  is true iff B >= 0 or
        // B^2 <= D        <=>
        // B^2 <= B^2-4AC  <=>
        // 0   <= -4AC     <=>
        // 0   <= C
        // That is,  -B <= sqrt(D) iff (B >= 0) or (C >= 0)  .

        // The second one:
        // sqrt(D) < 2A-B  is true iff 2A-B > 0 and
        // D    < 4A^2+B^2-4AB  <=>
        // -4AC < 4A^2-4AB      <=>
        // 0    < 4A^2-4AB+4AC  <=>
        // 0    > A-B+C
        // That is,  sqrt(D) < 2A-B <=> (2A-B > 0) and (A-B+C < 0)  .
        if (((B >= 0) || (C >= 0)) && (2*A-B > 0) && (A-B+C) < 0)
        {
            solSub = true;
        }
    }

    // If there are some solutions we still don't know whether they are on the
    // ray. Remember that s = c^{-1}((e-2g+j)t^2+2(g-e)t+e-a). To see if our
    // solutions lie on the ray, we need to check that s >= 0, that is:
    // c^{-1}((e-2g+j)t^2+2(g-e)t+e-a) >= 0  .
    // We note that  c^{-1} > 0 iff c > 0. Therefore the above inequality
    // simplifies to:
    // c((e-2g+j)t^2+2(g-e)t+e-a) >= 0  .

    // We wish to see for which t the polynomial above is positive. So we need
    // to solve it and check that our t's indeed lie such that it is positive.
    // By setting
    auto E = c*(e-2*g+j);
    auto F = c*(2*(g-e));
    auto G = c*(e-a);
    auto H = F*F-4*E*G;
    // the inequality can be written as  p(t) = Et^2 + Ft + G >= 0  . We note
    // that if  E > 0  and  H <= 0  we get  p(t) >= 0 no matter the value of t.
    // This can be seen by noting that only E*t^2 matters asymptotically, and
    // therefore  E > 0  gives that for some t,  p(t) > 0  . Since  H < 0
    // p(t) != 0  for all  t  and as  p(t)  is continuous, it must be positive
    // everywhere. Therefore our solutions will be correct.
    if (A == 0)
    {
        // Equation is now  Bt+C = 0  <=>  t = -C/B  . We have:
        // 0 <= t < 1     <=>
        // 0 <= -C/B < 1
        // In case  B >= 0  , this reduces to  0 <= -C < B  and otherwise
        // 0 >= -C > B:
        if ((B >= 0 && C <= 0 && -C < B) || (B < 0 && C >= 0 && -C > B))
        {
            // It is a solution. Now we plug it into the next polynomial.
            if (E > 0 && H <= 0) RETURN(1);///return 1;
            if (E < 0 && H < 0) RETURN(0);///return 0;
            // We need to check whether p(-C/B) >= 0. We get:
            // E*(-C/B)^2-F*C/B+G   >= 0  <=>
            // E*(-C)^2-B*C*F+B^2*G >= 0  <=>
            // C^2*E-B*C*F+B^2*G    >= 0  <=>
            // C*(C*E-B*F)+B^2*G    >= 0
            if (C*(C*E-B*F)+B*B*G >= 0) RETURN(1);///return 1;
            RETURN(0);///return 0;
        }
        else
        {
            RETURN(0);///return 0;
        }
    }
    if (E > 0 && H <= 0) RETURN(sol+solSub);///return sol+solSub;
    // Likewise, if  E < 0  and  H < 0  ,  p(t) < 0  no matter the value of t.
    if (E < 0 && H < 0) RETURN(0);///return 0;

    // Now we know all solutions where the line intersects the curve. If there
    // are none, the curve and ray will not intersect.
    if (sol + solSub == 0) RETURN(0);///return 0;

    // We start by dealing with specific degenerate cases (the easy ones).
    if (E == 0)
    {
        if (F == 0)
        {
            // p(t) >= 0  degenerates to  G >= 0
            if (G >= 0) RETURN(sol+solSub);///return sol+solSub;
            RETURN(0);///return 0;
        }
        // p(t) >= 0  degenerates to  Ft+G >= 0 iff Ft >= -G.
        if (D == 0)
        {
            // We have only one solution: t = -B/2A. It is a solution if:
            // F*(-B/2A) >= -G      <=>
            // F*B/2A    <= G       <=>
            // 2A*F*B    <= 4A^2*G  <=>
            // A*F*B     <= 2A^2*G
            if (A*F*B <= 2*A*A*G) RETURN(sol);///return sol;
            RETURN(0);///return 0;
        }
    }
    // Let # be shorthand for +- (and -# is then -+). The solutions can now be
    // written as  -B/2A#sqrt(D)/2A  . We now insert that into  p(t):
    // E(-B/2A#sqrt(D)/2A)^2+F(-B/2A#sqrt(D)/2A)+G    >= 0             <=>
    // E(-B#sqrt(D))^2+2AF(-B#sqrt(D))+4A^2G          >= 0             <=>
    // E(B^2+D-#2B*sqrt(D))-2ABF#2AF*sqrt(D)+4A^2G    >= 0             <=>
    // E(2B^2-4AC-#2B*sqrt(D))-2ABF#2AF*sqrt(D)+4A^2G >= 0             <=>
    // E(B^2-2AC-#B*sqrt(D))-ABF#AF*sqrt(D)+2A^2G     >= 0             <=>
    // B^2*E - 2ACE -# BE*sqrt(D) # AF*sqrt(D)        >= ABF - 2A^2*G  <=>
    // (#sqrt(D))*(AF-BE) >= ABF + 2ACE - 2A^2*G - B^2*E  <=>
    // (#sqrt(D))*(AF-BE) >= A(BF+2(CE-AG))-B^2*E   =: S
    auto S = A*(B*F+2*(C*E-A*G))-B*B*E;
    auto L = A*F-B*E;

    // (#sqrt(D))*L >= S
    // Now we start looking at +sqrt(D):
    // if L >= 0 then:
    //   S <= 0 or (S > 0 && D*L^2 > S^2)  <=> plus is a root.
    // if L < 0 then:
    //   (S <= 0 && D*L^2 <= S^2)  <=> plus is a root.
    if (L >= 0 && !(S <= 0 || (S > 0 && D*L*L > S*S))) sol = false;
    if (L < 0 && !(S <= 0 && D*L*L <= S*S)) sol = false;

    // Now look at -sqrt(D):
    // We get sqrt(D)*L <= -S
    // if L > 0 then:
    //   (S <= 0 && D*L^2 <= S^2)  <=> minus is a root
    // if L <= 0 then:
    //   S <= 0 or (S > 0 && D*L^2 >= S*S)  <=> minus is a root
    if (L > 0 && !(S <= 0 && D*L*L <= S*S)) solSub = false;
    if (L <= 0 && !(S <= 0 || (S > 0 && D*L*L >= S*S))) solSub = false;

    RETURN(sol+solSub);///return sol+solSub;
}

*/

bool Glyph::isInside(ivec2 pos) const
{
    size_t intersectCount = 0;
    size_t cBegin = 0;
    for (size_t i = 0; i < contours(); ++i)
    {
        size_t contourLength = contourEnd(i) - cBegin;
        if (contourLength < 3) continue; // Degenerate contour.
        LineSegment cLine;
        cLine.pos = position(cBegin + contourLength - 1);
        for (size_t j = cBegin; j < contourEnd(i); ++j)
        {
            if (isControl(j))
            {
                throw std::logic_error("Bézier curve implementation disabled.");
                /*QuadraticBezier qb;
                qb.p0 = cLine.pos;
                qb.p1 = position(j);
                qb.p2 = position((j-cBegin+1)%contourLength+cBegin);
                auto ic = intersectionCount(pos, qb);
                ///std::cout << "Bezier curve gives " << ic << " intersections.\n";
                intersectCount += ic;
                */
            }
            else
            {
                cLine.dir = position(j) - cLine.pos;


                bool rayBeginOnSegment = false;
                bool ict        = intersects(pos,
                                             cLine,
                                             rayBeginOnSegment,
                                             isZeroAcceptable(j));
                if (rayBeginOnSegment)
                {
                    // The set of points which the glyph consists of contains
                    // all curves. Therefore if the current point is on a
                    // segment, it is on the glyph.
                    return true;
                }
                intersectCount += ict;
            }
            cLine.pos = position(j);
        }
        cBegin = contourEnd(i);
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
    int maxFontWidth = info.bboxMax.x - info.bboxMin.x;
    int maxFontHeight = info.bboxMax.y - info.bboxMin.y;
    if (width <= 0)
    {
        if (height <= 0)
        {
            throw std::runtime_error("Bad render size.");
        }
        pixelHeight = (height * maxFontHeight) / info.emSize;
        if (pixelHeight < 2) pixelHeight = 2;
        pixelWidth = pixelHeight * glyph.info().width / glyph.info().height;
    }
    else
    {
        pixelWidth = (width * maxFontWidth) / info.emSize;
        if (pixelWidth < 2) pixelWidth = 2;
        pixelHeight = pixelWidth * glyph.info().height / glyph.info().width;
    }

    Image img(pixelWidth, pixelHeight);

    for (int y = 0; y < pixelHeight; ++y)
    {
        for (int x = 0; x < pixelWidth; ++x)
        {
            ivec2 glyphPos;
            glyphPos.x = glyph.info().hCursorX + x*glyph.info().width/pixelWidth;
            glyphPos.y = glyph.info().hCursorY - y*glyph.info().height/pixelHeight;
            img.setPixel(x, y, 0x3d3d5c);
            if (auto cnt = glyph.isInside(glyphPos))
            {
                std::cout << std::hex << cnt;
                if (cnt) img.setPixel(x, y, 0xffffff);
            }
            else
            {
                std::cout << "0";
            }
        }
        std::cout << "\n";
    }
    return img;
}
