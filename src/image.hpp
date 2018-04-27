#ifndef IMAGE_HPP_INCLUDED
#define IMAGE_HPP_INCLUDED

#include "types.hpp"
#include <string>
#include <vector>

struct Colour
{
    U8 r;
    U8 g;
    U8 b;
    U8 a;
    Colour() : r{0}, g{0}, b{0}, a{255} {}
    Colour(U8 rx, U8 gx, U8 bx) : r{rx}, g{gx}, b{bx}, a{255} {}
    Colour(U8 rx, U8 gx, U8 bx, U8 ax) : r{rx}, g{gx}, b{bx}, a{ax} {}
    Colour(U32 c) : r((c>>16)&0xFF),
                    g((c>>8)&0xFF),
                    b(c&0xFF),
                    a{255} {}
};

struct Image
{
    std::string name;
    size_t width;
    size_t height;
    std::vector<U8> p;
    Image() = default;
    Image(size_t w, size_t h, std::string n = "")
    : name{n}, width{w}, height{h}
    {
        p.resize(width*height*4);
    }

    void resize(size_t w, size_t h)
    {
        p.resize(w*h*4);
        width = w;
        height = h;
        for (size_t x = 0; x < width; ++x)
        {
            for (size_t y = 0; y < height; ++y)
            {
                setPixel(x, y, (x+y)&1 ? 0 : 0xffffff);
            }
        }
    }

    inline Colour pixel(size_t x, size_t y) const
    {
        return {p[4*width*y+4*x  ], p[4*width*y+4*x+1],
                p[4*width*y+4*x+2], p[4*width*y+4*x+3]};
    }

    inline void setPixel(size_t x, size_t y, Colour c)
    {
        p[4*width*y+4*x] = c.r;
        p[4*width*y+4*x+1] = c.g;
        p[4*width*y+4*x+2] = c.b;
        p[4*width*y+4*x+3] = c.a;
    }
};

void writeImage(const Image& img);

#endif // IMAGE_HPP_INCLUDED

