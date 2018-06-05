#include "image.hpp"

#include <fstream>
#include <stdexcept>

namespace
{

void writePnmHeader(std::ofstream& file, size_t width, size_t height)
{
    file << "P6\n" << width << ' ' << height << "\n255\n";
}

} // End anonymous namespace

void writeImage(const Image& img)
{
    std::ofstream file;
    std::string fname = img.name;
    if (fname.length() < 4 ||
        fname.substr(fname.length()-4) != ".pnm")
    {
        fname += ".pnm";
    }
    file.open(fname.c_str());
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open " + fname + " for writing.");
    }
    writePnmHeader(file, img.width, img.height);
    if (img.p.size() != 4*img.width*img.height)
    {
        throw std::runtime_error("Image width and/or height is wrong.");
    }
    std::vector<U8> data(3*img.width*img.height);
    size_t dPos = 0;

    for (size_t i = 0; i < img.p.size(); ++i)
    {
        if ((i&3)==3) continue;
        data.at(dPos++) = img.p[i];
    }
    file.write(reinterpret_cast<const char*>(data.data()), data.size());

    // Ugly (but quick) hack to get compressed output.
    file.close();
    int res = system(("compresspnm "+fname+' '+fname.substr(0, fname.size() - 4)+".png").c_str());
    (void)res;
}
