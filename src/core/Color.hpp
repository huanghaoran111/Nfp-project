#include<cinttypes>
#include<imgui.h>
namespace NFP
{
    struct Color{
        unsigned char r, g, b, a;
        Color(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) : r(r), g(g), b(b), a(a) {}
        Color() : r(255), g(255), b(255) {}

        uint32_t toInt() const{
            return ((r << 16) | (g << 8) | b);
        }

        void setColor(uint32_t color){
            r = (color >> 16) & 0xFF;
            g = (color >> 8) & 0xFF;
            b = color & 0xFF;
        }

        static ImVec4 toVec4f(Color color){
            return {color.r / 255.0f, color.g / 255.0f, color.b / 255.0f, color.a / 255.0f};
        }

    };
}
