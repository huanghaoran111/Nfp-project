#pragma once
#include <cstdint>
#include <imgui.h>

struct Color{
    unsigned char r, g, b, a;
    Color() : r(0), g(0), b(0), a(255) {}
    constexpr Color(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) : r(r), g(g), b(b), a(a) {}
    Color(uint32_t color) : a((color >> 24) & 0xFF), b((color >> 16) & 0xFF), g((color >> 8) & 0xFF), r(color & 0xFF) {}
    Color(ImVec4 color) : r(color.x * 255), g(color.y * 255), b(color.z * 255), a(color.w * 255) {}
    
    uint32_t toInt() const{
        return ((r << 24) | (g << 16) | (b << 8) | a);
    }

    void setColor(uint32_t color){
        r = (color >> 24) & 0xFF;
        g = (color >> 16) & 0xFF;
        b = (color >> 8) & 0xFF;
        a = color & 0xFF;
    }

    static ImU32 toImU32(Color color){
        return IM_COL32(color.r, color.g, color.b, color.a);
    }

};

namespace Colors{
    constexpr uint32_t WHITE = IM_COL32(255, 255, 255, 255);
    constexpr uint32_t RED = IM_COL32(255, 0, 0, 255);
    constexpr uint32_t GREEN = IM_COL32(0, 255, 0, 255);
    constexpr uint32_t BLUE = IM_COL32(0, 0, 255, 255);
    constexpr uint32_t BLACK = IM_COL32(0, 0, 0, 255);
}


// constexpr IM_COL32 WHITE = 0xFFFFFFFF;
// constexpr IM_COL32 BLACK = 0xFF000000;

