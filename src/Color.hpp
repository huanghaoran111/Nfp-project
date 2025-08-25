#pragma once
#include <cstdint>
#include <imgui.h>

struct Color{
    unsigned char r, g, b, a;
    Color() : r(0), g(0), b(0), a(255) {}
    Color(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) : r(r), g(g), b(b), a(a) {}
    Color(uint32_t color) : r((color >> 16) & 0xFF), g((color >> 8) & 0xFF), b(color & 0xFF) {}
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
        return ImColor(color.r, color.g, color.b, color.a);
    }

};