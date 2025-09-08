#pragma once
#include <iostream>
#include <vector>
#include <imgui.h>

const float PI = 3.14159265359f;
constexpr float EPSILON = 1e-6;

struct Vec2{
    float x, y;
    Vec2(): x(0), y(0) {}
    Vec2(float x, float y): x(x), y(y) {}

    ImVec2 ToImVec2() const {return ImVec2(x, y);}

    Vec2 operator-() const {return Vec2(-this->x, -this->y);}
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator+(const Vec2& other) const;
    Vec2 operator*(float scalar) const;
    float operator*(const Vec2& other) const;
    Vec2& operator=(const Vec2& other);
    bool operator==(const Vec2& other) const;
    bool operator!=(const Vec2& other) const;
    friend Vec2 operator*(float scalar, const Vec2& vec);
    float operator^(const Vec2& other) const;
    float Cross(const Vec2& other) const;
    Vec2 RotateClockwise90() const;
    Vec2 RotateCounterClockwise90() const;
    double angle() const;
};
Vec2 operator*(float scalar, const Vec2& vec);
