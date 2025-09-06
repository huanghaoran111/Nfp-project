#include <MathCore.h>

Vec2 Vec2::operator-(const Vec2& other) const {
    return Vec2(x - other.x, y - other.y);
}

Vec2 Vec2::operator+(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

float Vec2::operator*(const Vec2& other) const {
    return (x * other.x) + (y * other.y);
}

Vec2& Vec2::operator=(const Vec2& other) {
    this->x = other.x;
    this->y = other.y;
    return *this;
}

Vec2 Vec2::operator*(float scalar) const {
    return Vec2(x * scalar, y * scalar);
}

Vec2 operator*(float scalar, const Vec2& vec){
    return vec * scalar;
}

float Vec2::operator^(const Vec2& other) const{
    return x * other.y - y * other.x;
}

float Vec2::Cross(const Vec2& other) const{
    return *this ^ other;
}

bool Vec2::operator==(const Vec2& other) const{
    return (x == other.x) && (y == other.y);
}

bool Vec2::operator!=(const Vec2& other) const{
    return !(*this == other);
}

Vec2 Vec2::RotateCounterClockwise90()const {
    return { -this->y, this->x };
}

Vec2 Vec2::RotateClockwise90() const {
    return { this->y, -this->x };
}