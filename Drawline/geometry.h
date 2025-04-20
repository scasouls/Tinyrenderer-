#pragma once
#include <iostream>
#include <cmath>

template<class T>
struct Vec2 {
    union {
        struct { T u, v; };
        struct { T x, y; };
        T raw[2];
    };

    Vec2() : u(0), v(0) {}
    Vec2(T _u, T _v) : u(_u), v(_v) {}

    inline Vec2<T> operator+(const Vec2<T>& V) const { return Vec2<T>(u + V.u, v + V.v); }
    inline Vec2<T> operator-(const Vec2<T>& V) const { return Vec2<T>(u - V.u, v - V.v); }
    inline Vec2<T> operator*(float a) const { return Vec2<T>(u * a, v * a); }
};

template<class T>
struct Vec3 {
    union {
        struct { T x, y, z; };
        struct { T vert, iuv, normal; };
        T raw[3];
    };

    Vec3() : x(0), y(0), z(0) {}
    Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

    inline Vec3<T> operator+(const Vec3<T>& V) const { return Vec3<T>(x + V.x, y + V.y, z + V.z); }
    inline Vec3<T> operator-(const Vec3<T>& V) const { return Vec3<T>(x - V.x, y - V.y, z - V.z); }
    inline Vec3<T> operator*(float a) const { return Vec3<T>(x * a, y * a, z * a); }
    inline float operator*(const Vec3<T>& V) const { return x * V.x + y * V.y + z * V.z; } // 点积
    inline Vec3<T> operator^(const Vec3<T>& V) const { // 叉积
        return Vec3<T>(
            y * V.z - z * V.y,
            z * V.x - x * V.z,
            x * V.y - y * V.x
        );
    }
};

// 类型别名
typedef Vec2<float> Vec2f;
typedef Vec2<int>   Vec2i;
typedef Vec3<float> Vec3f;
typedef Vec3<int>   Vec3i;

// 输出重载
template<class T>
std::ostream& operator<<(std::ostream& s, const Vec2<T>& v) {
    s << "(" << v.x << ", " << v.y << ")";
    return s;
}

template<class T>
std::ostream& operator<<(std::ostream& s, const Vec3<T>& v) {
    s << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return s;
}
