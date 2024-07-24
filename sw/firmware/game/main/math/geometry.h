#pragma once

#include <tuple>
#include <cmath>
#include <string>

#include "./lmath.h"


namespace linalg {


template<typename T>
class Vector;

template<typename T>
class Point {
public:
    T x;
    T y;

    Point() : x(0), y(0) {}
    Point(T x, T y) : x(x), y(y) {}

    Point operator+(Vector<T> vec) const {
        return Point(x + vec.x, y + vec.y);
    }

    Point operator-(Vector<T> vec) const {
        return Point(x - vec.x, y - vec.y);
    }

    Vector<T> operator-(Point other) const;

    template<int I>
    T get() const {
        if constexpr (I == 0) {
            return x;
        } else if constexpr (I == 1) {
            return y;
        }
    }

    template<typename U>
    Point<U> cast() const {
        return Point<U>(static_cast<U>(x), static_cast<U>(y));
    }
};


template<typename T>
class Vector {
public:
    T x;
    T y;

    Vector() : x(0), y(0) {}
    Vector(T x, T y) : x(x), y(y) {}

    Vector operator+(Vector other) const {
        return Vector(x + other.x, y + other.y);
    }

    Vector operator-(Vector other) const {
        return Vector(x - other.x, y - other.y);
    }

    Vector operator-() const {
        return Vector(-x, -y);
    }

    Vector operator*(T scalar) const {
        return Vector(x * scalar, y * scalar);
    }

    Vector operator/(T scalar) const {
        return Vector(x / scalar, y / scalar);
    }

    bool operator==(Vector other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(Vector other) const {
        return x != other.x || y != other.y;
    }

    T dot(Vector other) const {
        return x * other.x + y * other.y;
    }

    T cross(Vector other) const {
        return x * other.y - y * other.x;
    }

    float angle() const {
        return lmath::atan2(y, x);
    }

    float angle(Vector other) const {
        return lmath::atan2(cross(other), dot(other));
    }

    T length2() const {
        return x * x + y * y;
    }

    T length() const {
        return lmath::sqrt(length2());
    }

    Vector normalize() const {
        return *this / length();
    }

    template<bool Dir>
    Vector normal() const {
        if constexpr (Dir) {
            return Vector(-y, x);
        } else {
            return Vector(y, -x);
        }
    }

    Vector rotate(T angle) const {
        T cos = lmath::cos(angle);
        T sin = lmath::sin(angle);
        return Vector(
            x * cos - y * sin,
            x * sin + y * cos
        );
    }

    template<int I>
    T get() const {
        if constexpr (I == 0) {
            return x;
        } else if constexpr (I == 1) {
            return y;
        }
    }

    template<typename U>
    Vector<U> cast() const {
        return Vector<U>(static_cast<U>(x), static_cast<U>(y));
    }
};


template<typename T>
Vector<T> Point<T>::operator-(Point<T> other) const {
    return Vector(x - other.x, y - other.y);
}


template<typename T>
struct tuple_size;

template<int I, typename U>
struct tuple_element;


template<typename T>
struct tuple_size<Point<T>> : std::integral_constant<size_t, 2> {};

template<typename T>
struct tuple_element<0, Point<T>> {
    using type = T;
};

template<typename T>
struct tuple_element<1, Point<T>> {
    using type = T;
};

template<typename T>
std::string to_string(Point<T> point) {
    return "Point(" + std::to_string(point.x) + "," + std::to_string(point.y) + ")";
}

template<typename T>
struct tuple_size<Vector<T>> : std::integral_constant<size_t, 2> {};

template<typename T>
struct tuple_element<0, Vector<T>> {
    using type = T;
};

template<typename T>
struct tuple_element<1, Vector<T>> {
    using type = T;
};

template<typename T>
std::string to_string(Vector<T> vector) {
    return "Vector(" + std::to_string(vector.x) + "," + std::to_string(vector.y) + ")";
}


} // namespace linalg
