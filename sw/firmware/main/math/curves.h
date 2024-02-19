#pragma once

#include <optional>
#include <memory>

#include "geometry.h"
#include "lmath.h"


namespace linalg {


template<typename T>
class Raycastable {
public:
    /**
     * @brief Raycasts a ray from origin in direction.
     * @param origin The origin of the ray.
     * @param direction The direction of the ray.
     * @return The point of intersection and the distance from origin to the point of intersection.
     */
    virtual std::optional<std::pair<Point<T>, T>> raycast(Point<T> origin, Vector<T> direction) const = 0;
    virtual std::string to_string() const = 0;
    virtual ~Raycastable() = default;
};


template<typename T>
class Line : public Raycastable<T> {
public:
    Point<T> p1;
    Point<T> p2;

    Line(Point<T> p1, Point<T> p2) : p1(p1), p2(p2) {}

    std::optional<std::pair<Point<T>, T>> raycast(Point<T> origin, Vector<T> direction) const override {
        auto [x1, y1] = p1;
        auto [x2, y2] = p2;
        auto [x3, y3] = origin;
        auto [x4, y4] = origin + direction;
        auto denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if (denominator == 0) {
            return std::nullopt;
        }

        auto t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
        auto u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

        if (t >= 0 && t <= 1 && u >= 0) {
            return std::make_pair(origin + direction * u, u);
        } else {
            return std::nullopt;
        }
    }

    T distance(Point<T> point) const {
        auto [x1, y1] = p1;
        auto [x2, y2] = p2;
        auto [x3, y3] = point;
        auto numerator = lmath::abs((x2 - x1) * (y1 - y3) - (x1 - x3) * (y2 - y1));
        auto denominator = lmath::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        return numerator / denominator;
    }

    std::string to_string() const override {
        return "Line(" + std::to_string(p1) + "," + std::to_string(p2) + ")";
    }
};

template<typename T>
std::string to_string(const Line<T> &object) {
    return object.to_string();
}


template<typename T>
class Circle : public Raycastable<T> {
    Point<T> center;
    T radius;
public:
    Circle(Point<T> center, T radius) : center(center), radius(radius) {}

    std::optional<std::pair<Point<T>, T>> raycast(Point<T> origin, Vector<T> direction) const override {
        auto a = direction.length2();
        auto b = 2 * direction.dot(origin - center);
        auto c = (origin - center).length2() - radius * radius;

        auto discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return std::nullopt;
        }
        auto t = (-b - lmath::sqrt(discriminant)) / (2 * a);

        if (t < 0) {
            return std::nullopt;
        }

        return std::make_pair(origin + direction * t, t);

    }

    std::string to_string() const override {
        return "Circle(" + std::to_string(center) + "," + std::to_string(radius) + ")";
    }
};

template<typename T>
std::string to_string(const Circle<T> &object) {
    return object.to_string();
}


template<typename T>
class HermiteCubic {
    std::pair<Point<T>, Vector<T>> start;
    std::pair<Point<T>, Vector<T>> end;
public:
    HermiteCubic(std::pair<Point<T>, Vector<T>> start, std::pair<Point<T>, Vector<T>> end) : start(start), end(end) {}

    Point<T> get(float t) const {
        return Point<float>(0, 0) + (
            (start.first - Point<float>(0, 0)) * (2*t*t*t - 3*t*t + 1) +
            (start.second) * (t*t*t - 2*t*t + t) +
            (end.first - Point<float>(0, 0)) * (-2*t*t*t + 3*t*t) +
            (end.second) * (t*t*t - t*t)
        );
    }
};

template<typename T>
class HermiteSpline {
public:
    std::vector<std::pair<Point<T>, Vector<T>>> points;
    std::vector<float> knots;
    HermiteSpline(std::vector<std::pair<Point<T>, Vector<T>>> points) : points(points) {
        knots.emplace_back(0);
        for (int i = 1; i < points.size(); ++i) {
            knots.emplace_back(knots.back() + (points[i].first - points[i - 1].first).length());
        }
        for (int i = 0; i < knots.size(); ++i) {
            knots[i] /= knots.back();
        }
    }


    Point<T> get(float t) const {
        if (t <= 0) {
            return points.front().first;
        }

        if (t >= 1) {
            return points.back().first;
        }

        int i = std::upper_bound(knots.begin(), knots.end(), t) - knots.begin() - 1;

        float t1 = knots[i];
        float t2 = knots[i + 1];

        return HermiteCubic<T>(points[i], points[i + 1]).get((t - t1) / (t2 - t1));
    }
};


} // namespace linalg
