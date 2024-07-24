#pragma once

#include <vector>

#include "./core.h"


namespace logic {


class JaggedNavigator {
    using Spline = linalg::JaggedSpline<num>;

    Spline _spline;

    unsigned _currentSegment = 0;
public:
    JaggedNavigator() = default;

    void setSpline(Spline spline) {
        _spline = std::move(spline);
    }

    void findNearestSegment(Point pos) {
        num minDistance = std::numeric_limits<num>::max();
        unsigned minIndex = 0;
        for (unsigned i = 0; i < _spline.points.size() - 1; ++i) {
            num distance = _spline.segment(i).distance(pos);
            if (distance < minDistance) {
                minDistance = distance;
                minIndex = i;
            }
        }
    }

    void updateSegment(Point pos) {
        if (_currentSegment == _spline.points.size() - 1) {
            return;
        }

        if (_spline.segment(_currentSegment).distance(pos) < _spline.segment(_currentSegment + 1).distance(pos)) {
            return;
        }

        _currentSegment++;
    }

    std::pair<num, num> getMotorPowers(Point pos, Vector dir, bool allowBackwards) {
        Line segment = _spline.segment(_currentSegment);

        auto angle = segment.asVector().angle(dir);
        auto backwards = allowBackwards && lmath::abs(angle) > lmath::radians<decltype(angle)>(90);

        // fix the angle
        if (backwards) {
            if (angle > 0) {
                angle = angle - lmath::radians<decltype(angle)>(180);
            }
            else {
                angle = angle + lmath::radians<decltype(angle)>(180);
            }
        }

        if (lmath::abs(angle) > lmath::radians<decltype(angle)>(45)) {
            // angle is too big, we need to turn

            if (angle > 0 != backwards) {
                return {100, -100};
            }
            else {
                return {-100, 100};
            }
        }
        else {
            num distance = segment.distance(pos);

            Vector target(1_n, distance);
            if (backwards) {
                target = -target;
            }

            auto [left, right] = target.normalize();

            return {left, right};
        }
    }
};


} // namespace logic
