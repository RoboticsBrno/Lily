# cython: language_level=3
# cython: boundscheck=False
# cython: wraparound=False
# cython: cdivision=True

import numpy as np
cimport numpy as np
cimport cython
from libc.math cimport sin, cos, sqrt, fmod, M_PI

np.import_array()


cdef inline double _mod2pi(double angle):
    cdef double twopi = 2.0 * M_PI
    angle = fmod(angle, twopi)
    if angle < 0:
        angle += twopi
    return angle


cdef class BearDetectorCore:
    """Cython core for the BearDetector hot loop.

    Manages the sliding window, feature-vector update and per-point
    classification entirely in C speed.
    """

    cdef int _feature_detection_points
    cdef int _mid
    cdef double _last_angle
    cdef double[2] _feat_vec
    cdef double _min_distance
    cdef double _max_distance
    cdef double _feature_threshold
    cdef double _bear_feature_norm_bias

    # Circular buffer for waiting points (always full)
    cdef double[:, ::1] _waiting_pts
    cdef double[::1] _waiting_angles
    cdef double[::1] _waiting_dists
    cdef int _waiting_head

    def __init__(self, int feature_detection_points, double min_distance,
                 double max_distance, double feature_threshold,
                 double bear_feature_norm_bias):
        if feature_detection_points < 3:
            raise ValueError("feature_detection_points must be >= 3")
        self._feature_detection_points = feature_detection_points
        self._mid = feature_detection_points // 2
        self._min_distance = min_distance
        self._max_distance = max_distance
        self._feature_threshold = feature_threshold
        self._bear_feature_norm_bias = bear_feature_norm_bias

        self._last_angle = 0.0
        self._feat_vec[0] = 0.0
        self._feat_vec[1] = 0.0

        self._waiting_pts = np.zeros((feature_detection_points, 2), dtype=np.float64)
        self._waiting_angles = np.zeros(feature_detection_points, dtype=np.float64)
        self._waiting_dists = np.full(feature_detection_points, 1.0, dtype=np.float64)
        self._waiting_head = 0

    cdef inline void _push_waiting(self, double x, double y, double angle, double dist):
        cdef int idx = self._waiting_head
        self._waiting_pts[idx, 0] = x
        self._waiting_pts[idx, 1] = y
        self._waiting_angles[idx] = angle
        self._waiting_dists[idx] = dist
        self._waiting_head = (self._waiting_head + 1) % self._feature_detection_points

    cdef inline double _get_waiting_x(self, int logical_idx):
        cdef int physical = (self._waiting_head + logical_idx) % self._feature_detection_points
        return self._waiting_pts[physical, 0]

    cdef inline double _get_waiting_y(self, int logical_idx):
        cdef int physical = (self._waiting_head + logical_idx) % self._feature_detection_points
        return self._waiting_pts[physical, 1]

    cdef inline double _get_waiting_angle(self, int logical_idx):
        cdef int physical = (self._waiting_head + logical_idx) % self._feature_detection_points
        return self._waiting_angles[physical]

    cdef inline double _get_waiting_dist(self, int logical_idx):
        cdef int physical = (self._waiting_head + logical_idx) % self._feature_detection_points
        return self._waiting_dists[physical]

    cpdef list process(self, double robot_x, double robot_y, double robot_yaw,
                       double[:] dxs, double[:] dys, double[:] angles, double[:] distances):
        cdef int n = dxs.shape[0]
        cdef int i
        cdef double dist, x, y, angle
        cdef double sin_ = sin(robot_yaw)
        cdef double cos_ = cos(robot_yaw)
        cdef double fd = <double>self._feature_detection_points
        cdef double feat_val, scale
        cdef double mid_x, mid_y, mid_angle, mid_dist
        cdef double w0_x, w0_y, wmid_x, wmid_y, wmid1_x, wmid1_y
        cdef int mid = self._mid
        cdef list events = []

        for i in range(n):
            x = dxs[i] * cos_ - dys[i] * sin_ + robot_x
            y = dxs[i] * sin_ + dys[i] * cos_ + robot_y
            angle = _mod2pi(angles[i] + robot_yaw)
            dist = distances[i]

            if angle > 1.5 * M_PI and self._last_angle < 0.5 * M_PI:
                events.append((0,))

            self._last_angle = angle

            if dist < self._min_distance or dist >= self._max_distance:
                continue

            # Save values needed for feat_vec update before push overwrites oldest slot
            w0_x = self._get_waiting_x(0)
            w0_y = self._get_waiting_y(0)
            wmid_x = self._get_waiting_x(mid)
            wmid_y = self._get_waiting_y(mid)
            wmid1_x = self._get_waiting_x(mid + 1)
            wmid1_y = self._get_waiting_y(mid + 1)

            # Push current measurement into circular buffer
            self._push_waiting(x, y, angle, dist)

            # Update feature vector: feat_vec += wmid * fd - wmid1 * fd - w0 + w_end
            self._feat_vec[0] += (wmid_x - wmid1_x) * fd - w0_x + x
            self._feat_vec[1] += (wmid_y - wmid1_y) * fd - w0_y + y

            # Retrieve the mid point from the NEW buffer state
            mid_x = self._get_waiting_x(mid)
            mid_y = self._get_waiting_y(mid)
            mid_angle = self._get_waiting_angle(mid)
            mid_dist = self._get_waiting_dist(mid)

            # Compute feature value and scaled vector
            scale = 1.0 / (self._feature_detection_points * (mid_dist + self._bear_feature_norm_bias))
            feat_val = sqrt(self._feat_vec[0] * self._feat_vec[0] +
                            self._feat_vec[1] * self._feat_vec[1]) * scale

            if feat_val > self._feature_threshold:
                events.append((2, mid_x, mid_y,
                               self._feat_vec[0] * scale,
                               self._feat_vec[1] * scale,
                               mid_angle, feat_val))
            else:
                events.append((1, mid_x, mid_y,
                               self._feat_vec[0] * scale,
                               self._feat_vec[1] * scale,
                               mid_angle, feat_val))

        return events
