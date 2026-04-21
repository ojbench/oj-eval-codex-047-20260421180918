// OJ 2284 - 迅影的机器人
// Baseline controller implementing a conservative velocity policy.
// The OJ will include Vec, Monitor, TIME_INTERVAL, etc. We only implement Controller::get_v_next().

#pragma once

struct Vec { // fallback to satisfy local static analysis; OJ provides real version
    double x, y;
    Vec(double x_=0, double y_=0): x(x_), y(y_) {}
    Vec operator+(const Vec& o) const { return Vec(x+o.x, y+o.y); }
    Vec operator-(const Vec& o) const { return Vec(x-o.x, y-o.y); }
    Vec operator*(double k) const { return Vec(x*k, y*k); }
    Vec operator/(double k) const { return Vec(x/k, y/k); }
    double len() const { return std::sqrt(x*x + y*y); }
};

class Monitor; // forward decl; real definition is provided by OJ

class Controller {
public:
    // Exposed by framework (read-only in our logic):
    Vec pos_cur;    // current position
    Vec v_cur;      // current velocity
    Vec pos_tar;    // target position
    double r;       // radius
    double v_max;   // max speed
    int id;         // robot id (0-based)
    Monitor* monitor; // global monitor for observing others

    // Decide the velocity for next interval.
    Vec get_v_next();
};

// Utility: clamp vector magnitude to max_len (if needed)
static inline Vec clamp_vec(const Vec& v, double max_len){
    double L = v.len();
    if (L <= 1e-12) return Vec(0,0);
    if (L <= max_len) return v;
    double k = max_len / L;
    return Vec(v.x*k, v.y*k);
}

// Conservative baseline strategy:
// - Head towards the target with speed up to v_max.
// - Apply a simple short-range repulsion to reduce immediate overlap risk.
// - Slow down when very close to the target to avoid overshoot jitter.
// This focuses on safety and simplicity; the judge performs collision checks.

#include <cmath>
#include <vector>

// Declarations that match expected Monitor API (only signatures we call).
class Monitor {
public:
    bool get_speeding(int) const;
    std::vector<int> get_collision(int) const;
    bool get_warning() const;
    Vec get_pos_cur(int) const;
    Vec get_v_cur(int) const;
    double get_r(int) const;
    bool get_done() const;
    int get_robot_number() const;
    int get_test_id() const;
};

Vec Controller::get_v_next() {
    // If already effectively at target, keep current velocity minimal to avoid churn
    Vec to_tar = pos_tar - pos_cur;
    double dist = to_tar.len();

    // small threshold relative to radius to avoid oscillation near target
    double arrive_eps = std::max(1e-3, r * 0.1);

    // Desired speed scaling: slow down when close
    double desired_speed = v_max;
    if (dist < 5 * r + 1e-9) {
        // Linearly scale down within 5 radii
        desired_speed = v_max * std::max(0.0, dist / std::max(1e-9, 5 * r));
    }

    Vec v_des(0,0);
    if (dist > arrive_eps) {
        // unit direction to target
        if (dist > 1e-12) v_des = to_tar * (1.0 / dist) * desired_speed;
    } else {
        // close enough: gently brake
        v_des = Vec(0,0);
    }

    // Simple short-range repulsion from nearby robots (only using observable info)
    // This is heuristic; judge will validate no-collision. We keep it conservative.
    int n = 0;
    if (monitor) n = monitor->get_robot_number();
    Vec repel(0,0);
    if (n > 0 && monitor) {
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            double rj = monitor->get_r(j);
            Vec d = pos_cur - pj;
            double L = d.len();
            double safe = r + rj; // touching distance
            if (L < 1e-9) {
                // identical position: push arbitrarily outward with small magnitude
                repel = repel + Vec(0.0, (id < j ? 1.0 : -1.0)) * (v_max * 0.2);
                continue;
            }
            // If within 1.5x safety distance, apply repulsion that grows as we get closer
            double horizon = 1.5 * safe;
            if (L < horizon) {
                double k = (horizon - L) / horizon; // in (0,1)
                // Scale relative to available speed; stronger when too close
                double mag = v_max * 0.6 * k;
                repel = repel + d * (mag / L); // direction away from neighbor
            }
        }
    }

    // Combine and cap to v_max
    Vec v_next = v_des + repel;
    v_next = clamp_vec(v_next, v_max);

    return v_next;
}

