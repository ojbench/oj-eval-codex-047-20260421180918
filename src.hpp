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

Vec Controller::get_v_next() {
    // If already effectively at target, keep current velocity minimal to avoid churn
    Vec to_tar = pos_tar - pos_cur;
    double dist = to_tar.len();

    // small threshold relative to radius to avoid oscillation near target
    double arrive_eps = r * 0.1;
    if (arrive_eps < 1e-3) arrive_eps = 1e-3;

    // Desired speed scaling: slow down when close
    double desired_speed = v_max;
    {
        double horizon = 5 * r;
        if (horizon < 1e-9) horizon = 1e-9;
        if (dist < horizon) {
            double ratio = dist / horizon;
            if (ratio < 0.0) ratio = 0.0;
            if (ratio > 1.0) ratio = 1.0;
            // Linearly scale down within 5 radii
            desired_speed = v_max * ratio;
        }
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
