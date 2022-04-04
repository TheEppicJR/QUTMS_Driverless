use super::pathperimitives;
use pathperimitives::PointF2M;

pub fn ccw(p1: &PointF2M, p2: &PointF2M, p3: &PointF2M) -> bool {
    (p3.y - p1.y) * (p2.x - p1.x) - (p2.y - p1.y) * (p3.x - p1.x) > 0.0
}

pub fn lines_intersect(p1: &PointF2M, p2: &PointF2M, p3: &PointF2M, p4: &PointF2M) -> bool {
    ccw(p1, p3, p4) != ccw(p2, p3, p4) && ccw(p1, p2, p3) != ccw(p1, p2, p4)
}

pub fn get_line_intersection(
    p1: &PointF2M,
    p2: &PointF2M,
    p3: &PointF2M,
    p4: &PointF2M,
) -> Option<PointF2M> {
    let a1 = p2.y - p1.y;
    let b1 = p1.x - p2.x;
    let c1 = a1 * p1.x + b1 * p1.y;

    let a2 = p4.y - p3.y;
    let b2 = p3.x - p4.x;
    let c2 = a2 * p3.x + b2 * p3.y;

    let delta = a1 * b2 - a2 * b1;

    if delta == 0.0 {
        return None;
    }

    Some(PointF2M::new(
        (b2 * c1 - b1 * c2) / delta,
        (a1 * c2 - a2 * c1) / delta,
    ))
}

pub fn distance_between_points(p1: &PointF2M, p2: &PointF2M) -> f64 {
    ((p1 - p2).x * (p1 - p2).x + (p1 - p2).y * (p1 - p2).y).sqrt() as f64
}

pub fn is_left_cone(p1: &PointF2M, p2: &PointF2M, yaw: f64) -> bool {
    let v1 = p2 - p1;
    let angle = v1[1].atan2(v1[0]) as f64;
    let delta = wrap_pi(angle - yaw);
    delta > 0.0
}

pub fn wrap_pi(angle: f64) -> f64 {
    let mut angle = angle;
    while angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    while angle < -std::f64::consts::PI {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}
