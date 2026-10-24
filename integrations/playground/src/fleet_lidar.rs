//! Measured-cloud brake governor for the optional arcade taxi lab.
//! The sensor simulator knows weather; the governor receives only returned
//! points, detector geometry, measured speed and a supplied-map turn intent.
use std::cell::RefCell;
use rustdriving_core::{Detection, LidarScan, Perception, PlanarLidarCalibration, PlanarLidarSweep, PlanarRayOutcome, Pose, Vec2};
use rustdriving_perception::LidarClusters;
use rustdriving_sim::lidar_weather::{WeatherConfig, apply};

const RAYS: usize = 360;
const RANGE: f64 = 45.0;
#[derive(Clone)]
struct Sensor {
    stamp: f64,
    packet_stamp: f64,
    ready: bool,
    ranges: Vec<f64>,
    points: Vec<Vec2>,
    detections: Vec<Detection>,
    limit: f64,
    nearest: f64,
    scans: u64,
    base_returns: usize,
    weather: u32,
    strength: f64,
    braking: bool,
}
impl Default for Sensor {
    fn default() -> Self {
        Self { stamp: -1.0, packet_stamp: -1.0, ready: false, ranges: Vec::new(), points: Vec::new(),
            detections: Vec::new(), limit: 0.0, nearest: RANGE, scans: 0, base_returns: 0,
            weather: 0, strength: 0.0, braking: false }
    }
}
struct Lab { slots: Vec<Sensor>, current: Option<usize> }
thread_local! {
    static LAB: RefCell<Lab> = RefCell::new(Lab { slots: vec![Sensor::default(); 16], current: None });
}
fn path_at(s: f64, curvature: f64, bend: f64) -> Vec2 {
    if curvature.abs() < 1e-8 { return Vec2::new(s, 0.0); }
    let curve = s.min(bend);
    let angle = curvature * curve;
    Vec2::new(angle.sin() / curvature + (s - curve) * angle.cos(),
        (1.0 - angle.cos()) / curvature + (s - curve) * angle.sin())
}
fn govern(points: &[Vec2], detections: &[Detection], speed: f64, curvature: f64, bend: f64) -> (f64, f64) {
    let horizon = (speed * speed / 10.0 + speed * 0.3 + 8.0).clamp(12.0, RANGE);
    let path: Vec<_> = (0..=(horizon * 2.0).ceil() as usize).map(|i| (i as f64 * 0.5, path_at(i as f64 * 0.5, curvature, bend))).collect();
    let mut nearest = RANGE;
    let close = points.iter().filter(|p| p.x > 0.0 && p.x < 3.5 && p.y.abs() < 1.15).count();
    if close >= 2 { nearest = 0.0; }
    for point in points {
        // Ignore unsupported speckles outside the immediate emergency region.
        if !detections.iter().any(|d| point.distance(d.center) <= d.radius + 0.05) { continue; }
        for (along, center) in &path {
            if point.distance(*center) < 1.15 { nearest = nearest.min(*along); break; }
        }
    }
    let limit = ((2.25 + 10.0 * (nearest - 3.5).max(0.0)).sqrt() - 1.5).clamp(0.0, 14.0);
    (limit, nearest)
}
#[unsafe(no_mangle)]
pub extern "C" fn fleet_lidar_begin(slot: u32, stamp: f64, weather: u32, strength: f64) -> bool {
    LAB.with(|lab| {
        let mut lab = lab.borrow_mut();
        lab.current = None;
        let Some(sensor) = lab.slots.get_mut(slot as usize) else { return false; };
        sensor.ready = false; sensor.limit = 0.0; sensor.ranges.clear();
        if !stamp.is_finite() || stamp < 0.0 || stamp <= sensor.stamp ||
            weather > 2 || !strength.is_finite() || !(0.0..=1.0).contains(&strength) { return false; }
        sensor.packet_stamp = stamp; sensor.weather = weather; sensor.strength = strength;
        lab.current = Some(slot as usize); true
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn fleet_lidar_ray(beam: u32, range: f64) -> bool {
    LAB.with(|lab| {
        let mut lab = lab.borrow_mut();
        let Some(slot) = lab.current else { return false; };
        let sensor = &mut lab.slots[slot];
        if beam as usize != sensor.ranges.len() || beam as usize >= RAYS ||
            !range.is_finite() || !(0.2..=RANGE).contains(&range) {
            sensor.ready = false; sensor.limit = 0.0; lab.current = None; return false;
        }
        sensor.ranges.push(range); true
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn fleet_lidar_finish(slot: u32, speed: f64, curvature: f64, bend: f64) -> bool {
    LAB.with(|lab| {
        let mut lab = lab.borrow_mut();
        if lab.current.take() != Some(slot as usize) { return false; }
        let sensor = &mut lab.slots[slot as usize];
        if sensor.ranges.len() != RAYS || ![speed, curvature, bend].into_iter().all(f64::is_finite) ||
            !(0.0..=20.0).contains(&speed) || curvature.abs() > 1.0 || !(0.0..=RANGE).contains(&bend) { return false; }
        let mut input = LidarScan { stamp: sensor.packet_stamp, points: Vec::new(), sweep: None };
        let rays = sensor.ranges.iter().enumerate().map(|(i, range)| {
            if *range >= RANGE { return PlanarRayOutcome::Clear; }
            let angle = -std::f64::consts::PI + std::f64::consts::TAU * i as f64 / RAYS as f64;
            let index = input.points.len(); input.points.push(Vec2::new(angle.cos() * range, angle.sin() * range));
            PlanarRayOutcome::Return(index)
        }).collect();
        input.sweep = Some(PlanarLidarSweep { calibration: PlanarLidarCalibration {
            ray_count: RAYS, first_angle_rad: -std::f64::consts::PI, angular_step_rad: std::f64::consts::TAU / RAYS as f64,
            min_range_m: 0.2, max_range_m: RANGE, range_error_m: 0.05 }, rays });
        sensor.base_returns = input.points.len();
        let s = sensor.strength;
        let config = match sensor.weather {
            1 => WeatherConfig { extinction_m_inv: 0.018*s, backscatter_rate_m_inv: 0.0025*s, range_jitter_m: 0.12*s },
            2 => WeatherConfig { extinction_m_inv: 0.01*s, backscatter_rate_m_inv: 0.015*s, range_jitter_m: 0.3*s },
            _ => WeatherConfig::CLEAR,
        };
        let Ok(observed) = apply(&input, config, 1701 ^ u64::from(slot).wrapping_mul(7919)) else { return false; };
        let detections = LidarClusters.detect(&observed.scan, Pose { position: Vec2::default(), yaw: 0.0 });
        // No weather label, clean scan, source label or unavailable-ray
        // diagnostic enters this function.
        let (limit, nearest) = govern(&observed.scan.points, &detections, speed, curvature, bend);
        sensor.points = observed.scan.points; sensor.detections = detections;
        sensor.limit = limit; sensor.nearest = nearest; sensor.braking = limit < speed - 0.25;
        sensor.stamp = sensor.packet_stamp; sensor.ready = true; sensor.scans += 1; true
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn fleet_lidar_read(slot: u32, field: u32) -> f64 {
    LAB.with(|lab| {
        let lab = lab.borrow(); let Some(s) = lab.slots.get(slot as usize) else { return f64::NAN; };
        match field { 0 => s.stamp, 1 => if s.ready { 1.0 } else { 0.0 }, 2 => s.points.len() as f64, 3 => s.detections.len() as f64,
            4 => s.nearest, 5 => s.limit, 6 => if s.braking { 1.0 } else { 0.0 }, 7 => s.base_returns as f64, 8 => s.scans as f64, _ => f64::NAN }
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn fleet_lidar_point(slot: u32, index: u32, axis: u32) -> f64 {
    LAB.with(|lab| {
        let lab = lab.borrow();
        let Some(p) = lab.slots.get(slot as usize).and_then(|s| s.points.get(index as usize)) else { return f64::NAN; };
        match axis { 0 => p.x, 1 => p.y, _ => f64::NAN }
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn fleet_lidar_detection(slot: u32, index: u32, field: u32) -> f64 {
    LAB.with(|lab| {
        let lab = lab.borrow();
        let Some(d) = lab.slots.get(slot as usize).and_then(|s| s.detections.get(index as usize)) else { return f64::NAN; };
        match field { 0 => d.center.x, 1 => d.center.y, 2 => d.radius, _ => f64::NAN }
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    fn scan(slot: u32, time: f64, weather: u32, strength: f64, range: f64) {
        assert!(fleet_lidar_begin(slot, time, weather, strength));
        for i in 0..360 {
            let angle = -std::f64::consts::PI + std::f64::consts::TAU * i as f64 / 360.0;
            let r = if angle.abs() < 0.1 { range } else { 45.0 };
            assert!(fleet_lidar_ray(i, r));
        }
        assert!(fleet_lidar_finish(slot, 9.0, 0.0, 0.0));
    }
    #[test]
    fn measured_wall_brakes_without_actor_positions() {
        scan(0, 1.0, 0, 0.0, 8.0);
        assert!(fleet_lidar_read(0, 3) >= 1.0);
        assert!(fleet_lidar_read(0, 5) < 8.0);
        assert!(fleet_lidar_read(0, 4) < 9.0);
    }
    #[test]
    fn empty_scan_and_own_map_turn_do_not_invent_obstacles() {
        let (limit, distance) = govern(&[], &[], 9.0, 0.2, 7.0);
        assert_eq!(limit, 14.0); assert_eq!(distance, 45.0);
        let end = path_at(8.0, 0.2, std::f64::consts::FRAC_PI_2 / 0.2);
        assert!((end.x - 5.0).abs() < 0.01 && end.y > 5.0);
    }
    #[test]
    fn side_objects_do_not_block_the_measured_corridor() {
        let points = vec![Vec2::new(4.0, 2.6), Vec2::new(5.0, 2.6), Vec2::new(6.0, 2.6)];
        let detections = vec![Detection { center: Vec2::new(5.0, 2.6), radius: 2.0 }];
        assert_eq!(govern(&points, &detections, 9.0, 0.0, 0.0).0, 14.0);
    }
    #[test]
    fn missing_invalid_and_duplicate_packets_hold_the_governor() {
        assert!(fleet_lidar_begin(1, 1.0, 0, 0.0));
        assert!(!fleet_lidar_finish(1, 9.0, 0.0, 0.0));
        assert_eq!(fleet_lidar_read(1, 1), 0.0); assert_eq!(fleet_lidar_read(1, 5), 0.0);
        scan(1, 2.0, 0, 0.0, 45.0);
        assert!(!fleet_lidar_begin(1, 2.0, 0, 0.0));
        assert_eq!(fleet_lidar_read(1, 5), 0.0);
        assert!(fleet_lidar_begin(1, 3.0, 0, 0.0));
        assert!(!fleet_lidar_ray(0, f64::NAN));
        assert!(!fleet_lidar_finish(1, 9.0, 0.0, 0.0));
    }
    #[test]
    fn weather_changes_the_acquired_cloud_with_no_truth_labels_in_control() {
        scan(2, 1.0, 0, 0.0, 15.0);
        scan(3, 1.0, 2, 1.0, 15.0);
        assert_eq!(fleet_lidar_read(2, 7), fleet_lidar_read(3, 7));
        assert_ne!(fleet_lidar_read(2, 2), fleet_lidar_read(3, 2));
        assert!(fleet_lidar_read(3, 2) <= 360.0);
        assert!(fleet_lidar_point(3, 0, 0).is_finite());
    }
}
