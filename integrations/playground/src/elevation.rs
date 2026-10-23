//! Arcade terrain contact and ballistic flight, isolated from the research stack.
//! Ground coordinates are east/north metres; height is metres above city grade.
use super::{Vehicle, wrap};
use crate::structures::{BODY_HEIGHT, blocks_body, ceiling, excavation, upper};

const GRAVITY: f64 = 9.81;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Surface {
    pub height: f64,
    pub gx: f64,
    pub gy: f64,
}

pub(crate) fn surface(x: f64, y: f64) -> Surface {
    if let Some(floor) = excavation(x, y) {
        return floor;
    }
    if !x.is_finite() || !y.is_finite() || !(110.0..=190.0).contains(&y) {
        return Surface::default();
    }
    if (200.0..340.0).contains(&x) {
        let t = (x - 200.0) / 140.0;
        Surface {
            height: 48.0 * t * t * (3.0 - 2.0 * t),
            gx: 48.0 * 6.0 * t * (1.0 - t) / 140.0,
            gy: 0.0,
        }
    } else if (340.0..372.0).contains(&x) {
        Surface {
            height: 48.0,
            ..Surface::default()
        }
    } else if (372.0..380.0).contains(&x) {
        Surface {
            height: 48.0 + (x - 372.0) * 0.125,
            gx: 0.125,
            gy: 0.0,
        }
    } else {
        Surface::default()
    }
}

// Pick only a layer reachable from this height. Lower roads survive at the same XY.
fn support(x: f64, y: f64, reachable_height: f64) -> Surface {
    let floor = surface(x, y);
    upper(x, y)
        .filter(|s| s.height <= reachable_height + 0.000_001 && s.height >= floor.height)
        .unwrap_or(floor)
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Elevation {
    height: f64,
    vertical: f64,
    grounded: bool,
    pitch: f64,
    roll: f64,
    air_time: f64,
    max_air_time: f64,
    landings: u32,
    landing_speed: f64,
    vx: f64,
    vy: f64,
}

impl Default for Elevation {
    fn default() -> Self {
        Self {
            height: 0.0,
            vertical: 0.0,
            grounded: true,
            pitch: 0.0,
            roll: 0.0,
            air_time: 0.0,
            max_air_time: 0.0,
            landings: 0,
            landing_speed: 0.0,
            vx: 0.0,
            vy: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct DrivingVehicle {
    car: Vehicle,
    elevation: Elevation,
}

impl DrivingVehicle {
    pub fn reset(&mut self, x: f64, y: f64, yaw: f64) -> bool {
        if !self.car.reset(x, y, yaw) {
            return false;
        }
        self.elevation = Elevation {
            height: surface(x, y).height,
            ..Elevation::default()
        };
        self.attitude(surface(x, y));
        true
    }

    fn velocity(&self) -> (f64, f64) {
        let (s, c) = self.car.yaw.sin_cos();
        (
            c * self.car.forward - s * self.car.lateral,
            s * self.car.forward + c * self.car.lateral,
        )
    }

    fn attitude(&mut self, ground: Surface) {
        let (s, c) = self.car.yaw.sin_cos();
        self.elevation.pitch = (ground.gx * c + ground.gy * s).atan();
        self.elevation.roll = (ground.gy * c - ground.gx * s).atan();
    }

    pub fn step(&mut self, throttle: f64, steering: f64, hand: f64, dt: f64) -> bool {
        if ![throttle, steering, hand, dt]
            .into_iter()
            .all(f64::is_finite)
            || !(0.000_1..=0.05).contains(&dt)
        {
            return false;
        }
        let ticks = (dt / 0.005).ceil() as usize;
        let h = dt / ticks as f64;
        for _ in 0..ticks {
            if self.elevation.grounded {
                self.on_ground(throttle, steering, hand, h);
            } else {
                self.in_air(h);
            }
        }
        true
    }

    fn on_ground(&mut self, throttle: f64, steering: f64, hand: f64, dt: f64) {
        let old = self.car;
        let ground = support(old.x, old.y, self.elevation.height);
        let (s, c) = old.yaw.sin_cos();
        let gravity_scale = -GRAVITY / (1.0 + ground.gx * ground.gx + ground.gy * ground.gy);
        self.car.forward += gravity_scale * (ground.gx * c + ground.gy * s) * dt;
        self.car.lateral += gravity_scale * (-ground.gx * s + ground.gy * c) * dt;
        self.car.step(throttle, steering, hand, dt);
        let dx = self.car.x - old.x;
        let dy = self.car.y - old.y;
        let next = support(
            self.car.x,
            self.car.y,
            self.elevation.height + 0.8 * dx.hypot(dy) + 0.001,
        );
        // A vertical face cannot be climbed by entering its height-field cell.
        if blocks_body(
            self.car.x,
            self.car.y,
            self.elevation.height.max(next.height),
        ) || (next.height > self.elevation.height + 0.01
            && next.height - ground.height > 0.8 * dx.hypot(dy) + 0.001)
        {
            self.car.x = old.x;
            self.car.y = old.y;
            self.car.forward = 0.0;
            self.car.lateral = 0.0;
            self.car.yaw_rate = 0.0;
            self.car.contacts = self.car.contacts.saturating_add(1);
            return;
        }
        let (vx, vy) = self.velocity();
        let vz = ground.gx * vx + ground.gy * vy;
        let free_height =
            self.elevation.height + ground.gx * dx + ground.gy * dy - 0.5 * GRAVITY * dt * dt;
        if free_height > next.height + 0.000_001 {
            self.elevation.grounded = false;
            self.elevation.height = free_height;
            self.elevation.vertical = vz - GRAVITY * dt;
            self.elevation.vx = vx;
            self.elevation.vy = vy;
            self.elevation.air_time = dt;
            self.elevation.max_air_time = self.elevation.max_air_time.max(dt);
            self.car.rear_slip = 0.0;
            self.car.front_slip = 0.0;
        } else {
            self.elevation.height = next.height;
            self.elevation.vertical = next.gx * vx + next.gy * vy;
            self.attitude(next);
        }
    }

    fn in_air(&mut self, dt: f64) {
        let previous = self.car;
        let ground = support(previous.x, previous.y, self.elevation.height);
        self.car.x += self.elevation.vx * dt;
        self.car.y += self.elevation.vy * dt;
        let mut next = support(self.car.x, self.car.y, self.elevation.height);
        let distance = (self.car.x - previous.x).hypot(self.car.y - previous.y);
        if blocks_body(
            self.car.x,
            self.car.y,
            self.elevation.height.max(next.height),
        ) || (self.elevation.height < next.height
            && next.height - ground.height > 0.8 * distance + 0.001)
        {
            self.car.x = previous.x;
            self.car.y = previous.y;
            self.elevation.vx *= -0.15;
            self.elevation.vy *= -0.15;
            self.car.contacts = self.car.contacts.saturating_add(1);
            next = ground;
        }
        let roof = ceiling(self.car.x, self.car.y, self.elevation.height);
        self.elevation.height += self.elevation.vertical * dt - 0.5 * GRAVITY * dt * dt;
        if self.elevation.height + BODY_HEIGHT > roof {
            self.elevation.height = roof - BODY_HEIGHT;
            self.elevation.vertical = self.elevation.vertical.min(0.0);
            self.car.contacts = self.car.contacts.saturating_add(1);
        }
        self.elevation.vertical -= GRAVITY * dt;
        self.elevation.air_time += dt;
        self.elevation.max_air_time = self.elevation.max_air_time.max(self.elevation.air_time);
        self.car.distance += (self.car.x - previous.x).hypot(self.car.y - previous.y);
        self.car.yaw = wrap(self.car.yaw + self.car.yaw_rate * dt);
        self.car.yaw_rate *= (-0.25 * dt).exp();
        let speed = self.elevation.vx.hypot(self.elevation.vy);
        let pitch = self.elevation.vertical.atan2(speed.max(0.1));
        self.elevation.pitch += (pitch - self.elevation.pitch) * (1.0 - (-2.0 * dt).exp());
        self.elevation.roll *= (-2.0 * dt).exp();
        if self.elevation.height <= next.height {
            self.elevation.landing_speed = (next.gx * self.elevation.vx
                + next.gy * self.elevation.vy
                - self.elevation.vertical)
                .max(0.0);
            self.elevation.height = next.height;
            self.elevation.vertical = next.gx * self.elevation.vx + next.gy * self.elevation.vy;
            self.elevation.grounded = true;
            self.elevation.landings = self.elevation.landings.saturating_add(1);
            self.elevation.air_time = 0.0;
            let retention = if self.elevation.landing_speed > 12.0 {
                0.65
            } else {
                0.96
            };
            self.elevation.vx *= retention;
            self.elevation.vy *= retention;
            self.car.yaw_rate *= 0.5;
            self.attitude(next);
        }
        let (s, c) = self.car.yaw.sin_cos();
        self.car.forward = c * self.elevation.vx + s * self.elevation.vy;
        self.car.lateral = -s * self.elevation.vx + c * self.elevation.vy;
    }

    /// Fracture a breakaway barrier by spending normal kinetic energy (joules).
    /// The authored arcade vehicle mass is 1,250 kg. No position/height impulse.
    pub fn break_barrier(&mut self, nx: f64, ny: f64, energy: f64) -> bool {
        if ![nx, ny, energy].into_iter().all(f64::is_finite) || energy <= 0.0 {
            return false;
        }
        let length = nx.hypot(ny);
        if length < 0.000_001 { return false; }
        let (nx, ny) = (nx / length, ny / length);
        let (vx, vy) = self.velocity();
        let incoming = -(vx * nx + vy * ny);
        let remaining = incoming * incoming - 2.0 * energy / 1_250.0;
        if incoming <= 0.0 || remaining <= 0.0 { return false; }
        let lost = incoming - remaining.sqrt();
        let (vx, vy) = (vx + nx * lost, vy + ny * lost);
        let (s, c) = self.car.yaw.sin_cos();
        self.car.forward = c * vx + s * vy;
        self.car.lateral = -s * vx + c * vy;
        self.car.contacts = self.car.contacts.saturating_add(1);
        if self.elevation.grounded {
            let ground = support(self.car.x, self.car.y, self.elevation.height);
            self.elevation.vertical = ground.gx * vx + ground.gy * vy;
        } else {
            self.elevation.vx = vx;
            self.elevation.vy = vy;
        }
        true
    }

    pub fn contact(&mut self, nx: f64, ny: f64, penetration: f64) -> bool {
        let old = self.car;
        if !self.car.contact(nx, ny, penetration) {
            return false;
        }
        if self.elevation.grounded {
            let next = support(self.car.x, self.car.y, self.elevation.height + 0.01);
            if next.height > self.elevation.height + 1.0
                || blocks_body(self.car.x, self.car.y, self.elevation.height)
            {
                self.car.x = old.x;
                self.car.y = old.y;
            } else if next.height < self.elevation.height - 1.0 {
                self.elevation.grounded = false;
                (self.elevation.vx, self.elevation.vy) = self.velocity();
            } else {
                self.elevation.height = next.height;
                self.attitude(next);
            }
        } else {
            (self.elevation.vx, self.elevation.vy) = self.velocity();
        }
        true
    }

    pub fn read(&self, index: u32) -> f64 {
        let e = &self.elevation;
        match index {
            11 => e.height,
            12 => e.vertical,
            13 => {
                if e.grounded {
                    1.0
                } else {
                    0.0
                }
            }
            14 => e.pitch,
            15 => e.roll,
            16 => e.air_time,
            17 => f64::from(e.landings),
            18 => e.landing_speed,
            19 => e.max_air_time,
            20 => support(self.car.x, self.car.y, e.height).height,
            21 => ceiling(self.car.x, self.car.y, e.height),
            _ => self.car.read(index),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn launch() -> DrivingVehicle {
        let mut car = DrivingVehicle::default();
        car.reset(190.0, 150.0, 0.0);
        for _ in 0..6000 {
            car.step(1.0, 0.0, 0.0, 1.0 / 120.0);
            if !car.elevation.grounded && car.car.x > 380.0 {
                return car;
            }
        }
        panic!("The keyboard-equivalent full-throttle run never left the cliff");
    }

    #[test]
    fn full_run_climbs_launches_and_lands() {
        let mut car = launch();
        assert!(car.elevation.height > 48.5);
        let start = car.car.x;
        for _ in 0..1200 {
            car.step(1.0, 0.0, 0.0, 1.0 / 120.0);
            if car.elevation.grounded {
                assert!(car.car.x > start + 35.0 && car.car.x < 550.0);
                assert!(car.elevation.height == 0.0);
                assert!(car.elevation.max_air_time > 3.0);
                assert!(car.elevation.landing_speed > 20.0);
                assert_eq!(car.elevation.landings, 1);
                return;
            }
        }
        panic!("The cliff dive never landed");
    }

    #[test]
    fn free_fall_matches_an_independent_ballistic_equation_and_ignores_air_controls() {
        let mut coast = launch();
        let mut controls = coast;
        let initial = coast.elevation;
        let x = coast.car.x;
        let duration = 0.2;
        for _ in 0..40 {
            coast.step(0.0, 0.0, 0.0, 0.005);
            controls.step(1.0, -1.0, 1.0, 0.005);
        }
        assert!(!coast.elevation.grounded);
        assert!(
            (coast.elevation.height
                - (initial.height + initial.vertical * duration
                    - 0.5 * 9.81 * duration * duration))
                .abs()
                < 1e-9
        );
        assert!((coast.car.x - x - initial.vx * duration).abs() < 1e-9);
        assert_eq!(coast, controls);
    }

    #[test]
    fn slope_gravity_rolls_a_stopped_car_downhill() {
        let mut car = DrivingVehicle::default();
        car.reset(260.0, 150.0, 0.0);
        assert!(car.elevation.pitch > 0.4);
        for _ in 0..240 {
            car.step(0.0, 0.0, 0.0, 1.0 / 120.0);
        }
        assert!(car.car.x < 257.0 && car.car.forward < -2.0);
        assert!(car.elevation.grounded);
    }

    #[test]
    fn the_cliff_face_blocks_entry_from_below_instead_of_teleporting_up() {
        let mut car = DrivingVehicle::default();
        car.reset(382.0, 150.0, std::f64::consts::PI);
        for _ in 0..600 {
            car.step(1.0, 0.0, 0.0, 1.0 / 120.0);
        }
        assert!(car.car.x >= 380.0 && car.elevation.height == 0.0);
        assert!(car.car.contacts > 0);
    }

    #[test]
    fn reset_and_invalid_input_preserve_the_vertical_contract() {
        let mut car = launch();
        let before = car;
        assert!(!car.step(f64::NAN, 0.0, 0.0, 0.01));
        assert!(!car.reset(f64::INFINITY, 0.0, 0.0));
        assert_eq!(car, before);
        assert!(car.reset(190.0, 150.0, 0.0));
        assert!(car.elevation.grounded && car.elevation.height == 0.0);
        assert_eq!(car.elevation.landings, 0);
        assert_eq!(car.elevation.max_air_time, 0.0);
    }
    #[test]
    fn bridge_and_lower_road_keep_distinct_heights_at_the_same_crossing() {
        let mut above = DrivingVehicle::default();
        above.reset(190.0, 20.0, 0.0);
        for _ in 0..6000 {
            above.step(1.0, 0.0, 0.0, 1.0 / 120.0);
            if above.car.x >= 350.0 {
                break;
            }
        }
        assert!(above.car.x >= 350.0, "The bridge climb was blocked");
        assert!((above.elevation.height - 12.0).abs() < 0.1);
        let mut below = DrivingVehicle::default();
        below.reset(350.0, -55.0, std::f64::consts::FRAC_PI_2);
        for _ in 0..3000 {
            below.step(1.0, 0.0, 0.0, 1.0 / 120.0);
            assert!(below.elevation.height.abs() < 1e-10);
            if below.car.y > 35.0 {
                return;
            }
        }
        panic!("The lower road did not cross underneath the bridge");
    }

    #[test]
    fn the_excavated_road_goes_below_grade_and_exits_through_the_far_ramp() {
        let mut car = DrivingVehicle::default();
        car.reset(190.0, -105.0, 0.0);
        let mut underground = false;
        for _ in 0..6000 {
            car.step(1.0, 0.0, 0.0, 1.0 / 120.0);
            if (300.0..400.0).contains(&car.car.x) {
                assert!((car.elevation.height + 8.0).abs() < 1e-7);
                assert_eq!(car.read(21), -1.2);
                underground = true;
            }
            if car.car.x > 505.0 && car.elevation.grounded {
                assert!(underground && car.elevation.height == 0.0);
                return;
            }
        }
        panic!("The tunnel run never returned to city grade");
    }

    #[test]
    fn the_tunnel_roof_can_be_crossed_above_without_falling_to_the_lower_road() {
        let mut car = DrivingVehicle::default();
        car.reset(350.0, -145.0, std::f64::consts::FRAC_PI_2);
        let mut crossed = false;
        for _ in 0..2400 {
            car.step(1.0, 0.0, 0.0, 1.0 / 120.0);
            assert!(car.elevation.height.abs() < 1e-10);
            crossed |= (-114.0..-96.0).contains(&car.car.y);
            if car.car.y > -80.0 {
                assert!(crossed);
                return;
            }
        }
        panic!("The upper road did not cross the tunnel roof");
    }

    #[test]
    fn rising_into_a_bridge_hits_its_underside_without_teleporting_onto_it() {
        let mut car = DrivingVehicle::default();
        car.reset(350.0, 20.0, 0.0);
        car.elevation.height = 8.5;
        car.elevation.vertical = 12.0;
        car.elevation.grounded = false;
        for _ in 0..100 {
            car.step(1.0, 1.0, 1.0, 0.005);
            assert!(car.elevation.height + BODY_HEIGHT <= 10.9 + 1e-10);
        }
        assert!(car.car.contacts > 0 && car.elevation.vertical < 0.0);
    }

    #[test]
    fn a_falling_car_lands_on_the_bridge_upper_layer() {
        let mut car = DrivingVehicle::default();
        car.reset(350.0, 20.0, 0.0);
        car.elevation.height = 18.0;
        car.elevation.grounded = false;
        for _ in 0..600 {
            car.step(0.0, 0.0, 0.0, 0.005);
            if car.elevation.grounded {
                assert!((car.elevation.height - 12.0).abs() < 1e-10);
                assert_eq!(car.elevation.landings, 1);
                return;
            }
        }
        panic!("The car fell through the bridge");
    }

    #[test]
    fn tunnel_side_walls_do_not_snap_the_lower_car_up_to_grade() {
        let mut car = DrivingVehicle::default();
        car.reset(350.0, -105.0, std::f64::consts::FRAC_PI_2);
        for _ in 0..600 {
            car.step(1.0, 0.0, 0.0, 1.0 / 120.0);
        }
        assert!(car.car.y <= -95.0 && car.elevation.height == -8.0);
        assert!(car.car.contacts > 0);
    }
    #[test]
    fn barrier_fracture_spends_normal_energy_and_preserves_pose_and_tangent() {
        let mut car = DrivingVehicle::default();
        car.reset(360.0, 150.0, 0.0);
        car.car.forward = 30.0;
        car.car.lateral = 4.0;
        let before = car;
        assert!(car.break_barrier(-2.0, 0.0, 90_000.0));
        assert!((0.5 * 1_250.0 * (900.0 - car.car.forward.powi(2)) - 90_000.0).abs() < 1e-8);
        assert_eq!(car.car.lateral, 4.0);
        assert_eq!((car.car.x, car.car.y, car.elevation.height), (before.car.x, before.car.y, 48.0));
        assert!(car.elevation.grounded && car.car.contacts == 1);
    }

    #[test]
    fn a_slow_or_outgoing_car_cannot_fracture_the_guardrail() {
        let mut car = DrivingVehicle::default();
        car.car.forward = 10.0;
        let before = car;
        assert!(!car.break_barrier(-1.0, 0.0, 90_000.0));
        assert!(!car.break_barrier(1.0, 0.0, 1_000.0));
        for bad in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
            assert!(!car.break_barrier(-1.0, 0.0, bad));
        }
        assert!(!car.break_barrier(0.0, 0.0, 1_000.0));
        assert_eq!(car, before);
    }

    #[test]
    fn an_airborne_barrier_impact_keeps_ballistic_velocity_consistent() {
        let mut car = launch();
        let z = car.elevation.height;
        let vz = car.elevation.vertical;
        assert!(car.break_barrier(-1.0, 0.0, 90_000.0));
        let vx = car.elevation.vx;
        let x = car.car.x;
        car.step(0.0, 0.0, 0.0, 0.005);
        assert!((car.car.x - x - vx * 0.005).abs() < 1e-10);
        assert!((car.elevation.height - z - vz * 0.005 + 0.5 * 9.81 * 0.005_f64.powi(2)).abs() < 1e-10);
    }

}
