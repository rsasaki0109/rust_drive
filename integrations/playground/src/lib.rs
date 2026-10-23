//! Manual-driving arcade model, separate from the autonomous research pipeline.
//! SI units; X/Y ground plane, positive yaw and steering turn left.
//! The two-axle tire model integrates lateral speed and yaw independently.
//! A handbrake reduces rear grip. Contact uses an explicit arcade impulse.
use std::cell::RefCell;
mod elevation;
mod structures;
use elevation::{DrivingVehicle, surface};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vehicle {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub forward: f64,
    pub lateral: f64,
    pub yaw_rate: f64,
    pub steering: f64,
    pub rear_slip: f64,
    pub front_slip: f64,
    pub distance: f64,
    pub contacts: u32,
}

impl Vehicle {
    pub fn reset(&mut self, x: f64, y: f64, yaw: f64) -> bool {
        if ![x, y, yaw].into_iter().all(f64::is_finite) || x.abs() > 10_000. || y.abs() > 10_000. {
            return false;
        }
        *self = Self {
            x,
            y,
            yaw: wrap(yaw),
            ..Self::default()
        };
        true
    }

    pub fn step(&mut self, throttle: f64, steering: f64, handbrake: f64, dt: f64) -> bool {
        if ![throttle, steering, handbrake, dt]
            .into_iter()
            .all(f64::is_finite)
            || !(0.000_1..=0.05).contains(&dt)
        {
            return false;
        }
        let ticks = (dt / 0.005).ceil() as usize;
        for _ in 0..ticks {
            self.integrate(
                throttle.clamp(-1., 1.),
                steering.clamp(-1., 1.),
                handbrake.clamp(0., 1.),
                dt / ticks as f64,
            );
        }
        true
    }

    fn integrate(&mut self, throttle: f64, steering: f64, handbrake: f64, dt: f64) {
        let target = steering * 0.56 / (1. + self.forward.abs() * 0.023);
        self.steering += (target - self.steering).clamp(-2.2 * dt, 2.2 * dt);
        let mass = 1_250.;
        let front_axle = 1.18;
        let rear_axle = 1.42;
        let length = front_axle + rear_axle;
        let front_load = mass * 9.81 * rear_axle / length;
        let rear_load = mass * 9.81 * front_axle / length;
        let braking = throttle < 0. && self.forward > 0.8 || throttle > 0. && self.forward < -0.8;
        let drive = if braking {
            -throttle.abs() * 10_000. * self.forward.signum()
        } else {
            throttle * if throttle < 0. { 4_000. } else { 6_800. }
        };
        let rear_grip = 1.08 * rear_load * (1. - 0.87 * handbrake);
        let engine_force = drive.clamp(-rear_load * 1.15, rear_load * 1.15);
        let drag = 0.40 * self.forward * self.forward.abs()
            + 32. * self.forward
            + if self.forward.abs() > 0.2 {
                120. * self.forward.signum()
            } else {
                0.
            };
        let hand_force = handbrake * 1_300. * self.forward.signum();
        let previous = self.forward;
        self.forward +=
            ((engine_force - drag - hand_force) / mass + self.lateral * self.yaw_rate) * dt;
        if throttle == 0. && previous * self.forward < 0. {
            self.forward = 0.;
        }
        self.forward = self.forward.clamp(-10., 40.);
        if self.forward.abs() < 2.2 {
            self.yaw_rate = self.forward / length * self.steering.tan();
            self.lateral = self.yaw_rate * rear_axle;
            self.rear_slip = 0.;
            self.front_slip = 0.;
        } else {
            let vx = self.forward;
            self.front_slip =
                ((self.lateral + front_axle * self.yaw_rate) / vx).atan() - self.steering;
            self.rear_slip = ((self.lateral - rear_axle * self.yaw_rate) / vx).atan();
            let front_limit = front_load * 1.08;
            let front_force = front_limit * (-65_000. * self.front_slip / front_limit).tanh();
            let rear_force = rear_grip * (-72_000. * self.rear_slip / rear_grip.max(1.)).tanh();
            let lateral_acceleration =
                (front_force * self.steering.cos() + rear_force) / mass - vx * self.yaw_rate;
            let yaw_acceleration =
                (front_axle * front_force * self.steering.cos() - rear_axle * rear_force) / 1_750.;
            self.lateral = (self.lateral + lateral_acceleration * dt).clamp(-28., 28.);
            self.yaw_rate =
                (self.yaw_rate + (yaw_acceleration - 0.55 * self.yaw_rate) * dt).clamp(-3.8, 3.8);
        }
        let heading = self.yaw + self.yaw_rate * dt * 0.5;
        let (s, c) = heading.sin_cos();
        let dx = (c * self.forward - s * self.lateral) * dt;
        let dy = (s * self.forward + c * self.lateral) * dt;
        self.x += dx;
        self.y += dy;
        self.distance += dx.hypot(dy);
        self.yaw = wrap(self.yaw + self.yaw_rate * dt);
    }

    /// Resolve a circular arcade contact. The caller supplies geometry only.
    pub fn contact(&mut self, nx: f64, ny: f64, penetration: f64) -> bool {
        if ![nx, ny, penetration].into_iter().all(f64::is_finite)
            || !(0.0..=6.).contains(&penetration)
            || (nx.hypot(ny) - 1.).abs() > 0.01
        {
            return false;
        }
        self.x += nx * penetration;
        self.y += ny * penetration;
        let (s, c) = self.yaw.sin_cos();
        let mut vx = c * self.forward - s * self.lateral;
        let mut vy = s * self.forward + c * self.lateral;
        let entering = vx * nx + vy * ny;
        if entering < 0. {
            vx -= 1.2 * entering * nx;
            vy -= 1.2 * entering * ny;
            self.forward = (c * vx + s * vy) * 0.78;
            self.lateral = (-s * vx + c * vy) * 0.78;
            self.yaw_rate *= 0.5;
            self.contacts = self.contacts.saturating_add(1);
        }
        true
    }

    pub fn read(&self, index: u32) -> f64 {
        match index {
            0 => self.x,
            1 => self.y,
            2 => self.yaw,
            3 => self.forward,
            4 => self.lateral,
            5 => self.yaw_rate,
            6 => self.steering,
            7 => self.rear_slip,
            8 => self.front_slip,
            9 => self.distance,
            10 => f64::from(self.contacts),
            _ => 0.,
        }
    }
}

fn wrap(yaw: f64) -> f64 {
    (yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

thread_local! { static CAR: RefCell<DrivingVehicle> = RefCell::new(DrivingVehicle::default()); }

// Numeric-only WebAssembly ABI; no raw pointers or shared mutable globals.
#[unsafe(no_mangle)]
pub extern "C" fn drive_reset(x: f64, y: f64, yaw: f64) -> bool {
    CAR.with(|car| car.borrow_mut().reset(x, y, yaw))
}
#[unsafe(no_mangle)]
pub extern "C" fn drive_step(throttle: f64, steering: f64, handbrake: f64, dt: f64) -> bool {
    CAR.with(|car| car.borrow_mut().step(throttle, steering, handbrake, dt))
}
#[unsafe(no_mangle)]
pub extern "C" fn drive_contact(nx: f64, ny: f64, penetration: f64) -> bool {
    CAR.with(|car| car.borrow_mut().contact(nx, ny, penetration))
}
#[unsafe(no_mangle)]
pub extern "C" fn drive_read(index: u32) -> f64 {
    CAR.with(|car| car.borrow().read(index))
}

/// Authored arcade terrain, shared by physics and the renderer; not operational sensing.
#[unsafe(no_mangle)]
pub extern "C" fn drive_surface(x: f64, y: f64, index: u32) -> f64 {
    let ground = surface(x, y);
    match index {
        0 => ground.height,
        1 => ground.gx,
        2 => ground.gy,
        _ => 0.0,
    }
}

/// Height of an authored upper deck, or a finite sentinel outside its footprint.
#[unsafe(no_mangle)]
pub extern "C" fn drive_deck(x: f64, y: f64, index: u32) -> f64 {
    structures::upper(x, y).map_or(-1_000_000.0, |s| match index {
        0 => s.height,
        1 => s.gx,
        2 => s.gy,
        _ => 0.0,
    })
}

/// Nearest overhead slab from this height; OPEN_SKY means no cover.
#[unsafe(no_mangle)]
pub extern "C" fn drive_ceiling(x: f64, y: f64, base_height: f64) -> f64 {
    if ![x, y, base_height].into_iter().all(f64::is_finite) {
        return structures::OPEN_SKY;
    }
    structures::ceiling(x, y, base_height)
}

/// Reachable road height for ground-attached effects and camera clearance.
#[unsafe(no_mangle)]
pub extern "C" fn drive_support(x: f64, y: f64, base_height: f64) -> f64 {
    let floor = surface(x, y).height;
    structures::upper(x, y)
        .filter(|s| s.height <= base_height + 0.05 && s.height >= floor)
        .map_or(floor, |s| s.height)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn acceleration_braking_reverse_and_straight_motion() {
        let mut car = Vehicle::default();
        for _ in 0..600 {
            assert!(car.step(1., 0., 0., 1. / 120.));
        }
        assert!(car.forward > 15. && car.x > 35. && car.y.abs() < 1e-10);
        for _ in 0..1_200 {
            car.step(-1., 0., 0., 1. / 120.);
        }
        assert!(car.forward < -3. && car.forward >= -10.);
    }
    #[test]
    fn handbrake_creates_real_rear_slip_and_lateral_motion() {
        let mut grip = Vehicle::default();
        for _ in 0..400 {
            grip.step(1., 0., 0., 1. / 120.);
        }
        let mut drift = grip;
        let mut maximum_slip = 0_f64;
        let mut maximum_lateral = 0_f64;
        for _ in 0..150 {
            grip.step(0.35, 0.6, 0., 1. / 120.);
            drift.step(0.35, 0.6, 1., 1. / 120.);
            maximum_slip = maximum_slip.max(drift.rear_slip.abs());
            maximum_lateral = maximum_lateral.max(drift.lateral.abs());
        }
        assert!(maximum_slip > 15_f64.to_radians());
        assert!((drift.x - grip.x).hypot(drift.y - grip.y) > 1.);
        assert!(maximum_lateral > 1.);
    }
    #[test]
    fn invalid_inputs_do_not_poison_or_advance_the_car() {
        let mut car = Vehicle::default();
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!car.step(value, 0., 0., 0.01));
            assert!(!car.reset(value, 0., 0.));
            assert!(!car.contact(1., 0., value));
        }
        assert!(!car.step(1., 1., 1., 1.));
        assert!(!car.contact(0., 0., 1.));
        assert_eq!(car, Vehicle::default());
    }
    #[test]
    fn impact_reduces_speed_and_recovers_finite_state() {
        let mut car = Vehicle {
            forward: 20.,
            ..Vehicle::default()
        };
        assert!(car.contact(-1., 0., 0.5));
        assert!(car.forward.abs() < 20. && car.x == -0.5 && car.contacts == 1);
        for _ in 0..8_000 {
            car.step(1., 0.8, 1., 1. / 120.);
        }
        assert!((0..11).all(|i| car.read(i).is_finite()));
        car.reset(2., 3., 7.);
        assert!(
            car.forward == 0.
                && car.x == 2.
                && car.y == 3.
                && car.yaw.abs() <= std::f64::consts::PI
        );
    }
}
