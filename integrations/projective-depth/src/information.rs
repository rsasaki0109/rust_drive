//! Directional information at fixed metre/radian scales, before equilibration.
use crate::{Equation, H, Pose, eigenvalues, photometric};
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct Spectrum {
    pub translation_scale_m: f64,
    pub rotation_scale_rad: f64,
    pub matrix_scale: f64,
    pub eigenvalues: [f64; 6],
    pub condition: Option<f64>,
}
#[derive(Debug, Serialize)]
pub struct InformationFailure {
    pub final_pose: bool,
    pub scale: usize,
    pub pose: Pose,
    pub equation: Equation,
    pub photometric: Option<photometric::PhotoEquation>,
    pub spectrum: Option<Spectrum>,
}
fn spectrum(h: H) -> Option<Spectrum> {
    if !h.iter().flatten().all(|v| v.is_finite()) {
        return None;
    }
    let scale = (0..6).map(|i| h[i][i]).fold(0., f64::max);
    if scale <= 0. {
        return None;
    }
    let mut values = eigenvalues(h.map(|row| row.map(|v| v / scale)));
    values.sort_by(f64::total_cmp);
    let condition = (values[0] > 0.)
        .then(|| values[5] / values[0])
        .filter(|c| c.is_finite());
    Some(Spectrum {
        translation_scale_m: 1.,
        rotation_scale_rad: 1.,
        matrix_scale: scale,
        eigenvalues: values,
        condition,
    })
}
pub(super) fn check(
    final_pose: bool,
    scale: usize,
    pose: Pose,
    equation: &Equation,
    photo: Option<&photometric::PhotoEquation>,
    failure: &mut Option<InformationFailure>,
) -> Result<(), String> {
    let (h, _) = photometric::combine(equation, photo);
    let spectrum = spectrum(h);
    if spectrum
        .as_ref()
        .and_then(|s| s.condition)
        .is_some_and(|c| c <= 1e6)
    {
        return Ok(());
    }
    *failure = Some(InformationFailure {
        final_pose,
        scale,
        pose,
        equation: equation.clone(),
        photometric: photo.cloned(),
        spectrum,
    });
    Err("insufficient physical observability condition".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn diagonal_equilibration_cannot_invent_a_weak_pose_direction() {
        let weak: H = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                if i != j {
                    0.
                } else if i == 0 {
                    1e-12
                } else {
                    1.
                }
            })
        });
        assert!(crate::solve_normal(weak, [0.; 6], 1e6).is_ok());
        assert!(spectrum(weak).unwrap().condition.unwrap() > 1e6);
        let mut failure = None;
        let equation = Equation {
            h: weak,
            g: [0.; 6],
            count: 200,
            rms_m: 0.,
            direction_counts: None,
        };
        assert!(check(false, 4, Pose::identity(), &equation, None, &mut failure).is_err());
        assert_eq!(failure.unwrap().equation.count, 200);
    }
    #[test]
    fn spectrum_preserves_physical_axes_under_rotation_and_global_residual_scale() {
        let h: H = std::array::from_fn(|i| {
            std::array::from_fn(|j| if i == j { (i + 1) as f64 } else { 0. })
        });
        let r = Pose::identity()
            .increment([0., 0., 0., 0.2, -0.3, 0.1])
            .rotation;
        let q: H = std::array::from_fn(|i| {
            std::array::from_fn(|j| if i / 3 == j / 3 { r[i % 3][j % 3] } else { 0. })
        });
        let rotated: H = std::array::from_fn(|i| {
            std::array::from_fn(|j| (0..6).map(|k| q[i][k] * h[k][k] * q[j][k]).sum())
        });
        for factor in [1e-20, 1., 1e20] {
            let s = spectrum(rotated.map(|row| row.map(|v| v * factor))).unwrap();
            assert!((s.condition.unwrap() - 6.).abs() < 1e-12);
        }
    }
}
