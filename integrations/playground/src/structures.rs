//! Authored stacked road surfaces. Heights are metres above city grade.
use crate::elevation::Surface;

pub(crate) const BODY_HEIGHT: f64 = 1.55;
pub(crate) const OPEN_SKY: f64 = 1_000_000.0;

fn ramp(t: f64, height: f64, length: f64) -> Surface {
    Surface {
        height: height * t * t * (3.0 - 2.0 * t),
        gx: height * 6.0 * t * (1.0 - t) / length,
        gy: 0.0,
    }
}

/// The excavation includes both open approach ramps and the covered lower road.
pub(crate) fn excavation(x: f64, y: f64) -> Option<Surface> {
    if !(190.0..=500.0).contains(&x) || !(-115.0..=-95.0).contains(&y) {
        return None;
    }
    Some(if x < 260.0 {
        ramp((x - 190.0) / 70.0, -8.0, 70.0)
    } else if x <= 430.0 {
        Surface {
            height: -8.0,
            ..Surface::default()
        }
    } else {
        let mut s = ramp((500.0 - x) / 70.0, -8.0, 70.0);
        s.gx = -s.gx;
        s
    })
}

/// Upper layers are separate from the excavation/ground; XY alone is ambiguous.
pub(crate) fn upper(x: f64, y: f64) -> Option<Surface> {
    // Pedestrian bridge over the hillside avenue; cars remain on the road below.
    if (353.5..=356.5).contains(&x) && (137.0..=163.0).contains(&y) {
        let floor = crate::elevation::surface(x, y);
        return Some(Surface { height: floor.height + 5.8, ..floor });
    }
    // Urban railway and the wider station platform have a physical underside.
    if ((170.0..=540.0).contains(&x) && (100.0..=108.0).contains(&y))
        || ((460.0..=505.0).contains(&x) && (98.0..=112.0).contains(&y)) {
        return Some(Surface { height: 10.0, ..Surface::default() });
    }

    if (260.0..=430.0).contains(&x) && (-115.0..=-95.0).contains(&y) {
        return Some(Surface::default());
    }
    if !(190.0..=500.0).contains(&x) || !(11.0..=29.0).contains(&y) {
        return None;
    }
    Some(if x < 260.0 {
        ramp((x - 190.0) / 70.0, 12.0, 70.0)
    } else if x <= 430.0 {
        Surface {
            height: 12.0,
            ..Surface::default()
        }
    } else {
        let mut s = ramp((500.0 - x) / 70.0, 12.0, 70.0);
        s.gx = -s.gx;
        s
    })
}

fn thickness(y: f64) -> f64 {
    if y < -70.0 { 1.2 } else { 1.1 }
}

pub(crate) fn ceiling(x: f64, y: f64, base_height: f64) -> f64 {
    upper(x, y)
        .filter(|s| base_height < s.height - 0.01)
        .map_or(OPEN_SKY, |s| s.height - thickness(y))
}

/// A slab side is solid; being underneath an upper surface never snaps a car up.
pub(crate) fn blocks_body(x: f64, y: f64, base_height: f64) -> bool {
    upper(x, y).is_some_and(|s| {
        base_height < s.height - 0.01
            && base_height + BODY_HEIGHT > s.height - thickness(y) + 0.000_001
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramps_have_independent_smoothstep_heights_and_derivatives() {
        for i in 1..70 {
            let x = 190.0 + f64::from(i);
            let t = f64::from(i) / 70.0;
            let bridge = upper(x, 20.0).unwrap();
            let tunnel = excavation(x, -105.0).unwrap();
            assert!((bridge.height - 12.0 * (3.0 * t * t - 2.0 * t * t * t)).abs() < 1e-10);
            assert!((tunnel.height + 8.0 * (3.0 * t * t - 2.0 * t * t * t)).abs() < 1e-10);
            let derivative = (upper(x + 0.001, 20.0).unwrap().height
                - upper(x - 0.001, 20.0).unwrap().height)
                / 0.002;
            assert!((bridge.gx - derivative).abs() < 1e-7);
        }
        assert!(upper(f64::NAN, 20.0).is_none());
        assert!(excavation(350.0, f64::INFINITY).is_none());
    }

    #[test]
    fn lower_roads_have_real_clearance_and_deck_sides_are_solid() {
        assert_eq!(ceiling(350.0, 20.0, 0.0), 10.9);
        assert_eq!(ceiling(350.0, -105.0, -8.0), -1.2);
        assert_eq!(ceiling(350.0, -105.0, 0.0), OPEN_SKY);
        assert!(!blocks_body(350.0, 20.0, 0.0));
        assert!(blocks_body(350.0, 20.0, 11.0));
        assert!(blocks_body(350.0, -105.0, -1.0));
    }
    #[test]
    fn urban_railway_and_pedestrian_bridge_keep_the_road_below_clear() {
        assert!((upper(350.0, 104.0).unwrap().height - 10.0).abs() < 1e-10);
        assert!((ceiling(350.0, 104.0, 0.0) - 8.9).abs() < 1e-10);
        assert!((upper(355.0, 150.0).unwrap().height - 53.8).abs() < 1e-10);
        assert!((ceiling(355.0, 150.0, 48.0) - 52.7).abs() < 1e-10);
        assert!(!blocks_body(355.0, 150.0, 48.0));
        assert!((upper(482.0, 111.0).unwrap().height - 10.0).abs() < 1e-10);
    }

}
