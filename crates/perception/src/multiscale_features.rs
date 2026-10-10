//! Additive three-level FAST/rotated-BRIEF prototype. This module is exercised
//! by the standalone RGB-D pair binary; it is not exported by the library yet.
//! Descriptor extraction and mutual-ratio matching use the original frontend.
use crate::image_features::{
    FeatureError, FeatureMatch, GrayImage, ImageFeature, MAX_FEATURES, extract_features,
    match_features,
};

/// Fixed BEFORE viewed-data regression: preserve 200 native, 120 half and 80
/// quarter features. Unused allocations are not redistributed or data-tuned.
pub const LEVEL_BUDGETS: [usize; 3] = [200, 120, 80];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultiscaleFeature {
    /// Original-camera coordinates, including the averaging-cell centre offset.
    pub feature: ImageFeature,
    pub level: usize,
    pub level_x: f64,
    pub level_y: f64,
}

/// Floor-sized 2x2 box average, rounded to nearest integer (ties upward).
/// Incomplete trailing cells are excluded. Allocates at most 76,800 bytes.
pub fn downsample_half(
    width: usize,
    height: usize,
    pixels: &[u8],
) -> Result<(usize, usize, Vec<u8>), FeatureError> {
    GrayImage::new(width, height, pixels)?;
    let (w, h) = (width / 2, height / 2);
    let mut output = Vec::with_capacity(w * h);
    for y in 0..h {
        for x in 0..w {
            let i = 2 * y * width + 2 * x;
            let sum = pixels[i] as u16
                + pixels[i + 1] as u16
                + pixels[i + width] as u16
                + pixels[i + width + 1] as u16;
            output.push(((sum + 2) / 4) as u8);
        }
    }
    Ok((w, h, output))
}

/// Native + half + quarter, with a GLOBAL 400-feature bound. Levels too small
/// for the original 17-pixel descriptor border are skipped, without padding.
/// No spatial/descriptor deduplication: ambiguous cross-level descriptors are
/// deliberately left to the original strict bidirectional ratio gate.
pub fn extract_multiscale(
    width: usize,
    height: usize,
    pixels: &[u8],
) -> Result<Vec<MultiscaleFeature>, FeatureError> {
    GrayImage::new(width, height, pixels)?;
    let mut features = Vec::with_capacity(MAX_FEATURES);
    let mut owned = Vec::new();
    let (mut w, mut h) = (width, height);
    for (level, budget) in LEVEL_BUDGETS.into_iter().enumerate() {
        let image_pixels = if level == 0 { pixels } else { &owned };
        if w <= 34 || h <= 34 {
            break;
        }
        let scale = (1_usize << level) as f64;
        let offset = (scale - 1.) / 2.;
        for mut feature in extract_features(GrayImage::new(w, h, image_pixels)?)
            .into_iter()
            .take(budget)
        {
            let (level_x, level_y) = (feature.x, feature.y);
            feature.x = scale * level_x + offset;
            feature.y = scale * level_y + offset;
            features.push(MultiscaleFeature {
                feature,
                level,
                level_x,
                level_y,
            });
        }
        if level < 2 {
            let next = downsample_half(w, h, image_pixels)?;
            (w, h, owned) = next;
        }
    }
    Ok(features)
}

/// Original mutual best/second-best ratio <0.8 in BOTH directions, maximum
/// Hamming 64, one-to-one, globally <=256. No extra geometric/scale permission.
pub fn match_multiscale(
    previous: &[MultiscaleFeature],
    current: &[MultiscaleFeature],
) -> Result<Vec<FeatureMatch>, FeatureError> {
    if previous.len() > MAX_FEATURES || current.len() > MAX_FEATURES {
        return Err(FeatureError::TooManyFeatures);
    }
    let a: Vec<_> = previous.iter().map(|f| f.feature).collect();
    let b: Vec<_> = current.iter().map(|f| f.feature).collect();
    match_features(&a, &b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn texture(w: usize, h: usize) -> Vec<u8> {
        (0..w * h)
            .map(|i| {
                let (x, y) = (i % w, i / w);
                let check = if (x / 7 + y / 11) % 2 == 0 { 40 } else { 170 };
                check + ((x * 97 + y * 193 + x * y * 17) % 61) as u8
            })
            .collect()
    }

    #[test]
    fn exact_averaging_odd_edges_and_camera_centres() {
        let mut pixels = vec![0; 37 * 35];
        pixels[0] = 1;
        pixels[1] = 2;
        pixels[37] = 3;
        pixels[38] = 4;
        pixels[36] = 255; // discarded final column
        let (w, h, p) = downsample_half(37, 35, &pixels).unwrap();
        assert_eq!((w, h, p.len(), p[0], p[17]), (18, 17, 306, 3, 0));
        let f = extract_multiscale(640, 480, &texture(640, 480)).unwrap();
        for item in &f {
            let scale = (1_usize << item.level) as f64;
            assert_eq!(item.feature.x, item.level_x * scale + (scale - 1.) / 2.);
            assert_eq!(item.feature.y, item.level_y * scale + (scale - 1.) / 2.);
        }
        assert!(f.iter().any(|x| x.level == 2 && x.feature.x.fract() == 0.5));
    }

    #[test]
    fn limits_determinism_blank_and_invalid_images() {
        let p = texture(640, 480);
        let a = extract_multiscale(640, 480, &p).unwrap();
        assert_eq!(a, extract_multiscale(640, 480, &p).unwrap());
        assert!(a.len() <= 400);
        for (level, cap) in LEVEL_BUDGETS.into_iter().enumerate() {
            assert!(a.iter().filter(|f| f.level == level).count() <= cap);
        }
        let m = match_multiscale(&a, &a).unwrap();
        assert!(m.len() <= 256);
        assert!(m.iter().all(|m| m.previous_index == m.current_index));
        assert!(
            extract_multiscale(640, 480, &vec![127; 640 * 480])
                .unwrap()
                .is_empty()
        );
        assert!(extract_multiscale(usize::MAX, 480, &[]).is_err());
        assert!(extract_multiscale(641, 480, &[]).is_err());
        assert!(extract_multiscale(100, 100, &[]).is_err());
        assert_eq!(
            match_multiscale(&vec![a[0]; 401], &a),
            Err(FeatureError::TooManyFeatures)
        );
    }

    #[test]
    fn exact_twofold_scale_has_cross_level_geometric_matches() {
        let (w, h) = (160, 120);
        let p = texture(w, h);
        let expanded: Vec<_> = (0..4 * w * h)
            .map(|i| p[(i / (2 * w) / 2) * w + (i % (2 * w)) / 2])
            .collect();
        let a = extract_multiscale(w, h, &p).unwrap();
        let b = extract_multiscale(2 * w, 2 * h, &expanded).unwrap();
        let matches = match_multiscale(&a, &b).unwrap();
        let correct = matches
            .iter()
            .filter(|m| {
                let (a, b) = (a[m.previous_index], b[m.current_index]);
                b.level == a.level + 1
                    && b.feature.x == 2. * a.feature.x + 0.5
                    && b.feature.y == 2. * a.feature.y + 0.5
            })
            .count();
        assert!(correct >= 20, "only {correct} exact cross-level matches");
        assert!(
            matches
                .iter()
                .filter(|m| a[m.previous_index].level == 0 && b[m.current_index].level == 1)
                .count()
                >= 20
        );
        assert!(
            correct * 100 >= matches.len() * 90,
            "{correct}/{} correct",
            matches.len()
        );
    }

    #[test]
    fn rotation_and_translation_have_correct_pixel_geometry() {
        let (w, h) = (320, 240);
        let p = texture(w, h);
        let mut rotated = vec![0; w * h];
        let mut shifted = vec![0; w * h];
        for y in 0..h {
            for x in 0..w {
                rotated[x * h + h - 1 - y] = p[y * w + x];
                if x >= 8 && y >= 4 {
                    shifted[y * w + x] = p[(y - 4) * w + x - 8];
                }
            }
        }
        let a = extract_multiscale(w, h, &p).unwrap();
        for (b, rotation) in [
            (extract_multiscale(h, w, &rotated).unwrap(), true),
            (extract_multiscale(w, h, &shifted).unwrap(), false),
        ] {
            let matches = match_multiscale(&a, &b).unwrap();
            let correct = matches
                .iter()
                .filter(|m| {
                    let (a, b) = (a[m.previous_index].feature, b[m.current_index].feature);
                    if rotation {
                        b.x == h as f64 - 1. - a.y && b.y == a.x
                    } else {
                        b.x - a.x == 8. && b.y - a.y == 4.
                    }
                })
                .count();
            assert!(matches.len() >= 30, "only {} matches", matches.len());
            assert!(
                correct * 100 >= matches.len() * 90,
                "{correct}/{} correct",
                matches.len()
            );
        }
    }

    #[test]
    fn actual_duplicate_descriptors_remain_ambiguous() {
        let a = extract_multiscale(320, 240, &texture(320, 240)).unwrap();
        let duplicated = [a[0], a[0]];
        assert!(
            match_multiscale(&duplicated, &duplicated)
                .unwrap()
                .is_empty()
        );
        assert!(match_multiscale(&a[..1], &a[..1]).unwrap().is_empty());
    }
}
