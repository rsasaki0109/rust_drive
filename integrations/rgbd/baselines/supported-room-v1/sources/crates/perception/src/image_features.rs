//! Bounded classical image features: FAST-9, intensity-centroid orientation,
//! and a deterministic rotated 256-bit BRIEF descriptor.
//!
//! This is an original fixed-pattern descriptor, not OpenCV ORB's learned
//! sampling pattern. Pixels use the camera convention: x right, y down;
//! orientation is radians in this image plane. No scale invariance is claimed.

/// Hard limit on image storage and processing work (640 × 480 pixels).
pub const MAX_IMAGE_PIXELS: usize = 307_200;
pub const MAX_FEATURES: usize = 400;
pub const MAX_MATCHES: usize = 256;
const BORDER: usize = 17;
const TILE_SIZE: usize = 32;
const FAST_THRESHOLD: i16 = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeatureError {
    InvalidImageDimensions,
    InvalidPixelCount,
    TooManyFeatures,
    InvalidFeature,
}

/// Borrowed, tightly packed 8-bit grayscale image, without row padding.
#[derive(Clone, Copy, Debug)]
pub struct GrayImage<'a> {
    width: usize,
    height: usize,
    pixels: &'a [u8],
}

impl<'a> GrayImage<'a> {
    pub fn new(width: usize, height: usize, pixels: &'a [u8]) -> Result<Self, FeatureError> {
        let count = width
            .checked_mul(height)
            .filter(|&count| count <= MAX_IMAGE_PIXELS)
            .ok_or(FeatureError::InvalidImageDimensions)?;
        if width <= 2 * BORDER || height <= 2 * BORDER {
            return Err(FeatureError::InvalidImageDimensions);
        }
        if pixels.len() != count {
            return Err(FeatureError::InvalidPixelCount);
        }
        Ok(Self {
            width,
            height,
            pixels,
        })
    }

    pub fn width(self) -> usize {
        self.width
    }

    pub fn height(self) -> usize {
        self.height
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImageFeature {
    pub x: f64,
    pub y: f64,
    /// Minimum contrast in the strongest contiguous FAST-9 arc.
    pub score: u16,
    pub orientation: f64,
    pub descriptor: [u64; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FeatureMatch {
    pub previous_index: usize,
    pub current_index: usize,
    pub hamming_distance: u32,
}

const CIRCLE: [(isize, isize); 16] = [
    (0, -3),
    (1, -3),
    (2, -2),
    (3, -1),
    (3, 0),
    (3, 1),
    (2, 2),
    (1, 3),
    (0, 3),
    (-1, 3),
    (-2, 2),
    (-3, 1),
    (-3, 0),
    (-3, -1),
    (-2, -2),
    (-1, -3),
];

fn fast_score(image: GrayImage<'_>, x: usize, y: usize) -> u16 {
    let center = i16::from(image.pixels[y * image.width + x]);
    let mut differences = [0_i16; 16];
    for (index, (dx, dy)) in CIRCLE.iter().copied().enumerate() {
        let column = x.checked_add_signed(dx).expect("FAST border validated");
        let row = y.checked_add_signed(dy).expect("FAST border validated");
        differences[index] = i16::from(image.pixels[row * image.width + column]) - center;
    }
    // Most non-corners have fewer than nine pixels of either contrast sign.
    if differences.iter().filter(|&&v| v > FAST_THRESHOLD).count() < 9
        && differences.iter().filter(|&&v| v < -FAST_THRESHOLD).count() < 9
    {
        return 0;
    }
    let mut score = 0_i16;
    for start in 0..16 {
        let mut bright = 255_i16;
        let mut dark = 255_i16;
        for offset in 0..9 {
            let contrast = differences[(start + offset) % 16];
            bright = bright.min(contrast);
            dark = dark.min(-contrast);
        }
        score = score.max(bright).max(dark);
    }
    if score > FAST_THRESHOLD {
        score as u16
    } else {
        0
    }
}

/// Separable [1, 4, 6, 4, 1] / 16 blur. Accumulate before rounding once;
/// descriptor samples never access the uncomputed two-pixel border.
fn blurred(image: GrayImage<'_>) -> Vec<u8> {
    let mut horizontal = vec![0_u16; image.pixels.len()];
    let mut output = vec![0_u8; image.pixels.len()];
    let weights = [1_u16, 4, 6, 4, 1];
    for y in 0..image.height {
        for x in 2..image.width - 2 {
            let offset = y * image.width + x - 2;
            horizontal[y * image.width + x] = weights
                .iter()
                .enumerate()
                .map(|(i, &weight)| weight * u16::from(image.pixels[offset + i]))
                .sum();
        }
    }
    for y in 2..image.height - 2 {
        for x in 2..image.width - 2 {
            let sum: u32 = weights
                .iter()
                .enumerate()
                .map(|(i, &weight)| {
                    u32::from(weight) * u32::from(horizontal[(y + i - 2) * image.width + x])
                })
                .sum();
            output[y * image.width + x] = ((sum + 128) / 256) as u8;
        }
    }
    output
}

fn orientation(image: GrayImage<'_>, x: usize, y: usize) -> f64 {
    let mut moment_x = 0_i64;
    let mut moment_y = 0_i64;
    for dy in -15_isize..=15 {
        for dx in -15_isize..=15 {
            if dx * dx + dy * dy > 225 {
                continue;
            }
            let column = x
                .checked_add_signed(dx)
                .expect("orientation border validated");
            let row = y
                .checked_add_signed(dy)
                .expect("orientation border validated");
            let value = i64::from(image.pixels[row * image.width + column]);
            moment_x += dx as i64 * value;
            moment_y += dy as i64 * value;
        }
    }
    (moment_y as f64).atan2(moment_x as f64)
}

/// Fixed xorshift32 seed and rejection sampling in the radius-13 integer disk.
/// No image-dependent randomness, learned coefficients, or external state.
fn sample_point(state: &mut u32) -> (i32, i32) {
    loop {
        *state ^= *state << 13;
        *state ^= *state >> 17;
        *state ^= *state << 5;
        let x = (*state % 27) as i32 - 13;
        *state ^= *state << 13;
        *state ^= *state >> 17;
        *state ^= *state << 5;
        let y = (*state % 27) as i32 - 13;
        if x * x + y * y <= 169 {
            return (x, y);
        }
    }
}

fn pattern() -> [((i32, i32), (i32, i32)); 256] {
    let mut state = 0x9e37_79b9_u32;
    let mut pairs = [((0, 0), (0, 0)); 256];
    for pair in &mut pairs {
        let first = sample_point(&mut state);
        let mut second = sample_point(&mut state);
        while first == second {
            second = sample_point(&mut state);
        }
        *pair = (first, second);
    }
    pairs
}

/// Detect FAST-9 corners, apply 3×3 suppression, and retain at most two
/// strongest corners per 32×32 tile. Equal scores prefer top then left.
/// The final feature limit is 400. Descriptors use nearest integer sampling
/// after rotation, with a binomial 5×5 blur to reduce pixel noise sensitivity.
pub fn extract_features(image: GrayImage<'_>) -> Vec<ImageFeature> {
    let mut scores = vec![0_u16; image.pixels.len()];
    for y in BORDER - 1..image.height - BORDER + 1 {
        for x in BORDER - 1..image.width - BORDER + 1 {
            scores[y * image.width + x] = fast_score(image, x, y);
        }
    }
    let mut candidates = Vec::new();
    for y in BORDER..image.height - BORDER {
        for x in BORDER..image.width - BORDER {
            let index = y * image.width + x;
            let score = scores[index];
            if score == 0 {
                continue;
            }
            let suppressed = (y - 1..=y + 1).any(|row| {
                (x - 1..=x + 1).any(|column| {
                    let neighbor = row * image.width + column;
                    neighbor != index
                        && (scores[neighbor] > score
                            || (scores[neighbor] == score && neighbor < index))
                })
            });
            if !suppressed {
                candidates.push((score, x, y));
            }
        }
    }
    candidates.sort_unstable_by(|a, b| b.0.cmp(&a.0).then(a.2.cmp(&b.2)).then(a.1.cmp(&b.1)));
    let tile_columns = image.width.div_ceil(TILE_SIZE);
    let mut tile_counts = vec![0_u8; tile_columns * image.height.div_ceil(TILE_SIZE)];
    let blur = blurred(image);
    let pairs = pattern();
    let mut features = Vec::with_capacity(MAX_FEATURES);
    for (score, x, y) in candidates {
        let tile = y / TILE_SIZE * tile_columns + x / TILE_SIZE;
        if tile_counts[tile] == 2 {
            continue;
        }
        tile_counts[tile] += 1;
        let angle = orientation(image, x, y);
        let (sin, cos) = angle.sin_cos();
        let sample = |(dx, dy): (i32, i32)| {
            let rotated_x = (f64::from(dx) * cos - f64::from(dy) * sin).round() as isize;
            let rotated_y = (f64::from(dx) * sin + f64::from(dy) * cos).round() as isize;
            let column = x
                .checked_add_signed(rotated_x)
                .expect("descriptor border validated");
            let row = y
                .checked_add_signed(rotated_y)
                .expect("descriptor border validated");
            blur[row * image.width + column]
        };
        let mut descriptor = [0_u64; 4];
        for (bit, &(first, second)) in pairs.iter().enumerate() {
            if sample(first) < sample(second) {
                descriptor[bit / 64] |= 1_u64 << (bit % 64);
            }
        }
        features.push(ImageFeature {
            x: x as f64,
            y: y as f64,
            score,
            orientation: angle,
            descriptor,
        });
        if features.len() == MAX_FEATURES {
            break;
        }
    }
    features
}

fn distance(left: &ImageFeature, right: &ImageFeature) -> u32 {
    left.descriptor
        .iter()
        .zip(right.descriptor)
        .map(|(&a, b)| (a ^ b).count_ones())
        .sum()
}

fn unique_best(distances: impl Iterator<Item = (usize, u32)>) -> Option<(usize, u32)> {
    let mut best = (usize::MAX, u32::MAX);
    let mut second = u32::MAX;
    let mut count = 0;
    for (index, distance) in distances {
        count += 1;
        if distance < best.1 {
            second = best.1;
            best = (index, distance);
        } else {
            second = second.min(distance);
        }
    }
    // At least two candidates are required for an actual ratio test. Strict
    // integer inequality also rejects exact ties, including two zero distances.
    (count >= 2 && best.1 <= 64 && 5 * best.1 < 4 * second).then_some(best)
}

/// Mutual, unique nearest descriptor matches with Hamming ≤64 and strict
/// best/second-best ratio <0.8 in BOTH directions. Ties and single-candidate
/// comparisons are rejected. Output sorts by distance then previous/current
/// indices, capped at 256; no geometric pose claim is made by this function.
pub fn match_features(
    previous: &[ImageFeature],
    current: &[ImageFeature],
) -> Result<Vec<FeatureMatch>, FeatureError> {
    if previous.len() > MAX_FEATURES || current.len() > MAX_FEATURES {
        return Err(FeatureError::TooManyFeatures);
    }
    if previous.iter().chain(current).any(|feature| {
        !feature.x.is_finite()
            || !feature.y.is_finite()
            || !feature.orientation.is_finite()
            || feature.x < 0.0
            || feature.y < 0.0
    }) {
        return Err(FeatureError::InvalidFeature);
    }
    let forward: Vec<_> = previous
        .iter()
        .map(|feature| {
            unique_best(
                current
                    .iter()
                    .enumerate()
                    .map(|(index, target)| (index, distance(feature, target))),
            )
        })
        .collect();
    let backward: Vec<_> = current
        .iter()
        .map(|feature| {
            unique_best(
                previous
                    .iter()
                    .enumerate()
                    .map(|(index, target)| (index, distance(feature, target))),
            )
        })
        .collect();
    let mut matches = Vec::new();
    for (previous_index, best) in forward.into_iter().enumerate() {
        if let Some((current_index, hamming_distance)) = best
            && backward[current_index].is_some_and(|(index, _)| index == previous_index)
        {
            matches.push(FeatureMatch {
                previous_index,
                current_index,
                hamming_distance,
            });
        }
    }
    matches.sort_unstable_by_key(|item| {
        (
            item.hamming_distance,
            item.previous_index,
            item.current_index,
        )
    });
    matches.truncate(MAX_MATCHES);
    Ok(matches)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn textured(width: usize, height: usize) -> Vec<u8> {
        // Independently constructed aperiodic checker and thin line texture.
        (0..width * height)
            .map(|i| {
                let (x, y) = (i % width, i / width);
                let checker = if (x / 7 + y / 11) % 2 == 0 { 40 } else { 170 };
                let noise = ((x * 97 + y * 193 + x * y * 17) % 61) as u8;
                checker + noise
            })
            .collect()
    }

    #[test]
    fn bounds_and_constant_image_do_not_create_features() {
        assert!(matches!(
            GrayImage::new(usize::MAX, 35, &[]),
            Err(FeatureError::InvalidImageDimensions)
        ));
        assert!(matches!(
            GrayImage::new(641, 480, &[]),
            Err(FeatureError::InvalidImageDimensions)
        ));
        assert!(matches!(
            GrayImage::new(34, 35, &[0; 1190]),
            Err(FeatureError::InvalidImageDimensions)
        ));
        assert!(matches!(
            GrayImage::new(35, 35, &[]),
            Err(FeatureError::InvalidPixelCount)
        ));
        let pixels = vec![127; 640 * 480];
        assert!(extract_features(GrayImage::new(640, 480, &pixels).unwrap()).is_empty());
    }

    #[test]
    fn hand_drawn_square_has_corners_and_blank_background_has_none() {
        let mut pixels = vec![20_u8; 96 * 96];
        for y in 30..65 {
            for x in 30..65 {
                pixels[y * 96 + x] = 230;
            }
        }
        let features = extract_features(GrayImage::new(96, 96, &pixels).unwrap());
        assert_eq!(features.len(), 4);
        for &(x, y) in &[(30., 30.), (64., 30.), (30., 64.), (64., 64.)] {
            assert!(
                features
                    .iter()
                    .any(|feature| (feature.x - x).abs() <= 2. && (feature.y - y).abs() <= 2.)
            );
        }
        assert!(features.iter().all(|feature| feature.score == 210));
    }

    #[test]
    fn translated_texture_matches_actual_pixels_not_only_descriptors() {
        let (width, height) = (320, 240);
        let pixels = textured(width, height);
        let mut translated = vec![0; width * height];
        for y in 3..height {
            for x in 5..width {
                translated[y * width + x] = pixels[(y - 3) * width + x - 5];
            }
        }
        let first = extract_features(GrayImage::new(width, height, &pixels).unwrap());
        let second = extract_features(GrayImage::new(width, height, &translated).unwrap());
        let matches = match_features(&first, &second).unwrap();
        assert!(matches.len() >= 50, "{} matches", matches.len());
        let correct = matches
            .iter()
            .filter(|item| {
                let a = first[item.previous_index];
                let b = second[item.current_index];
                b.x - a.x == 5. && b.y - a.y == 3.
            })
            .count();
        assert!(
            correct * 100 >= matches.len() * 95,
            "{correct}/{} correct",
            matches.len()
        );
        assert_eq!(
            first,
            extract_features(GrayImage::new(width, height, &pixels).unwrap())
        );
    }

    #[test]
    fn quarter_turn_texture_retains_geometrically_correct_matches() {
        let (width, height) = (320, 240);
        let pixels = textured(width, height);
        let mut rotated = vec![0; width * height];
        for y in 0..height {
            for x in 0..width {
                rotated[x * height + height - 1 - y] = pixels[y * width + x];
            }
        }
        let first = extract_features(GrayImage::new(width, height, &pixels).unwrap());
        let second = extract_features(GrayImage::new(height, width, &rotated).unwrap());
        let matches = match_features(&first, &second).unwrap();
        assert!(matches.len() >= 40, "{} matches", matches.len());
        let correct = matches
            .iter()
            .filter(|item| {
                let a = first[item.previous_index];
                let b = second[item.current_index];
                b.x == height as f64 - 1. - a.y && b.y == a.x
            })
            .count();
        assert!(
            correct * 100 >= matches.len() * 95,
            "{correct}/{} correct",
            matches.len()
        );
    }

    fn feature(descriptor: [u64; 4]) -> ImageFeature {
        ImageFeature {
            x: 40.,
            y: 40.,
            score: 100,
            orientation: 0.,
            descriptor,
        }
    }

    #[test]
    fn ties_single_candidates_and_nonfinite_features_fail_closed() {
        let identical = [feature([0; 4]); 2];
        assert!(match_features(&identical, &identical).unwrap().is_empty());
        assert!(
            match_features(&identical[..1], &identical[..1])
                .unwrap()
                .is_empty()
        );
        let far = [feature([u64::MAX; 4]), feature([u64::MAX - 1; 4])];
        assert!(match_features(&identical, &far).unwrap().is_empty());
        let mut invalid = feature([0; 4]);
        invalid.x = f64::NAN;
        assert_eq!(
            match_features(&[invalid], &identical),
            Err(FeatureError::InvalidFeature)
        );
        assert_eq!(
            match_features(&vec![feature([0; 4]); MAX_FEATURES + 1], &[]),
            Err(FeatureError::TooManyFeatures)
        );
    }

    #[test]
    fn strict_ratio_and_reverse_unique_match_are_required() {
        // Distances to zero: 4 and 5. The 0.8 boundary is deliberately rejected.
        let zero = feature([0; 4]);
        let boundary = [feature([15, 0, 0, 0]), feature([31, 0, 0, 0])];
        assert!(
            match_features(&[zero, feature([u64::MAX; 4])], &boundary)
                .unwrap()
                .is_empty()
        );
        let other = feature([u64::MAX; 4]);
        let clear = [feature([1, 0, 0, 0]), other];
        let matches = match_features(&[zero, other], &clear).unwrap();
        assert_eq!(matches.len(), 2);
        assert_eq!(matches[0].hamming_distance, 0);
        assert_eq!(matches[1].hamming_distance, 1);
    }

    #[test]
    fn dense_image_respects_feature_and_tile_limits() {
        let pixels = textured(640, 480);
        let features = extract_features(GrayImage::new(640, 480, &pixels).unwrap());
        assert_eq!(features.len(), MAX_FEATURES);
        let mut counts = std::collections::BTreeMap::new();
        for feature in features {
            let key = (
                feature.x as usize / TILE_SIZE,
                feature.y as usize / TILE_SIZE,
            );
            *counts.entry(key).or_insert(0) += 1;
        }
        assert!(counts.values().all(|&count| count <= 2));
    }

    #[test]
    fn match_count_limit_preserves_unique_correspondences() {
        let mut features = Vec::new();
        let mut state = 5_u64;
        for index in 0..MAX_FEATURES {
            let mut descriptor = [0; 4];
            for word in &mut descriptor {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1);
                *word = state;
            }
            let mut item = feature(descriptor);
            item.x = index as f64;
            features.push(item);
        }
        let matches = match_features(&features, &features).unwrap();
        assert_eq!(matches.len(), MAX_MATCHES);
        assert!(matches.iter().all(|item| item.previous_index == item.current_index && item.hamming_distance == 0));
    }
}
