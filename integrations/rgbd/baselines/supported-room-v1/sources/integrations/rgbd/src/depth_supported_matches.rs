//! Restrict the unchanged mutual descriptor matcher to measured-depth support.
use rustdriving_perception::image_features::{
    FeatureMatch, ImageFeature, MAX_FEATURES, match_features,
};

/// Masks refer to the complete, unchanged extraction order. Selection preserves
/// that order, so mapping back also preserves the original capped sort order.
pub(super) fn match_depth_supported(
    previous: &[ImageFeature],
    current: &[ImageFeature],
    previous_eligible: &[bool],
    current_eligible: &[bool],
) -> Result<Vec<FeatureMatch>, String> {
    if previous.len() != previous_eligible.len() || current.len() != current_eligible.len() {
        return Err("depth eligibility mask length differs from feature count".into());
    }
    if previous.len() > MAX_FEATURES || current.len() > MAX_FEATURES {
        return Err("depth-supported matcher exceeds original feature bound".into());
    }
    if previous.iter().chain(current).any(|feature| {
        !feature.x.is_finite()
            || !feature.y.is_finite()
            || !feature.orientation.is_finite()
            || feature.x < 0.0
            || feature.y < 0.0
    }) {
        return Err("depth-supported matcher has invalid original feature".into());
    }
    let select = |features: &[ImageFeature], mask: &[bool]| {
        features
            .iter()
            .zip(mask)
            .enumerate()
            .filter_map(|(index, (feature, &eligible))| eligible.then_some((index, *feature)))
            .unzip::<_, _, Vec<_>, Vec<_>>()
    };
    let (previous_ids, previous_supported) = select(previous, previous_eligible);
    let (current_ids, current_supported) = select(current, current_eligible);
    let matched = match_features(&previous_supported, &current_supported)
        .map_err(|e| format!("depth-supported descriptor matching: {e:?}"))?;
    matched
        .into_iter()
        .map(|item| {
            Ok(FeatureMatch {
                previous_index: *previous_ids
                    .get(item.previous_index)
                    .ok_or("filtered previous feature index out of bounds")?,
                current_index: *current_ids
                    .get(item.current_index)
                    .ok_or("filtered current feature index out of bounds")?,
                hamming_distance: item.hamming_distance,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn feature(bits: u64) -> ImageFeature {
        ImageFeature {
            x: 20.,
            y: 20.,
            score: 40,
            orientation: 0.,
            descriptor: [bits; 4],
        }
    }
    #[test]
    fn invalid_nearest_cannot_compete_and_original_indices_survive() {
        let previous = [feature(0), feature(u64::MAX)];
        let current = [feature(0), feature(1), feature(u64::MAX)];
        let matches =
            match_depth_supported(&previous, &current, &[true; 2], &[false, true, true]).unwrap();
        assert!(matches.contains(&FeatureMatch {
            previous_index: 0,
            current_index: 1,
            hamming_distance: 4,
        }));
        assert!(matches.contains(&FeatureMatch {
            previous_index: 1,
            current_index: 2,
            hamming_distance: 0,
        }));
    }
    #[test]
    fn eligible_competitor_remains_in_ratio_and_nearest_selection() {
        let previous = [feature(0), feature(u64::MAX)];
        let current = [feature(0), feature(1), feature(u64::MAX)];
        let matches = match_depth_supported(&previous, &current, &[true; 2], &[true; 3]).unwrap();
        assert!(
            matches
                .iter()
                .any(|m| m.previous_index == 0 && m.current_index == 0)
        );
        assert!(
            !matches
                .iter()
                .any(|m| m.previous_index == 0 && m.current_index == 1)
        );
        // Best/second-best = 4/5 is exactly the original strict-ratio boundary.
        let current = [feature(0b1111), feature(0b11111), feature(u64::MAX)];
        let matches = match_depth_supported(&previous, &current, &[true; 2], &[true; 3]).unwrap();
        assert!(!matches.iter().any(|m| m.previous_index == 0));
    }
    #[test]
    fn supported_equal_descriptors_remain_ambiguous() {
        let previous = [feature(0), feature(u64::MAX)];
        let current = [feature(0), feature(0), feature(u64::MAX)];
        let matched = match_depth_supported(&previous, &current, &[true; 2], &[true; 3]).unwrap();
        assert!(!matched.iter().any(|m| m.previous_index == 0));
    }
    #[test]
    fn single_supported_candidate_in_either_direction_is_not_a_ratio() {
        let features = [feature(0), feature(u64::MAX)];
        for (previous_mask, current_mask) in
            [([true, false], [true, true]), ([true, true], [true, false])]
        {
            assert!(
                match_depth_supported(&features, &features, &previous_mask, &current_mask)
                    .unwrap()
                    .is_empty()
            );
        }
    }
    #[test]
    fn permutation_preserves_descriptor_associations_in_original_index_space() {
        let previous = [feature(0), feature(1), feature(u64::MAX)];
        let current = [feature(u64::MAX), feature(1), feature(0)];
        let matched = match_depth_supported(
            &previous,
            &current,
            &[true, false, true],
            &[true, false, true],
        )
        .unwrap();
        assert_eq!(
            matched,
            vec![
                FeatureMatch {
                    previous_index: 0,
                    current_index: 2,
                    hamming_distance: 0
                },
                FeatureMatch {
                    previous_index: 2,
                    current_index: 0,
                    hamming_distance: 0
                },
            ]
        );
    }
    #[test]
    fn bounds_and_invalid_eligible_features_reject() {
        let features = [feature(0), feature(u64::MAX)];
        assert!(match_depth_supported(&features, &features, &[true], &[true; 2]).is_err());
        assert!(match_depth_supported(&features, &features, &[true; 2], &[true]).is_err());
        let oversized = vec![feature(0); MAX_FEATURES + 1];
        assert!(
            match_depth_supported(
                &oversized,
                &features,
                &vec![false; oversized.len()],
                &[true; 2]
            )
            .is_err()
        );
        let mut invalid = features;
        invalid[0].orientation = f64::NAN;
        assert!(match_depth_supported(&invalid, &features, &[true; 2], &[true; 2]).is_err());
        assert!(match_depth_supported(&invalid, &features, &[false; 2], &[false; 2]).is_err());
        for value in [f64::NEG_INFINITY, f64::INFINITY, f64::NAN, -1.] {
            invalid[0] = features[0];
            invalid[0].x = value;
            assert!(match_depth_supported(&invalid, &features, &[false; 2], &[false; 2]).is_err());
            invalid[0] = features[0];
            invalid[0].y = value;
            assert!(match_depth_supported(&invalid, &features, &[false; 2], &[false; 2]).is_err());
        }
    }
    #[test]
    fn eligible_original_mutual_pairs_survive_with_two_candidates_and_no_binding_cap() {
        let previous = [feature(0), feature(u64::MAX), feature(0x5555555555555555)];
        let current = [feature(1), feature(u64::MAX), feature(0xaaaaaaaaaaaaaaaa)];
        let mask = [true, true, false];
        let original = match_features(&previous, &current).unwrap();
        let retained: Vec<_> = original
            .into_iter()
            .filter(|m| mask[m.previous_index] && mask[m.current_index])
            .collect();
        assert_eq!(retained.len(), 2);
        let supported = match_depth_supported(&previous, &current, &mask, &mask).unwrap();
        assert!(retained.iter().all(|m| supported.contains(m)));
    }
    #[test]
    fn full_domain_matches_original_matcher_and_cap_exactly() {
        let features: Vec<_> = (0..MAX_FEATURES)
            .map(|i| {
                let mut state = i as u64 + 1;
                let mut descriptor = [0; 4];
                for bits in &mut descriptor {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    *bits = state.wrapping_mul(0x9e3779b97f4a7c15);
                }
                ImageFeature {
                    descriptor,
                    ..feature(0)
                }
            })
            .collect();
        let supported = match_depth_supported(
            &features,
            &features,
            &vec![true; MAX_FEATURES],
            &vec![true; MAX_FEATURES],
        )
        .unwrap();
        let original = match_features(&features, &features).unwrap();
        assert_eq!(supported, original);
        assert_eq!(supported.len(), 256);
        assert!(
            supported
                .iter()
                .all(|m| m.previous_index == m.current_index)
        );
    }
}
