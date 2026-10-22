//! Compare the production detector with a frozen predecessor on fixed inputs.
use rustdriving_core::{Detection, LidarScan, Perception, Pose, Vec2};
use rustdriving_perception::LidarClusters;
use std::hint::black_box;
use std::time::Instant;

mod legacy {
    use rustdriving_core::{Detection, LidarScan, Pose, Vec2};
    include!("../tests/fixtures/legacy_cluster_detector.rs");

    pub(super) fn detect(scan: &LidarScan, pose: Pose, sparse: bool) -> Vec<Detection> {
        detect_clusters(scan, pose, sparse)
    }
}

fn signature(values: &[Detection]) -> Vec<[u64; 3]> {
    values
        .iter()
        .map(|d| {
            [
                d.center.x.to_bits(),
                d.center.y.to_bits(),
                d.radius.to_bits(),
            ]
        })
        .collect()
}

fn checksum(values: impl Iterator<Item = u64>) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    for value in values {
        for byte in value.to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}

fn sample(run: impl FnOnce() -> Vec<Detection>, expected: &[[u64; 3]]) -> u128 {
    let started = Instant::now();
    let actual = black_box(run());
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(signature(&actual), expected);
    elapsed
}

fn circle(count: usize) -> Vec<Vec2> {
    (0..count)
        .map(|i| {
            let a = i as f64 * std::f64::consts::TAU / count as f64;
            Vec2::new(30. * a.cos(), 30. * a.sin())
        })
        .collect()
}

fn arcs(count: usize, sparse: bool) -> Vec<Vec2> {
    let members = if sparse { 4 } else { 8 };
    (0..count)
        .map(|i| {
            let group = i / members;
            let angle = 2.4 + (i % members) as f64 * 0.3;
            Vec2::new(
                5. + (group % 16) as f64 * 4. + angle.cos(),
                -16. + (group / 16) as f64 * 4. + angle.sin(),
            )
        })
        .collect()
}

fn main() {
    assert_eq!(
        std::env::args().len(),
        1,
        "all frozen workloads are mandatory"
    );
    let cases = [
        ("small-64", circle(64), false),
        ("circle-720", circle(720), false),
        ("circle-4096", circle(4096), false),
        (
            "compact-8192",
            (0..8192)
                .map(|i| Vec2::new(5. + (i % 64) as f64 * 1e-4, -3. + (i / 64) as f64 * 1e-4))
                .collect(),
            false,
        ),
        (
            "separated-8192",
            (0..8192)
                .map(|i| Vec2::new(1. + (i % 128) as f64 * 3., 1. + (i / 128) as f64 * 3.))
                .collect(),
            false,
        ),
        ("circles-512", arcs(512, false), false),
        ("sparse-arcs-512", arcs(512, true), true),
    ];
    let pose = Pose {
        position: Vec2::new(12., -7.),
        yaw: 0.7,
    };
    for (name, points, sparse) in cases {
        let scan = LidarScan {
            stamp: 42.,
            points,
            sweep: None,
        };
        let input_hash = checksum(
            scan.points
                .iter()
                .flat_map(|p| [p.x.to_bits(), p.y.to_bits()]),
        );
        let expected = signature(&legacy::detect(&scan, pose, sparse));
        let new = || {
            if sparse {
                LidarClusters.detect_with_sparse_circles(black_box(&scan), black_box(pose))
            } else {
                LidarClusters.detect(black_box(&scan), black_box(pose))
            }
        };
        let old = || legacy::detect(black_box(&scan), black_box(pose), black_box(sparse));
        assert_eq!(signature(&new()), expected);
        let mut legacy_ns = Vec::new();
        let mut indexed_ns = Vec::new();
        for repetition in 0..9 {
            if repetition % 2 == 0 {
                legacy_ns.push(sample(old, &expected));
                indexed_ns.push(sample(new, &expected));
            } else {
                indexed_ns.push(sample(new, &expected));
                legacy_ns.push(sample(old, &expected));
            }
        }
        let output_hash = checksum(expected.iter().flat_map(|v| v.iter().copied()));
        println!(
            r#"{{"case":"{name}","points":{},"sparse_circles":{sparse},"repetitions":9,"input_fnv64":"{input_hash:016x}","output_fnv64":"{output_hash:016x}","detections":{},"bitwise_equal":true,"legacy_ns":{legacy_ns:?},"indexed_ns":{indexed_ns:?}}}"#,
            scan.points.len(),
            expected.len(),
        );
    }
}
