//! Sparse axial-depth rasterization of an existing KITTI projection report.
//! Pixel cells are half-open unit squares; smaller camera Z wins, then row ID.
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const MAX_POINTS: usize = 200_000;

/// The operational subset of the projection tool's version-1 report.
/// Calibration and provenance fields stay in the source report, bound by its hash.
#[derive(Debug, Deserialize)]
pub struct Projection {
    pub schema_version: u32,
    pub kind: String,
    pub camera_index: usize,
    pub effective_model: Canvas,
    pub rows: Vec<Point>,
    pub summary: ProjectionCounts,
}

#[derive(Debug, Deserialize)]
pub struct Canvas {
    pub camera_index: usize,
    pub rectified_size: [usize; 2],
}

#[derive(Debug, Deserialize)]
pub struct Point {
    pub index: usize,
    pub velodyne_xyz_m: [f64; 3],
    pub reflectance: f64,
    pub camera_xyz_m: [f64; 3],
    pub pixel: Option<[f64; 2]>,
    pub status: ProjectionStatus,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ProjectionStatus {
    Inframe,
    Outside,
    Behind,
}

#[derive(Debug, Deserialize, PartialEq)]
pub struct ProjectionCounts {
    pub points: usize,
    pub inframe: usize,
    pub outside: usize,
    pub behind: usize,
}

/// One occupied cell. Missing cells carry no inferred or interpolated depth.
#[derive(Debug, Serialize, PartialEq)]
pub struct DepthPixel {
    pub pixel: [usize; 2],
    pub depth_m: f64,
    pub point_index: usize,
    pub support_count: usize,
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum AssignmentStatus {
    Selected,
    Occluded,
    Outside,
    Behind,
}

/// Every original row is retained, including a link to the cell's winning row.
#[derive(Debug, Serialize, PartialEq)]
pub struct Assignment {
    pub point_index: usize,
    pub status: AssignmentStatus,
    pub pixel: Option<[usize; 2]>,
    pub selected_point_index: Option<usize>,
}

#[derive(Debug, Serialize, PartialEq)]
pub struct Summary {
    pub points: usize,
    pub inframe_points: usize,
    pub occupied_pixels: usize,
    pub occluded_points: usize,
    pub outside: usize,
    pub behind: usize,
}

#[derive(Debug, Serialize, PartialEq)]
pub struct SparseDepth {
    pub rectified_size: [usize; 2],
    /// Sorted by y, then x, independently of source point order.
    pub pixels: Vec<DepthPixel>,
    pub assignments: Vec<Assignment>,
    pub summary: Summary,
}

/// Validate the complete input before selecting any depth. This validates a
/// projected-row contract, not the authenticity or accuracy of calibration.
pub fn rasterize(input: &Projection) -> Result<SparseDepth, String> {
    let [width, height] = input.effective_model.rectified_size;
    if input.schema_version != 1
        || input.kind != "kitti_processed_projection_analytic_tool"
        || input.camera_index > 3
        || input.effective_model.camera_index != input.camera_index
    {
        return Err("expected a version-1 KITTI projection report and matching camera".into());
    }
    if !(1..=8192).contains(&width) || !(1..=8192).contains(&height) || width * height > 16_777_216
    {
        return Err("rectified canvas size/capacity bound".into());
    }
    if input.rows.is_empty() || input.rows.len() > MAX_POINTS {
        return Err("projection must contain 1..200000 rows".into());
    }
    let mut counts = ProjectionCounts {
        points: input.rows.len(),
        inframe: 0,
        outside: 0,
        behind: 0,
    };
    for (index, row) in input.rows.iter().enumerate() {
        if row.index != index
            || !row.reflectance.is_finite()
            || row
                .velodyne_xyz_m
                .iter()
                .chain(&row.camera_xyz_m)
                .any(|v| !v.is_finite())
        {
            return Err(format!(
                "invalid point index or nonfinite point at row {index}"
            ));
        }
        if row.camera_xyz_m[2] <= 0. {
            if row.pixel.is_some() || row.status != ProjectionStatus::Behind {
                return Err(format!(
                    "nonpositive depth must be behind without a pixel: {index}"
                ));
            }
            counts.behind += 1;
        } else {
            let [u, v] = row.pixel.ok_or("positive depth requires a pixel")?;
            if !u.is_finite() || !v.is_finite() {
                return Err("nonfinite projected pixel".into());
            }
            let inframe = u >= 0. && v >= 0. && u < width as f64 && v < height as f64;
            let expected = if inframe {
                counts.inframe += 1;
                ProjectionStatus::Inframe
            } else {
                counts.outside += 1;
                ProjectionStatus::Outside
            };
            if row.status != expected {
                return Err(format!(
                    "pixel status contradicts half-open canvas: {index}"
                ));
            }
        }
    }
    if counts != input.summary {
        return Err("projection summary disagrees with complete rows".into());
    }

    // Sparse storage scales with observed cells, never the full image canvas.
    let mut cells: BTreeMap<(usize, usize), DepthPixel> = BTreeMap::new();
    for row in &input.rows {
        if row.status != ProjectionStatus::Inframe {
            continue;
        }
        let [u, v] = row.pixel.expect("validated positive-depth pixel");
        let [x, y] = [u.floor() as usize, v.floor() as usize];
        let cell = cells.entry((y, x)).or_insert(DepthPixel {
            pixel: [x, y],
            depth_m: row.camera_xyz_m[2],
            point_index: row.index,
            support_count: 0,
        });
        cell.support_count += 1;
        // Rows are validated in increasing index order. An exact depth tie
        // keeps the earlier original index, without an epsilon tie interval.
        if row.camera_xyz_m[2] < cell.depth_m {
            cell.depth_m = row.camera_xyz_m[2];
            cell.point_index = row.index;
        }
    }
    let assignments = input
        .rows
        .iter()
        .map(|row| {
            let (status, pixel, selected_point_index) = match row.status {
                ProjectionStatus::Inframe => {
                    let [u, v] = row.pixel.expect("validated pixel");
                    let cell = &cells[&(v.floor() as usize, u.floor() as usize)];
                    let status = if row.index == cell.point_index {
                        AssignmentStatus::Selected
                    } else {
                        AssignmentStatus::Occluded
                    };
                    (status, Some(cell.pixel), Some(cell.point_index))
                }
                ProjectionStatus::Outside => (AssignmentStatus::Outside, None, None),
                ProjectionStatus::Behind => (AssignmentStatus::Behind, None, None),
            };
            Assignment {
                point_index: row.index,
                status,
                pixel,
                selected_point_index,
            }
        })
        .collect();
    let summary = Summary {
        points: counts.points,
        inframe_points: counts.inframe,
        occupied_pixels: cells.len(),
        occluded_points: counts.inframe - cells.len(),
        outside: counts.outside,
        behind: counts.behind,
    };
    Ok(SparseDepth {
        rectified_size: [width, height],
        pixels: cells.into_values().collect(),
        assignments,
        summary,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn projection() -> Projection {
        // Row 1 has shorter axial Z but longer Euclidean range than row 0.
        // Rows 1/2 tie; row 3 lies in the next cell, not a rounded neighbour.
        let raw = [
            ([0., 0., 5.], Some([0.9, 0.9]), ProjectionStatus::Inframe),
            ([20., 0., 4.], Some([0.1, 0.1]), ProjectionStatus::Inframe),
            ([0., 0., 4.], Some([0., 0.]), ProjectionStatus::Inframe),
            ([0., 0., 3.], Some([1., 0.]), ProjectionStatus::Inframe),
            ([0., 0., 1.], Some([2., 0.]), ProjectionStatus::Outside),
            ([0., 0., 0.], None, ProjectionStatus::Behind),
        ];
        Projection {
            schema_version: 1,
            kind: "kitti_processed_projection_analytic_tool".into(),
            camera_index: 2,
            effective_model: Canvas {
                camera_index: 2,
                rectified_size: [2, 2],
            },
            rows: raw
                .into_iter()
                .enumerate()
                .map(|(index, (camera_xyz_m, pixel, status))| Point {
                    index,
                    velodyne_xyz_m: [0.; 3],
                    reflectance: 0.,
                    camera_xyz_m,
                    pixel,
                    status,
                })
                .collect(),
            summary: ProjectionCounts {
                points: 6,
                inframe: 4,
                outside: 1,
                behind: 1,
            },
        }
    }

    #[test]
    fn axial_depth_occlusion_exact_ties_and_floor_cells() {
        let depth = rasterize(&projection()).unwrap();
        assert_eq!(
            depth.pixels,
            vec![
                DepthPixel {
                    pixel: [0, 0],
                    depth_m: 4.,
                    point_index: 1,
                    support_count: 3
                },
                DepthPixel {
                    pixel: [1, 0],
                    depth_m: 3.,
                    point_index: 3,
                    support_count: 1
                },
            ]
        );
        assert_eq!(
            depth
                .assignments
                .iter()
                .map(|a| a.status)
                .collect::<Vec<_>>(),
            vec![
                AssignmentStatus::Occluded,
                AssignmentStatus::Selected,
                AssignmentStatus::Occluded,
                AssignmentStatus::Selected,
                AssignmentStatus::Outside,
                AssignmentStatus::Behind
            ]
        );
        assert_eq!(depth.assignments[0].selected_point_index, Some(1));
        assert_eq!(depth.assignments[2].selected_point_index, Some(1));
        assert_eq!(depth.summary.occluded_points, 2);
    }

    #[test]
    fn empty_canvas_and_row_major_order_do_not_require_a_dense_image() {
        let mut input = projection();
        input.effective_model.rectified_size = [8192, 2048];
        input.rows[0].pixel = Some([8191.9, 2047.9]);
        input.rows[3].pixel = Some([0., 1.]);
        // Row 4 stays outside the large canvas.
        input.rows[4].pixel = Some([-0.01, 0.]);
        let depth = rasterize(&input).unwrap();
        assert_eq!(
            depth.pixels.iter().map(|p| p.pixel).collect::<Vec<_>>(),
            vec![[0, 0], [0, 1], [8191, 2047]]
        );
        for row in &mut input.rows {
            row.camera_xyz_m[2] = -1.;
            row.status = ProjectionStatus::Behind;
            row.pixel = None;
        }
        input.summary = ProjectionCounts {
            points: 6,
            inframe: 0,
            outside: 0,
            behind: 6,
        };
        let depth = rasterize(&input).unwrap();
        assert!(depth.pixels.is_empty());
        assert_eq!(depth.assignments.len(), 6);
    }

    #[test]
    fn malformed_rows_and_summaries_are_rejected() {
        for mutate in [
            |p: &mut Projection| p.rows[0].index = 1,
            |p: &mut Projection| p.rows[0].camera_xyz_m[2] = f64::NAN,
            |p: &mut Projection| p.rows[0].velodyne_xyz_m[0] = f64::INFINITY,
            |p: &mut Projection| p.rows[0].reflectance = f64::NAN,
            |p: &mut Projection| p.rows[0].pixel = Some([f64::INFINITY, 0.]),
            |p: &mut Projection| p.rows[0].pixel = None,
            |p: &mut Projection| p.rows[0].status = ProjectionStatus::Outside,
            |p: &mut Projection| p.rows[5].pixel = Some([0., 0.]),
            |p: &mut Projection| p.summary.points = 5,
            |p: &mut Projection| p.effective_model.camera_index = 1,
            |p: &mut Projection| p.effective_model.rectified_size = [8192, 8192],
            |p: &mut Projection| p.effective_model.rectified_size = [0, 2],
            |p: &mut Projection| p.schema_version = 2,
            |p: &mut Projection| p.rows.clear(),
        ] {
            let mut input = projection();
            mutate(&mut input);
            assert!(rasterize(&input).is_err());
        }
    }

    #[test]
    fn too_many_otherwise_valid_points_are_rejected() {
        let mut input = projection();
        input.rows = (0..=MAX_POINTS)
            .map(|index| Point {
                index,
                velodyne_xyz_m: [0.; 3],
                reflectance: 0.,
                camera_xyz_m: [0.; 3],
                pixel: None,
                status: ProjectionStatus::Behind,
            })
            .collect();
        input.summary = ProjectionCounts {
            points: input.rows.len(),
            inframe: 0,
            outside: 0,
            behind: input.rows.len(),
        };
        assert!(rasterize(&input).unwrap_err().contains("1..200000"));
    }
}
