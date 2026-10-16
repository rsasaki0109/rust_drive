//! Sparse axial-depth cells from an existing calibrated KITTI projection report.
mod depth;
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{Read, Write},
    path::{Component, Path, PathBuf},
};

const MAX_REPORT_BYTES: usize = 64 * 1024 * 1024;

fn sha(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn bounded(path: &Path) -> Result<Vec<u8>, String> {
    let meta = fs::symlink_metadata(path).map_err(|e| e.to_string())?;
    if !meta.is_file() || meta.file_type().is_symlink() || meta.len() > MAX_REPORT_BYTES as u64 {
        return Err("projection must be a regular file of at most 64 MiB".into());
    }
    let mut bytes = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(MAX_REPORT_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > MAX_REPORT_BYTES {
        return Err("projection report byte bound".into());
    }
    Ok(bytes)
}

fn fresh(path: &Path) -> Result<fs::File, String> {
    if path
        .as_os_str()
        .as_encoded_bytes()
        .split(|&b| b == b'/')
        .any(|part| part == b"." || part == b"..")
    {
        return Err("output path must not contain dot or parent components".into());
    }
    let absolute = if path.is_absolute() {
        path.to_path_buf()
    } else {
        std::env::current_dir()
            .map_err(|e| e.to_string())?
            .join(path)
    };
    let mut current = PathBuf::new();
    for component in absolute
        .parent()
        .ok_or("output parent missing")?
        .components()
    {
        match component {
            Component::RootDir | Component::Normal(_) => current.push(component.as_os_str()),
            _ => return Err("invalid output directory component".into()),
        }
        match fs::symlink_metadata(&current) {
            Ok(meta) if meta.is_dir() && !meta.file_type().is_symlink() => (),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                fs::create_dir(&current).map_err(|e| e.to_string())?;
                let meta = fs::symlink_metadata(&current).map_err(|e| e.to_string())?;
                if !meta.is_dir() || meta.file_type().is_symlink() {
                    return Err("output parent changed during admission".into());
                }
            }
            _ => return Err("output parent must be a real directory".into()),
        }
    }
    fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(absolute)
        .map_err(|e| format!("fresh output admission: {e}"))
}

fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 4 {
        return Err("require --projection <report.json> --output <new-depth.json>".into());
    }
    let mut options = BTreeMap::new();
    for pair in args.chunks_exact(2) {
        if !["--projection", "--output"].contains(&pair[0].as_str())
            || options.insert(pair[0].as_str(), pair[1].as_str()).is_some()
        {
            return Err("unknown or duplicate CLI option".into());
        }
    }
    // Match the projection tool's exclusive output admission before input reads.
    let mut output = fresh(Path::new(options["--output"]))?;
    let bytes = bounded(Path::new(options["--projection"]))?;
    let input: depth::Projection = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    let depth = depth::rasterize(&input)?;
    let report = json!({
        "schema_version":1,"kind":"kitti_sparse_axial_depth",
        "camera_index":input.camera_index,
        "input_projection":{"bytes":bytes.len(),"sha256":sha(&bytes)},
        "sources":{
            "binary":sha(include_bytes!("main.rs")),
            "rasterizer":sha(include_bytes!("depth.rs")),
            "cargo_manifest":sha(include_bytes!("../Cargo.toml")),
            "cargo_lock":sha(include_bytes!("../Cargo.lock"))
        },
        "depth":depth,
        "depth_units":"metres; positive selected-camera axial Z",
        "pixel_rule":"floor(u), floor(v); exact half-open canvas",
        "selection_rule":"minimum axial Z, then minimum original point index; no epsilon",
        "missing_depth":"unobserved; sparse cells only; no interpolation or densification",
        "scope":"Validated projected rows; source hash identifies bytes, not calibration authenticity or measured accuracy. Cell occlusion is a raster rule, not physical visibility. No synchronization, deskew, images, inference or driving integration."
    });
    let serialized = serde_json::to_vec(&report).map_err(|e| e.to_string())?;
    if serialized.len() > MAX_REPORT_BYTES {
        return Err("depth report byte bound".into());
    }
    output.write_all(&serialized).map_err(|e| e.to_string())?;
    println!("{}", report["depth"]["summary"]);
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("KITTI sparse depth: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_roundtrip_preserves_adjacent_depths_and_pixel_boundaries() {
        let input: depth::Projection = serde_json::from_str(
            r#"{
            "schema_version":1,"kind":"kitti_processed_projection_analytic_tool",
            "camera_index":0,"effective_model":{"camera_index":0,"rectified_size":[2,1]},
            "rows":[
                {"index":0,"velodyne_xyz_m":[0,0,0],"reflectance":0,
                 "camera_xyz_m":[0,0,4],"pixel":[0.1,0],"status":"inframe"},
                {"index":1,"velodyne_xyz_m":[0,0,0],"reflectance":0,
                 "camera_xyz_m":[0,0,3.9999999999999996],
                 "pixel":[0.9999999999999999,0],"status":"inframe"},
                {"index":2,"velodyne_xyz_m":[0,0,0],"reflectance":0,
                 "camera_xyz_m":[0,0,5],"pixel":[1,0],"status":"inframe"}],
            "summary":{"points":3,"inframe":3,"outside":0,"behind":0}}
        "#,
        )
        .unwrap();
        let result = depth::rasterize(&input).unwrap();
        assert_eq!(result.pixels.len(), 2);
        assert_eq!(result.pixels[0].point_index, 1);
        assert_eq!(result.pixels[0].depth_m.to_bits(), 4_f64.to_bits() - 1);
        assert_eq!(result.pixels[0].pixel, [0, 0]);
        assert_eq!(result.pixels[1].point_index, 2);
        assert_eq!(result.pixels[1].pixel, [1, 0]);
    }
}
