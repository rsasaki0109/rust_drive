//! Exact calibrated KITTI processed-image point projection; no dataset download.
#[path = "../kitti_calibration.rs"]
mod kitti_calibration;
use kitti_calibration::{Calibration, MAX_CALIBRATION_BYTES, Status};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{Read, Write},
    path::{Component, Path, PathBuf},
};
const PROTOCOL_SHA: &str = "f19310ee577bf9e77b8b2969763d5ec58f7e0cb1884671ac076f8d987a52832c";
const AUTHORITY_SHA: &str = "2eff9f3660d23430c8639f68abac8da1161d53334a3459a2c4018d6043db5fee";
const PROTOCOL_BYTES: &[u8] = include_bytes!("../../../../assets/kitti-projection-v1/design.json");
const AUTHORITY_BYTES: &[u8] =
    include_bytes!("../../../../assets/kitti-projection-v1/source-authority.json");
const MAX_POINTS: usize = 200_000;
const MAX_POINT_BYTES: usize = MAX_POINTS * 16;
const MAX_REPORT_BYTES: usize = 64 * 1024 * 1024;
fn sha(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn sources() -> Value {
    json!({"binary":sha(include_bytes!("rustdriving-kitti-project.rs")),
        "calibration":sha(include_bytes!("../kitti_calibration.rs")),
        "design":sha(PROTOCOL_BYTES),"source_authority":sha(AUTHORITY_BYTES),
        "cargo_manifest":sha(include_bytes!("../../Cargo.toml")),
        "cargo_lock":sha(include_bytes!("../../Cargo.lock")),
        "rust_toolchain":sha(include_bytes!("../../../../rust-toolchain.toml"))})
}
fn bounded(path: &Path, max: usize) -> Result<Vec<u8>, String> {
    let meta = fs::symlink_metadata(path).map_err(|e| format!("input metadata: {e}"))?;
    if !meta.is_file() || meta.file_type().is_symlink() || meta.len() > max as u64 {
        return Err("regular input file/byte bound".into());
    }
    let mut bytes = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(max as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > max {
        return Err("input byte bound".into());
    }
    Ok(bytes)
}
fn fresh(path: &Path) -> Result<fs::File, String> {
    // Path::components normalizes interior `.`; check supplied bytes first.
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
    let parent = absolute.parent().ok_or("output parent missing")?;
    let mut current = PathBuf::new();
    for component in parent.components() {
        match component {
            Component::RootDir | Component::Normal(_) => current.push(component.as_os_str()),
            _ => return Err("output path must not contain dot or parent components".into()),
        }
        match fs::symlink_metadata(&current) {
            Ok(meta) => {
                if meta.file_type().is_symlink() || !meta.is_dir() {
                    return Err("output parent must be a real directory, never symlink".into());
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                fs::create_dir(&current).map_err(|e| e.to_string())?;
                let meta = fs::symlink_metadata(&current).map_err(|e| e.to_string())?;
                if meta.file_type().is_symlink() || !meta.is_dir() {
                    return Err("output parent changed during admission".into());
                }
            }
            Err(e) => return Err(e.to_string()),
        }
    }
    fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(absolute)
        .map_err(|e| format!("fresh output admission: {e}"))
}
fn parse_points(bytes: &[u8]) -> Result<Vec<([f64; 3], f64)>, String> {
    if bytes.is_empty() || bytes.len() > MAX_POINT_BYTES || !bytes.len().is_multiple_of(16) {
        return Err("Velodyne input must contain1..200000 complete16B points".into());
    }
    let mut points = Vec::with_capacity(bytes.len() / 16);
    for (index, row) in bytes.chunks_exact(16).enumerate() {
        let values: [f64; 4] = std::array::from_fn(|i| {
            f64::from(f32::from_le_bytes(
                row[4 * i..4 * i + 4].try_into().expect("fixed4B chunk"),
            ))
        });
        if values.iter().any(|v| !v.is_finite()) {
            return Err(format!("nonfinite point/reflectance at index{index}"));
        }
        points.push(([values[0], values[1], values[2]], values[3]));
    }
    Ok(points)
}
fn text(bytes: &[u8]) -> Result<&str, String> {
    std::str::from_utf8(bytes).map_err(|e| e.to_string())
}
fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mut options = BTreeMap::new();
    if !args.len().is_multiple_of(2) {
        return Err("every option requires a value".into());
    }
    for pair in args.chunks_exact(2) {
        if ![
            "--calib-cam",
            "--calib-velo",
            "--calib-imu",
            "--points",
            "--camera",
            "--output",
        ]
        .contains(&pair[0].as_str())
            || options.insert(pair[0].as_str(), pair[1].as_str()).is_some()
        {
            return Err("unknown or duplicate CLI option".into());
        }
    }
    if options.len() != 6 {
        return Err(
            "require --calib-cam --calib-velo --calib-imu --points --camera --output".into(),
        );
    }
    let camera: usize = options["--camera"]
        .parse()
        .map_err(|_| "invalid camera index")?;
    if camera > 3 {
        return Err("camera index must be0..3".into());
    }
    // Admission precedes ALL reads of camera/transform/point source files.
    let mut output = fresh(Path::new(options["--output"]))?;
    if sha(PROTOCOL_BYTES) != PROTOCOL_SHA || sha(AUTHORITY_BYTES) != AUTHORITY_SHA {
        return Err("fixed projection protocol/authority SHA mismatch".into());
    }
    let protocol: Value = serde_json::from_slice(PROTOCOL_BYTES).map_err(|e| e.to_string())?;
    let authority: Value = serde_json::from_slice(AUTHORITY_BYTES).map_err(|e| e.to_string())?;
    let cam = bounded(Path::new(options["--calib-cam"]), MAX_CALIBRATION_BYTES)?;
    let velo = bounded(Path::new(options["--calib-velo"]), MAX_CALIBRATION_BYTES)?;
    let imu = bounded(Path::new(options["--calib-imu"]), MAX_CALIBRATION_BYTES)?;
    let calibration = Calibration::from_texts(text(&cam)?, text(&velo)?, text(&imu)?)?;
    let model = calibration.model(camera)?;
    let bytes = bounded(Path::new(options["--points"]), MAX_POINT_BYTES)?;
    let points = parse_points(&bytes)?;
    let mut counts = [0usize; 3];
    let mut rows = Vec::with_capacity(points.len());
    for (index, (point, reflectance)) in points.into_iter().enumerate() {
        let projected = model.project(point)?;
        counts[match projected.status {
            Status::Inframe => 0,
            Status::Outside => 1,
            Status::Behind => 2,
        }] += 1;
        rows.push(json!({"index":index,"velodyne_xyz_m":point,"reflectance":reflectance,
            "camera_xyz_m":projected.camera_xyz_m,"pixel":projected.pixel,"status":projected.status}));
    }
    let inputs = [
        ("calib_cam_to_cam", &cam),
        ("calib_velo_to_cam", &velo),
        ("calib_imu_to_velo", &imu),
        ("velodyne_points", &bytes),
    ];
    let hashes: BTreeMap<_, _> = inputs
        .into_iter()
        .map(|(key, b)| (key, json!({"bytes":b.len(),"sha256":sha(b)})))
        .collect();
    let report = json!({"schema_version":1,"kind":"kitti_processed_projection_analytic_tool",
        "camera_index":camera,"protocol_sha256":PROTOCOL_SHA,"protocol":protocol,
        "source_authority_sha256":AUTHORITY_SHA,"source_authority":authority,"source_hashes":hashes,"sources":sources(),"calibration":calibration,"effective_model":model,
        "rows":rows,"summary":{"points":rows.len(),"inframe":counts[0],"outside":counts[1],"behind":counts[2]},
        "projection_chain":"P_rect_selected * embed(R_rect_00) * T_cam0_from_velo * X_velo",
        "camera_axes":"right,down,forward","velodyne_and_imu_axes":"forward,left,up","translation_units":"metres",
        "native_camera_parameters":"provenance only; never applied to already processed/rectified pixels",
        "compatibility_metadata":"calib_time/corner_dist/delta_f/delta_c accepted as metadata; not used in projection",
        "camera_images_loaded":false,"labels_loaded":false,"ground_truth_operational":false,"data_downloaded":false,
        "limits":["Requires user-provided authorized raw calibration/Velodyne files; no dataset acquisition or accuracy claim.","Projection only; no motion compensation, deskew, timestamps, pose evaluation, tracking, uncertainty or driving integration.","Input hashes identify supplied bytes; they do not authenticate factory calibration, a common acquisition day, or archive membership. Calibration processing times may differ.","Reflectance is retained, finite checked, and unused in geometry.","All input rows retained; nonpositive camera depth is behind, without a pixel."]});
    let serialized = serde_json::to_vec(&report).map_err(|e| e.to_string())?;
    if serialized.len() > MAX_REPORT_BYTES {
        return Err("report byte bound".into());
    }
    output.write_all(&serialized).map_err(|e| e.to_string())?;
    println!("{}", report["summary"]);
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("KITTI projection: {e}");
        std::process::exit(2);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hand_little_endian_xyz_reflectance_and_malformed_rows() {
        let bytes: [u8; 16] = [0, 0, 128, 63, 0, 0, 0, 192, 0, 0, 96, 64, 0, 0, 0, 63];
        assert_eq!(parse_points(&bytes).unwrap(), vec![([1., -2., 3.5], 0.5)]);
        for n in [0, 1, 15, 17] {
            assert!(parse_points(&vec![0; n]).is_err());
        }
        assert!(parse_points(&vec![0; MAX_POINT_BYTES + 16]).is_err());
        for index in 0..4 {
            let mut invalid = bytes;
            invalid[index * 4..index * 4 + 4].copy_from_slice(&f32::INFINITY.to_le_bytes());
            assert!(parse_points(&invalid).is_err());
        }
    }
    #[test]
    fn output_exclusive_admission_preserves_existing_bytes_and_rejects_symlink_parent() {
        let dir = std::env::temp_dir().join(format!(
            "rustdriving-kitti-admission-{}",
            std::process::id()
        ));
        fs::create_dir(&dir).unwrap();
        let path = dir.join("proof.json");
        fresh(&path).unwrap().write_all(b"preserve").unwrap();
        assert!(fresh(&path).is_err());
        assert_eq!(fs::read(&path).unwrap(), b"preserve");
        #[cfg(unix)]
        {
            let link = dir.join("link");
            std::os::unix::fs::symlink(&dir, &link).unwrap();
            assert!(fresh(&link.join("escape.json")).is_err());
        }
        assert!(fresh(&dir.join("../escape.json")).is_err());
        assert!(fresh(&dir.join("./escape.json")).is_err());
        fs::remove_dir_all(dir).unwrap();
    }
}
