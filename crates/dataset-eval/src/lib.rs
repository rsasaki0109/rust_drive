//! Native bounded research point-cloud readers; labels belong to the evaluator.
use rustdriving_core::Vec3;
use std::collections::BTreeMap;
use std::io::Read;
use std::path::Path;

pub const MAX_FILE_BYTES: usize = 64 * 1024 * 1024;
pub const MAX_POINTS: usize = 500_000;

pub fn read_bounded(path: &Path) -> Result<Vec<u8>, String> {
    let file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let size = file.metadata().map_err(|e| e.to_string())?.len();
    if size > MAX_FILE_BYTES as u64 {
        return Err("point-cloud file exceeds 64 MiB bound".into());
    }
    let mut bytes = Vec::new();
    file.take(MAX_FILE_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > MAX_FILE_BYTES {
        return Err("point-cloud file grew beyond bound".into());
    }
    Ok(bytes)
}
/// PCL's LZF block decoder, including overlapping back references. Output size
/// comes from the independently validated point/field dimensions, not the block.
pub fn decompress_lzf(input: &[u8], expected: usize) -> Result<Vec<u8>, String> {
    if expected > MAX_POINTS * 12 || input.len() > MAX_FILE_BYTES {
        return Err("LZF allocation bound exceeded".into());
    }
    let mut output = Vec::with_capacity(expected);
    let mut cursor = 0usize;
    while cursor < input.len() {
        let control = input[cursor];
        cursor += 1;
        if control < 32 {
            let length = usize::from(control) + 1;
            if cursor
                .checked_add(length)
                .is_none_or(|end| end > input.len())
                || output
                    .len()
                    .checked_add(length)
                    .is_none_or(|end| end > expected)
            {
                return Err("LZF literal exceeds input or declared output".into());
            }
            output.extend_from_slice(&input[cursor..cursor + length]);
            cursor += length;
        } else {
            let mut length = usize::from(control >> 5);
            if length == 7 {
                length += usize::from(*input.get(cursor).ok_or("truncated LZF extended length")?);
                cursor += 1;
            }
            let low = usize::from(*input.get(cursor).ok_or("truncated LZF back reference")?);
            cursor += 1;
            let distance = (usize::from(control & 31) << 8) + low + 1;
            length += 2;
            if distance > output.len()
                || output
                    .len()
                    .checked_add(length)
                    .is_none_or(|end| end > expected)
            {
                return Err("invalid LZF reference or output overflow".into());
            }
            for _ in 0..length {
                output.push(output[output.len() - distance]);
            }
        }
    }
    if output.len() != expected {
        return Err("LZF decoded size differs from point dimensions".into());
    }
    Ok(output)
}
fn finite_point(p: Vec3) -> Result<Vec3, String> {
    if !p.finite() || [p.x, p.y, p.z].iter().any(|v| v.abs() > 10_000_000.0) {
        Err("point is nonfinite or exceeds coordinate bound".into())
    } else {
        Ok(p)
    }
}
/// PCD binary_compressed XYZ float32, supporting any XYZ field order. PCL stores
/// compressed data by field (all X then all Y then all Z), not interleaved XYZ.
pub fn parse_pcd(bytes: &[u8]) -> Result<Vec<Vec3>, String> {
    if bytes.len() > MAX_FILE_BYTES {
        return Err("PCD file bound exceeded".into());
    }
    let mut header = BTreeMap::new();
    let mut cursor = 0usize;
    loop {
        let remaining = bytes.get(cursor..).ok_or("missing PCD DATA header")?;
        let length = remaining
            .iter()
            .position(|b| *b == b'\n')
            .ok_or("truncated PCD header")?;
        if cursor + length > 16_384 {
            return Err("PCD header exceeds 16 KiB".into());
        }
        let line = std::str::from_utf8(&remaining[..length])
            .map_err(|_| "PCD header is not ASCII")?
            .trim();
        cursor += length + 1;
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut words = line.split_whitespace();
        let key = words.next().ok_or("empty PCD header")?;
        if ![
            "VERSION",
            "FIELDS",
            "SIZE",
            "TYPE",
            "COUNT",
            "WIDTH",
            "HEIGHT",
            "VIEWPOINT",
            "POINTS",
            "DATA",
        ]
        .contains(&key)
        {
            return Err("unsupported PCD header field".into());
        }
        let values: Vec<_> = words.map(str::to_owned).collect();
        if header.insert(key.to_owned(), values).is_some() {
            return Err("duplicate PCD header field".into());
        }
        if key == "DATA" {
            break;
        }
    }
    let values = |key: &str| header.get(key).ok_or_else(|| format!("missing PCD {key}"));
    let count = |key: &str| -> Result<usize, String> {
        let words = values(key)?;
        if words.len() != 1 {
            return Err(format!("invalid PCD {key}"));
        }
        words[0].parse().map_err(|_| format!("invalid PCD {key}"))
    };
    let fields = values("FIELDS")?;
    if fields.len() != 3
        || !["x", "y", "z"]
            .iter()
            .all(|axis| fields.iter().filter(|s| s.as_str() == *axis).count() == 1)
        || values("SIZE")? != &["4", "4", "4"]
        || values("TYPE")? != &["F", "F", "F"]
        || header.get("COUNT").is_some_and(|v| v != &["1", "1", "1"])
        || values("DATA")? != &["binary_compressed"]
    {
        return Err("requires compressed scalar XYZ float32 PCD".into());
    }
    let points = count("POINTS")?;
    if points == 0
        || points > MAX_POINTS
        || count("WIDTH")?.checked_mul(count("HEIGHT")?) != Some(points)
    {
        return Err("invalid or oversized PCD dimensions".into());
    }
    let prefix = bytes
        .get(cursor..cursor + 8)
        .ok_or("missing PCD compressed block sizes")?;
    let compressed = u32::from_le_bytes(prefix[..4].try_into().unwrap()) as usize;
    let uncompressed = u32::from_le_bytes(prefix[4..].try_into().unwrap()) as usize;
    let end = cursor
        .checked_add(8)
        .and_then(|n| n.checked_add(compressed))
        .ok_or("PCD block offset overflow")?;
    if uncompressed != points * 12 || end > bytes.len() || compressed > MAX_FILE_BYTES {
        return Err("PCD block dimensions or trailing payload mismatch".into());
    }
    // These historical PCL files reserve a 4096-byte header in their file size,
    // but place the compressed block immediately after the actual text header.
    // Accept only the exact unused reservation as zero padding, never payload.
    let padding = &bytes[end..];
    if !padding.is_empty()
        && (cursor > 4096 || padding.len() != 4096 - cursor || padding.iter().any(|b| *b != 0))
    {
        return Err("PCD trailing bytes are not the exact zero header-reservation padding".into());
    }
    let block = decompress_lzf(&bytes[cursor + 8..end], uncompressed)?;
    let axes: Vec<_> = ["x", "y", "z"]
        .iter()
        .map(|axis| fields.iter().position(|s| s == axis).unwrap())
        .collect();
    (0..points)
        .map(|index| {
            let coordinate = |axis: usize| {
                let offset = (axes[axis] * points + index) * 4;
                f64::from(f32::from_le_bytes(
                    block[offset..offset + 4].try_into().unwrap(),
                ))
            };
            finite_point(Vec3::new(coordinate(0), coordinate(1), coordinate(2)))
        })
        .collect()
}
/// Legacy ASCII VTK POLYDATA point section. Subsequent attributes are ignored;
/// they do not become coordinates, labels or point registration evidence.
pub fn parse_vtk(bytes: &[u8]) -> Result<Vec<Vec3>, String> {
    if bytes.len() > MAX_FILE_BYTES {
        return Err("VTK file bound exceeded".into());
    }
    let text = std::str::from_utf8(bytes).map_err(|_| "VTK must be ASCII text")?;
    let mut lines = text.lines();
    if !lines
        .next()
        .is_some_and(|line| line.starts_with("# vtk DataFile Version "))
    {
        return Err("missing legacy VTK version header".into());
    }
    lines.next().ok_or("missing VTK title")?;
    if lines.next().map(str::trim) != Some("ASCII")
        || lines.next().map(str::trim) != Some("DATASET POLYDATA")
    {
        return Err("requires ASCII VTK POLYDATA".into());
    }
    let point_header = lines.next().ok_or("missing VTK POINTS")?;
    let words: Vec<_> = point_header.split_whitespace().collect();
    if words.len() != 3 || words[0] != "POINTS" || !["float", "double"].contains(&words[2]) {
        return Err("invalid VTK POINTS header".into());
    }
    let count: usize = words[1].parse().map_err(|_| "invalid VTK point count")?;
    if count == 0 || count > MAX_POINTS {
        return Err("VTK point count exceeds bound".into());
    }
    let mut numbers = lines.flat_map(str::split_whitespace);
    let mut coordinate = || -> Result<f64, String> {
        let value = numbers
            .next()
            .ok_or("truncated VTK point section")?
            .parse::<f64>()
            .map_err(|_| "invalid VTK numeric coordinate")?;
        Ok(value)
    };
    (0..count)
        .map(|_| finite_point(Vec3::new(coordinate()?, coordinate()?, coordinate()?)))
        .collect()
}
/// Uncompressed legacy LAS 1.0–1.3 point formats 0–3. Scaling is applied to
/// signed integer XYZ; classification flags never enter the geometry result.
#[derive(Debug)]
pub struct LasGeometry {
    pub points: Vec<Vec3>,
    pub version_minor: u8,
    pub point_format: u8,
    pub scales: [f64; 3],
    pub offsets: [f64; 3],
}
struct LasHeader {
    offset: usize,
    record_bytes: usize,
    count: usize,
    version_minor: u8,
    point_format: u8,
    scales: [f64; 3],
    offsets: [f64; 3],
}
fn las_header(bytes: &[u8]) -> Result<LasHeader, String> {
    if bytes.len() > MAX_FILE_BYTES || bytes.len() < 227 || &bytes[..4] != b"LASF" {
        return Err("invalid or oversized LAS header".into());
    }
    if bytes[24] != 1 || bytes[25] > 3 {
        return Err("requires LAS version 1.0–1.3".into());
    }
    let u16_at = |i| u16::from_le_bytes(bytes[i..i + 2].try_into().unwrap()) as usize;
    let u32_at = |i| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap()) as usize;
    let f64_at = |i| f64::from_le_bytes(bytes[i..i + 8].try_into().unwrap());
    let header_bytes = u16_at(94);
    let offset = u32_at(96);
    let record_bytes = u16_at(105);
    let count = u32_at(107);
    let point_format = bytes[104];
    let min_record = match point_format {
        0 => 20,
        1 => 28,
        2 => 26,
        3 => 34,
        _ => return Err("compressed or unsupported LAS point format".into()),
    };
    if header_bytes < if bytes[25] == 3 { 235 } else { 227 }
        || header_bytes > offset
        || offset > bytes.len()
        || record_bytes < min_record
        || record_bytes > 256
        || count == 0
        || count > MAX_POINTS
        || count
            .checked_mul(record_bytes)
            .and_then(|n| offset.checked_add(n))
            .is_none_or(|end| end > bytes.len())
    {
        return Err("LAS header offsets, records, or point count exceed bounds".into());
    }
    // Verify each variable-length record ends before point data. Never search
    // for coordinates through metadata or trust an unchecked point offset.
    let vlrs = u32_at(100);
    if vlrs > 4096 {
        return Err("too many LAS variable-length records".into());
    }
    let mut cursor = header_bytes;
    for _ in 0..vlrs {
        let vlr = bytes
            .get(cursor..cursor.checked_add(54).ok_or("LAS VLR overflow")?)
            .ok_or("truncated LAS VLR")?;
        let length = u16::from_le_bytes(vlr[20..22].try_into().unwrap()) as usize;
        cursor = cursor
            .checked_add(54 + length)
            .ok_or("LAS VLR size overflow")?;
        if cursor > offset {
            return Err("LAS VLR overlaps point data".into());
        }
    }
    let scales = [f64_at(131), f64_at(139), f64_at(147)];
    let offsets = [f64_at(155), f64_at(163), f64_at(171)];
    if scales
        .iter()
        .any(|v| !v.is_finite() || *v <= 0. || *v > 1000.)
        || offsets
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 10_000_000.)
    {
        return Err("LAS scales/offsets invalid or unbounded".into());
    }
    Ok(LasHeader {
        offset,
        record_bytes,
        count,
        version_minor: bytes[25],
        point_format,
        scales,
        offsets,
    })
}
pub fn parse_las_points(bytes: &[u8]) -> Result<LasGeometry, String> {
    let h = las_header(bytes)?;
    let mut points = Vec::with_capacity(h.count);
    for i in 0..h.count {
        let record = &bytes[h.offset + i * h.record_bytes..h.offset + (i + 1) * h.record_bytes];
        let coordinate = |axis: usize| {
            f64::from(i32::from_le_bytes(
                record[axis * 4..axis * 4 + 4].try_into().unwrap(),
            )) * h.scales[axis]
                + h.offsets[axis]
        };
        points.push(finite_point(Vec3::new(
            coordinate(0),
            coordinate(1),
            coordinate(2),
        ))?);
    }
    Ok(LasGeometry {
        points,
        version_minor: h.version_minor,
        point_format: h.point_format,
        scales: h.scales,
        offsets: h.offsets,
    })
}
/// Evaluator-only classification decoder: class 2 ground, known classes other
/// than 0/1/7/8/12 non-ground; unknown/unclassified/noise/overlap or withheld
/// returns have no accuracy label. Coordinates are not selected using labels.
pub fn parse_las_labels(bytes: &[u8]) -> Result<Vec<Option<bool>>, String> {
    let h = las_header(bytes)?;
    Ok((0..h.count)
        .map(|i| {
            let classification = bytes[h.offset + i * h.record_bytes + 15];
            let class = classification & 31;
            if classification & 128 != 0 || ![2, 3, 4, 5, 6, 9, 10, 11].contains(&class) {
                None
            } else {
                Some(class == 2)
            }
        })
        .collect())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn las() -> Vec<u8> {
        let mut b = vec![0; 227 + 3 * 34];
        b[..4].copy_from_slice(b"LASF");
        b[24] = 1;
        b[25] = 2;
        b[94..96].copy_from_slice(&227u16.to_le_bytes());
        b[96..100].copy_from_slice(&227u32.to_le_bytes());
        b[104] = 3;
        b[105..107].copy_from_slice(&34u16.to_le_bytes());
        b[107..111].copy_from_slice(&3u32.to_le_bytes());
        for axis in 0..3 {
            b[131 + axis * 8..139 + axis * 8]
                .copy_from_slice(&[0.01f64, 0.02, 0.1][axis].to_le_bytes());
            b[155 + axis * 8..163 + axis * 8]
                .copy_from_slice(&[100f64, 200., 300.][axis].to_le_bytes());
        }
        for (i, class) in [2u8, 6, 128 | 2].into_iter().enumerate() {
            let base = 227 + i * 34;
            for (axis, value) in [-100i32, 250, 10].into_iter().enumerate() {
                b[base + axis * 4..base + axis * 4 + 4].copy_from_slice(&value.to_le_bytes());
            }
            b[base + 15] = class;
        }
        b
    }
    #[test]
    fn las_integer_scaling_and_labels_remain_separate() {
        let bytes = las();
        let geometry = parse_las_points(&bytes).unwrap();
        assert_eq!(geometry.points, vec![Vec3::new(99., 205., 301.); 3]);
        assert_eq!(geometry.scales, [0.01, 0.02, 0.1]);
        assert_eq!(
            parse_las_labels(&bytes).unwrap(),
            [Some(true), Some(false), None]
        );
        let mut changed = bytes.clone();
        changed[227 + 15] = 7;
        assert_eq!(parse_las_points(&changed).unwrap().points, geometry.points);
        assert_eq!(parse_las_labels(&changed).unwrap()[0], None);
    }
    #[test]
    fn las_rejects_truncation_compression_invalid_scaling_and_offsets() {
        let original = las();
        let mut compressed = original.clone();
        compressed[104] = 128 | 3;
        let mut wrong_offset = original.clone();
        wrong_offset[96..100].copy_from_slice(&220u32.to_le_bytes());
        let mut bad_scale = original.clone();
        bad_scale[131..139].copy_from_slice(&f64::NAN.to_le_bytes());
        let mut overlap = original.clone();
        overlap[100..104].copy_from_slice(&1u32.to_le_bytes());
        let mut enormous = original.clone();
        enormous[107..111].copy_from_slice(&500_001u32.to_le_bytes());
        for bytes in [
            compressed,
            wrong_offset,
            bad_scale,
            overlap,
            enormous,
            original[..original.len() - 1].to_vec(),
        ] {
            assert!(parse_las_points(&bytes).is_err());
            assert!(parse_las_labels(&bytes).is_err());
        }
    }
    #[test]
    fn lzf_overlap_and_long_reference_are_real_copies() {
        assert_eq!(
            decompress_lzf(&[0, b'a', 224, 4, 0], 14).unwrap(),
            vec![b'a'; 14]
        );
        assert_eq!(
            decompress_lzf(&[2, b'a', b'b', b'c', 128, 2], 9).unwrap(),
            b"abcabcabc"
        );
    }
    #[test]
    fn malformed_lzf_cannot_reference_before_output_or_overflow() {
        for (input, size) in [
            (&[32, 0][..], 3),
            (&[2, b'a'][..], 3),
            (&[224][..], 3),
            (&[0, b'a'][..], 2),
            (&[0, b'a', 32, 0][..], 3),
        ] {
            assert!(decompress_lzf(input, size).is_err());
        }
        assert!(decompress_lzf(&[], MAX_POINTS * 12 + 1).is_err());
    }
    fn pcd(fields: &str, values: [f32; 6]) -> Vec<u8> {
        let mut result=format!("VERSION .7\nFIELDS {fields}\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary_compressed\n").into_bytes();
        let raw: Vec<_> = values.into_iter().flat_map(f32::to_le_bytes).collect();
        result.extend_from_slice(&(25u32).to_le_bytes());
        result.extend_from_slice(&(24u32).to_le_bytes());
        result.push(23);
        result.extend(raw);
        result
    }
    #[test]
    fn compressed_pcd_uses_field_major_layout_and_validates_dimensions() {
        let raw = pcd("z x y", [3.0, 6.0, 1.0, 4.0, 2.0, 5.0]);
        assert_eq!(
            parse_pcd(&raw).unwrap(),
            [Vec3::new(1., 2., 3.), Vec3::new(4., 5., 6.)]
        );
        let mut bad = raw.clone();
        bad.push(0);
        assert!(parse_pcd(&bad).is_err());
        assert!(parse_pcd(&pcd("x y z", [f32::NAN, 0., 0., 0., 0., 0.])).is_err());
        let truncated = &raw[..raw.len() - 1];
        assert!(parse_pcd(truncated).is_err());
    }
    #[test]
    fn historical_pcl_padding_is_exactly_bounded_and_must_be_all_zero() {
        let mut raw = pcd("x y z", [1., 4., 2., 5., 3., 6.]);
        let header = raw.len() - 8 - 25;
        raw.resize(raw.len() + 4096 - header, 0);
        assert_eq!(parse_pcd(&raw).unwrap().len(), 2);
        let mut changed = raw.clone();
        *changed.last_mut().unwrap() = 1;
        assert!(parse_pcd(&changed).is_err());
        raw.push(0);
        assert!(parse_pcd(&raw).is_err());
    }
    #[test]
    fn vtk_points_are_bounded_and_attributes_do_not_supply_missing_xyz() {
        let file=b"# vtk DataFile Version 3.0\ntitle\nASCII\nDATASET POLYDATA\nPOINTS 2 float\n1 2 3\n4 5 6\nPOINT_DATA 2\n";
        assert_eq!(
            parse_vtk(file).unwrap(),
            [Vec3::new(1., 2., 3.), Vec3::new(4., 5., 6.)]
        );
        for file in [b"# vtk DataFile Version 3.0\ntitle\nASCII\nDATASET POLYDATA\nPOINTS 2 float\n1 2 3\nPOINT_DATA 2\n".as_slice(),b"# vtk DataFile Version 3.0\ntitle\nASCII\nDATASET POLYDATA\nPOINTS 9999999 float\n".as_slice()] {assert!(parse_vtk(file).is_err());}
    }
}
