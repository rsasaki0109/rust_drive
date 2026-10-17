//! YOLOX raw BGR letterbox using OpenCV's generic uint8 INTER_LINEAR arithmetic.
//!
//! The interpolation contract follows OpenCV 4.13.0 resize.cpp (Apache-2.0,
//! copyright Intel Corporation and OpenCV contributors). Independent cv2
//! execution checks every output byte; IPP/OpenCL paths are outside this claim.
use image::RgbImage;
use serde::Serialize;

#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    Triangle,
    Linear,
}

pub struct Prepared {
    pub values: Vec<f32>,
    pub ratio: f32,
    pub resized: [u32; 2],
}

fn taps(index: u32, source: u32, destination: u32, horizontal: bool) -> (u32, u32, i32, i32) {
    let scale = 1. / (f64::from(destination) / f64::from(source));
    let coordinate = ((f64::from(index) + 0.5) * scale - 0.5) as f32;
    let mut low = coordinate.floor() as i32;
    let mut fraction = coordinate - low as f32;
    if horizontal && (low < 0 || low >= source as i32 - 1) {
        low = low.clamp(0, source as i32 - 1);
        fraction = 0.;
    }
    let a = ((1. - fraction) * 2048.).round_ties_even() as i32;
    let b = (fraction * 2048.).round_ties_even() as i32;
    (
        low.clamp(0, source as i32 - 1) as u32,
        (low + 1).clamp(0, source as i32 - 1) as u32,
        a,
        b,
    )
}

pub fn prepare(image: &RgbImage, size: usize, mode: Mode) -> Result<Prepared, String> {
    let (width, height) = image.dimensions();
    if width == 0
        || height == 0
        || u64::from(width) * u64::from(height) > 16_000_000
        || !(1..=640).contains(&size)
    {
        return Err("bounded positive source and input dimensions required".into());
    }
    // The old path keeps its original f32 ratio and minimum-one-pixel rule.
    // Upstream Python instead uses f64 arithmetic before truncating dimensions.
    let (ratio, rw, rh) = match mode {
        Mode::Triangle => {
            let r = (size as f32 / width as f32).min(size as f32 / height as f32);
            (
                r,
                (width as f32 * r).floor().max(1.) as u32,
                (height as f32 * r).floor().max(1.) as u32,
            )
        }
        Mode::Linear => {
            let r = (size as f64 / f64::from(width)).min(size as f64 / f64::from(height));
            (
                r as f32,
                (f64::from(width) * r) as u32,
                (f64::from(height) * r) as u32,
            )
        }
    };
    if rw == 0 || rh == 0 || rw as usize > size || rh as usize > size {
        return Err("upstream resize would have an empty or out-of-bounds dimension".into());
    }
    let mut values = vec![114.; 3 * size * size];
    if mode == Mode::Triangle {
        let resized = image::imageops::resize(image, rw, rh, image::imageops::FilterType::Triangle);
        for (x, y, pixel) in resized.enumerate_pixels() {
            for c in 0..3 {
                values[c * size * size + y as usize * size + x as usize] = f32::from(pixel[2 - c]);
            }
        }
    } else {
        let xs: Vec<_> = (0..rw).map(|x| taps(x, width, rw, true)).collect();
        for y in 0..rh {
            let (y0, y1, b0, b1) = taps(y, height, rh, false);
            for (x, &(x0, x1, a0, a1)) in xs.iter().enumerate() {
                for c in 0..3 {
                    let pixel = if width == rw && height == rh {
                        // cv::resize directly copies an identically sized image.
                        image.get_pixel(x as u32, y)[2 - c]
                    } else if width == 2 * rw && height == 2 * rh {
                        // OpenCV substitutes its exact two-by-two area fast path.
                        let mut sum = 0_u32;
                        for dy in 0..2 {
                            for dx in 0..2 {
                                sum += u32::from(
                                    image.get_pixel(2 * x as u32 + dx, 2 * y + dy)[2 - c],
                                );
                            }
                        }
                        ((sum + 2) >> 2) as u8
                    } else {
                        let s0 = i32::from(image.get_pixel(x0, y0)[2 - c]) * a0
                            + i32::from(image.get_pixel(x1, y0)[2 - c]) * a1;
                        let s1 = i32::from(image.get_pixel(x0, y1)[2 - c]) * a0
                            + i32::from(image.get_pixel(x1, y1)[2 - c]) * a1;
                        ((((b0 * (s0 >> 4)) >> 16) + ((b1 * (s1 >> 4)) >> 16) + 2) >> 2) as u8
                    };
                    values[c * size * size + y as usize * size + x] = f32::from(pixel);
                }
            }
        }
    }
    Ok(Prepared {
        values,
        ratio,
        resized: [rw, rh],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_order_padding_and_identity_are_explicit() {
        let image = RgbImage::from_raw(2, 1, vec![1, 2, 3, 201, 202, 203]).unwrap();
        let result = prepare(&image, 2, Mode::Linear).unwrap();
        assert_eq!(
            result.values,
            vec![
                3., 203., 114., 114., 2., 202., 114., 114., 1., 201., 114., 114.
            ]
        );
        assert_eq!(result.resized, [2, 1]);
        assert_eq!(result.ratio, 1.);
    }

    #[test]
    fn area_two_by_two_rounds_before_float_conversion() {
        let image =
            RgbImage::from_raw(2, 2, vec![0, 10, 100, 1, 11, 101, 2, 12, 102, 3, 13, 103]).unwrap();
        assert_eq!(
            prepare(&image, 1, Mode::Linear).unwrap().values,
            vec![102., 12., 2.]
        );
    }

    #[test]
    fn invalid_and_extreme_aspect_geometry_is_rejected() {
        assert!(prepare(&RgbImage::new(0, 1), 416, Mode::Linear).is_err());
        assert!(prepare(&RgbImage::new(1, 1), 0, Mode::Linear).is_err());
        assert!(prepare(&RgbImage::new(1, 1), 641, Mode::Linear).is_err());
        assert!(prepare(&RgbImage::new(2000, 1), 416, Mode::Linear).is_err());
        assert!(prepare(&RgbImage::new(2000, 1), 416, Mode::Triangle).is_ok());
    }
}
