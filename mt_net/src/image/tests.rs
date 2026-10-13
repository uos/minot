use super::*;
use crate::{CompressedImage, Image};

fn raw(encoding: &str, width: u32, height: u32, step: u32, data: Vec<u8>) -> Image {
    Image {
        encoding: encoding.into(),
        width,
        height,
        step,
        data,
        ..Default::default()
    }
}

#[test]
fn color_order_and_alpha() {
    for (encoding, data, expected) in [
        ("rgb8", vec![10, 20, 30], [10, 20, 30, 255]),
        ("bgr8", vec![30, 20, 10], [10, 20, 30, 255]),
        ("rgba8", vec![10, 20, 30, 40], [10, 20, 30, 40]),
        ("bgra8", vec![30, 20, 10, 40], [10, 20, 30, 40]),
    ] {
        let message = raw(encoding, 1, 1, data.len() as u32, data);
        assert_eq!(
            message.to_image().unwrap().into_rgba8().get_pixel(0, 0).0,
            expected
        );
    }
}

#[test]
fn rows_strip_padding() {
    let message = raw("bgr8", 1, 2, 5, vec![3, 2, 1, 99, 99, 6, 5, 4, 88, 88]);
    assert_eq!(
        message.to_image().unwrap().into_rgb8().into_raw(),
        [1, 2, 3, 4, 5, 6]
    );
    assert_eq!(message.data[3], 99);
}

#[test]
fn mono8_preserves_values() {
    for encoding in ["mono8", "8UC1"] {
        assert_eq!(
            raw(encoding, 2, 1, 2, vec![0, 255])
                .to_image()
                .unwrap()
                .into_luma8()
                .into_raw(),
            [0, 255]
        );
    }
}

#[test]
fn mono16_handles_byte_order_and_padding() {
    for encoding in ["mono16", "16UC1"] {
        for bigendian in [0, 1] {
            let samples = [0x1234_u16, 0xabcd];
            let mut data = vec![];
            for value in samples {
                data.extend(if bigendian == 1 {
                    value.to_be_bytes()
                } else {
                    value.to_le_bytes()
                });
                data.push(99);
            }
            let mut message = raw(encoding, 1, 2, 3, data);
            message.is_bigendian = bigendian;
            assert_eq!(
                message.to_image().unwrap().into_luma16().into_raw(),
                samples
            );
        }
    }
}

#[test]
fn color16_preserves_channels_and_alpha() {
    for (encoding, samples, expected) in [
        ("rgb16", vec![1000_u16, 2000, 3000], vec![1000, 2000, 3000]),
        ("bgr16", vec![3000, 2000, 1000], vec![1000, 2000, 3000]),
        (
            "rgba16",
            vec![1000, 2000, 3000, 4000],
            vec![1000, 2000, 3000, 4000],
        ),
        (
            "bgra16",
            vec![3000, 2000, 1000, 4000],
            vec![1000, 2000, 3000, 4000],
        ),
    ] {
        for bigendian in [0, 1] {
            let data: Vec<_> = samples
                .iter()
                .flat_map(|v| {
                    if bigendian == 1 {
                        v.to_be_bytes()
                    } else {
                        v.to_le_bytes()
                    }
                })
                .collect();
            let mut message = raw(encoding, 1, 1, data.len() as u32, data);
            message.is_bigendian = bigendian;
            let image = message.to_image().unwrap();
            let actual = if samples.len() == 3 {
                image.into_rgb16().into_raw()
            } else {
                image.into_rgba16().into_raw()
            };
            assert_eq!(actual, expected);
        }
    }
}

fn compressed(format: ::image::ImageFormat) -> CompressedImage {
    let pixels =
        DynamicImage::ImageRgb8(ImageBuffer::from_pixel(2, 1, ::image::Rgb([120, 60, 30])));
    let mut cursor = std::io::Cursor::new(Vec::new());
    pixels.write_to(&mut cursor, format).unwrap();
    CompressedImage {
        format: if format == ::image::ImageFormat::Png {
            "rgb8; png compressed bgr8".into()
        } else {
            "rgb8; jpeg compressed bgr8".into()
        },
        data: cursor.into_inner(),
        ..Default::default()
    }
}

#[test]
fn png_decodes_without_swapping_rgb() {
    let message = compressed(::image::ImageFormat::Png);
    let pixels = message.to_image().unwrap().into_rgb8();
    assert_eq!(pixels.dimensions(), (2, 1));
    assert_eq!(pixels.get_pixel(0, 0).0, [120, 60, 30]);
}

#[test]
fn jpeg_decodes() {
    let pixels = compressed(::image::ImageFormat::Jpeg)
        .to_image()
        .unwrap()
        .into_rgb8();
    assert_eq!(pixels.dimensions(), (2, 1));
    for (actual, expected) in pixels.get_pixel(0, 0).0.into_iter().zip([120_u8, 60, 30]) {
        assert!(actual.abs_diff(expected) <= 5);
    }
}

#[test]
fn compressed_format_can_be_empty() {
    let mut message = compressed(::image::ImageFormat::Png);
    message.format.clear();
    assert_eq!(
        message.to_image().unwrap().into_rgb8().get_pixel(0, 0).0,
        [120, 60, 30]
    );
}

#[test]
fn png16_preserves_depth_values() {
    let pixels =
        DynamicImage::ImageLuma16(ImageBuffer::from_raw(2, 1, vec![42_u16, 65535]).unwrap());
    let mut cursor = std::io::Cursor::new(vec![]);
    pixels
        .write_to(&mut cursor, ::image::ImageFormat::Png)
        .unwrap();
    let message = CompressedImage {
        data: cursor.into_inner(),
        format: "mono16; png compressed".into(),
        ..Default::default()
    };
    assert_eq!(
        message.to_image().unwrap().into_luma16().into_raw(),
        [42, 65535]
    );
}

#[test]
fn archived_messages_convert_without_deserializing() {
    let message = raw("bgra8", 1, 1, 4, vec![30, 20, 10, 40]);
    let bytes = rkyv::to_bytes::<rkyv::rancor::Error>(&message).unwrap();
    let archived = rkyv::access::<rkyv::Archived<Image>, rkyv::rancor::Error>(&bytes).unwrap();
    assert_eq!(
        archived_image_to_image(archived)
            .unwrap()
            .into_rgba8()
            .get_pixel(0, 0)
            .0,
        [10, 20, 30, 40]
    );
    let message = compressed(::image::ImageFormat::Png);
    let bytes = rkyv::to_bytes::<rkyv::rancor::Error>(&message).unwrap();
    let archived =
        rkyv::access::<rkyv::Archived<CompressedImage>, rkyv::rancor::Error>(&bytes).unwrap();
    assert_eq!(
        archived_compressed_image_to_image(archived)
            .unwrap()
            .into_rgb8()
            .get_pixel(0, 0)
            .0,
        [120, 60, 30]
    );
}

#[test]
fn invalid_layouts_return_errors() {
    for message in [
        raw("rgb8", 2, 1, 5, vec![0; 5]),
        raw("mono16", 1, 2, 2, vec![0; 3]),
        raw("mono8", 1, 1, 1, vec![0; 2]),
        raw("mono8", 0, 1, 0, vec![]),
        raw("mono8", 1, 0, 1, vec![]),
        raw("rgba16", u32::MAX, u32::MAX, u32::MAX, vec![]),
        raw("32FC1", 1, 1, 4, vec![0; 4]),
        raw("bayer_rggb8", 1, 1, 1, vec![0]),
    ] {
        assert!(message.to_image().is_err(), "{}", message.encoding);
    }
    let mut message = raw("mono8", 1, 1, 1, vec![0]);
    message.is_bigendian = 2;
    assert!(message.to_image().is_err());
}

#[test]
fn invalid_compressed_and_depth_transport_return_errors() {
    let message = CompressedImage {
        data: vec![1, 2, 3],
        ..Default::default()
    };
    assert!(message.to_image().is_err());
    let mut message = compressed(::image::ImageFormat::Png);
    message.format = "16UC1; compressedDepth png".into();
    assert!(
        message
            .to_image()
            .unwrap_err()
            .to_string()
            .contains("compressedDepth")
    );
    message.format.clear();
    message.data.truncate(20);
    assert!(message.to_image().is_err());
}

fn dynamic_samples() -> Vec<DynamicImage> {
    vec![
        DynamicImage::ImageLuma8(ImageBuffer::from_raw(2, 1, vec![1_u8, 255]).unwrap()),
        DynamicImage::ImageRgb8(
            ImageBuffer::from_raw(2, 1, vec![10_u8, 20, 30, 40, 50, 60]).unwrap(),
        ),
        DynamicImage::ImageRgba8(
            ImageBuffer::from_raw(2, 1, vec![10_u8, 20, 30, 40, 50, 60, 70, 80]).unwrap(),
        ),
        DynamicImage::ImageLuma16(ImageBuffer::from_raw(2, 1, vec![0x1234_u16, 65535]).unwrap()),
        DynamicImage::ImageRgb16(
            ImageBuffer::from_raw(2, 1, vec![1_u16, 256, 65535, 1000, 2000, 3000]).unwrap(),
        ),
        DynamicImage::ImageRgba16(
            ImageBuffer::from_raw(2, 1, vec![1_u16, 256, 65535, 999, 1000, 2000, 3000, 4000])
                .unwrap(),
        ),
    ]
}

#[test]
fn encode_raw_roundtrip_preserves_samples() {
    for pixels in dynamic_samples() {
        let message = image_from_dynamic(&pixels).unwrap();
        assert_eq!(message.width, 2);
        assert_eq!(message.height, 1);
        assert_eq!(message.step as usize, pixels.as_bytes().len());
        assert_eq!(message.header, Default::default());
        let decoded = message.to_image().unwrap();
        assert_eq!(decoded.color(), pixels.color());
        assert_eq!(decoded.as_bytes(), pixels.as_bytes());
    }
}

#[test]
fn encode_png_roundtrip_preserves_samples_and_alpha() {
    for pixels in dynamic_samples() {
        let message = compressed_image_from_dynamic(&pixels, ImageFormat::Png).unwrap();
        assert_eq!(message.format, "png");
        assert_eq!(message.header, Default::default());
        let decoded = message.to_image().unwrap();
        assert_eq!(decoded.color(), pixels.color());
        assert_eq!(decoded.as_bytes(), pixels.as_bytes());
    }
}

#[test]
fn encode_jpeg_roundtrip() {
    let pixels =
        DynamicImage::ImageRgb8(ImageBuffer::from_pixel(2, 1, ::image::Rgb([120, 60, 30])));
    let message = compressed_image_from_dynamic(&pixels, ImageFormat::Jpeg).unwrap();
    assert_eq!(message.format, "jpeg");
    let decoded = message.to_image().unwrap().into_rgb8();
    assert_eq!(decoded.dimensions(), (2, 1));
    for (actual, expected) in decoded.get_pixel(0, 0).0.into_iter().zip([120_u8, 60, 30]) {
        assert!(actual.abs_diff(expected) <= 5);
    }
}

#[test]
fn encoders_reject_unsupported_types_instead_of_losing_samples() {
    let rgba = DynamicImage::new_rgba8(1, 1);
    let mono16 = DynamicImage::new_luma16(1, 1);
    assert!(compressed_image_from_dynamic(&rgba, ImageFormat::Jpeg).is_err());
    assert!(compressed_image_from_dynamic(&mono16, ImageFormat::Jpeg).is_err());
    assert!(compressed_image_from_dynamic(&rgba, ImageFormat::Gif).is_err());
    assert!(image_from_dynamic(&DynamicImage::new_rgb32f(1, 1)).is_err());
    assert!(image_from_dynamic(&DynamicImage::new_luma_a8(1, 1)).is_err());
    let empty = DynamicImage::new_rgb8(0, 0);
    assert!(image_from_dynamic(&empty).is_err());
    assert!(compressed_image_from_dynamic(&empty, ImageFormat::Png).is_err());
}

#[test]
fn fluent_headers_preserve_pixels_and_setter_order() {
    let pixels = DynamicImage::ImageRgb8(ImageBuffer::from_pixel(2, 1, ::image::Rgb([1, 2, 3])));
    let header = Header {
        frame_id: "original".into(),
        stamp: Time { sec: 7, nanosec: 8 },
    };
    let stamp = Time {
        sec: 42,
        nanosec: 123,
    };
    let raw = image_from_dynamic(&pixels).unwrap();
    let raw_data = raw.data.clone();
    let raw = raw
        .with_header(header.clone())
        .with_frame_id("camera")
        .with_stamp(stamp.clone());
    assert_eq!(raw.header.frame_id, "camera");
    assert_eq!(raw.header.stamp, stamp);
    assert_eq!(raw.data, raw_data);
    let encoded = compressed_image_from_dynamic(&pixels, ImageFormat::Png).unwrap();
    let encoded_data = encoded.data.clone();
    let encoded = encoded
        .with_stamp(stamp)
        .with_frame_id("overwritten")
        .with_header(header.clone());
    assert_eq!(encoded.header, header);
    assert_eq!(encoded.data, encoded_data);
    assert_eq!(
        encoded.to_image().unwrap().into_rgb8().get_pixel(0, 0).0,
        [1, 2, 3]
    );
}
