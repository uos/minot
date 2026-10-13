//! Decode and encode camera messages with the `image` crate.
//!
//! The `image` feature is enabled by default on `mt_net` and `mt_pubsub`. JPEG and PNG
//! codecs are included; additional `image` codec features can be enabled by
//! the application. Raw RGB/BGR, RGBA/BGRA, and mono images preserve their
//! 8-bit or 16-bit samples, strip row padding, and respect byte order.
//! `8UC1` and `16UC1` are treated as grayscale, without scaling depth values.
//! Bayer, YUV, floating-point depth, and compressedDepth require specialized
//! processing and return errors rather than silently changing their meaning.
//!
//! ```
//! use mt_net::{Image, image::ToImage};
//!
//! let message = Image {
//!     width: 1, height: 1, encoding: "bgr8".into(), step: 3,
//!     data: vec![10, 20, 30], ..Default::default()
//! };
//! let pixels = message.to_image()?.into_rgb8();
//! assert_eq!(pixels.get_pixel(0, 0).0, [30, 20, 10]);
//! # Ok::<(), anyhow::Error>(())
//! ```

use ::image::ImageBuffer;
pub use ::image::{DynamicImage, ImageFormat};
use anyhow::{Context, Result, bail, ensure};
pub use ros2_interfaces_jazzy_rkyv::{builtin_interfaces::msg::Time, std_msgs::msg::Header};

/// Fluent header setters for raw and compressed camera messages.
///
/// Each setter consumes and returns the message without copying its pixels.
/// Setters apply in order: `with_header` replaces the complete header, while
/// `with_frame_id` and `with_stamp` replace only their respective fields.
///
/// ```
/// use mt_net::image::{DynamicImage, ImageMessageExt, Time, image_from_dynamic};
///
/// let message = image_from_dynamic(&DynamicImage::new_rgb8(2, 1))?
///     .with_frame_id("camera_optical_frame")
///     .with_stamp(Time { sec: 42, nanosec: 123 });
/// assert_eq!(message.header.frame_id, "camera_optical_frame");
/// # Ok::<(), anyhow::Error>(())
/// ```
pub trait ImageMessageExt: Sized {
    fn header_mut(&mut self) -> &mut Header;

    fn with_header(mut self, header: Header) -> Self {
        *self.header_mut() = header;
        self
    }

    fn with_frame_id(mut self, frame_id: impl Into<String>) -> Self {
        self.header_mut().frame_id = frame_id.into();
        self
    }

    fn with_stamp(mut self, stamp: Time) -> Self {
        self.header_mut().stamp = stamp;
        self
    }
}

impl ImageMessageExt for crate::Image {
    fn header_mut(&mut self) -> &mut Header {
        &mut self.header
    }
}

impl ImageMessageExt for crate::CompressedImage {
    fn header_mut(&mut self) -> &mut Header {
        &mut self.header
    }
}

/// Decode an image message into an owned `image::DynamicImage`.
///
/// Implemented for owned Minot camera messages. Archived views can use
/// [`archived_image_to_image`] and [`archived_compressed_image_to_image`].
/// The message remains unchanged. Unsupported encodings, malformed row
/// layouts, and invalid compressed payloads return errors.
pub trait ToImage {
    fn to_image(&self) -> Result<DynamicImage>;
}

impl ToImage for crate::Image {
    fn to_image(&self) -> Result<DynamicImage> {
        raw_image(
            self.width,
            self.height,
            &self.encoding,
            self.step,
            self.is_bigendian,
            &self.data,
        )
    }
}

/// Convert an archived raw image without first deserializing its payload.
pub fn archived_image_to_image(message: &rkyv::Archived<crate::Image>) -> Result<DynamicImage> {
    raw_image(
        message.width.to_native(),
        message.height.to_native(),
        message.encoding.as_str(),
        message.step.to_native(),
        message.is_bigendian,
        message.data.as_slice(),
    )
}

impl ToImage for crate::CompressedImage {
    fn to_image(&self) -> Result<DynamicImage> {
        compressed_image(&self.format, &self.data)
    }
}

/// Decode an archived compressed image without first copying its payload.
pub fn archived_compressed_image_to_image(
    message: &rkyv::Archived<crate::CompressedImage>,
) -> Result<DynamicImage> {
    compressed_image(message.format.as_str(), message.data.as_slice())
}

/// Encode pixels as a tightly packed raw ROS image with a default header.
///
/// Supports grayscale, RGB, and RGBA at 8 or 16 bits. Sixteen-bit samples
/// retain native byte order and set `is_bigendian` accordingly. Unsupported
/// sample types return an error. Set `header` after conversion as needed.
pub fn image_from_dynamic(pixels: &DynamicImage) -> Result<crate::Image> {
    let (encoding, pixel_bytes, sixteen_bit) = match pixels {
        DynamicImage::ImageLuma8(_) => ("mono8", 1, false),
        DynamicImage::ImageRgb8(_) => ("rgb8", 3, false),
        DynamicImage::ImageRgba8(_) => ("rgba8", 4, false),
        DynamicImage::ImageLuma16(_) => ("mono16", 2, true),
        DynamicImage::ImageRgb16(_) => ("rgb16", 6, true),
        DynamicImage::ImageRgba16(_) => ("rgba16", 8, true),
        _ => bail!(
            "Unsupported DynamicImage color type for raw ROS encoding: {:?}",
            pixels.color()
        ),
    };
    ensure!(
        pixels.width() > 0 && pixels.height() > 0,
        "Image dimensions must be nonzero"
    );
    let step = pixels
        .width()
        .checked_mul(pixel_bytes)
        .context("ROS image step overflows")?;
    Ok(crate::Image {
        header: Default::default(),
        width: pixels.width(),
        height: pixels.height(),
        encoding: encoding.into(),
        is_bigendian: u8::from(sixteen_bit && cfg!(target_endian = "big")),
        step,
        data: pixels.as_bytes().to_vec(),
    })
}

/// Encode pixels as a JPEG or PNG ROS compressed image with a default header.
///
/// PNG preserves supported sample precision and alpha; JPEG is lossy. The
/// `image` crate reports color types that the chosen codec cannot encode.
/// Set `header` after conversion as needed.
pub fn compressed_image_from_dynamic(
    pixels: &DynamicImage,
    format: ::image::ImageFormat,
) -> Result<crate::CompressedImage> {
    let codec = match format {
        ::image::ImageFormat::Jpeg => "jpeg",
        ::image::ImageFormat::Png => "png",
        _ => bail!("Compressed ROS image encoding supports JPEG and PNG"),
    };
    ensure!(
        pixels.width() > 0 && pixels.height() > 0,
        "Image dimensions must be nonzero"
    );
    let mut output = std::io::Cursor::new(Vec::new());
    ::image::write_buffer_with_format(
        &mut output,
        pixels.as_bytes(),
        pixels.width(),
        pixels.height(),
        pixels.color(),
        format,
    )
    .context("Cannot encode compressed image")?;
    Ok(crate::CompressedImage {
        header: Default::default(),
        format: codec.into(),
        data: output.into_inner(),
    })
}

fn compressed_image(format: &str, data: &[u8]) -> Result<DynamicImage> {
    ensure!(
        !format.to_ascii_lowercase().contains("compresseddepth"),
        "compressedDepth images require depth transport decoding"
    );
    // Detect the codec from the actual payload: ROS format strings can include
    // pixel encodings (e.g. "rgb8; jpeg compressed bgr8") or be empty.
    ::image::ImageReader::new(std::io::Cursor::new(data))
        .with_guessed_format()
        .context("Cannot identify compressed image format")?
        .decode()
        .context("Cannot decode compressed image")
}

fn raw_image(
    width: u32,
    height: u32,
    encoding: &str,
    step: u32,
    bigendian: u8,
    data: &[u8],
) -> Result<DynamicImage> {
    let (channels, sample_bytes, bgr) = match encoding {
        "mono8" | "8UC1" => (1, 1, false),
        "rgb8" => (3, 1, false),
        "bgr8" => (3, 1, true),
        "rgba8" => (4, 1, false),
        "bgra8" => (4, 1, true),
        "mono16" | "16UC1" => (1, 2, false),
        "rgb16" => (3, 2, false),
        "bgr16" => (3, 2, true),
        "rgba16" => (4, 2, false),
        "bgra16" => (4, 2, true),
        _ => bail!("Unsupported ROS image encoding: {encoding}"),
    };
    ensure!(width > 0 && height > 0, "Image dimensions must be nonzero");
    ensure!(bigendian <= 1, "Image is_bigendian must be 0 or 1");
    let row_bytes = usize::try_from(width)?
        .checked_mul(channels * sample_bytes)
        .context("Image row size overflows")?;
    let stride = usize::try_from(step)?;
    ensure!(
        stride >= row_bytes,
        "Image step {step} is smaller than row size {row_bytes}"
    );
    let expected = stride
        .checked_mul(usize::try_from(height)?)
        .context("Image data size overflows")?;
    ensure!(
        data.len() == expected,
        "Image data has {} bytes, expected {expected}",
        data.len()
    );
    let packed_len = row_bytes
        .checked_mul(usize::try_from(height)?)
        .context("Packed image size overflows")?;
    let mut packed = Vec::new();
    packed
        .try_reserve_exact(packed_len)
        .context("Cannot allocate image pixels")?;
    for row in data.chunks_exact(stride) {
        packed.extend_from_slice(&row[..row_bytes]);
    }

    macro_rules! build {
        ($pixels:expr, $mono:ident, $rgb:ident, $rgba:ident) => {{
            // Layout validation above guarantees every buffer has the required
            // number of samples; from_raw also checks the crate's size limits.
            match channels {
                1 => DynamicImage::$mono(
                    ImageBuffer::from_raw(width, height, $pixels)
                        .context("Invalid grayscale image dimensions")?,
                ),
                3 => DynamicImage::$rgb(
                    ImageBuffer::from_raw(width, height, $pixels)
                        .context("Invalid RGB image dimensions")?,
                ),
                4 => DynamicImage::$rgba(
                    ImageBuffer::from_raw(width, height, $pixels)
                        .context("Invalid RGBA image dimensions")?,
                ),
                _ => unreachable!(),
            }
        }};
    }
    if sample_bytes == 1 {
        if bgr {
            for pixel in packed.chunks_exact_mut(channels) {
                pixel.swap(0, 2);
            }
        }
        Ok(build!(packed, ImageLuma8, ImageRgb8, ImageRgba8))
    } else {
        let mut samples: Vec<u16> = packed
            .chunks_exact(2)
            .map(|bytes| {
                let bytes = [bytes[0], bytes[1]];
                if bigendian == 1 {
                    u16::from_be_bytes(bytes)
                } else {
                    u16::from_le_bytes(bytes)
                }
            })
            .collect();
        if bgr {
            for pixel in samples.chunks_exact_mut(channels) {
                pixel.swap(0, 2);
            }
        }
        Ok(build!(samples, ImageLuma16, ImageRgb16, ImageRgba16))
    }
}

#[cfg(test)]
mod tests;
