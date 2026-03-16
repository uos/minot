#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_and_generate_simple_struct() {
        let input = "
            struct Simple {
                a: u32,
                b: f64,
            }
        ";
        let output = parse_and_generate(input, "Simple").unwrap();
        assert!(output.contains("uint32_t a;"));
        assert!(output.contains("double b;"));
        assert!(output.contains("} minot_Simple;"));
    }

    #[test]
    fn test_parse_and_generate_complex_struct() {
        let input = "
            struct Header {
                seq: u32,
                stamp: Time,
                frame_id: String,
            }
            struct Time {
                sec: i32,
                nanosec: u32,
            }
            struct Imu {
                header: Header,
                orientation: Quaternion,
                orientation_covariance: [f64; 9],
                linear_acceleration: Vector3,
                linear_acceleration_covariance: [f64; 9],
            }
        ";
        let output = parse_and_generate(input, "").unwrap();
        
        // Check Header
        assert!(output.contains("uint32_t seq;"));
        assert!(output.contains("minot_Time stamp;"));
        assert!(output.contains("minot_rkyv_string frame_id;"));
        assert!(output.contains("} minot_Header;"));

        // Check Time
        assert!(output.contains("int32_t sec;"));
        assert!(output.contains("uint32_t nanosec;"));
        assert!(output.contains("} minot_Time;"));
        
        // Check Imu
        assert!(output.contains("minot_Header header;"));
        assert!(output.contains("minot_Quaternion orientation;"));
        assert!(output.contains("double orientation_covariance[9];"));
        assert!(output.contains("minot_Vector3 linear_acceleration;"));
        assert!(output.contains("double linear_acceleration_covariance[9];"));
        assert!(output.contains("} minot_Imu;"));
    }

    #[test]
    fn test_parse_and_generate_arrays_and_vecs() {
        let input = "
            struct Data {
                data: Vec<u8>,
                matrix: [f32; 16],
                flags: [bool; 4],
            }
        ";
        let output = parse_and_generate(input, "Data").unwrap();
        
        assert!(output.contains("minot_rkyv_string data;"));
        assert!(output.contains("float matrix[16];"));
        assert!(output.contains("bool flags[4];"));
    }
}
