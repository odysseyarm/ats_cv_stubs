#![cfg_attr(all(not(feature = "std"), not(test)), no_std)]

use core::fmt::Debug;
use nalgebra::{
    ComplexField, Isometry3, Matrix3, Point2, Point3, Quaternion, RealField, Rotation2, Rotation3,
    SMatrix, SVector, Scalar, Translation2, UnitQuaternion, Vector2, Vector3,
};
#[cfg(feature = "std")]
use nalgebra::{MatrixXx1, MatrixXx2};
use opencv_ros_camera::{Distortion, RosOpenCvIntrinsics};

#[cfg(feature = "std")]
pub mod foveated;
#[cfg(feature = "std")]
pub mod helpers;
pub mod ocv_types;

use num::{
    traits::{float::TotalOrder, Float},
    FromPrimitive,
};

#[cfg(feature = "std")]
use sqpnp::types::{SQPSolution, SolverParameters};
#[cfg(feature = "std")]
use std::error::Error;
#[cfg(feature = "std")]
use std::io::Read;

#[cfg(feature = "std")]
pub fn pnp(_3dpoints: &[Vector3<f32>], projections: &[Vector2<f32>]) -> Option<SQPSolution> {
    let solver = sqpnp::PnpSolver::new(_3dpoints, projections, None, SolverParameters::default());
    if let Some(mut solver) = solver {
        solver.solve();
        if solver.number_of_solutions() == 1 {
            return Some(*solver.solution_ptr(0).unwrap());
        }
    }
    None
}

pub fn transform_aim_point<F: Float + FromPrimitive>(
    aim_point: Point2<F>,
    p1: Point2<F>,
    p2: Point2<F>,
    p3: Point2<F>,
    p4: Point2<F>,
    np1: Point2<F>,
    np2: Point2<F>,
    np3: Point2<F>,
    np4: Point2<F>,
) -> Option<Point2<F>>
where
    F: ComplexField,
    F: RealField,
{
    Some(get_perspective_transform(p1, p2, p3, p4, np1, np2, np3, np4)?.transform_point(&aim_point))
}

pub fn get_perspective_transform<F: Float + FromPrimitive>(
    p1: Point2<F>,
    p2: Point2<F>,
    p3: Point2<F>,
    p4: Point2<F>,
    np1: Point2<F>,
    np2: Point2<F>,
    np3: Point2<F>,
    np4: Point2<F>,
) -> Option<SMatrix<F, 3, 3>>
where
    F: ComplexField,
    F: RealField,
{
    #[rustfmt::skip]
    let a = SMatrix::<F, 8, 8>::from_row_slice(&[
        p1.x, p1.y, F::one(), F::zero(), F::zero(), F::zero(), -np1.x * p1.x, -np1.x * p1.y,
        F::zero(), F::zero(), F::zero(), p1.x, p1.y, F::one(), -np1.y * p1.x, -np1.y * p1.y,
        p2.x, p2.y, F::one(), F::zero(), F::zero(), F::zero(), -np2.x * p2.x, -np2.x * p2.y,
        F::zero(), F::zero(), F::zero(), p2.x, p2.y, F::one(), -np2.y * p2.x, -np2.y * p2.y,
        p3.x, p3.y, F::one(), F::zero(), F::zero(), F::zero(), -np3.x * p3.x, -np3.x * p3.y,
        F::zero(), F::zero(), F::zero(), p3.x, p3.y, F::one(), -np3.y * p3.x, -np3.y * p3.y,
        p4.x, p4.y, F::one(), F::zero(), F::zero(), F::zero(), -np4.x * p4.x, -np4.x * p4.y,
        F::zero(), F::zero(), F::zero(), p4.x, p4.y, F::one(), -np4.y * p4.x, -np4.y * p4.y,
    ]);
    let axp = SVector::from([np1.x, np1.y, np2.x, np2.y, np3.x, np3.y, np4.x, np4.y]);
    let decomp = a.lu();
    let p = decomp.solve(&axp)?;
    Some(Matrix3::new(
        p[0],
        p[1],
        p[2],
        p[3],
        p[4],
        p[5],
        p[6],
        p[7],
        F::one(),
    ))
}

fn split_column<F: Float + FromPrimitive>(
    p: &mut [Point2<F>],
    threshold: F,
) -> (&mut [Point2<F>], &mut [Point2<F>])
where
    F: ComplexField,
    F: RealField,
{
    let [first, rest @ ..] = p else {
        return (&mut [], p);
    };
    for i in 0..rest.len() {
        if Float::abs(rest[i].x - first.x) > threshold {
            return p.split_at_mut(i + 1);
        }
    }
    (p, &mut [])
}

pub fn choose_rectangle_markers<F: Float + FromPrimitive>(
    p: &mut [Point2<F>],
    screen_id: u8,
    column_threshold: F,
) -> Option<[Point2<F>; 4]>
where
    F: ComplexField + RealField + TotalOrder,
{
    if screen_id == 0 {
        p.sort_unstable_by(|a, b| a.x.total_cmp(&b.x));
    } else {
        p.sort_unstable_by(|a, b| b.x.total_cmp(&a.x));
    }
    let n2048 = F::from(2048).unwrap();
    let (col1, rest) = split_column(p, column_threshold);
    let (col2, _) = split_column(rest, column_threshold);
    let &col1_down = col1
        .iter()
        .filter(|p| p.y > n2048)
        .min_by(|a, b| a.y.total_cmp(&b.y))?;
    let &col1_up = col1
        .iter()
        .filter(|p| p.y < n2048)
        .max_by(|a, b| a.y.total_cmp(&b.y))?;
    let &col2_down = col2
        .iter()
        .filter(|p| p.y > n2048)
        .min_by(|a, b| a.y.total_cmp(&b.y))?;
    let &col2_up = col2
        .iter()
        .filter(|p| p.y < n2048)
        .max_by(|a, b| a.y.total_cmp(&b.y))?;
    Some([col1_up, col2_up, col2_down, col1_down])
}

#[cfg(feature = "std")]
pub fn calculate_screen_id<F: Float + FromPrimitive>(markers: &[Point2<F>]) -> Option<u8>
where
    F: ComplexField,
    F: RealField,
{
    let mut closest_left = F::from(9001).unwrap();
    let mut closest_right = F::from(9001).unwrap();
    let mut left_cnt = 0;
    let mut right_cnt = 0;
    for marker in markers {
        let v = marker.x - F::from_i32(2048).unwrap();
        if v < F::from_i32(0).unwrap() {
            closest_left = Float::min(closest_left, -v);
            left_cnt += 1;
        } else {
            closest_right = Float::min(closest_right, v);
            right_cnt += 1;
        }
    }
    if left_cnt + right_cnt < 5 {
        return None;
    }
    if left_cnt < right_cnt {
        return Some(0);
    }
    if left_cnt > right_cnt {
        return Some(1);
    }
    // left_cnt == right_cnt
    if closest_left < closest_right {
        return Some(0);
    }
    Some(1)
}

#[cfg(feature = "std")]
pub fn mot_rotate<F: Float + FromPrimitive + Debug>(a: &[Point2<F>], angle: F) -> Vec<Point2<F>>
where
    F: ComplexField + RealField,
{
    let center = F::from(2047.5).unwrap(); // Define the center of rotation
    let translation_to_center = Translation2::new(-center, -center); // Translation to origin
    let rotation = Rotation2::new(angle); // Define the rotation transformation
    let translation_back = Translation2::new(center, center); // Translation back to original position

    a.iter()
        .map(|point| {
            // Apply transformations
            let p = translation_to_center.transform_point(point); // Move point to origin
            let rotated_p = rotation.transform_point(&p); // Rotate the point
            translation_back.transform_point(&rotated_p) // Move point back
        })
        .collect()
}

#[cfg(feature = "std")]
pub fn calculate_projections<F: Float + FromPrimitive + RealField>(
    screen_points: &[Point2<F>],
    focal_length_pixels: Vector2<F>, // Focal length in pixels
    resolution: Vector2<F>,          // Resolution of the camera (width, height)
) -> Vec<Vector2<F>> {
    let projections = screen_points
        .iter()
        .map(|pos| {
            let x = (pos.x - resolution.x / F::from_f32(2.).unwrap()) / focal_length_pixels.x;
            let y = (pos.y - resolution.y / F::from_f32(2.).unwrap()) / focal_length_pixels.y;
            Vector2::new(x, y)
        })
        .collect();

    projections
}

#[cfg(feature = "std")]
/// Solves the Perspective-n-Point (PnP) problem using given 3D points and their 2D projections.
/// Returns an option containing the solution (rotation and translation vectors) if successful.
pub fn marker_pnp<F: Float + FromPrimitive + RealField + ComplexField>(
    _3dpoints: &[Vector3<F>],   // Array of marker 3D points in world coordinates
    projections: &[Vector2<F>], // Corresponding 2D projections of the 3D points on the camera image
    weights: Option<&[F]>,      // How much weight to give to each 2D projection when minimizing RMS
    previous_solution: Option<&SQPSolution>, // Previous solution to choose the closest new solution from
) -> Option<SQPSolution>
where
    F: ComplexField + RealField,
{
    // The SQPnP library's solver expects points in f32, so we convert from F to f32 if necessary
    let projections_f32 = projections
        .iter()
        .map(|p| Vector2::new(p.x.to_f32().unwrap(), p.y.to_f32().unwrap()))
        .collect::<Vec<_>>();
    let _3dpoints_f32 = _3dpoints
        .iter()
        .map(|p| {
            Vector3::new(
                p.x.to_f32().unwrap(),
                p.y.to_f32().unwrap(),
                p.z.to_f32().unwrap(),
            )
        })
        .collect::<Vec<_>>();
    let weights_f32 = weights.map(|w| w.iter().map(|x| x.to_f32().unwrap()).collect::<Vec<_>>());

    // Create a new PnP solver instance with the converted points
    let solver_parameters: SolverParameters = SolverParameters::default();

    let solver = sqpnp::PnpSolver::new(
        &_3dpoints_f32,
        &projections_f32,
        weights_f32.as_deref(),
        solver_parameters,
    );

    // Attempt to solve the PnP problem
    if let Some(mut solver) = solver {
        solver.solve(); // Execute the solver algorithm

        // Check if there is at least one solution
        if solver.number_of_solutions() == 0 {
            return None;
        }
        let mut best_solution = solver.solution_ptr(0).unwrap();
        // println!("Number of solutions: {}", solver.number_of_solutions());
        for i in 1..solver.number_of_solutions() {
            let solution = solver.solution_ptr(i);
            if let Some(previous_solution) = previous_solution {
                // if it's closer to the previous solution, update the best solution
                let solution = solution.unwrap();
                if (solution.t - previous_solution.t).norm()
                    < (best_solution.t - previous_solution.t).norm()
                {
                    best_solution = solution;
                }
            } else if solution.unwrap().sq_error < best_solution.sq_error {
                best_solution = solution.unwrap();
            }
        }
        return Some(*best_solution);
    }
    None
}

#[cfg(feature = "std")]
pub fn intersection_to_screen_space<F: Float>(
    intersection: Point3<F>,
    screen_calibration: &crate::ScreenCalibration<F>,
) -> Point2<F>
where
    F: ComplexField,
    F: RealField,
{
    screen_calibration
        .homography
        .transform_point(&intersection.xy())
}

#[cfg(feature = "std")]
/// Calculates the 3D coordinates of markers for a virtual screen in front of a camera.
/// The markers are positioned based on width and height ratios relative to the center of the screen, scaled by the screen dimensions.
pub fn calculate_marker_3dpoints<F: Float + FromPrimitive>(
    point_ratios: &[Point2<F>], // Relative positions of the markers on the screen
    aspect_ratio: F,            // Width to height ratio of the screen.
    screen_scale: F,            // Normalized height of the screen, typically 1.0.
) -> Vec<Vector3<F>>
where
    F: ComplexField + RealField,
{
    point_ratios
        .iter()
        .map(|&point_ratio| {
            let x = point_ratio.x * aspect_ratio * screen_scale; // Compute horizontal position relative to center
            let y = point_ratio.y * screen_scale; // Compute vertical position relative to center
            let z = F::zero(); // Assume markers are on a flat screen at z = 0
            Vector3::new(x, y, z)
        })
        .collect()
}

#[cfg(feature = "std")]
pub fn calculate_screen_3dpoints<F: Float + FromPrimitive + RealField + 'static>(
    screen_scale: F, // Scale factor for screen dimensions, typically the height in normalized space
    screen_aspect_ratio: F, // Screen aspect ratio
) -> [Vector3<F>; 4] {
    let _0 = F::from_f32(0.).unwrap();
    let _2 = F::from_f32(2.).unwrap();
    [
        Vector3::new(
            (-screen_scale / _2) * screen_aspect_ratio,
            -screen_scale / _2,
            _0,
        ),
        Vector3::new(
            (screen_scale / _2) * screen_aspect_ratio,
            -screen_scale / _2,
            _0,
        ),
        Vector3::new(
            (screen_scale / _2) * screen_aspect_ratio,
            screen_scale / _2,
            _0,
        ),
        Vector3::new(
            (-screen_scale / _2) * screen_aspect_ratio,
            screen_scale / _2,
            _0,
        ),
    ]
}

#[cfg(feature = "std")]
/// Solves the Perspective-n-Point (PnP) problem using dynamically calculated 3D points based on given 2D projections and configuration ratios for markers on a screen, scaled by screen dimensions.
pub fn solve_pnp_with_dynamic_screen_points<
    F: Float + FromPrimitive + RealField + ComplexField + Debug,
>(
    projections: &[Vector2<F>], // 2D projections of the 3D points on the screen
    point_ratios: &[Point2<F>], // Relative positions of the markers on the screen
    aspect_ratio: F,            // Aspect ratio of the screen
    screen_scale: F,            // Normalized height of the screen, typically 1.0.
) -> Option<SQPSolution>
where
    F: ComplexField + RealField,
{
    let _3dpoints = calculate_marker_3dpoints(point_ratios, aspect_ratio, screen_scale);
    marker_pnp(&_3dpoints, projections, None, None) // Use the core marker_pnp function here
}

#[cfg(feature = "std")]
pub fn ray_plane_intersection<F: Float + FromPrimitive + Scalar + Debug + 'static>(
    ray_origin: &Vector3<F>,
    ray_direction: &Vector3<F>,
    plane_normal: &Vector3<F>,
    plane_point: &Point3<F>,
) -> (Point3<F>, F)
where
    F: ComplexField,
    F: RealField,
{
    let ray_direction = ray_direction.normalize();
    let d = plane_normal.dot(&plane_point.coords);
    let t = (d - plane_normal.dot(ray_origin)) / plane_normal.dot(&ray_direction);
    (Point3::from(ray_origin + ray_direction.scale(t)), t)
}

#[cfg(feature = "std")]
pub fn calculate_aimpoint<F: Float + FromPrimitive + RealField>(
    isometry: &nalgebra::Isometry<F, Rotation3<F>, 3>,
    screen_calibration: &ScreenCalibration<F>,
) -> Option<Point2<F>> {
    let _0 = F::from_f32(0.).unwrap();
    let _1 = F::from_f32(1.).unwrap();

    let (aimpoint, _) = ray_plane_intersection(
        &isometry.translation.vector,
        &isometry.transform_vector(&Vector3::z()),
        &Vector3::z(),
        &Point3::new(_0, _0, _0),
    );

    Some(intersection_to_screen_space(aimpoint, &screen_calibration))
}

#[cfg(feature = "std")]
pub fn calculate_aimpoint_and_distance<F: Float + FromPrimitive + RealField>(
    isometry: &nalgebra::Isometry<F, Rotation3<F>, 3>,
    screen_calibration: &ScreenCalibration<F>,
) -> Option<(Point2<F>, F)> {
    let _0 = F::from_f32(0.).unwrap();
    let _1 = F::from_f32(1.).unwrap();

    let (aimpoint, d) = ray_plane_intersection(
        &isometry.translation.vector,
        &isometry.transform_vector(&Vector3::z()),
        &Vector3::z(),
        &Point3::new(_0, _0, _0),
    );

    Some((
        intersection_to_screen_space(aimpoint, &screen_calibration),
        d,
    ))
}

#[cfg(feature = "std")]
pub fn match_markers<F: Float + FromPrimitive + Scalar>(
    primary_markers: &[Point2<F>],
    secondary_markers: &[Point2<F>],
) -> Vec<Option<Point2<F>>> {
    let mut index_claim_map = [None::<usize>; 16];
    let mut dist_map = [F::max_value(); 16];

    for (i, primary_point) in primary_markers.iter().enumerate() {
        for (j, secondary_point) in secondary_markers.iter().enumerate() {
            let dist = (primary_point.x - secondary_point.x).powi(2)
                + (primary_point.y - secondary_point.y).powi(2);
            if dist_map[i] > dist {
                if index_claim_map[j].is_none() || dist_map[index_claim_map[j].unwrap()] > dist {
                    index_claim_map[j] = Some(i);
                    dist_map[i] = dist;
                }
            }
        }
    }

    // Mapping each primary marker to the nearest secondary marker if available
    primary_markers
        .iter()
        .enumerate()
        .map(|(i, _)| {
            index_claim_map
                .iter()
                .position(|&primary_index| primary_index == Some(i))
                .map(|secondary_index| secondary_markers[secondary_index])
        })
        .collect()
}

#[cfg(feature = "std")]
pub fn wf_to_nf_points<F: Float + FromPrimitive + 'static>(
    widefield_markers: &[Point2<F>],
    nf_intrinsics: &RosOpenCvIntrinsics<F>,
    wf_intrinsics: &RosOpenCvIntrinsics<F>,
    stereo_iso: Isometry3<F>,
) -> Vec<Point2<F>>
where
    F: ComplexField,
    F: RealField,
    F: TotalOrder,
{
    let _n1 = F::from_f32(-1.).unwrap();
    let _0 = F::from_f32(0.).unwrap();
    let _1 = F::from_f32(1.).unwrap();

    let wf_fx = wf_intrinsics.p.m11;
    let wf_fy = wf_intrinsics.p.m22;
    let wf_cx = wf_intrinsics.p.m13;
    let wf_cy = wf_intrinsics.p.m23;

    let nf_fx = nf_intrinsics.p.m11;
    let nf_fy = nf_intrinsics.p.m22;
    let nf_cx = nf_intrinsics.p.m13;
    let nf_cy = nf_intrinsics.p.m23;

    widefield_markers
        .iter()
        .map(|point| {
            let point3d = stereo_iso.rotation.transform_point(&Point3::new(
                (point.x - wf_cx) / wf_fx,
                (point.y - wf_cy) / wf_fy,
                _1,
            ));
            let intrinsic_matrix_3d = Matrix3::new(nf_fx, _0, nf_cx, _0, nf_fy, nf_cy, _0, _0, _1);
            let projected_point = intrinsic_matrix_3d * point3d.coords;
            Point2::new(
                projected_point.x / projected_point.z,
                projected_point.y / projected_point.z,
            )
        })
        .collect::<Vec<_>>()
}

pub fn wf_to_nf_point<F: Float + FromPrimitive + 'static>(
    widefield_marker: Point2<F>,
    nf_intrinsics: &RosOpenCvIntrinsics<F>,
    wf_intrinsics: &RosOpenCvIntrinsics<F>,
    stereo_iso: &Isometry3<F>,
) -> Point2<F>
where
    F: ComplexField,
    F: RealField,
    F: TotalOrder,
{
    let _n1 = F::from_f32(-1.).unwrap();
    let _0 = F::from_f32(0.).unwrap();
    let _1 = F::from_f32(1.).unwrap();

    let wf_fx = wf_intrinsics.p.m11;
    let wf_fy = wf_intrinsics.p.m22;
    let wf_cx = wf_intrinsics.p.m13;
    let wf_cy = wf_intrinsics.p.m23;

    let nf_fx = nf_intrinsics.p.m11;
    let nf_fy = nf_intrinsics.p.m22;
    let nf_cx = nf_intrinsics.p.m13;
    let nf_cy = nf_intrinsics.p.m23;

    let point3d = stereo_iso.rotation.transform_point(&Point3::new(
        (widefield_marker.x - wf_cx) / wf_fx,
        (widefield_marker.y - wf_cy) / wf_fy,
        _1,
    ));
    let intrinsic_matrix_3d = Matrix3::new(nf_fx, _0, nf_cx, _0, nf_fy, nf_cy, _0, _0, _1);
    let projected_point = intrinsic_matrix_3d * point3d.coords;
    Point2::new(
        projected_point.x / projected_point.z,
        projected_point.y / projected_point.z,
    )
}

pub fn to_normalized_image_coordinates<F>(
    point: Point2<F>,
    intrinsics: &RosOpenCvIntrinsics<F>,
    stereo_iso: Option<&Isometry3<F>>,
) -> Point2<F>
where
    F: Float + FromPrimitive + RealField + 'static,
{
    let _n1 = F::from_f32(-1.).unwrap();
    let _0 = F::from_f32(0.).unwrap();
    let _1 = F::from_f32(1.).unwrap();

    let fx = intrinsics.p.m11;
    let fy = intrinsics.p.m22;
    let cx = intrinsics.p.m13;
    let cy = intrinsics.p.m23;

    let mut point = Vector2::new((point.x - cx) / fx, (point.y - cy) / fy);
    if let Some(stereo_iso) = stereo_iso {
        let point2 = stereo_iso.rotation.transform_vector(&point.push(_1));
        point = point2.xy() / point2.z;
    }
    point.into()
}

pub fn peak_detect_128(
    samples: &mut [f32; 128],
    min_freq: f32,
    max_freq: f32,
    sample_rate: f32,
) -> f32 {
    // Compute the RFFT of the samples
    let spectrum = microfft::real::rfft_128(samples);

    // Since the real-valued coefficient at the Nyquist frequency is packed into the
    // imaginary part of the DC bin, it must be cleared before computing the amplitudes
    spectrum[0].im = 0.0;

    let mut max_freq_bin = 0.0;
    let mut max_amplitude = 0.0;
    let mut max_index = 0;

    // Iterate through the spectrum to find the most dominant frequency with an aligned index
    for i in 0..spectrum.len() {
        let freq_bin = i as f32 * sample_rate / 128.;
        let amplitude = spectrum[i].norm();

        if freq_bin > min_freq && freq_bin < max_freq && amplitude > max_amplitude {
            max_freq_bin = freq_bin;
            max_amplitude = amplitude;
            max_index = i;
        }
    }

    // If no aligned frequency bin was found, return 0.0
    if max_amplitude == 0.0 {
        return 0.0;
    }

    if max_index > 0 {
        spectrum[max_index - 1].re = 0.0;
        spectrum[max_index - 1].im = 0.0;
    }
    if max_index - 1 > 0 {
        spectrum[max_index - 2].re = 0.0;
        spectrum[max_index - 2].im = 0.0;
    }
    if max_index < spectrum.len() - 1 {
        spectrum[max_index + 1].re = 0.0;
        spectrum[max_index + 1].im = 0.0;
    }
    if max_index + 1 < spectrum.len() - 1 {
        spectrum[max_index + 2].re = 0.0;
        spectrum[max_index + 2].im = 0.0;
    }

    spectrum[max_index].re = 0.0;
    spectrum[max_index].im = 0.0;

    // Create an array of amplitudes and find the third-highest amplitude
    let mut amplitudes: [f32; 128] = [0.0; 128];
    for i in 0..spectrum.len() {
        let freq_bin = i as f32 * sample_rate / 128.;
        if freq_bin > 21. {
            amplitudes[i] = spectrum[i].norm();
        }
    }

    // Use select_nth_unstable_by to find the third-highest amplitude
    let (_, third_amplitude, _) = amplitudes.select_nth_unstable_by(1, |a, b| b.total_cmp(a));

    // Calculate the ratio and determine the dominant frequency
    let ratio = max_amplitude / *third_amplitude;
    if ratio > 1.2 && max_freq_bin >= min_freq && max_freq_bin <= max_freq {
        return max_freq_bin;
    }

    0.0
}

// fn harmonic_product_spectrum(
//     samples: &mut [f32; 128],
//     sample_rate: f32,
//     max_harmonic: usize,
//     min_freq: f32,
// ) -> f32 {
//     let mut spectrum = microfft::real::rfft_128(samples);
//
//     // Since the real-valued coefficient at the Nyquist frequency is packed into the
//     // imaginary part of the DC bin, it must be cleared before computing the amplitudes
//     spectrum[0].im = 0.0;
//
//     // Calculate the frequency resolution (bin width)
//     let bin_width = sample_rate / 128.0;
//
//     // Calculate the bin indices corresponding to the min and max frequencies
//     let min_bin = (min_freq / bin_width).ceil() as usize;
//
//     // Initialize the harmonic product spectrum in-place
//     let mut harmonics = [0.0; 64]; // Only half the bins are useful due to symmetry
//
//     for h in 1..=max_harmonic {
//         for i in min_bin..64 {
//             let index = i * h;
//             if index < harmonics.len() {
//                 let amplitude = (spectrum[index].re.powi(2) + spectrum[index].im.powi(2)).sqrt();
//                 harmonics[i] += amplitude;
//             }
//         }
//     }
//
//     // Find the frequency bin with the maximum harmonic product within the specified range
//     let max_bin = harmonics[min_bin..64]
//         .iter()
//         .enumerate()
//         .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
//         .map(|(idx, _)| idx + min_bin)
//         .unwrap_or(min_bin);
//
//     // Convert the bin index to a frequency
//     let dominant_frequency = max_bin as f32 * bin_width;
//
//     dominant_frequency
// }

// pub fn peak_detect_128(samples: &mut [f32; 128], min_freq: f32, max_freq: f32, sample_rate: f32) -> f32 {
//     let dominant_freq = harmonic_product_spectrum(samples, sample_rate, 3, 40.);
//
//     if dominant_freq >= min_freq && dominant_freq <= max_freq {
//         dominant_freq
//     } else {
//         0.
//     }
// }

#[cfg(feature = "std")]
pub fn get_intrinsics_from_opencv_camera_calibration_json<
    R: RealField + serde::de::DeserializeOwned + Copy,
>(
    mut reader: impl Read,
) -> Result<RosOpenCvIntrinsics<R>, Box<dyn Error>> {
    let mut contents = String::new();
    reader.read_to_string(&mut contents)?;
    let parsed_json: ocv_types::CameraCalibrationParams<R> = serde_json::from_str(&contents)?;

    Ok(parsed_json.into())
}

#[cfg(feature = "std")]
pub fn get_isometry_from_opencv_stereo_calibration_json<
    R: RealField + serde::de::DeserializeOwned + Copy,
>(
    mut reader: impl Read,
) -> Result<Isometry3<R>, Box<dyn Error>> {
    let mut contents = String::new();
    reader.read_to_string(&mut contents)?;
    let parsed_json: ocv_types::StereoCalibrationParams<R> = serde_json::from_str(&contents)?;

    Ok(parsed_json.into())
}

#[cfg(feature = "std")]
pub fn write_opencv_minimal_camera_calibration_json<R: RealField + serde::Serialize>(
    intrinsics: &RosOpenCvIntrinsics<R>,
    mut writer: impl std::io::Write,
) -> Result<(), Box<dyn Error>> {
    let json = serde_json::to_string_pretty(&ocv_types::MinimalCameraCalibrationParams::from(
        intrinsics.clone(),
    ))?;
    writer.write_all(json.as_bytes())?;
    Ok(())
}

#[cfg(feature = "std")]
pub fn write_opencv_minimal_stereo_calibration_json<R: RealField + serde::Serialize>(
    stereo_iso: &Isometry3<R>,
    mut writer: impl std::io::Write,
) -> Result<(), Box<dyn Error>> {
    let json = serde_json::to_string_pretty(&ocv_types::MinimalStereoCalibrationParams::from(
        stereo_iso.clone(),
    ))?;
    writer.write_all(json.as_bytes())?;
    Ok(())
}

#[cfg(feature = "std")]
pub fn undistort_points<F: Float + FromPrimitive + RealField + Debug>(
    intrinsics: &RosOpenCvIntrinsics<F>,
    points: &[Point2<F>],
) -> Vec<Point2<F>> {
    let x_mat = MatrixXx1::from_column_slice(&points.iter().map(|p| p.x as F).collect::<Vec<_>>());
    let y_mat = MatrixXx1::from_column_slice(&points.iter().map(|p| p.y as F).collect::<Vec<_>>());
    let points = MatrixXx2::from_columns(&[x_mat, y_mat]);
    let distorted = cam_geom::Pixels::new(points);
    let undistorted = intrinsics.undistort(&distorted);

    let x_vec = undistorted.data.as_slice()[..undistorted.data.len() / 2].to_vec();
    let y_vec = undistorted.data.as_slice()[undistorted.data.len() / 2..].to_vec();
    x_vec
        .into_iter()
        .zip(y_vec)
        .map(|(x, y)| Point2::new(x, y))
        .collect::<Vec<_>>()
}

#[cfg(feature = "std")]
pub fn ros_opencv_intrinsics_type_convert<A: simba::scalar::SubsetOf<B>, B: From<A>>(
    intrinsics: &RosOpenCvIntrinsics<A>,
) -> RosOpenCvIntrinsics<B>
where
    A: RealField + Copy + FromPrimitive,
    B: RealField + Copy + FromPrimitive,
{
    let _0 = B::from_f32(0.).unwrap();

    let fx = intrinsics.p.m11.into();
    let fy = intrinsics.p.m22.into();
    let cx = intrinsics.p.m13.into();
    let cy = intrinsics.p.m23.into();

    let distortion = Distortion::from_opencv_vec(intrinsics.distortion.opencv_vec().cast());

    RosOpenCvIntrinsics::from_params_with_distortion(fx, _0, fy, cx, cy, distortion)
}

#[cfg(feature = "std")]
/// Calculate a rotation to be applied to `p1` which minimizes the squared distances. In other
/// words, find `$\mathbf{R}$` which minimizes `$\sum^n_{i=1} ||\mathbf{R}\vec{p_{1,i}} -
/// \vec{p_{2,i}}||^2$`
pub fn calculate_rotational_offset(
    p1: &[Vector3<f32>],
    p2: &[Vector3<f32>],
) -> UnitQuaternion<f32> {
    // https://web.stanford.edu/class/cs273/refs/Absolute-OPT.pdf
    let m: Matrix3<f32> = std::iter::zip(p1, p2)
        .map(|(p1, p2)| p1 * p2.transpose())
        .sum();
    let [ // column major
        [sxx, syx, szx],
        [sxy, syy, szy],
        [sxz, syz, szz],
    ] = m.data.0;
    let n = nalgebra::matrix![
        sxx + syy + szz, syz - szy, szx - sxz, sxy - syx;
        syz - szy, sxx - syy - szz, sxy + syx, szx + sxz;
        szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy;
        sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz;
    ];
    let eigen = n.symmetric_eigen(); // TODO this could be replaced with a specialized routine
    let (_, [w, i, j, k]) =
        std::iter::zip(eigen.eigenvalues.iter().copied(), eigen.eigenvectors.data.0)
            .max_by(|(ev1, _), (ev2, _)| ev1.total_cmp(ev2))
            .unwrap();
    UnitQuaternion::from_quaternion(Quaternion::new(w, i, j, k))
}

#[derive(serde::Serialize, serde::Deserialize, Clone, Copy)]
pub struct Plane<F: Float + FromPrimitive + Scalar> {
    pub origin: nalgebra::Point3<F>,
    pub normal: nalgebra::Vector3<F>,
}

impl<F: Float + FromPrimitive + Scalar> Plane<F> {
    pub fn cast<G>(&self) -> Plane<G>
    where
        G: Float + FromPrimitive + Scalar + nalgebra::RealField,
        F: simba::scalar::SubsetOf<G>,
    {
        Plane {
            origin: self.origin.cast(),
            normal: self.normal.cast(),
        }
    }
}

/// `rotation` describes the rotation to transform coordinates in world space to physical screen
/// space.
///
/// `object_points` is the positions of the markers in meters in physical screen space, where the
/// physical screen lies on the XY plane.
///
/// `homography` is used to map from physical screen space to
/// the unit square, i.e. go from `(x, y)` in meters to `(x', y')` where `x', y' ∈ [0, 1]`.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct ScreenCalibration<F: Float + FromPrimitive + Scalar + nalgebra::RealField> {
    #[serde(default)]
    pub rotation: nalgebra::UnitQuaternion<F>,
    pub homography: nalgebra::Matrix3<F>,
    pub object_points: [nalgebra::Point3<F>; crate::foveated::MARKER_PATTERN_LEN],
}

impl<F: Float + FromPrimitive + Scalar + nalgebra::RealField> ScreenCalibration<F> {
    pub fn cast<G>(&self) -> ScreenCalibration<G>
    where
        G: Float + FromPrimitive + Scalar + nalgebra::RealField,
        F: simba::scalar::SubsetOf<G>,
    {
        ScreenCalibration {
            rotation: self.rotation.cast(),
            homography: self.homography.cast(),
            object_points: self.object_points.map(|point| point.cast()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_aimpoint() {
        let center_aim = Point2::new(0.5, 0.5);

        let p1 = Point2::new(0.5, 1.);
        let p2 = Point2::new(0., 0.5);
        let p3 = Point2::new(0.5, 0.);
        let p4 = Point2::new(1., 0.5);

        let result = transform_aim_point(
            center_aim,
            p1,
            p2,
            p3,
            p4,
            Point2::new(0.5, 1.),
            Point2::new(0., 0.5),
            Point2::new(0.5, 0.),
            Point2::new(1., 0.5),
        );

        assert_eq!(result, Some(Point2::new(0.5, 0.5)));
    }

    #[test]
    fn side_on_transform_aimpoint() {
        let r = transform_aim_point(
            Point2::new(1976., 808.),
            Point2::new(1768., 637.),
            Point2::new(353., 2583.),
            Point2::new(2834., 665.),
            Point2::new(4173., 2652.),
            Point2::new(0., 0.),
            Point2::new(8., 0.),
            Point2::new(0., 4.),
            Point2::new(8., 4.),
        )
        .unwrap();
        assert!(f32::abs(r.x - 2.) < 0.03);
        assert!(f32::abs(r.y - 1.) < 0.03);
    }

    #[test]
    fn columns() {
        let mut p = &mut [
            Point2::new(600., 3019.),
            Point2::new(653., 1442.),
            Point2::new(713., 799.),
            Point2::new(1481., 3082.),
            Point2::new(1529., 1479.),
            Point2::new(1599., 978.),
            Point2::new(2181., 1563.),
            Point2::new(2221., 869.),
            Point2::new(2231., 3154.),
            Point2::new(2927., 3168.),
            Point2::new(2946., 1555.),
            Point2::new(2967., 947.),
        ][..];
        let mut cnt = 0;
        loop {
            let (a, b) = split_column(p, 400.);
            assert_eq!(a.len(), 3, "{a:?}, {b:?}");
            p = b;
            cnt += 1;
            if p.is_empty() {
                break;
            }
        }
        assert_eq!(cnt, 4);
    }

    #[test]
    fn rectangle_screen_0() {
        let mut p = [
            Point2::new(713., 799.),
            Point2::new(1599., 978.),
            Point2::new(1529., 1479.),
            Point2::new(2927., 3168.),
            Point2::new(653., 1442.),
            Point2::new(2946., 1555.),
            Point2::new(2967., 947.),
            Point2::new(1481., 3082.),
            Point2::new(2221., 869.),
            Point2::new(600., 3019.),
            Point2::new(2181., 1563.),
            Point2::new(2231., 3154.),
        ];
        let x = choose_rectangle_markers(&mut p, 0, 300.).unwrap();
        assert_eq!(
            x,
            [
                Point2::new(653., 1442.),
                Point2::new(1529., 1479.),
                Point2::new(1481., 3082.),
                Point2::new(600., 3019.),
            ]
        );
    }

    #[test]
    fn rectangle_screen_1() {
        let mut p = [
            Point2::new(713., 799.),
            Point2::new(1599., 978.),
            Point2::new(1529., 1479.),
            Point2::new(2927., 3168.),
            Point2::new(653., 1442.),
            Point2::new(2946., 1555.),
            Point2::new(2967., 947.),
            Point2::new(1481., 3082.),
            Point2::new(2221., 869.),
            Point2::new(600., 3019.),
            Point2::new(2181., 1563.),
            Point2::new(2231., 3154.),
        ];
        let x = choose_rectangle_markers(&mut p, 1, 300.).unwrap();
        assert_eq!(
            x,
            [
                Point2::new(2946., 1555.),
                Point2::new(2181., 1563.),
                Point2::new(2231., 3154.),
                Point2::new(2927., 3168.),
            ]
        );
    }

    #[test]
    fn test_marker_3dpoints() {
        let point_ratios = [
            Point2::new(0.35, 0.),
            Point2::new(0.65, 0.),
            Point2::new(0.65, 1.),
            Point2::new(0.35, 1.),
        ];
        let aspect_ratio = 16.0 / 9.0;
        let screen_scale = 1.0;
        let points =
            calculate_marker_3dpoints(&point_ratios, aspect_ratio.into(), screen_scale.into());
        assert_eq!(
            points,
            [
                Vector3::new(0.35 * 16. / 9., 0., 0.),
                Vector3::new(0.65 * 16. / 9., 0., 0.),
                Vector3::new(0.65 * 16. / 9., 1., 0.),
                Vector3::new(0.35 * 16. / 9., 1., 0.),
            ]
        );
    }

    #[test]
    fn test_screen_3dpoints() {
        let screen_scale = 1.0;
        let screen_aspect_ratio = 16. / 9.;
        let points = calculate_screen_3dpoints(screen_scale.into(), screen_aspect_ratio.into());
        assert_eq!(
            points,
            [
                Vector3::new(0.0 * 16. / 9., 0., 0.),
                Vector3::new(1.0 * 16. / 9., 0., 0.),
                Vector3::new(1.0 * 16. / 9., 1., 0.),
                Vector3::new(0.0 * 16. / 9., 1., 0.),
            ]
        );
    }

    #[test]
    fn test_match_markers() {
        // Define some sample primary and secondary markers using nalgebra's Point2
        let primary_markers = [Point2::new(1.0, 1.0), Point2::new(5.0, 5.0)];
        let secondary_markers = [
            Point2::new(1.5, 1.5),
            Point2::new(4.5, 4.5),
            Point2::new(2.0, 2.0),
        ];

        // Expected results
        let expected = [
            Some(Point2::new(1.5, 1.5)), // Closest to primary_markers[0]
            Some(Point2::new(4.5, 4.5)), // Closest to primary_markers[1]
        ];

        // Run the match_markers function
        let result = match_markers(&primary_markers, &secondary_markers);

        // Assert each matched point is as expected
        assert_eq!(result.len(), expected.len());
        for (i, expected_point) in expected.iter().enumerate() {
            match expected_point {
                Some(point) => assert!(
                    (result[i].unwrap() - *point).norm() < 1e-6,
                    "Mismatch at index {}: expected {:?}, got {:?}",
                    i,
                    expected_point,
                    result[i]
                ),
                None => assert!(
                    result[i].is_none(),
                    "Expected None at index {}, got {:?}",
                    i,
                    result[i]
                ),
            }
        }
    }

    #[test]
    fn test_ocv_types() {
        let bytes: [u8; 170] = [
            2, 50, 0, 97, 26, 17, 67, 0, 0, 0, 0, 4, 171, 63, 66, 0, 0, 0, 0, 159, 74, 17, 67, 48,
            153, 69, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 15, 194, 80, 190, 230, 29, 15, 64,
            186, 243, 191, 59, 97, 239, 180, 186, 71, 174, 248, 192, 104, 93, 9, 66, 0, 0, 0, 0,
            70, 244, 66, 66, 0, 0, 0, 0, 88, 148, 9, 66, 43, 244, 71, 66, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 128, 63, 219, 26, 35, 61, 247, 144, 35, 189, 200, 121, 225, 57, 205, 8, 158, 186,
            95, 190, 173, 59, 37, 255, 127, 63, 120, 123, 167, 187, 187, 43, 61, 185, 233, 101,
            166, 59, 183, 24, 127, 63, 235, 151, 171, 189, 17, 99, 31, 58, 221, 150, 171, 61, 143,
            25, 127, 63, 155, 65, 63, 188, 92, 101, 170, 190, 35, 71, 73, 62, 171, 122, 205, 203,
            83, 43, 0,
        ];
        let bytes = &mut &bytes[3..];
        let nf_intrinsics = ocv_types::MinimalCameraCalibrationParams::parse(bytes).unwrap();
        assert_eq!(
            nf_intrinsics.camera_matrix.data,
            [
                145.10303635182407,
                0.,
                47.917007513463489,
                0.,
                145.29149528441428,
                49.399597700110256,
                0.0,
                0.0,
                1.0,
            ]
        );
        assert_eq!(
            nf_intrinsics.dist_coeffs.data,
            [
                -0.20386528104463086,
                2.2361997667805928,
                0.0058579118546963271,
                -0.0013804251043507982,
                -7.7712738787306455,
            ]
        );
        let wf_intrinsics = ocv_types::MinimalCameraCalibrationParams::parse(bytes).unwrap();
        assert_eq!(
            wf_intrinsics.camera_matrix.data,
            [
                34.34121647200962,
                0.,
                48.738547789766642,
                0.,
                34.394866762375322,
                49.988446965249153,
                0.0,
                0.0,
                1.0,
            ]
        );
        assert_eq!(
            wf_intrinsics.dist_coeffs.data,
            [
                0.039820534469617412,
                -0.039933169314557031,
                0.00043006078813049756,
                -0.0012057066028621883,
                0.0053022349797757964,
            ]
        );
        let stereo_iso = ocv_types::MinimalStereoCalibrationParams::parse(bytes).unwrap();
        assert_eq!(
            stereo_iso.r.data,
            [
                0.99998692169365289,
                -0.0051111539633757353,
                -0.00018040735794555248,
                0.0050780667355570033,
                0.99647084468055069,
                -0.083785851668757322,
                0.00060801306019016873,
                0.083783839771118418,
                0.99648376731049981,
            ]
        );
        assert_eq!(
            stereo_iso.t.data,
            [
                -0.011673356870756095,
                -0.33280456937540659,
                0.19656043337961257
            ]
        );
        let ros_opencv_nf_intrinsics: RosOpenCvIntrinsics<f32> = nf_intrinsics.into();
        assert_eq!(ros_opencv_nf_intrinsics.p[0], 145.10303635182407);
        let ros_opencv_nf_intrinsicsf32: RosOpenCvIntrinsics<f32> =
            ros_opencv_intrinsics_type_convert(&ros_opencv_nf_intrinsics);

        assert!(
            f32::abs(ros_opencv_nf_intrinsicsf32.p[0] - 145.10303635182407) < 1e-5,
            "Mismatch: {:?}",
            ros_opencv_nf_intrinsicsf32.p[0]
        );

        let back_into_nf_intrinsics: ocv_types::MinimalCameraCalibrationParams<f32> =
            ros_opencv_nf_intrinsics.into();
        assert!(
            f32::abs(back_into_nf_intrinsics.camera_matrix.data[0] - 145.10303635182407) < 1e-5,
            "Mismatch: {:?}",
            back_into_nf_intrinsics.camera_matrix.data[0]
        );

        let ros_opencv_nf_intrinsics: RosOpenCvIntrinsics<f32> = back_into_nf_intrinsics.into();
        assert!(
            f32::abs(ros_opencv_nf_intrinsics.p[0] - 145.10303635182407) < 1e-5,
            "Mismatch: {:?}",
            ros_opencv_nf_intrinsicsf32.p[0]
        );

        let mut buf = Vec::new();
        nf_intrinsics.serialize(&mut buf);

        let deserialized = ocv_types::MinimalCameraCalibrationParams::parse(&mut &buf[..]).unwrap();
        assert!(
            f32::abs(deserialized.camera_matrix.data[0] - 145.10303635182407) < 1e-5,
            "Mismatch: {:?}",
            deserialized.camera_matrix.data[0]
        );

        let ros_opencv_stereo_iso: Isometry3<f32> = stereo_iso.into();
        assert!(
            f32::abs(
                ros_opencv_stereo_iso
                    .rotation
                    .to_rotation_matrix()
                    .matrix()
                    .m11
                    - 0.99998692169365289
            ) < 1e-5,
            "Mismatch: {:?}",
            ros_opencv_stereo_iso
                .rotation
                .to_rotation_matrix()
                .matrix()
                .m11
        );
        assert!(
            f32::abs(
                ros_opencv_stereo_iso
                    .rotation
                    .to_rotation_matrix()
                    .matrix()
                    .m21
                    - 0.0050780667355570033
            ) < 1e-5,
            "Mismatch: {:?}",
            ros_opencv_stereo_iso
                .rotation
                .to_rotation_matrix()
                .matrix()
                .m21
        );

        let back_into_stereo_iso: ocv_types::MinimalStereoCalibrationParams<f32> =
            ros_opencv_stereo_iso.into();
        assert!(
            f32::abs(back_into_stereo_iso.r.data[0] - 0.99998692169365289) < 1e-5,
            "Mismatch: {:?}",
            back_into_stereo_iso.r.data[0]
        );
    }

    #[test]
    fn test_peak_detect() {
        let sample_count = 128;
        let sample_rate = 200.; // Sample rate in Hz
        let sample_interval = 1.0 / sample_rate as f32;
        let mut samples: [f32; 128] = [0.0; 128];

        let min_freq = 43.;
        let max_freq = 80.;

        let signal_freqs = [43.75, 50., 56.25, 62.5, 68.75, 75.];

        for signal_freq in signal_freqs.iter() {
            for i in 0..sample_count {
                samples[i] =
                    (2.0 * core::f32::consts::PI * signal_freq * sample_interval * i as f32).sin();
            }

            let result = peak_detect_128(&mut samples, min_freq, max_freq, sample_rate);
            assert!(f32::abs(result - *signal_freq) < 0.0001);
        }
    }
}
