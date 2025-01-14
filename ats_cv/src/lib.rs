#![cfg_attr(all(not(feature = "std"), not(test)), no_std)]

use core::fmt::Debug;
use nalgebra::{
    ComplexField, Isometry3, Point2, Point3, RealField, Rotation3,
    SMatrix, Scalar, UnitQuaternion, Vector2, Vector3,
};
#[cfg(feature = "std")]
use opencv_ros_camera::RosOpenCvIntrinsics;

#[cfg(feature = "std")]
pub mod foveated;
#[cfg(feature = "std")]
pub mod helpers;

use num::{
    traits::{float::TotalOrder, Float},
    FromPrimitive,
};

#[cfg(feature = "std")]
use sqpnp::types::SQPSolution;

#[cfg(feature = "std")]
pub fn pnp(_: &[Vector3<f32>], _: &[Vector2<f32>]) -> Option<SQPSolution> {
    Default::default()
}

pub fn transform_aim_point<F: Float + FromPrimitive>(
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
) -> Option<Point2<F>>
where
    F: ComplexField,
    F: RealField,
{
    Default::default()
}

pub fn get_perspective_transform<F: Float + FromPrimitive>(
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
    _: Point2<F>,
) -> Option<SMatrix<F, 3, 3>>
where
    F: ComplexField,
    F: RealField,
{
    Default::default()
}

pub fn choose_rectangle_markers<F: Float + FromPrimitive>(
    _: &mut [Point2<F>],
    _: u8,
    _: F,
) -> Option<[Point2<F>; 4]>
where
    F: ComplexField + RealField + TotalOrder,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn calculate_screen_id<F: Float + FromPrimitive>(_: &[Point2<F>]) -> Option<u8>
where
    F: ComplexField,
    F: RealField,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn mot_rotate<F: Float + FromPrimitive + Debug>(_: &[Point2<F>], _: F) -> Vec<Point2<F>>
where
    F: ComplexField + RealField,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn calculate_projections<F: Float + FromPrimitive + RealField>(
    _: &[Point2<F>],
    _: Vector2<F>,
    _: Vector2<F>,
) -> Vec<Vector2<F>> {
    Default::default()
}

#[cfg(feature = "std")]
pub fn marker_pnp<F: Float + FromPrimitive + RealField + ComplexField>(
    _: &[Vector3<F>],
    _: &[Vector2<F>],
    _: Option<&[F]>,
    _: Option<&SQPSolution>,
) -> Option<SQPSolution>
where
    F: ComplexField + RealField,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn intersection_to_screen_space<F: Float>(
    _: Point3<F>,
    _: &crate::ScreenCalibration<F>,
) -> Point2<F>
where
    F: ComplexField,
    F: RealField,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn calculate_marker_3dpoints<F: Float + FromPrimitive>(
    _: &[Point2<F>],
    _: F,
    _: F,
) -> Vec<Vector3<F>>
where
    F: ComplexField + RealField,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn calculate_screen_3dpoints<F: Float + FromPrimitive + RealField + 'static>(
    _: F,
    _: F,
) -> [Vector3<F>; 4] {
    let _0 = F::zero();
    [
        Vector3::new(_0, _0, _0),
        Vector3::new(_0, _0, _0),
        Vector3::new(_0, _0, _0),
        Vector3::new(_0, _0, _0),
    ]
}

#[cfg(feature = "std")]
pub fn solve_pnp_with_dynamic_screen_points<
    F: Float + FromPrimitive + RealField + ComplexField + Debug,
>(
    _: &[Vector2<F>],
    _: &[Point2<F>],
    _: F,
    _: F,
) -> Option<SQPSolution>
where
    F: ComplexField + RealField,
{
    Default::default()
}

#[cfg(feature = "std")]
pub fn ray_plane_intersection<F: Float + FromPrimitive + Scalar + Debug + 'static>(
    _: &Vector3<F>,
    _: &Vector3<F>,
    _: &Vector3<F>,
    _: &Point3<F>,
) -> (Point3<F>, F)
where
    F: ComplexField,
    F: RealField,
{
    (Default::default(), F::from_f32(0.).unwrap())
}

#[cfg(feature = "std")]
pub fn calculate_aimpoint<F: Float + FromPrimitive + RealField>(
    _: &nalgebra::Isometry<F, Rotation3<F>, 3>,
    _: &ScreenCalibration<F>,
) -> Option<Point2<F>> {
    Default::default()
}

#[cfg(feature = "std")]
pub fn calculate_aimpoint_and_distance<F: Float + FromPrimitive + RealField>(
    _: &nalgebra::Isometry<F, Rotation3<F>, 3>,
    _: &ScreenCalibration<F>,
) -> Option<(Point2<F>, F)> {
    Default::default()
}

#[cfg(feature = "std")]
pub fn match_markers<F: Float + FromPrimitive + Scalar>(
    _: &[Point2<F>],
    _: &[Point2<F>],
) -> Vec<Option<Point2<F>>> {
    Default::default()
}

#[cfg(feature = "std")]
pub fn wf_to_nf_points<F: Float + FromPrimitive + 'static>(
    _: &[Point2<F>],
    _: &RosOpenCvIntrinsics<F>,
    _: &RosOpenCvIntrinsics<F>,
    _: Isometry3<F>,
) -> Vec<Point2<F>>
where
    F: ComplexField,
    F: RealField,
    F: TotalOrder,
{
    Default::default()
}

pub fn wf_to_nf_point<F: Float + FromPrimitive + 'static>(
    _: Point2<F>,
    _: &RosOpenCvIntrinsics<F>,
    _: &RosOpenCvIntrinsics<F>,
    _: &Isometry3<F>,
) -> Point2<F>
where
    F: ComplexField,
    F: RealField,
    F: TotalOrder,
{
    Default::default()
}

pub fn to_normalized_image_coordinates<F>(
    _: Point2<F>,
    _: &RosOpenCvIntrinsics<F>,
    _: Option<&Isometry3<F>>,
) -> Point2<F>
where
    F: Float + FromPrimitive + RealField + 'static,
{
    Default::default()
}

pub fn peak_detect_128(
    _: &mut [f32; 128],
    _: f32,
    _: f32,
    _: f32,
) -> f32 {
    0.
}

#[cfg(feature = "std")]
pub fn calculate_rotational_offset(
    _: &[Vector3<f32>],
    _: &[Vector3<f32>],
) -> UnitQuaternion<f32> {
    Default::default()
}

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

#[cfg(feature = "std")]
pub fn undistort_points<F: Float + FromPrimitive + RealField + Debug>(
    _: &RosOpenCvIntrinsics<F>,
    _: &[Point2<F>],
) -> Vec<Point2<F>> {
    Default::default()
}
