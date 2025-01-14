use nalgebra::{Isometry3, Matrix3, Point2, Rotation3, Translation3, UnitQuaternion, Vector3};

pub fn get_raycast_aimpoint(
    _: &crate::foveated::FoveatedAimpointState,
    _: &crate::ScreenCalibration<f32>,
    _: Option<Isometry3<f32>>
) -> (
    Rotation3<f32>,
    Translation3<f32>,
    Option<(Point2<f32>, f32)>,
) {
    (
        Rotation3::identity(),
        Translation3::identity(),
        None,
    )
}

pub fn calculate_zero_offset_quat(
    _: Translation3<f32>,
    _: Point2<f32>,
    _: &arrayvec::ArrayVec<
        (u8, crate::ScreenCalibration<f32>),
        { (crate::foveated::MAX_SCREEN_ID + 1) as usize },
    >,
    _: &crate::foveated::FoveatedAimpointState,
) -> Option<UnitQuaternion<f32>> {
    None
}

pub fn raycast_update(
    _: &arrayvec::ArrayVec<
        (u8, crate::ScreenCalibration<f32>),
        { (crate::foveated::MAX_SCREEN_ID + 1) as usize },
    >,
    _: &mut crate::foveated::FoveatedAimpointState,
    _: Option<Isometry3<f32>>,
) -> (
    Option<(Matrix3<f32>, Vector3<f32>)>,
    Option<(Point2<f32>, f32)>,
) {
    (None, None)
}

pub fn get_screen_calibration(
    _: &arrayvec::ArrayVec<
        (u8, crate::ScreenCalibration<f32>),
        { (crate::foveated::MAX_SCREEN_ID + 1) as usize },
    >,
    _: u8,
) -> Option<&crate::ScreenCalibration<f32>> {
    None
}
