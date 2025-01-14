use core::ops::ControlFlow;
use core::time::Duration;

use arrayvec::ArrayVec;
use nalgebra::{
    ComplexField, Isometry3, Point2, Point3,
    RealField, Scalar, UnitVector3,
    Vector2, Vector3,
};
use num::{Float, FromPrimitive};

pub const MAX_SCREEN_ID: u8 = 5;
pub const MARKER_PATTERN_LEN: usize = 6;

#[derive(Debug)]
pub struct FoveatedAimpointState {
    pub filter: eskf::ESKF,
    pub screen_id: u8,
}

impl FoveatedAimpointState {
    pub fn new() -> Self {
        let filter = eskf::Builder::new().build();
        Self {
            filter,
            screen_id: 0,
        }
    }

    pub fn reset(&mut self) {}

    pub fn predict(&mut self, _: Vector3<f32>, _: Vector3<f32>, _: Duration) {}

    pub fn observe_markers(
        &mut self,
        _: &[Marker],
        _: &[Marker],
        _: UnitVector3<f32>,
        _: &ArrayVec<
            (u8, crate::ScreenCalibration<f32>),
            { (MAX_SCREEN_ID + 1) as usize },
        >,
    ) -> bool {
        true
    }

    pub fn match_markers_from_eskf(
        &self,
        _: &[Marker],
        _: &[Marker],
        _: &[Point3<f32>; MARKER_PATTERN_LEN],
    ) -> MarkerMatching {
        let reprojection_match = MarkerMatching {
            nf: [None; MARKER_PATTERN_LEN],
            wf: [None; MARKER_PATTERN_LEN],
        };

        reprojection_match
    }

    pub fn init(&self) -> bool {
        false
    }
}

pub fn do_pnp(
    _: MarkerMatching,
    _: &[Marker],
    _: &[Marker],
    _: UnitVector3<f32>,
    _: &[Point3<f32>; MARKER_PATTERN_LEN],
) -> Option<Isometry3<f32>> {
    None
}

#[derive(Debug)]
pub struct Marker {
    pub position: Point2<f32>,
}

#[derive(Clone, Default, Debug)]
pub struct MarkerMatching {
    pub nf: [Option<usize>; MARKER_PATTERN_LEN],
    pub wf: [Option<usize>; MARKER_PATTERN_LEN],
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum MarkerMatchSource {
    Nf(usize),
    Wf(usize),
}

pub fn identify_markers_nf_wf2(
    _: &[Marker],
    _: &[Marker],
    _: UnitVector3<f32>,
    _: &[(u8, crate::ScreenCalibration<f32>)],
) -> (MarkerMatching, Option<u8>) {
    (MarkerMatching::default(), None)
}

pub fn identify_markers(
    _: &[Point2<f32>],
    _: UnitVector3<f32>,
    _: &[(u8, crate::ScreenCalibration<f32>)],
) -> Option<(
    [usize; MARKER_PATTERN_LEN],
    [Vector2<f32>; MARKER_PATTERN_LEN],
    u8,
)> {
    None
}

pub fn identify_markers_from_screen_calibration(
    _: &[Point2<f32>],
    _: UnitVector3<f32>,
    _: &crate::ScreenCalibration<f32>,
) -> Option<(
    [usize; MARKER_PATTERN_LEN],
    [Vector2<f32>; MARKER_PATTERN_LEN],
    f32,
)> {
    None
}

pub fn match2<F>(_: &[Point2<F>], _: &[Vector2<F>]) -> ([Option<usize>; 16], F)
where
    F: Float + Scalar + std::ops::SubAssign + ComplexField + RealField,
{
    ([None; 16], F::from(0).unwrap())
}

pub fn match3<const N: usize, F>(
    _: &[Point2<F>],
    _: &[Vector2<F>; N],
) -> ([Option<usize>; N], F)
where
    F: Float + Scalar + std::ops::SubAssign + ComplexField + RealField,
{
    ([None; N], F::from(0).unwrap())
}

pub fn visit_collinear<F, G>(_: &[Point2<F>], _: G)
where
    F: Float + FromPrimitive + ComplexField + RealField,
    G: FnMut(&[Point2<F>]) -> ControlFlow<()>,
{
}

pub fn visit_collinear_generic<F, G, const CAP: usize>(
    _: &[Point2<F>],
    _: G,
    _: F,
) where
    F: Float + FromPrimitive + ComplexField + RealField,
    G: FnMut(&[Point2<F>]) -> ControlFlow<()>,
{
}
