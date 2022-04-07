use na::{Matrix3, Point};
use nalgebra as na;
use num_derive::FromPrimitive;
use spade::handles::{DirectedEdgeHandle, UndirectedEdgeHandle, VertexHandle};
use spade::{DelaunayTriangulation, HasPosition, Point2};

#[derive(FromPrimitive, Debug, PartialEq, Copy, Clone)]
pub enum PointColor {
    Blue = 0,
    Yellow = 1,
    BigOrange = 2,
    SmallOrange = 3,
    Unknown = 4,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum EdgeType {
    LeftHand,
    RightHand,
    MiddleOfTrack,
    Unknown,
    Discarded,
    StartFinish,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum EdgeColor {
    B2B,
    Y2Y,
    O2O,
    B2Y,
    B2O,
    Y2O,
    B2U,
    Y2U,
    O2U,
    U2U,
    Unknown,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum TileType {
    Ontrack,
    Unknown,
    Offtrack,
}

pub type PointF2 = Point2<f64>;
pub type PointF3 = Point<f64, 3>;
pub type PointF2M = Point<f64, 2>;

#[derive(Debug, Clone, Copy)]
pub struct TrackEdge {
    pub start: VertexType,
    pub end: VertexType,
}

impl TrackEdge {
    pub fn new(start: VertexType, end: VertexType) -> Self {
        Self {
            start: start,
            end: end,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Track {
    pub left_hand: Vec<TrackEdge>,
    pub right_hand: Vec<TrackEdge>,
    pub right_start: VertexType,
    pub left_start: VertexType,
}

impl Track {
    pub fn new() -> Self {
        Self {
            right_start: VertexType::empty(),
            left_start: VertexType::empty(),
            left_hand: Vec::new(),
            right_hand: Vec::new(),
        }
    }
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub struct VertexType {
    pub position: PointF2,
    pub position2: PointF2M,
    pub position3: PointF3,
    pub color: PointColor,
    pub covariance: Matrix3<f64>,
}

impl VertexType {
    pub fn new(x: f64, y: f64, z: f64, color: PointColor, _id: u32, cov: Matrix3<f64>) -> Self {
        Self {
            position: PointF2::new(x, y),
            position2: PointF2M::new(x, y),
            position3: PointF3::new(x, y, z),
            color: color,
            covariance: cov,
        }
    }
    pub fn empty() -> Self {
        Self {
            position: PointF2::new(0.0, 0.0),
            position2: PointF2M::new(0.0, 0.0),
            position3: PointF3::new(0.0, 0.0, 0.0),
            color: PointColor::Unknown,
            covariance: Matrix3::default(),
        }
    }
    pub fn two_d_only(point: PointF2M) -> Self {
        Self {
            position: PointF2::new(point.x, point.y),
            position2: point,
            position3: PointF3::new(point.x, point.y, 0.0),
            color: PointColor::Unknown,
            covariance: Matrix3::default(),
        }
    }
}

impl HasPosition for VertexType {
    type Scalar = f64;

    fn position(&self) -> Point2<f64> {
        Point2::new(self.position.x, self.position.y)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct UndirectedEdgeType {
    pub edgetype: EdgeType,
    pub edgecolor: EdgeColor,
}

// this is from the example and it probably dosent need to be here
impl AsRef<UndirectedEdgeType> for UndirectedEdgeType {
    fn as_ref(&self) -> &UndirectedEdgeType {
        self
    }
}

impl Default for UndirectedEdgeType {
    fn default() -> Self {
        Self {
            edgetype: EdgeType::Unknown,
            edgecolor: EdgeColor::Unknown,
        }
    }
}

// should make this have a enum to declare if it is a side of the track
// then the RRT can see not only if the tree has crossed the bounds of the track but see if it is entering or eziting the track
#[derive(Default, Debug, Clone, Copy)]
pub struct DirectedEdgeType {}

#[derive(Debug, Clone, Copy)]
pub struct FaceType {
    pub tiletype: TileType,
}

impl Default for FaceType {
    fn default() -> Self {
        Self {
            tiletype: TileType::Unknown,
        }
    }
}


pub type Triangulation =
    DelaunayTriangulation<VertexType, DirectedEdgeType, UndirectedEdgeType, FaceType>;

pub type VertexHandleType<'a> =
    VertexHandle<'a, VertexType, DirectedEdgeType, UndirectedEdgeType, FaceType>;

pub type UndirectedEdgeHandleType<'a> =
    UndirectedEdgeHandle<'a, VertexType, DirectedEdgeType, UndirectedEdgeType, FaceType>;

pub type DirectedEdgeHandleType<'a> =
    DirectedEdgeHandle<'a, VertexType, DirectedEdgeType, UndirectedEdgeType, FaceType>;

pub fn get_edge_color(p1: &VertexHandleType, p2: &VertexHandleType) -> EdgeColor {
    match p1.data().color {
        PointColor::Unknown => match p2.data().color {
            PointColor::Unknown => {
                return EdgeColor::U2U;
            }
            PointColor::Blue => {
                return EdgeColor::B2U;
            }
            PointColor::Yellow => {
                return EdgeColor::Y2U;
            }
            PointColor::BigOrange | PointColor::SmallOrange => {
                return EdgeColor::O2U;
            }
        },
        PointColor::Yellow => match p2.data().color {
            PointColor::Unknown => {
                return EdgeColor::Y2U;
            }
            PointColor::Blue => {
                return EdgeColor::B2Y;
            }
            PointColor::Yellow => {
                return EdgeColor::Y2Y;
            }
            PointColor::BigOrange | PointColor::SmallOrange => {
                return EdgeColor::Y2O;
            }
        },
        PointColor::Blue => match p2.data().color {
            PointColor::Unknown => {
                return EdgeColor::B2U;
            }
            PointColor::Blue => {
                return EdgeColor::B2B;
            }
            PointColor::Yellow => {
                return EdgeColor::B2Y;
            }
            PointColor::BigOrange | PointColor::SmallOrange => {
                return EdgeColor::B2O;
            }
        },
        PointColor::BigOrange | PointColor::SmallOrange => match p2.data().color {
            PointColor::Unknown => {
                return EdgeColor::O2U;
            }
            PointColor::Blue => {
                return EdgeColor::B2O;
            }
            PointColor::Yellow => {
                return EdgeColor::Y2O;
            }
            PointColor::BigOrange | PointColor::SmallOrange => {
                return EdgeColor::O2O;
            }
        },
    }
}
