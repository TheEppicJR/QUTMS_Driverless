use nalgebra as na;
use spade::{DelaunayTriangulation, Triangulation, Point2, PositionInTriangulation, DirectedEdgeHandle, HasPosition};
use spade::handles::{FixedFaceHandle, FixedDirectedEdgeHandle, VertexHandle, UndirectedEdgeHandle};
use na::{Vector3, Rotation3, Matrix3, Point};
use T;
use num_derive::FromPrimitive;    
use num_traits::FromPrimitive;

#[derive(FromPrimitive)]
pub enum PointColor{
    Blue = 0,
    Yellow = 1,
    BigOrange = 2,
    SmallOrange = 3,
    Unknown = 4,
}

pub enum EdgeType{
    LeftHand,
    RightHand,
    MiddleOfTrack,
    Unknown,
    Discarded,
    StartFinish,
}

pub enum EdgeColor{
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

pub enum TileType{
    Ontrack,
    Unknown,
    Offtrack,
}

pub struct TrackEdge {
    pub start: VertexType;
    pub end: VertexType;
}

impl TrackEdge {
    pub fn new(start: VertexType, end: VertexType) -> Self {

        Self {
            start: start,
            end: end,

        }
    }
}

pub struct Track {
    pub left_hand: Vec<TrackEdge>;
    pub right_hand: Vec<TrackEdge>;
    pub right_start: VertexType;
    pub left_start: VertexType;

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

pub struct VertexType {
    pub position: Point,
    pub position3: Point!,
    pub color: PointColor,
    pub covariance: Matrix3,
}

impl VertexType {
    pub fn new(x: f64, y: f64, z: f64, color: PointColor, id: u32, cov: Matrix3) -> Self {

        Self {
            position: Point::new(x, y),
            position3: Point![x, y, z],
            color: color,
            covariance: cov,

        }
    }
    pub fn empty() -> Self {

        Self {
            position: Point::new(0.0, 0.0);
            position3d: Point![0.0, 0.0, 0.0],
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

pub struct UndirectedEdgeType {
    pub edgetype: EdgeType,
    pub edgecolor: EdgeColor,

}

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
#[derive(Default)]
pub struct DirectedEdgeType {}

#[derive(Clone, Copy, Debug)]
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

pub type Triangulation = DelaunayTriangulation<VertexType, DirectedEdgeType, UndirectedEdgeType, FaceType>;

pub fn get_edge_color(p1: &VertexHandle, p2: &VertexHandle) -> EdgeColor {
    match p1.data().color {
        PointColor::Unknown => {
            match p2.data().color {
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
            }
        }
        PointColor::Yellow => {
            match p2.data().color {
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
            }
        }
        PointColor::Blue => {
            match p2.data().color {
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
            }
        }
        PointColor::BigOrange | PointColor::SmallOrange => {
            match p2.data().color {
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
            }
        }
    }
}