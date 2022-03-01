use spade::{DelaunayTriangulation, Triangulation, Point2};
use PointWithCovariance;
use nalgebra as na;
use na::{Vector3, Rotation3, Matrix3, Point};
use num_derive::FromPrimitive;    
use num_traits::FromPrimitive;

#[derive(FromPrimitive)]
enum PointColor{
    Blue = 0,
    Yellow = 1,
    BigOrange = 2,
    SmallOrange = 3,
    Unknown = 4,
}

enum EdgeType{
    LeftHand,
    RightHand,
    MiddleOfTrack,
    Unknown,
    Discarded,
    StartFinish,
}

enum EdgeColor{
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
}

enum TileType{
    Ontrack,
    Unknown,
    Offtrack,
}

pub struct VertexType {
    pub position: Point,
    pub position3: Point!,
    pub mut color: PointColor,
    pub mut covariance: Matrix3,
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
}

impl HasPosition for VertexType {
    type Scalar = f64;

    fn position(&self) -> Point2<f64> {
        Point2::new(self.position.x, self.position.y)
    }
}

pub struct UndirectedEdgeType {
    pub mut edgetype: EdgeType,
    pub mut edgecolor: EdgeColor,

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
            edgecolor: EdgeColor::U2U,
        }
    }
}

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

fn get_edges(points: Vec<PointWithCovariance>) -> Triangulation<_> {

    let mut triangulation: Triangulation = Triangulation::new();

    for point in &points{
        let x = point.position.x;
        let y = point.position.y;
        let z = point.position.z;
        let id = point.id;
        let color = PointColor::from_i8(point.color);
        let cov = Matrix3::from_row_slice(point.covariance);
        triangulation.insert(VertexType<x, y, z, color, id, cov>);
    }

}
    