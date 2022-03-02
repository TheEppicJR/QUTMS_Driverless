use nalgebra as na;
use na::{Vector3, Rotation3, Matrix3, Point};

pub struct TrackEdge {
    pub mut start: VertexType;
    pub mut end: VertexType;
}
mod edgefinder;

impl TrackEdge {
    pub fn new(start: VertexType, end: VertexType) -> Self {

        Self {
            start: start,
            end: end,

        }
    }
}

pub struct Track {
    pub mut left_hand: Vec<TrackEdge>;
    pub mut right_hand: Vec<TrackEdge>;
    pub mut right_start: VertexType;
    pub mut left_start: VertexType;

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