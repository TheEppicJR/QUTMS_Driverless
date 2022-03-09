use r2r;
use r2r::driverless_msgs::msg::PointWithCovariance;
use r2r::driverless_msgs::msg::PointWithCovarianceArrayStamped;
use r2r::visualization_msgs::msg::{Marker, MarkerArray}
use r2r::geometry_msgs::msg::{Point}
use r2r::std_msgs::msg::{ColorRGBA}
use r2r::nav_msgs::msg::Odometry;
use r2r::QosProfile;
use nalgebra as na;
mod edgefinder;
mod pathperimitives;

pub fn generate_edges(triangulation_obj: &Triangulation) -> Marker {
    let mut marker_points: Vec<Point>;
    let mut marker_colors: Vec<ColorRGBA>;
    for edge in triangulation_obj.undirected_edges() {
        let edge_type = edge.data().edgetype;
        // Depending on the type of edge it has been determined we want to shade it diffrently, not totally sure what I want to do with this yet
        // match edge_type {
        //     EdgeType::LeftHand | EdgeType::RightHand {

        //     }
        // }
        for point in edge.vertices() {
            let color_rgba = get_color_RGBA(&point.data);
            let point = get_point_msg(&point.position)
        }
    }
}

fn get_point_msg(point: &PointF2) -> Point {
    return Point { x: point.x, y: point.y }
}

fn get_color_RGBA(conevertex: &VertexType) -> ColorRGBA {
    match conevertex.color {
        PointColor::Blue => {
            return ColorRGBA { r: 0.0, g: 0.0, b: 1.0, a: 1.0}
        }
        PointColor::Yellow => {
            return ColorRGBA { r: 1.0, g: 1.0, b: 0.0, a: 1.0}
        }
        PointColor::SmallOrange | PointColor::BigOrange => {
            return ColorRGBA { r: 1.0, g: 0.7, b: 0.0, a: 1.0}
        }
        PointColor::Unknown => {
            return ColorRGBA { r: 0.5, g: 0.5, b: 0.5, a: 1.0}
        }
    }
}