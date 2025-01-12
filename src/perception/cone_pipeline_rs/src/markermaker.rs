use super::pathperimitives;
use super::rrt;
use colorgrad::spectral;
use pathperimitives::{PointColor, PointF2, Triangulation, VertexType, PointF2M};
use r2r;
use r2r::geometry_msgs::msg::Point;
use r2r::std_msgs::msg::ColorRGBA;
use r2r::visualization_msgs::msg::Marker;
use rrt::RRTNode;
use spade::Triangulation as OtherTriangulation;

pub fn generate_edges(
    triangulation_obj: &Triangulation,
) -> Result<Marker, Box<dyn std::error::Error>> {
    let mut marker_points: Vec<Point> = Vec::new();
    let mut marker_colors: Vec<ColorRGBA> = Vec::new();
    for edge in triangulation_obj.undirected_edges() {
        // let edge_type = &edge.data().edgetype;
        // Depending on the type of edge it has been determined we want to shade it diffrently, not totally sure what I want to do with this yet
        // match edge_type {
        //     EdgeType::LeftHand | EdgeType::RightHand {

        //     }
        // }
        for point in edge.vertices() {
            let color_rgba = get_color_rgba(&point.data());
            let point = get_point_msg_sp(&point.position());
            marker_points.push(point);
            marker_colors.push(color_rgba);
        }
    }

    let mut edge_marker = Marker::default();
    edge_marker.header.frame_id = "ekf/map".to_string();
    edge_marker.header.stamp = get_time()?;
    edge_marker.points = marker_points;
    edge_marker.colors = marker_colors;
    return Ok(edge_marker);
}

pub fn generate_nodes(
    nodes: &Vec<RRTNode>,
    maxcost: &f64,
) -> Result<Marker, Box<dyn std::error::Error>> {
    let mut marker_points: Vec<Point> = Vec::new();
    let mut marker_colors: Vec<ColorRGBA> = Vec::new();
    for node in nodes {
        match node.parent {
            Some(parent) => {
                let color_rgba = get_color_gradiant(&node.cost, maxcost);
                let point = get_point_msg(&node.point);
                marker_points.push(point);
                marker_colors.push(color_rgba);
                let color_rgba_parent = get_color_gradiant(&nodes[parent].cost, maxcost);
                let point_parent = get_point_msg(&nodes[parent].point);
                marker_points.push(point_parent);
                marker_colors.push(color_rgba_parent);
            }
            None => {
                let _color_rgba = 0;
            }
        }
    }
    let mut node_marker = Marker::default();
    node_marker.header.frame_id = "ekf/map".to_string();
    node_marker.header.stamp = get_time()?;
    node_marker.points = marker_points;
    node_marker.colors = marker_colors;
    return Ok(node_marker);
}

fn get_time() -> Result<r2r::builtin_interfaces::msg::Time, Box<dyn std::error::Error>> {
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;
    let now = clock.get_now()?;
    return Ok(r2r::Clock::to_builtin_time(&now));
}

fn get_point_msg(point: &PointF2M) -> Point {
    return Point {
        x: point.x,
        y: point.y,
        z: 0.0,
    };
}

fn get_point_msg_sp(point: &PointF2) -> Point {
    return Point {
        x: point.x,
        y: point.y,
        z: 0.0,
    };
}

fn get_color_gradiant(shade: &f64, scale: &f64) -> ColorRGBA {
    let mut color_rgba = ColorRGBA::default();
    let grad = spectral();
    let (r, g, b, a) = grad.at((shade / scale).into()).rgba_u8();
    color_rgba.r = r.into();
    color_rgba.g = g.into();
    color_rgba.b = b.into();
    color_rgba.a = a.into();
    return color_rgba;
}

fn get_color_rgba(conevertex: &VertexType) -> ColorRGBA {
    match conevertex.color {
        PointColor::Blue => {
            return ColorRGBA {
                r: 0.0,
                g: 0.0,
                b: 1.0,
                a: 1.0,
            }
        }
        PointColor::Yellow => {
            return ColorRGBA {
                r: 1.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            }
        }
        PointColor::SmallOrange | PointColor::BigOrange => {
            return ColorRGBA {
                r: 1.0,
                g: 0.7,
                b: 0.0,
                a: 1.0,
            }
        }
        PointColor::Unknown => {
            return ColorRGBA {
                r: 0.5,
                g: 0.5,
                b: 0.5,
                a: 1.0,
            }
        }
    }
}
