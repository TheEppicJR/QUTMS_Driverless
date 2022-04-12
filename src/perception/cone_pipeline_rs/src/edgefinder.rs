use super::pathperimitives;
use na::Matrix3;
use nalgebra as na;
use num_traits::FromPrimitive;
use pathperimitives::{EdgeColor, PointColor, PointF2, TileType, Track, Triangulation, VertexType};
use r2r::driverless_msgs::msg::PointWithCovariance;
use spade::handles::{
    FixedDirectedEdgeHandle, FixedFaceHandle, FixedUndirectedEdgeHandle, InnerTag,
};
use spade::{PositionInTriangulation, Triangulation as OtherTriangulation};

pub fn get_edges(points: Vec<PointWithCovariance>) -> Triangulation {
    let mut triangulation: Triangulation = Triangulation::new();

    for point in &points {
        let x = point.position.x;
        let y = point.position.y;
        let z = point.position.z;
        let id = point.id;
        let mut color = PointColor::Unknown;
        if let Some(colormsg) = PointColor::from_i8(point.color.try_into().unwrap()) {
            color = colormsg;
        }
        let cov = Matrix3::from_row_slice(&point.covariance);
        let f = triangulation.insert(VertexType::new(x, y, z, color, id, cov));
        match f {
            Ok(_) => {}
            Err(e) => {
                println!("{:?}", e);
            }
        }
    }
    return triangulation;
}

pub fn color_edges(triangulation_obj: &mut Triangulation) -> Option<f64> {
    for edgehandle in triangulation_obj.fixed_directed_edges() {
        color_edge(triangulation_obj, &edgehandle);
    }

    None
}

pub fn find_track_edges(
    triangulation_obj: &mut Triangulation,
    pathpoints: &Vec<PointF2>,
    _track: &Track,
) -> Option<f64> {
    if triangulation_obj.num_vertices() > 4 {
        color_edges(triangulation_obj);
        // For now im not going to use the Track object to maintain the state
        // ie its going to be stateless for now, just wipe Track and put in new shit every time
        // to make it stateless we have to go through and check for points in areas that are relivant and see if they changed and I dont wanna do that rn
        // Really I should be adding and removing points here so I dont have to iterate through the edges every time
        for pathpoint in pathpoints.iter() {
            // get whatever vertex, edge, or triangle the point is in/on
            let pointpos = triangulation_obj.locate(pathpoint.clone());
            // now use one of the analysis methods to start to fill in our understanding of the track
            // we only care if the point is on a line or in a triangle
            match pointpos {
                PositionInTriangulation::OnEdge(edgehandle) => {
                    analyse_edge(triangulation_obj, &edgehandle);
                }
                PositionInTriangulation::OnFace(facehandle) => {
                    analyse_face(triangulation_obj, &facehandle);
                }
                PositionInTriangulation::NoTriangulation => {
                    // Need to return blank shit
                }
                PositionInTriangulation::OnVertex(_vertexhandle) => {
                    // Need to figure out what to do in this case
                }
                PositionInTriangulation::OutsideOfConvexHull(_idkwhatthisis) => {
                    // Need to figure out what to do in this case
                }
            }
        }
    }
    None
}

fn analyse_face(
    triangulation_obj: &mut Triangulation,
    fixed_facehandle: &FixedFaceHandle<InnerTag>,
) -> Option<f64> {
    let facehandle = triangulation_obj.face(fixed_facehandle.clone());
    let facehandle_fixed = facehandle.fix();
    // get handdels for all the edges
    let edge_handle_arr = facehandle.adjacent_edges();
    // get fixed refrences for all those handles so we arent mutably borrowing them
    let edges_mut: [FixedUndirectedEdgeHandle; 3] = [
        edge_handle_arr[0].as_undirected().fix(),
        edge_handle_arr[1].as_undirected().fix(),
        edge_handle_arr[2].as_undirected().fix(),
    ];
    // This code is obfuscated to make it more 'readable'
    // I would like to do (x[0] == x[1] and x[0] == x[2]) and (x[0] == B or x[0] == Y) but that would mutably borrow more than one at a time, so all I can do is compaire them one by one to the ENUM
    if (triangulation_obj
        .undirected_edge_data_mut(edges_mut[0])
        .edgecolor
        == EdgeColor::B2B
        && triangulation_obj
            .undirected_edge_data_mut(edges_mut[1])
            .edgecolor
            == EdgeColor::B2B
        && triangulation_obj
            .undirected_edge_data_mut(edges_mut[0])
            .edgecolor
            == EdgeColor::B2B)
        || (triangulation_obj
            .undirected_edge_data_mut(edges_mut[0])
            .edgecolor
            == EdgeColor::Y2Y
            && triangulation_obj
                .undirected_edge_data_mut(edges_mut[1])
                .edgecolor
                == EdgeColor::Y2Y
            && triangulation_obj
                .undirected_edge_data_mut(edges_mut[0])
                .edgecolor
                == EdgeColor::Y2Y)
    {
        let mut face_data_mut = triangulation_obj.face_data_mut(facehandle_fixed);
        face_data_mut.tiletype = TileType::Offtrack;
        return None;
    }

    None
}

fn analyse_edge(
    triangulation_obj: &mut Triangulation,
    edgehandle: &FixedDirectedEdgeHandle,
) -> Option<f64> {
    let mut_edge = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected());
    // Now depending on color of the edge we can do diffrent things
    match mut_edge.edgecolor {
        EdgeColor::B2Y => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::B2O | EdgeColor::B2U => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::Y2O | EdgeColor::Y2U => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::Y2Y => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::B2B => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::O2O => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::U2U => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::O2U => {
            let _whatever = VertexType::empty();
        }
        EdgeColor::Unknown => {
            let _whatever = VertexType::empty();
        }
    }

    // let first_face_handle = edgehandle.face();
    // let second_face_handle = edgehandle.rev().face();
    None
}

fn color_edge(
    triangulation_obj: &mut Triangulation,
    edgehandle: &FixedDirectedEdgeHandle,
) -> Option<f64> {
    let edge = triangulation_obj.directed_edge(*edgehandle); // idk what the * does
    let p1 = edge.from();
    let p2 = edge.to();
    let edgecolor = pathperimitives::get_edge_color(&p1, &p2);
    let mut edge_mut = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected());
    edge_mut.edgecolor = edgecolor;

    None
}
