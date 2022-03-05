use spade::{PositionInTriangulation, Triangulation as OtherTriangulation};
use spade::handles::{FixedFaceHandle, PossiblyOuterTag, InnerTag};
use r2r::driverless_msgs::msg::PointWithCovariance;
use nalgebra as na;
use na::{Matrix3};
use super::pathperimitives;
use pathperimitives::{Triangulation, EdgeColor, Track, DirectedEdgeHandleType, PointColor, VertexType, PointF2};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

pub fn get_edges(points: Vec<PointWithCovariance>) -> Triangulation {

    let mut triangulation: Triangulation = Triangulation::new();

    for point in &points{
        let x = point.position.x;
        let y = point.position.y;
        let z = point.position.z;
        let id = point.id;
        let mut color = PointColor::Unknown;
        if let Some(colormsg) = PointColor::from_i8(point.color.try_into().unwrap()) {
            color = colormsg;
        }
        let cov = Matrix3::from_row_slice(&point.covariance);
        triangulation.insert(VertexType::new(x, y, z, color, id, cov));
    }
    return triangulation;

}

pub fn find_track_edges(triangulation_obj: &mut Triangulation, pathpoints: &Vec<PointF2>, track: &Track) -> Option<f64> {
    if triangulation_obj.num_vertices() > 4 {
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
                    let edge = triangulation_obj.directed_edge(edgehandle);
                    analyse_edge(triangulation_obj, &edge);
                }
                PositionInTriangulation::OnFace(facehandle) => {
                    analyse_face(triangulation_obj, &facehandle);
                }

            }
        }
    }
    
    None
}

fn analyse_face(triangulation_obj: &mut Triangulation, fixed_facehandle: &FixedFaceHandle<InnerTag>) -> Option<f64> {
    let facehandle = triangulation_obj.face(fixed_facehandle.clone());
    for edgehandle in facehandle.adjacent_edges() {
        let mut_edge = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected().fix());
        if let EdgeColor::Unknown = mut_edge.edgecolor {
            color_edge(triangulation_obj, &edgehandle);
        }
    }

    None
}

fn analyse_edge(triangulation_obj: &mut Triangulation, edgehandle: &DirectedEdgeHandleType) -> Option<f64> {
    let mut_edge = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected().fix());
    // If we havent figured out the color of the edge then figure that shit out
    if let EdgeColor::Unknown = mut_edge.edgecolor {
        color_edge(triangulation_obj, &edgehandle);
    }
    // Redeclare it so that triangulation_obj isnt borrowed mutably
    let mut_edge = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected().fix());
    // Now depending on color of the edge we can do diffrent things
    match mut_edge.edgecolor {
        EdgeColor::B2O => {
            let whatever = VertexType::empty();
        }
    }

    let first_face_handle = edgehandle.face();
    let second_face_handle = edgehandle.rev().face();
    
    None
}



fn color_edge(triangulation_obj: &mut Triangulation, edge: &DirectedEdgeHandleType) -> Option<f64> {
    let p1 = edge.from();
    let p2 = edge.to();
    let mut edge_mut = triangulation_obj.undirected_edge_data_mut(edge.as_undirected().fix());
    edge_mut.edgecolor = pathperimitives::get_edge_color(&p1, &p2);

    None
}
    
