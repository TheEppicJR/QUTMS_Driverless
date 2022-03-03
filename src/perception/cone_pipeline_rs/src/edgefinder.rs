use spade::{DelaunayTriangulation, Triangulation as OtherTriangulation, Point2, PositionInTriangulation, DirectedEdgeHandle};
use spade::handles::{FixedFaceHandle, FixedDirectedEdgeHandle, VertexHandle, UndirectedEdgeHandle};
use PointWithCovariance;
use nalgebra as na;
use na::{Vector3, Rotation3, Matrix3, Point};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use T;
mod pathperimitives;
mod pathperimitives::{get_edges, Track, EdgeColor}

pub fn get_edges(points: Vec<PointWithCovariance>) -> Triangulation<_> {

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
    return triangulation;

}

pub fn find_track_edges(triangulation_obj: &Triangulation<_>, pathpoints: &Vec<Point>, track: &Track) -> Option<T> {
    if triangulation_obj.num_vertices() > 4 {
        // For now im not going to use the Track object to maintain the state
        // ie its going to be stateless for now, just wipe Track and put in new shit every time
        // to make it stateless we have to go through and check for points in areas that are relivant and see if they changed and I dont wanna do that rn
        // Really I should be adding and removing points here so I dont have to iterate through the edges every time
        for pathpoint in pathpoints.iter() {
            // get whatever vertex, edge, or triangle the point is in/on
            let pointpos = triangulation_obj.locate(pathpoint);
            // now use one of the analysis methods to start to fill in our understanding of the track
            // we only care if the point is on a line or in a triangle
            match pointpos {
                PositionInTriangulation::OnEdge(edgehandle) => {
                    let mut edge = triangulation_obj.directed_edge(edgehandle);
                    analyse_edge(triangulation_obj, edge);
                }

            }
        }
    }
}

fn analyse_face(triangulation_obj: &Triangulation<_>, facehandle: &FixedFaceHandle) -> Option<T> {
    let mut mut_facehandle = triangulation_obj.face(facehandle);
    for edgehandle in mut_facehandle.adjacent_edges() {
        let mut mut_edge = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected());
        if mut_edge.data().color == EdgeColor::Unknown {
            color_edge(triangulation_obj, mut_edge);
        }
    }
}

fn analyse_edge(triangulation_obj: &Triangulation<_>, edgehandle: &DirectedEdgeHandle) -> Option<T> {
    let pts = edgehandle.vertices();
    let mut p1 = pts.from();
    let mut p2 = pts.to();
    let mut mut_edge = triangulation_obj.undirected_edge_data_mut(edgehandle.as_undirected());
    // If we havent figured out the color of the edge then figure that shit out
    if mut_edge.data().edgecolor == EdgeColor::Unknown {
        color_edge(triangulation_obj, mut_edge);
    }
    // Now depending on color of the edge we can do diffrent things
    match mut_edge.data().edgecolor {
        EdgeColor::B2O {

        }
    }

    let mut first_face_handle = edgehandle.face();
    let mut second_face_handle = edgehandle.rev().face();
    

}



fn color_edge(triangulation_obj: &Triangulation<_>, edge: &UndirectedEdgeHandle) -> Option<T> {
    let pts = edge.vertices();
    let mut p1 = pts.from();
    let mut p2 = pts.to();
    let mut mut_edge = triangulation_obj.undirected_edge_data_mut(edge.as_undirected());
    mut_edge.data().edgecolor = get_edge_color(p1, p2);
}
    
