use super::linefuncs;
use super::pathperimitives;
use kiddo::KdTree;
use kiddo::distance::squared_euclidean;
use linefuncs::{distance_between_points, is_left_cone, lines_intersect};
use pathperimitives::{EdgeColor, PointF2, PointF2M, TileType, Triangulation};
use rand::Rng;
use spade::handles::FixedDirectedEdgeHandle;
use spade::{PositionInTriangulation, Triangulation as OtherTriangulation};

#[derive(PartialEq, Debug, Copy, Clone)]
pub enum RRTNodeType {
    OutOfBounds,
    InBounds,
    ConeClearance,
    Leaf,
    Unknown,
}

#[derive(Debug, Clone)]
pub struct RRTNode {
    pub point: PointF2M,
    pub parent: Option<usize>,
    pub num_children: u32,
    pub cost: f64,
    pub yaw: f64,
    pub node_type: RRTNodeType,
    pub children: Vec<usize>,
}

impl RRTNode {
    pub fn new(point: PointF2M, parent: Option<usize>, yaw: f64) -> RRTNode {
        RRTNode {
            point: point,
            parent: parent,
            yaw: yaw,
            num_children: 0,
            cost: 0.0,
            node_type: RRTNodeType::Unknown,
            children: Vec::new(),
        }
    }
}

pub fn check_edge(
    triangulation: &Triangulation,
    standoff: &f64,
    random_point: &PointF2M,
    edge: &FixedDirectedEdgeHandle,
) -> RRTNodeType {
    let edge_handle = triangulation.directed_edge(*edge);
    let edge_color = edge_handle.as_undirected().data().edgecolor.clone();
    if edge_color == EdgeColor::B2B || edge_color == EdgeColor::Y2Y {
        // || edge_color == EdgeColor::O2O this one shouldnt be here but its more complicated than that
        return RRTNodeType::OutOfBounds;
    } else {
        let close_to = distance_between_points(&random_point, &edge_handle.to().data().position2);
        let close_from =
            distance_between_points(&random_point, &edge_handle.from().data().position2);
        if close_to > *standoff || close_from > *standoff {
            return RRTNodeType::ConeClearance;
        } else {
            return RRTNodeType::InBounds;
        }
    }
}

pub fn score_node(node: &RRTNode, front_cones: Vec<(f64, &PointF2M)>) -> f64 {
    let mut right = 0.0;
    let mut left = 0.0;
    for cone in front_cones {
        let isleft = is_left_cone(&PointF2M::new(cone.1.x, cone.1.y), &node.point, node.yaw);
        if isleft {
            left += 1.0;
        } else {
            right += 1.0;
        }
    }
    return 5.0 - left - right;
}

pub fn rrt(
    start: PointF2M,
    triangulation: &Triangulation,
    max_iterations: usize,
    expansion_radius: f64,
    step_size: f64,
    max_yaw_change: f64,
    yaw: f64,
    standoff: f64,
) -> Vec<RRTNode> {
    let mut nodes = vec![RRTNode::new(start, None, yaw)];
    let mut leafs: Vec<usize> = Vec::new();
    let mut iterations = 0;
    while iterations < max_iterations {
        let mut rng = rand::thread_rng();
        let random_node = rng.gen_range(0..nodes.len());
        if nodes[random_node].node_type == RRTNodeType::InBounds {
            let random_yaw = rng.gen_range(-1.0 * max_yaw_change..max_yaw_change) + nodes[random_node].yaw;
            let random_point = PointF2M::new(
                nodes[random_node].point.x + step_size * random_yaw.cos(),
                nodes[random_node].point.y + step_size * random_yaw.sin(),
            );
            let mut new_node = RRTNode::new(random_point, Some(random_node), random_yaw);
            let pos_in_triangulation =
                triangulation.locate(PointF2::new(random_point[0], random_point[1]));
            match pos_in_triangulation {
                PositionInTriangulation::OnEdge(edge) => {
                    new_node.node_type = check_edge(triangulation, &standoff, &random_point, &edge);
                }
                PositionInTriangulation::OnFace(face) => {
                    let edges = triangulation.face(face).adjacent_edges();
                    for edge in edges {
                        let to = edge.to();
                        let from = edge.from();
                        let intersection = lines_intersect(
                            &nodes[random_node].point,
                            &random_point,
                            &to.data().position2,
                            &from.data().position2,
                        );
                        if intersection {
                            let edge_type =
                                check_edge(triangulation, &standoff, &random_point, &edge.fix());
                            match edge_type {
                                RRTNodeType::OutOfBounds => {
                                    new_node.node_type = RRTNodeType::OutOfBounds;
                                    break;
                                }
                                RRTNodeType::InBounds => {
                                    new_node.node_type = RRTNodeType::InBounds;
                                    break;
                                }
                                RRTNodeType::ConeClearance => {
                                    new_node.node_type = RRTNodeType::ConeClearance;
                                    break;
                                }
                                RRTNodeType::Unknown => {
                                    new_node.node_type = RRTNodeType::Unknown;
                                }
                                RRTNodeType::Leaf => {
                                    new_node.node_type = RRTNodeType::Leaf;
                                    break;
                                }
                            }
                        }
                    }
                    if new_node.node_type != RRTNodeType::Unknown {
                        match triangulation.face(face).data().tiletype {
                            TileType::Offtrack => {
                                new_node.node_type = RRTNodeType::OutOfBounds;
                            }
                            TileType::Ontrack => {
                                new_node.node_type = RRTNodeType::InBounds;
                            }
                            TileType::Unknown => {
                                new_node.node_type = RRTNodeType::Unknown;
                            }
                        }
                    }
                }
                PositionInTriangulation::OutsideOfConvexHull(_edge) => {
                    new_node.node_type = RRTNodeType::OutOfBounds;
                }
                PositionInTriangulation::OnVertex(_vertex) => {
                    // this one needs to take into account if the vertex is even a valid cone
                    new_node.node_type = RRTNodeType::OutOfBounds;
                }
                PositionInTriangulation::NoTriangulation => {
                    new_node.node_type = RRTNodeType::Unknown;
                }
            }
            if new_node.node_type == RRTNodeType::InBounds
                && distance_between_points(&random_point, &start) > expansion_radius
            {
                new_node.node_type = RRTNodeType::Leaf;
                leafs.push(nodes.len());
            }
            nodes.push(new_node);
            nodes[random_node].num_children += 1;
            let child_id = nodes.len() - 1;
            nodes[random_node].children.push(child_id);
            iterations += 1;
        }
    }

    let mut kdtree: KdTree<f64, PointF2M, 2> = KdTree::new();
    for vertex in triangulation.vertices() {
        kdtree.add(
            &[vertex.position().x, vertex.position().y],
            vertex.data().position2,
        );
    }
    for node in &mut nodes {
        let loc = [node.point[0], node.point[1]];
        let front_cones_resp = kdtree.within(&loc, 12.0, &squared_euclidean);
        match front_cones_resp {
            std::result::Result::Ok(cones) => {
                node.cost = score_node(&node, cones);
            }
            std::result::Result::Err(_e) => {
                node.cost = 0.0;
            }
        }
    }
    let _sum = score_children(&mut nodes, 0);

    return nodes;
}

pub fn score_children(nodes: &mut Vec<RRTNode>, index: usize) -> Option<usize> {
    let node_cost = nodes[index].cost;
    let children = &nodes[index].children.clone();
    for child in children {
        nodes[*child].cost += node_cost;
        score_children(nodes, *child);
    }

    None
}

pub fn get_best_leaf(nodes: &Vec<RRTNode>) -> usize {
    let mut best_leaf = 0;
    let mut best_cost = nodes[0].cost;
    for (i, node) in nodes.iter().enumerate() {
        if node.node_type == RRTNodeType::Leaf {
            if node.cost > best_cost {
                best_leaf = i;
                best_cost = node.cost;
            }
        }
    }
    best_leaf
}
