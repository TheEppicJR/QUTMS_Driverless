use cgmath::{Euler, Quaternion};
use futures::future;
use futures::stream::StreamExt;
use r2r;
use r2r::driverless_msgs::msg::PointWithCovarianceArrayStamped;
use r2r::nav_msgs::msg::Odometry;
use r2r::QosProfile;
use std::sync::{Arc, Mutex};
use tokio::task;
mod edgefinder;
mod linefuncs;
mod markermaker;
mod pathperimitives;
mod rrt;

#[derive(Debug, Default)]
struct SharedState {
    pub state: pathperimitives::Triangulation,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "testnode", "")?;
    let arc_node = Arc::new(Mutex::new(node));

    // let mut pathpoints: Vec<pathperimitives::PointF2> = Vec::new();
    // let mut trackedges: pathperimitives::Track = pathperimitives::Track::new();
    // let mut triangulation: pathperimitives::Triangulation = pathperimitives::Triangulation::new();

    // let mut triangulation: Mutex<pathperimitives::Triangulation> =
    //     Mutex::new(pathperimitives::Triangulation::new());

    let state = Arc::new(Mutex::new(SharedState::default()));

    let an = arc_node.clone();
    let state2 = state.clone();
    task::spawn(async move { pts_subscriber(an, state2).await.unwrap() });

    let an = arc_node.clone();
    let state3 = state.clone();
    task::spawn(async move { odom_subscriber(an, state3).await.unwrap() });

    let handle = tokio::task::spawn_blocking(move || loop {
        {
            arc_node
                .lock()
                .unwrap()
                .spin_once(std::time::Duration::from_millis(10));
        }
        std::thread::sleep(std::time::Duration::from_millis(100))
    });

    handle.await?;

    Ok(())
}

async fn pts_subscriber(arc_node: Arc<Mutex<r2r::Node>>, triangulation: Arc<Mutex<SharedState>>) -> Result<(), r2r::Error> {
    let points_subscriber = arc_node
        .lock()
        .unwrap()
        .subscribe::<PointWithCovarianceArrayStamped>(
            "/cone_pipe/cone_detection_cov",
            QosProfile::default(),
        )?;
    let delaunay_publisher = arc_node
        .lock()
        .unwrap()
        .create_publisher::<r2r::visualization_msgs::msg::Marker>(
            "/cone_pipe/delaunay/markers",
            QosProfile::default(),
        )?;
    points_subscriber
        .for_each(|msg| {
            let mut cur_triangulation = edgefinder::get_edges(msg.points);
            let marker_result = markermaker::generate_edges(&cur_triangulation);
            match marker_result {
                Ok(marker) => {
                    delaunay_publisher.publish(&marker);
                }
                Err(e) => {
                    println!("{}", e);
                }
            }
            triangulation.lock().unwrap().state = cur_triangulation;
            future::ready(())
        });
    Ok(())
}

async fn odom_subscriber(arc_node: Arc<Mutex<r2r::Node>>, triangulation: Arc<Mutex<SharedState>>) -> Result<(), r2r::Error> {
    let odom_subscriber = arc_node
        .lock()
        .unwrap()
        .subscribe::<Odometry>("/odometry/global", QosProfile::default())?;
    let rrt_publisher = arc_node
        .lock()
        .unwrap()
        .create_publisher::<r2r::visualization_msgs::msg::Marker>(
            "/cone_pipe/rrt/markers",
            QosProfile::default(),
        )?;

    odom_subscriber
        .for_each(|msg| {
            let pos = msg.pose.pose.position;
            let pos_point = pathperimitives::PointF2M::new(pos.x, pos.y);
            let quat = Quaternion::new(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            );
            let euler = Euler::from(quat);
            let yaw = euler.z.0;
            let nodes = rrt::rrt(
                pos_point,
                &triangulation.lock().unwrap().state,
                2000,
                12.0,
                0.5,
                0.3141592653589793,
                yaw,
                0.7,
            );

            let best_leaf = rrt::get_best_leaf(&nodes);
            let max_cost = nodes[best_leaf].cost;
            let marker_result = markermaker::generate_nodes(&nodes, &max_cost);
            match marker_result {
                Ok(marker) => {
                    rrt_publisher.publish(&marker);
                }
                Err(e) => {
                    println!("{}", e);
                }
            }
            future::ready(())
        });
    Ok(())
}
