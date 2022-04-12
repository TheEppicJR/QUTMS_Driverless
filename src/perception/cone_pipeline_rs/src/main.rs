use cgmath::{Euler, Quaternion};
use futures::stream::StreamExt;
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

    let state = Arc::new(Mutex::new(SharedState::default()));

    println!("Begin init");

    let an = arc_node.clone();
    let state2 = state.clone();
    task::spawn(async move { pts_subscriber(an, state2).await.unwrap() });

    let an = arc_node.clone();
    let state3 = state.clone();
    task::spawn(async move { odom_subscriber(an, state3).await.unwrap() });

    let an = arc_node.clone();
    task::spawn(async move { publisher(an).await.unwrap() });

    println!("Finish init");

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

async fn publisher(arc_node: Arc<Mutex<r2r::Node>>) -> Result<(), r2r::Error> {
    let (mut timer, publisher) = {
        // Limiting the scope when locking the arc
        let mut node = arc_node.lock().unwrap();
        let timer = node.create_wall_timer(std::time::Duration::from_secs(2))?;
        let publisher =
            node.create_publisher::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
        (timer, publisher)
    };
    for _ in 1..10 {
        timer.tick().await?;
        let msg = r2r::std_msgs::msg::String {
            data: "hello from r2r".to_string(),
        };
        publisher.publish(&msg)?;
    }
    Ok(())
}

async fn pts_subscriber(
    arc_node: Arc<Mutex<r2r::Node>>,
    triangulation: Arc<Mutex<SharedState>>,
) -> Result<(), r2r::Error> {
    let (mut points_subscriber, delaunay_publisher) = {
        let mut node = arc_node.lock().unwrap();
        let points_subscriber = node.subscribe::<PointWithCovarianceArrayStamped>(
            "/cone_pipe/cone_detection_cov",
            QosProfile::default(),
        )?;
        let delaunay_publisher = node.create_publisher::<r2r::visualization_msgs::msg::Marker>(
            "/cone_pipe/delaunay/markers",
            QosProfile::default(),
        )?;
        (points_subscriber, delaunay_publisher)
    };

    loop {
        match points_subscriber.next().await {
            Some(msg) => {
                let cur_triangulation = edgefinder::get_edges(msg.points);
                let marker_result = markermaker::generate_edges(&cur_triangulation);
                match marker_result {
                    Ok(marker) => {
                        println!("Sucesfully generated marker");
                        let f = delaunay_publisher.publish(&marker);
                        match f {
                            Ok(_) => println!("Sucesfully published marker"),
                            Err(e) => println!("Failed to publish marker: {:?}", e),
                        }
                    }
                    Err(e) => {
                        println!("Suberr: {}", e);
                    }
                }
                triangulation.lock().unwrap().state = cur_triangulation;
            }
            None => {
                println!("Empty message");
            }
        }
    }
}

async fn odom_subscriber(
    arc_node: Arc<Mutex<r2r::Node>>,
    triangulation: Arc<Mutex<SharedState>>,
) -> Result<(), r2r::Error> {
    let (mut odom_subscriber, rrt_publisher) = {
        let mut node = arc_node.lock().unwrap();
        let odom_subscriber =
            node.subscribe::<Odometry>("/odometry/global", QosProfile::default())?;
        let rrt_publisher = node.create_publisher::<r2r::visualization_msgs::msg::Marker>(
            "/cone_pipe/rrt/markers",
            QosProfile::default(),
        )?;
        (odom_subscriber, rrt_publisher)
    };
    let mut cycle = 0;

    loop {
        match odom_subscriber.next().await {
            Some(msg) => {
                println!("Recived odom msg");
                if cycle % 10 == 0 {
                    let pos = msg.pose.pose.position;
                    let pos_point = pathperimitives::PointF2M::new(pos.x, pos.y);
                    let quat = Quaternion::new(
                        msg.pose.pose.orientation.w,
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
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
                            let f = rrt_publisher.publish(&marker);
                            match f {
                                Ok(_) => println!("Sucesfully published marker"),
                                Err(e) => println!("Failed to publish marker: {:?}", e),
                            }
                        }
                        Err(e) => {
                            println!("Puberr: {}", e);
                        }
                    }
                }
                cycle += 1;
            }
            None => {
                println!("Empty message");
            }
        }
    }
}
