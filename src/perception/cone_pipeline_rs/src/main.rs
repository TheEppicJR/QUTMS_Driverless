use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r;
use r2r::driverless_msgs::msg::PointWithCovarianceArrayStamped;
use r2r::nav_msgs::msg::Odometry;
use r2r::QosProfile;
use cgmath::{Quaternion, Euler};
use spade::{Triangulation as OtherTriangulation};
mod edgefinder;
mod pathperimitives;
mod markermaker;
mod rrt;
mod linefuncs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "node", "namespace")?;
    let points_subscriber = node
        .subscribe::<PointWithCovarianceArrayStamped>(
            "/cone_pipe/cone_detection_cov",
            QosProfile::default(),
        )?;
    let odom_subscriber = node.subscribe::<Odometry>("/odometry/global", QosProfile::default())?;
    let publisher = node.create_publisher::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
    let delaunay_publisher = node.create_publisher::<r2r::visualization_msgs::msg::Marker>(
        "/cone_pipe/delaunay/markers",
        QosProfile::default(),
    )?;
    let rrt_publisher = node.create_publisher::<r2r::visualization_msgs::msg::Marker>(
        "/cone_pipe/rrt/markers",
        QosProfile::default(),
    )?;
    let mut timer = node.create_wall_timer(std::time::Duration::from_millis(1000))?;

    // Set up a simple task executor.
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // let mut pathpoints: Vec<pathperimitives::PointF2> = Vec::new();
    // let mut trackedges: pathperimitives::Track = pathperimitives::Track::new();
    let mut triangulation: pathperimitives::Triangulation = pathperimitives::Triangulation::new();

    // Run the subscriber in one task, printing the messages
    spawner.spawn_local(async move {
        points_subscriber
            .for_each(|msg| {
                triangulation = edgefinder::get_edges(msg.points);
                let marker_result = markermaker::generate_edges(&triangulation);
                match marker_result {
                    Ok(marker) => {
                        delaunay_publisher.publish(&marker);
                    }
                    Err(e) => {
                        println!("{}", e);
                    }
                }
                future::ready(())
            })
            .await
    })?;

    spawner.spawn_local(async move {
        odom_subscriber
            .for_each(|msg| {
                let pos = msg.pose.pose.position;
                let pos_point = pathperimitives::PointF2M::new(pos.x, pos.y);
                let quat = Quaternion::new(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
                let euler = Euler::from(quat);
                let yaw = euler.z.0;
                let nodes = rrt::rrt(
                    pos_point,
                    &triangulation,
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
            })
            .await
    })?;

    // Run the publisher in another task
    spawner.spawn_local(async move {
        let mut counter = 0;
        loop {
            let _elapsed = timer.tick().await.unwrap();
            let msg = r2r::std_msgs::msg::String {
                data: format!("Hello, world! ({})", counter),
            };
            publisher.publish(&msg).unwrap();
            counter += 1;
        }
    })?;

    // Main loop spins ros.
    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
