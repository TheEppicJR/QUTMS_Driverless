use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r;
use r2r::driverless_msgs::msg::PointWithCovariance;
use r2r::driverless_msgs::msg::PointWithCovarianceArrayStamped;
use r2r::nav_msgs::msg::Odometry;
use r2r::QosProfile;
use nalgebra as na;
mod edgefinder;
mod pathperimitives;

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
    let mut timer = node.create_wall_timer(std::time::Duration::from_millis(1000))?;

    // Set up a simple task executor.
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let mut pathpoints: Vec<pathperimitives::PointF2> = Vec::new();
    let mut trackedges: pathperimitives::Track = pathperimitives::Track::new();

    // Run the subscriber in one task, printing the messages
    spawner.spawn_local(async move {
        points_subscriber
            .for_each(|msg| {
                let mut triangulation = edgefinder::get_edges(msg.points);

                future::ready(())
            })
            .await
    })?;

    spawner.spawn_local(async move {
        odom_subscriber
            .for_each(|msg| {
                // println!("got new msg odom: {}", msg.header.stamp);
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
