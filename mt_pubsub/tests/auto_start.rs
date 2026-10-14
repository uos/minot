//! Real subprocesses exercise the OS lock used by embedded coordinator election.
use std::{
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

#[test]
fn auto_start_worker() {
    let Ok(directory) = std::env::var("MINOT_TEST_AUTO_START_DIR") else {
        return;
    };
    let directory = std::path::PathBuf::from(directory);
    let index = std::env::var("MINOT_TEST_AUTO_START_INDEX").unwrap();
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        mt_sea::network::set_local_only(true);
        let deadline = Instant::now() + Duration::from_secs(25);
        while !directory.join("start").exists() {
            assert!(Instant::now() < deadline, "start barrier timed out");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let node = mt_pubsub::Node::create(
            mt_pubsub::NodeConfig::new(format!("startup_{index}"))
                .mode(mt_pubsub::Qos::BestEffort)
                .coord_mode(mt_pubsub::CoordMode::AutoStart),
        )
        .await
        .expect("simultaneous auto-start must register");
        std::fs::write(directory.join(format!("ready_{index}")), b"ready").unwrap();
        while !directory.join("finish").exists() {
            assert!(Instant::now() < deadline, "finish barrier timed out");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        drop(node);
    });
}

struct Workers(Vec<Child>);
impl Drop for Workers {
    fn drop(&mut self) {
        for child in &mut self.0 {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[test]
fn simultaneous_auto_start_on_independent_ports() {
    let ports: Vec<_> = (0..2)
        .map(|_| std::net::TcpListener::bind("127.0.0.1:0").unwrap())
        .collect();
    let addresses: Vec<_> = ports
        .iter()
        .map(|listener| listener.local_addr().unwrap().port())
        .collect();
    let directory = std::env::temp_dir().join(format!("minot-auto-start-{}", std::process::id()));
    std::fs::create_dir(&directory).unwrap();
    let mut workers = Workers(vec![]);
    for (group, port) in addresses.iter().enumerate() {
        for index in 0..8 {
            let child = Command::new(std::env::current_exe().unwrap())
                .args(["--exact", "auto_start_worker", "--nocapture"])
                .env("MINOT_ZENOH_PORT", port.to_string())
                .env("MINOT_DOMAIN_ID", (port % 60000 + 1).to_string())
                .env("MINOT_TEST_AUTO_START_DIR", &directory)
                .env("MINOT_TEST_AUTO_START_INDEX", format!("{group}_{index}"))
                .stdout(Stdio::null())
                .stderr(Stdio::inherit())
                .spawn()
                .unwrap();
            workers.0.push(child);
        }
    }
    drop(ports);
    std::fs::write(directory.join("start"), b"start").unwrap();
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut ready = false;
    while Instant::now() < deadline {
        for child in &mut workers.0 {
            if let Some(status) = child.try_wait().unwrap() {
                panic!("auto-start worker exited early: {status}");
            }
        }
        ready = (0..2).all(|group| {
            (0..8).all(|index| directory.join(format!("ready_{group}_{index}")).exists())
        });
        if ready {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    std::fs::write(directory.join("finish"), b"finish").unwrap();
    assert!(ready, "not every node registered on its configured port");
    for child in &mut workers.0 {
        assert!(child.wait().unwrap().success());
    }
    std::fs::remove_dir_all(&directory).unwrap();
}
