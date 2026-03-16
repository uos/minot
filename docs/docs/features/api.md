# Guide

- **Pub/Sub**: continuous data streams (sensor data, status updates).
- **Service**: request/reply RPC (ask once, get one response).
- **Action**: long-running goals with feedback + final result (and cancellation).

## Project setup

=== "Go"

    ```bash
    minot init-go my_go_robot_cli
    cd my_go_robot_cli
    minot add-go-msg std_msgs/String --project .
    go get -u github.com/charmbracelet/log
    go mod tidy
    ```

    You can directly run our examples after project setup.

    ```bash
    go run examples/pub.go
    go run examples/sub.go
    go run examples/service_server.go
    go run examples/service_client.go
    go run examples/action_client.go
    ```

    For action demo with Rust server:

    ```bash
    # in minot repo
    cd mt_action
    cargo run --example server
    ```

=== "Rust"

    Add `mt_pubsub` / `mt_service` / `mt_action` to `Cargo.toml`, then use the `ros2_interfaces_jazzy_rkyv` message types in your node code.

## Pub/Sub

=== "Go"

    ```go
    node, _ := minot.NewNode("pubsub_node", minot.QosReliable)
    pub, _ := node.NewStdMsgsStringPublisher("/some_topic", minot.QosReliable)
    _, _ = node.NewStdMsgsStringSubscriber("/some_topic", 10, minot.QosReliable, func(m minot.StdMsgsString) {
        fmt.Println(m.Data)
    })
    _ = pub.Publish(minot.StdMsgsString{Data: "Hello"})
    ```

=== "Rust"

    ```rust
    let node = Node::create(NodeConfig::new("pubsub_node")).await?;
    let pubber = node.create_publisher::<msg::String>("/some_topic".to_owned(), Qos::Reliable).await?;
    let mut subber = node.create_subscriber::<msg::String>("/some_topic".to_owned(), 10, Qos::Reliable).await?;
    pubber.publish(&msg::String { data: "Hello".to_owned() }).await?;
    ```

## Service

=== "Go"

    Server:

    ```go
    node, _ := minot.NewNode("service_server", minot.QosReliable)
    _, _ = node.NewStdMsgsStringServiceServer("my_cool_service", func(req minot.StdMsgsString) (minot.StdMsgsString, error) {
        return minot.StdMsgsString{Data: "Your message was: " + req.Data}, nil
    })
    <-node.ShutdownCh()
    ```

    Client:

    ```go
    node, _ := minot.NewNode("service_client", minot.QosReliable)
    client, _ := node.NewStdMsgsStringServiceClient("my_cool_service")
    resp, err := client.Request(minot.StdMsgsString{Data: "Hello"}, 10_000)
    if err == nil {
        fmt.Println(resp.Data)
    }
    ```

=== "Rust"

    ```rust
    let server = ServiceServer::<msg::String, msg::String>::new(node.clone(), "my_cool_service".to_owned()).await?;
    // ServiceServer::start(...) on server side
    let client = ServiceClient::<msg::String, msg::String>::new(node.clone(), "my_cool_service".to_owned()).await?;
    let res = client.request(msg::String{ data: "Hello".to_owned() }, Some(Duration::from_secs(10))).await?;
    ```

## Action

=== "Go"

    Client:

    ```go
    node, _ := minot.NewNode("action_client", minot.QosReliable)
    client, _ := node.NewStdMsgsStringActionClient("/my_amazing_action", minot.QosReliable)
    if !client.WaitForServer(5_000) { panic("server not ready") }

    accepted, _, _ := client.SendGoal(42, minot.StdMsgsString{Data: "Hello server"}, 10_000)
    fmt.Println("accepted:", accepted)

    goalID, fb, _ := client.GetFeedback()
    fmt.Println(goalID, fb.Data)

    state, result, _ := client.GetResult(42, 30_000)
    fmt.Println(state, result)
    ```

=== "Rust"

    ```rust
    let client: Arc<ActionClient<msg::String, msg::String, msg::String>> =
        ActionClient::new(node.clone(), "/my_amazing_action".to_owned(), Qos::Reliable).await?;
    let _ = ActionClient::send_goal(client.clone(), ActionSendGoalRequest { id: 42, goal: msg::String{ data: "Hello server".to_owned() } }, None).await?;
    ```

