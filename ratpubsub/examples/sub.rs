use ratpubsub::RatNode;

#[derive(Clone)]
struct StringMsg(String);

impl From<String> for StringMsg {
    fn from(value: String) -> Self {
        Self(value)
    }
}

impl ratpubsub::SeaSendableBuffer for StringMsg {
    fn to_packet(self) -> Vec<u8> {
        self.0.into_bytes()
    }

    fn set_from_packet(raw_data: Vec<u8>) -> anyhow::Result<Self> {
        Ok(Self(String::from_utf8_lossy(&raw_data).into_owned()))
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let node = RatNode::create("supercoolsubscriber").await?;

    let mut rx = node.subscribe::<StringMsg>("wow_interesting", 100).await?;

    while let Some(msg) = rx.recv().await {
        println!("Received Message! {}", &msg.0);
    }

    Ok(())
}
