use std::future::Future;

use log::{debug, error};
use mt_pubsub::{Node, Publisher, Qos, Subscriber};
use mt_sea::Sendable;
use std::collections::HashMap;
use std::marker::PhantomData;
use std::sync::Arc;
use tokio::select;
use tokio::sync::Mutex;
use tokio::sync::mpsc;
use tokio::time::{Duration, sleep};
use uuid::Uuid;

/*
 * TODO:
 * - verify types between server<->client and fail creation on mismatch
 */

pub struct ServiceServer<REQ, RES>
where
    REQ: Sendable,
    RES: Sendable,
{
    // reference to the managing node
    node: Arc<Node>,

    /// clients should publish their requests to this
    subber: Mutex<Subscriber<(u128, u64, REQ)>>,

    /// map of client identifiers to their respective publisher topics
    clients: Mutex<HashMap<Uuid, mpsc::Sender<(u64, REQ)>>>,

    _phantom_res: PhantomData<RES>,
}

impl<REQ, RES> ServiceServer<REQ, RES>
where
    REQ: Sendable,
    RES: Sendable,
{
    /// construct a new service server
    pub async fn new(node: Arc<Node>, topic: String) -> anyhow::Result<Arc<Self>> {
        // XXX: what is a good queue_size?
        let subber = Mutex::new(node.create_subscriber(topic, 10, Qos::Reliable).await?);
        let clients = Mutex::new(HashMap::new());
        Ok(Arc::new(Self {
            node,
            subber,
            clients,
            _phantom_res: PhantomData,
        }))
    }

    /**
     * start listening for requests; every client gets its own handler thread
     *
     * TODO:
     * - under *a lot* of pressure this sort of crumbles - essentially making
     *   the server thread spin at 100% cpu without making progress for a bit,
     *   should probably be investigated
     * - proper client disconnect - all that's needed is removing stale
     *   clients from the clients HashMap and the associated channel/thread
     *   *should* shut down automatically
     *
     * @param this      Arc of a ServiceServer instance
     * @param callback  the function implementing the service
     *                  this gets the request as its only argument
     */
    pub async fn start<F, Fut>(this: Arc<Self>, callback: Arc<F>)
    where
        F: Fn(REQ) -> Fut + Send + Sync + 'static,
        Fut: Future<Output = Result<RES, String>> + Send + Sync + 'static,
    {
        // once started these should stay locked for the entire runtime
        let mut subber = this.subber.lock().await;
        let mut clients = this.clients.lock().await;

        while let Some((client, seq_num, request)) = subber.next().await {
            let client = Uuid::from_u128(client);
            if !clients.contains_key(&client) {
                debug!("Registering new client: {}", &client);
                let (tx, rx) = mpsc::channel(100);
                clients.insert(client.to_owned(), tx);
                tokio::spawn(Self::client_handler(
                    this.node.clone(),
                    client.to_owned(),
                    rx,
                    callback.clone(),
                ));
            }

            if let Err(e) = clients
                .get(&client)
                .expect("client does not exist when it should")
                .send((seq_num, request))
                .await
            {
                error!("client handler died: {}", e);
            }
        }
    }

    /**
     * handler thread for a single unique client
     *
     * @param node     reference to the same Node used by the managing
     *                 ServiceServer
     * @param client   provided client ID
     * @param requests Receiver of the channel forwarding this client's requests
     * @param callback reference to the service callback
     */
    async fn client_handler<F, Fut>(
        node: Arc<Node>,
        client: Uuid,
        mut requests: mpsc::Receiver<(u64, REQ)>,
        callback: Arc<F>,
    ) where
        F: Fn(REQ) -> Fut + Send + Sync,
        Fut: Future<Output = Result<RES, String>>,
    {
        let pubber: Publisher<(u64, Result<RES, String>)> = match node
            .create_publisher(
                format!("/_service_responders/{}", &client.to_string()),
                Qos::Reliable,
            )
            .await
        {
            Ok(pubber) => pubber,
            Err(e) => {
                error!("Could not register new client: {}", e);
                return;
            }
        };

        while let Some((seq_num, request)) = requests.recv().await {
            let response = (seq_num, callback(request).await);
            match pubber.publish(&response).await {
                Ok(_) => {}
                Err(e) => {
                    error!(
                        "Error publishing response to {}: {}",
                        &client.to_string(),
                        e
                    );
                }
            }
        }
    }
}

pub struct ServiceClient<REQ, RES>
where
    REQ: Sendable,
    RES: Sendable,
{
    /// client id
    id: Uuid,

    /// sequence number for requests
    seq_num: Mutex<u64>,

    /// clients publish requests to this
    pubber: Publisher<(u128, u64, REQ)>,

    /// server responds over this subscribtion
    subber: Mutex<Subscriber<(u64, Result<RES, String>)>>,
}

impl<REQ, RES> ServiceClient<REQ, RES>
where
    REQ: Sendable,
    RES: Sendable,
{
    pub async fn new(node: Arc<Node>, topic: String) -> anyhow::Result<Self> {
        let id = Uuid::new_v4();
        let pubber = node.create_publisher(topic, Qos::Reliable).await?;
        // XXX: again what would be a good queue_size?
        let subber = node
            .create_subscriber(
                format!("/_service_responders/{}", &id.to_string()),
                10,
                Qos::Reliable,
            )
            .await?;
        Ok(Self {
            id,
            seq_num: Mutex::new(0),
            subber: Mutex::new(subber),
            pubber,
        })
    }

    /**
     * perform a request, keep in mind the returned future is *lazy*
     * i.e. processing likely waits until .await is called (unless spawned as a task via tokio::spawn)
     *
     * TODO: handle multiple async request calls?
     * this currently very much assumes 1 request -> 1 response
     *
     * @param request
     */
    pub async fn request(&self, request: REQ, timeout: Option<Duration>) -> Result<RES, String> {
        let seq_num = {
            let mut num = self.seq_num.lock().await;
            let n = *num;
            *num += 1;
            n
        };
        let myreq = (self.id.as_u128(), seq_num, request);
        let req_handle = self.pubber.publish(&myreq);
        let mut subber = self.subber.lock().await;

        if let Some(timeout) = timeout {
            select! {
                _ = sleep(timeout) => {
                    return Err("Timeout reached while sending request".to_owned());
                }
                res = req_handle => {
                    if let Err(e) = res {
                        return Err(format!("Error sending request: {}", &e));
                    }
                }
            }

            loop {
                select! {
                    _ = sleep(timeout) => {
                        return Err("Timeout reached while waiting for response".to_owned());
                    }
                    res = subber.next() => {
                        if let Some(res) = Self::handle_response(res, seq_num) {
                            return res;
                        }
                    }
                }
            }
        } else {
            if let Err(e) = req_handle.await {
                return Err(format!("Error sending request: {}", &e));
            }

            loop {
                let res = subber.next().await;
                if let Some(res) = Self::handle_response(res, seq_num) {
                    return res;
                }
            }
        }
    }

    /**
     * @param response  response as returned by subber.next()
     * @param seq_num   the expected sequence number
     * @returns         None if we should skip this response
     *                  Some(Ok(...)) if response is correct
     *                  Some(Err(...)) if response is bad
     */
    fn handle_response(
        response: Option<(u64, Result<RES, String>)>,
        seq_num: u64,
    ) -> Option<Result<RES, String>> {
        match response {
            Some((res_seq, res_data)) => match res_seq.cmp(&seq_num) {
                std::cmp::Ordering::Less => None,
                std::cmp::Ordering::Equal => match res_data {
                    Ok(res) => Some(Ok(res)),
                    Err(e) => Some(Err(format!(
                        "ServiceServer encountered an error processing request: {}",
                        e
                    ))),
                },
                std::cmp::Ordering::Greater => Some(Err(format!(
                    "Unexpected sequence number: {} (expected {})",
                    res_seq, seq_num
                ))),
            },
            None => Some(Err("None response from ServiceServer".to_owned())),
        }
    }
}
