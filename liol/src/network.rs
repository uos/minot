pub struct Rat {
    pub name: String,
    pub network_id: sea::ShipName,
    pub variables: Vec<String>,
    pub id: crate::RatId, // IDs are generated when a Rat is registered so we don't have to check in strings all the time
}

pub struct Wind {
    pub name: String,
    pub network_id: sea::ShipName,
}
