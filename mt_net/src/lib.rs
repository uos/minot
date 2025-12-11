pub use ros_pointcloud2::PointCloud2Msg;
pub use ros2_interfaces_jazzy_rkyv::nav_msgs::msg::Odometry;
pub use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::{Imu, PointCloud2};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

pub const COMPARE_NODE_NAME: &str = "#minot";

#[derive(Clone, Debug, Default, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
pub enum Qos {
    Sensor,
    #[default]
    SystemDefault,
    Custom(QosProfile),
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(untagged)]
pub enum HistoryValue {
    Numeric(i32),
    String(HistoryPolicy),
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum HistoryPolicy {
    KeepLast,
    KeepAll,
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(untagged)]
pub enum ReliabilityValue {
    Numeric(i32),
    String(ReliabilityPolicy),
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum ReliabilityPolicy {
    Reliable,
    BestEffort,
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(untagged)]
pub enum DurabilityValue {
    Numeric(i32),
    String(DurabilityPolicy),
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum DurabilityPolicy {
    Volatile,
    TransientLocal,
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(untagged)]
pub enum LivelinessValue {
    Numeric(i32),
    String(LivelinessPolicy),
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename_all = "snake_case")]
pub enum LivelinessPolicy {
    Automatic,
    ManualByTopic,
}

fn deserialize_history<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = HistoryValue::deserialize(deserializer)?;
    Ok(match value {
        HistoryValue::Numeric(1) => "keep_last".to_string(),
        HistoryValue::Numeric(2) => "keep_all".to_string(),
        HistoryValue::Numeric(_) => "keep_last".to_string(), // default
        HistoryValue::String(HistoryPolicy::KeepLast) => "keep_last".to_string(),
        HistoryValue::String(HistoryPolicy::KeepAll) => "keep_all".to_string(),
    })
}

fn deserialize_reliability<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = ReliabilityValue::deserialize(deserializer)?;
    Ok(match value {
        ReliabilityValue::Numeric(1) => "reliable".to_string(),
        ReliabilityValue::Numeric(2) => "best_effort".to_string(),
        ReliabilityValue::Numeric(_) => "reliable".to_string(), // default
        ReliabilityValue::String(ReliabilityPolicy::Reliable) => "reliable".to_string(),
        ReliabilityValue::String(ReliabilityPolicy::BestEffort) => "best_effort".to_string(),
    })
}

fn deserialize_durability<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = DurabilityValue::deserialize(deserializer)?;
    Ok(match value {
        DurabilityValue::Numeric(1) => "transient_local".to_string(),
        DurabilityValue::Numeric(2) => "volatile".to_string(),
        DurabilityValue::Numeric(_) => "volatile".to_string(), // default
        DurabilityValue::String(DurabilityPolicy::TransientLocal) => "transient_local".to_string(),
        DurabilityValue::String(DurabilityPolicy::Volatile) => "volatile".to_string(),
    })
}

fn deserialize_liveliness<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = LivelinessValue::deserialize(deserializer)?;
    Ok(match value {
        LivelinessValue::Numeric(1) => "automatic".to_string(),
        LivelinessValue::Numeric(3) => "manual_by_topic".to_string(),
        LivelinessValue::Numeric(_) => "automatic".to_string(), // default
        LivelinessValue::String(LivelinessPolicy::Automatic) => "automatic".to_string(),
        LivelinessValue::String(LivelinessPolicy::ManualByTopic) => "manual_by_topic".to_string(),
    })
}

#[derive(
    rkyv::Archive, Deserialize, Debug, Serialize, Clone, rkyv::Deserialize, rkyv::Serialize,
)]
pub struct QosProfile {
    #[serde(deserialize_with = "deserialize_history")]
    pub history: String,
    pub depth: i32,
    #[serde(deserialize_with = "deserialize_reliability")]
    pub reliability: String,
    #[serde(deserialize_with = "deserialize_durability")]
    pub durability: String,
    pub deadline: QosTime,
    pub lifespan: QosTime,
    #[serde(deserialize_with = "deserialize_liveliness")]
    pub liveliness: String,
    pub liveliness_lease_duration: QosTime,
    pub avoid_ros_namespace_conventions: bool,
}

#[derive(
    Deserialize, Serialize, rkyv::Serialize, rkyv::Deserialize, rkyv::Archive, Debug, Clone, Copy,
)]
pub struct QosTime {
    pub sec: u64,
    pub nsec: u64,
}

#[derive(Clone, Debug, rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)]
pub enum SensorTypeMapped {
    Lidar(PointCloud2),
    Imu(Imu),
    Odometry(Odometry),
    Any(Vec<u8>),
}

#[derive(Clone, Debug, rkyv::Serialize, rkyv::Deserialize, rkyv::Archive)]
pub struct BagMsg {
    pub topic: String,
    pub msg_type: String,
    pub data: SensorTypeMapped,
    pub qos: Option<Qos>,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum RatPubRegisterKind {
    Publish,
    Subscribe,
}

#[derive(PartialEq, Eq, Debug, Clone)]
pub struct Rules {
    store: HashMap<Variable, Vec<VariableHuman>>,
    pub cache: HashMap<Variable, HashSet<(Client, RatPubRegisterKind)>>,
    id_counter: u32,
}

pub type Client = String;
pub type Variable = String;

impl Default for Rules {
    fn default() -> Self {
        Self::new()
    }
}

impl Rules {
    pub fn new() -> Self {
        Self {
            store: HashMap::new(),
            cache: HashMap::new(),
            id_counter: 0,
        }
    }

    pub fn insert(&mut self, variable: String, strategies: Vec<VariableHuman>) {
        match self.store.get_mut(&variable) {
            Some(el) => {
                el.extend(strategies);
            }
            None => {
                self.store.insert(variable, strategies);
            }
        }
    }

    pub fn new_id(&mut self) -> u32 {
        self.id_counter += 1;
        self.id_counter
    }

    /// Removes a client (ship) from the rules and cache.
    /// - If the client is the ship performing a Shoot, Catch, Sail, or None action, the entire rule is removed but if there is another partner like in catch or shoot, the partner will be put into the cache with a subscriber/publisher state based on if was receiving it (subscriber) or sending it and the removing client wanted to catch it (publisher)
    /// - If the client is a target in a Shoot action, it is removed from the target list. If the list becomes empty, the Shoot rule is removed, AND the shooter ship is registered as a publisher in the cache.
    /// - Any cache registrations for the client are removed.
    pub fn remove_client(&mut self, client: &str) {
        // Process store rules
        let mut variables_to_remove_from_store = Vec::new();
        let mut cache_registrations_from_store: HashMap<
            Variable,
            HashSet<(Client, RatPubRegisterKind)>,
        > = HashMap::new();
        let mut store_updates: HashMap<Variable, Vec<VariableHuman>> = HashMap::new();

        for (variable, rules) in self.store.iter_mut() {
            let mut rules_to_keep_in_variable = Vec::new();
            let mut variable_has_remaining_rules = false;

            for rule in rules.drain(..) {
                // Use drain to efficiently iterate and remove
                if rule.ship == client {
                    // Case 1: Client is the main ship performing an action. Remove the rule.
                    match &rule.strategy {
                        Some(ActionPlan::Shoot { target, id: _ }) => {
                            // The client was the shooter, the targets were effectively subscribers
                            if !target.is_empty() {
                                cache_registrations_from_store
                                    .entry(variable.clone())
                                    .or_default()
                                    .extend(
                                        target
                                            .iter()
                                            .cloned()
                                            .map(|t| (t, RatPubRegisterKind::Subscribe)),
                                    );
                            }
                        }
                        Some(ActionPlan::Catch { source, id: _ }) => {
                            // The client was the catcher, the source was the publisher
                            cache_registrations_from_store
                                .entry(variable.clone())
                                .or_default()
                                .insert((source.clone(), RatPubRegisterKind::Publish));
                        }
                        _ => {
                            // Sail, None, or no strategy - no partners to register
                        }
                    }
                } else if let Some(ActionPlan::Shoot { target, id }) = &rule.strategy {
                    // Case 2: Rule is a Shoot action. Check if client is a target.
                    let initial_target_count = target.len();
                    let new_target = target
                        .iter()
                        .filter(|t| t != &client)
                        .cloned()
                        .collect::<Vec<_>>();

                    if new_target.len() < initial_target_count {
                        // The client was in the target list and has been removed.
                        if new_target.is_empty() {
                            // Target list is now empty, remove the Shoot rule.
                            // The shooter ship is registered as a publisher.
                            cache_registrations_from_store
                                .entry(variable.clone())
                                .or_default()
                                .insert((rule.ship.clone(), RatPubRegisterKind::Publish));
                        } else {
                            // Target list is not empty, keep the rule with the updated target list.
                            rules_to_keep_in_variable.push(VariableHuman {
                                ship: rule.ship.clone(),
                                strategy: Some(ActionPlan::Shoot {
                                    target: new_target,
                                    id: *id,
                                }),
                            });
                            variable_has_remaining_rules = true;
                        }
                    } else {
                        // Client was not in the target list, keep the rule as is.
                        rules_to_keep_in_variable.push(rule);
                        variable_has_remaining_rules = true;
                    }
                } else if let Some(ActionPlan::Catch { source, id: _ }) = &rule.strategy {
                    // Case 3: Rule is a Catch action. Check if client is the source.
                    if source == client {
                        // The client being removed is the source of this Catch rule. Remove the rule.
                        // The ship performing the Catch is registered as a subscriber.
                        cache_registrations_from_store
                            .entry(variable.clone())
                            .or_default()
                            .insert((rule.ship.clone(), RatPubRegisterKind::Subscribe));
                    } else {
                        // Client is not the source, keep the rule as is.
                        rules_to_keep_in_variable.push(rule);
                        variable_has_remaining_rules = true;
                    }
                } else {
                    // Case 4: Rule does not involve the client, keep it.
                    rules_to_keep_in_variable.push(rule);
                    variable_has_remaining_rules = true;
                }
            }

            // Decide whether to update or remove the variable entry in the store
            if variable_has_remaining_rules {
                store_updates.insert(variable.clone(), rules_to_keep_in_variable);
            } else {
                variables_to_remove_from_store.push(variable.clone());
            }
        }

        // Apply store updates and removals
        for (variable, updated_rules) in store_updates {
            self.store.insert(variable, updated_rules);
        }
        for var in variables_to_remove_from_store {
            self.store.remove(&var);
        }

        // Add collected cache registrations from store processing to the main cache
        for (variable, registrations) in cache_registrations_from_store {
            self.cache
                .entry(variable)
                .or_default()
                .extend(registrations);
        }

        // Process cache registrations
        let mut variables_to_remove_from_cache = Vec::new();
        for (variable, registrations) in self.cache.iter_mut() {
            let initial_count = registrations.len();
            registrations.retain(|(c, _)| c != client);
            if registrations.is_empty() && initial_count > 0 {
                variables_to_remove_from_cache.push(variable.clone());
            }
        }

        // Remove variables from the cache that have no remaining registrations
        for var in variables_to_remove_from_cache {
            self.cache.remove(&var);
        }

        // After removing clients from both store and cache, resolve the cache
        // to potentially generate new rules from remaining cache entries.
        self.resolve_cache();
    }

    pub fn clear(&mut self) {
        self.store.clear();
    }

    pub fn raw(&self) -> &std::collections::HashMap<String, Vec<VariableHuman>> {
        &self.store
    }

    /// Resolves cache entries into rules in the store based on existing rules and new registrations.
    /// Generates only Shoot actions, never Catch actions.
    /// - If a variable exists in the store AND has existing Shoot rules,
    ///   cache registrations for that variable are used to update/regenerate
    ///   the Shoot rules based on the combined set of publishers and subscribers
    ///   from the store and cache. The cache entry is removed.
    /// - If a variable is NOT in the store, OR is in the store but lacks Shoot rules,
    ///   it requires a publisher/subscriber pair *within the cache* to generate new
    ///   Shoot rules. These are added (either to a new entry or by extending
    ///   a non-Shoot entry in the store). If no pair exists in the cache,
    ///   the cache entry remains.
    pub fn resolve_cache(&mut self) {
        // Collect variables from cache keys to avoid borrowing issues while modifying
        let vars_to_process: Vec<Variable> = self.cache.keys().cloned().collect();

        for var_name in vars_to_process {
            // Get and remove cache registrations for this variable
            if let Some(registrations) = self.cache.remove(&var_name) {
                let mut new_pubs: HashSet<String> = HashSet::new();
                let mut new_subs: HashSet<String> = HashSet::new();

                for (client, kind) in registrations.iter() {
                    match kind {
                        RatPubRegisterKind::Publish => {
                            new_pubs.insert(client.clone());
                        }
                        RatPubRegisterKind::Subscribe => {
                            new_subs.insert(client.clone());
                        }
                    }
                }

                // Temporarily remove existing rules from the store
                let existing_rules_option = self.store.remove(&var_name);

                // Check if the existing store entry (if any) contains Shoot rules
                // We now only care about existing Shoot rules for the first case
                let had_existing_shoot = &existing_rules_option.as_ref().and_then(|rules| {
                    rules
                        .iter()
                        .find(|vh| matches!(vh.strategy, Some(ActionPlan::Shoot { .. })))
                        .map(|v| match v.strategy.as_ref() {
                            // TODO cleanup
                            Some(ap) => match ap {
                                ActionPlan::Sail => unreachable!(),
                                ActionPlan::Shoot { target: _, id } => id,
                                ActionPlan::Catch { .. } => unreachable!(),
                            },
                            None => unreachable!(),
                        })
                });

                let mut generated_rules: Vec<VariableHuman> = Vec::new();

                match had_existing_shoot {
                    Some(id) => {
                        // Case 1: Variable was in store AND had Shoot rules.
                        // Combine clients from existing rules and new cache registrations, then regenerate Shoot rules.
                        let mut all_pubs: HashSet<String> = HashSet::new();
                        let mut all_subs: HashSet<String> = HashSet::new();
                        let mut other_vh: Vec<VariableHuman> = Vec::new();
                        let id = **id;
                        if let Some(existing_rules) = existing_rules_option {
                            for vh in existing_rules {
                                match vh.strategy {
                                    Some(ActionPlan::Shoot { target, id: _ }) => {
                                        all_pubs.insert(vh.ship);
                                        // Also collect existing subscribers from targets of existing shoots
                                        all_subs.extend(target.into_iter());
                                    }
                                    // Treat existing Catch rules' ships as subscribers for combining lists
                                    Some(ActionPlan::Catch { source: _, id: _ }) => {
                                        all_subs.insert(vh.ship);
                                    }
                                    _ => {
                                        other_vh.push(vh);
                                    }
                                }
                            }
                        }
                        // Add new clients from cache registrations
                        all_pubs.extend(new_pubs.into_iter());
                        all_subs.extend(new_subs.into_iter());

                        let mut sorted_all_pubs: Vec<String> = all_pubs.into_iter().collect();
                        sorted_all_pubs.sort();
                        let mut sorted_all_subs: Vec<String> = all_subs.into_iter().collect();
                        sorted_all_subs.sort();
                        generated_rules.extend(other_vh);

                        if !sorted_all_pubs.is_empty() && !sorted_all_subs.is_empty() {
                            // Add Shoot rules for all publishers targeting all subscribers
                            for pub_client in &sorted_all_pubs {
                                generated_rules.push(VariableHuman {
                                    ship: pub_client.clone(),
                                    strategy: Some(ActionPlan::Shoot {
                                        target: sorted_all_subs.clone(),
                                        id,
                                    }),
                                });
                            }
                        }

                        // Re-insert the generated rules into the store
                        self.store.insert(var_name.clone(), generated_rules);
                    }
                    None => {
                        // Case 2: Variable was NOT in store, OR was in store but only had non-Shoot rules.
                        // Require a publisher/subscriber pair *in the cache* to generate rules.
                        if !new_pubs.is_empty() && !new_subs.is_empty() {
                            // Generate Shoot rules (new_pubs -> new_subs) from the cache pair
                            let mut sorted_new_pubs: Vec<String> = new_pubs.into_iter().collect();
                            sorted_new_pubs.sort();
                            let mut sorted_new_subs: Vec<String> = new_subs.into_iter().collect();
                            sorted_new_subs.sort();

                            let mut pair_generated_rules: Vec<VariableHuman> = Vec::new();
                            let pair_id = self.new_id();
                            for pub_client in &sorted_new_pubs {
                                pair_generated_rules.push(VariableHuman {
                                    ship: pub_client.clone(),
                                    strategy: Some(ActionPlan::Shoot {
                                        target: sorted_new_subs.clone(),
                                        id: pair_id,
                                    }),
                                });
                            }

                            if let Some(mut existing_rules) = existing_rules_option {
                                // Extend the existing non-Shoot rules with the newly generated pair Shoot rules
                                existing_rules.extend(pair_generated_rules);
                                self.store.insert(var_name.clone(), existing_rules);
                            } else {
                                // Insert a new entry with the generated pair Shoot rules
                                self.store.insert(var_name.clone(), pair_generated_rules);
                            }
                        } else {
                            // Case 3: No pair in cache, and no Shoot in store rules.
                            // Put the original store rules back (if any) and keep the cache entry.
                            if let Some(existing_rules) = existing_rules_option {
                                self.store.insert(var_name.clone(), existing_rules);
                            }
                            // Put original cache registrations back as they weren't processed
                            self.cache.insert(var_name.clone(), registrations);
                        }
                    }
                }
            }
        }
    }

    pub fn register(&mut self, var: String, client: String, kind: RatPubRegisterKind) {
        match self.cache.get_mut(&var) {
            Some(el) => {
                el.insert((client, kind));
            }
            None => {
                let mut set = HashSet::new();
                set.insert((client, kind));
                self.cache.insert(var, set);
            }
        }

        self.resolve_cache();
    }
}

#[derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize, Clone, Debug, PartialEq, Eq)]
pub struct VariableHuman {
    pub ship: String,
    pub strategy: Option<ActionPlan>,
}

#[derive(
    rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Debug, Clone, Default, PartialEq, Eq,
)]
pub enum ActionPlan {
    #[default]
    Sail,
    Shoot {
        target: Vec<String>,
        id: u32,
    },
    Catch {
        source: String,
        id: u32,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::{HashMap, HashSet};

    // Helper function to sort VariableHuman vectors for consistent comparison
    fn sort_variable_humans(humans: &mut Vec<VariableHuman>) {
        // Sort the outer vector based on ship and then strategy debug representation
        humans.sort_by(|a, b| {
            a.ship
                .cmp(&b.ship)
                .then(format!("{:?}", a.strategy).cmp(&format!("{:?}", b.strategy)))
        });
        // Also sort the target vector inside Shoot strategies for deterministic comparison
        for human in humans.iter_mut() {
            if let Some(ActionPlan::Shoot { target, id: _ }) = &mut human.strategy {
                target.sort();
            }
        }
    }
    fn create_rules() -> Rules {
        Rules::new()
    }

    #[test]
    fn test_remove_client_main_ship_no_partner() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "client1".to_string(),
                strategy: Some(ActionPlan::Sail),
            }],
        );

        rules.remove_client("client1");

        // client1's rule should be removed
        assert_eq!(rules.store.len(), 0);
    }

    #[test]
    fn test_remove_client_main_ship_catch() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "client1".to_string(),
                strategy: Some(ActionPlan::Catch {
                    source: "client2".to_string(),
                    id: 0,
                }),
            }],
        );

        rules.remove_client("client1");

        // client1's rule should be removed from store
        assert_eq!(rules.store.len(), 0);
        // client2 should be registered as a publisher in cache and resolved into a shoot rule
        assert_eq!(rules.cache.len(), 1); // Cache should be resolved
        let generated_rules = rules.cache.get("var1").unwrap();
        assert_eq!(generated_rules.len(), 1); // resolve_cache requires a pub/sub pair in cache if no existing shoot rule

        // Let's re-run with an existing subscriber in cache for the resolution to work
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "client1".to_string(),
                strategy: Some(ActionPlan::Catch {
                    source: "client2".to_string(),
                    id: 0,
                }),
            }],
        );
        rules
            .cache
            .entry("var1".to_string())
            .or_default()
            .insert(("client3".to_string(), RatPubRegisterKind::Subscribe));

        rules.remove_client("client1");
        assert_eq!(rules.store.len(), 1);
        assert!(rules.store.contains_key("var1"));
        let generated_rules = rules.store.get("var1").unwrap();
        assert_eq!(generated_rules.len(), 1);
        assert_eq!(generated_rules[0].ship, "client2");
        assert_eq!(
            generated_rules[0].strategy,
            Some(ActionPlan::Shoot {
                target: vec!["client3".to_string()],
                id: 1
            })
        );
        assert_eq!(rules.cache.len(), 0); // Cache should be resolved
    }

    #[test]
    fn test_remove_client_main_ship_shoot() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "client1".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["client2".to_string(), "client3".to_string()],
                    id: 0,
                }),
            }],
        );

        rules.remove_client("client1");

        // client1's rule should be removed from store
        assert_eq!(rules.store.len(), 0);
        // client2 and client3 should be registered as subscribers in cache and resolved
        let generated_rules = rules.cache.get("var1").unwrap();
        assert_eq!(generated_rules.len(), 2); // Cache should be resolved
    }

    #[test]
    fn test_remove_client_target_in_shoot_partial_removal() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![
                VariableHuman {
                    ship: "shooter1".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["client_to_remove".to_string(), "client2".to_string()],
                        id: 0,
                    }),
                },
                VariableHuman {
                    ship: "shooter2".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["client2".to_string(), "client_to_remove".to_string()],
                        id: 0,
                    }),
                },
            ],
        );

        rules.remove_client("client_to_remove");

        // client_to_remove should be removed from target lists
        assert_eq!(rules.store.len(), 1);
        assert!(rules.store.contains_key("var1"));
        let remaining_rules = rules.store.get("var1").unwrap();
        assert_eq!(remaining_rules.len(), 2);

        let rule1 = &remaining_rules[0];
        let rule2 = &remaining_rules[1];

        // Order might not be guaranteed, check both possibilities
        if rule1.ship == "shooter1" {
            assert_eq!(
                rule1.strategy,
                Some(ActionPlan::Shoot {
                    target: vec!["client2".to_string()],
                    id: 0
                })
            );
            assert_eq!(rule2.ship, "shooter2");
            assert_eq!(
                rule2.strategy,
                Some(ActionPlan::Shoot {
                    target: vec!["client2".to_string()],
                    id: 0
                })
            );
        } else {
            assert_eq!(rule1.ship, "shooter2");
            assert_eq!(
                rule1.strategy,
                Some(ActionPlan::Shoot {
                    target: vec!["client2".to_string()],
                    id: 0
                })
            );
            assert_eq!(rule2.ship, "shooter1");
            assert_eq!(
                rule2.strategy,
                Some(ActionPlan::Shoot {
                    target: vec!["client2".to_string()],
                    id: 0
                })
            );
        }

        assert_eq!(rules.cache.len(), 0); // No rules were fully removed, no new cache registrations from store
    }

    #[test]
    fn test_remove_client_target_in_shoot_full_removal() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "shooter1".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["client_to_remove".to_string()],
                    id: 0,
                }),
            }],
        );

        rules.remove_client("client_to_remove");

        assert_eq!(rules.cache.len(), 1);
        assert_eq!(rules.store.len(), 0);

        // Let's re-run with an existing subscriber in cache for the resolution to work
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "shooter1".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["client_to_remove".to_string()],
                    id: 0,
                }),
            }],
        );
        rules
            .cache
            .entry("var1".to_string())
            .or_default()
            .insert(("client2".to_string(), RatPubRegisterKind::Subscribe));

        rules.remove_client("client_to_remove");

        assert_eq!(rules.store.len(), 1);
        assert!(rules.store.contains_key("var1"));
        let generated_rules = rules.store.get("var1").unwrap();
        assert_eq!(generated_rules.len(), 1);
        assert_eq!(generated_rules[0].ship, "shooter1");
        assert_eq!(
            generated_rules[0].strategy,
            Some(ActionPlan::Shoot {
                target: vec!["client2".to_string()],
                id: 1
            })
        );
        assert_eq!(rules.cache.len(), 0); // Cache should be resolved
    }

    #[test]
    fn test_remove_client_from_cache_partial_removal() {
        let mut rules = create_rules();
        rules
            .cache
            .entry("var1".to_string())
            .or_default()
            .insert(("client_to_remove".to_string(), RatPubRegisterKind::Publish));
        rules
            .cache
            .entry("var1".to_string())
            .or_default()
            .insert(("client2".to_string(), RatPubRegisterKind::Subscribe));
        rules.cache.entry("var2".to_string()).or_default().insert((
            "client_to_remove".to_string(),
            RatPubRegisterKind::Subscribe,
        ));
        rules
            .cache
            .entry("var2".to_string())
            .or_default()
            .insert(("client3".to_string(), RatPubRegisterKind::Publish));

        rules.remove_client("client_to_remove");

        // client_to_remove's cache entries should be removed
        // var1 cache: should only have client2 (Subscribe)
        // var2 cache: should only have client3 (Publish)
        // The cache should then be resolved.

        assert_eq!(rules.cache.len(), 2); // Cache should be resolved
        assert_eq!(rules.store.len(), 0);
    }

    #[test]
    fn test_remove_client_from_cache_full_removal() {
        let mut rules = create_rules();
        rules
            .cache
            .entry("var1".to_string())
            .or_default()
            .insert(("client_to_remove".to_string(), RatPubRegisterKind::Publish));

        rules.remove_client("client_to_remove");

        // var1 cache should be empty and removed
        assert_eq!(rules.cache.len(), 0);
        assert_eq!(rules.store.len(), 0); // No cache entries to resolve into store rules
    }

    #[test]
    fn test_remove_client_involved_in_multiple_rules_and_variables() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![
                VariableHuman {
                    ship: "client_to_remove".to_string(),
                    strategy: Some(ActionPlan::Sail),
                },
                VariableHuman {
                    ship: "shooter1".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["client_to_remove".to_string(), "client2".to_string()],
                        id: 0,
                    }),
                },
            ],
        );
        rules.insert(
            "var2".to_string(),
            vec![VariableHuman {
                ship: "client3".to_string(),
                strategy: Some(ActionPlan::Catch {
                    source: "client_to_remove".to_string(),
                    id: 0,
                }),
            }],
        );
        rules.cache.entry("var1".to_string()).or_default().insert((
            "client_to_remove".to_string(),
            RatPubRegisterKind::Subscribe,
        ));
        rules
            .cache
            .entry("var3".to_string())
            .or_default()
            .insert(("client_to_remove".to_string(), RatPubRegisterKind::Publish));
        rules
            .cache
            .entry("var3".to_string())
            .or_default()
            .insert(("client4".to_string(), RatPubRegisterKind::Subscribe));

        rules.remove_client("client_to_remove");

        assert_eq!(rules.cache.len(), 2); // Cache should be resolved

        assert_eq!(rules.store.len(), 1);
        assert!(rules.store.contains_key("var1"));

        let var1_rules = rules.store.get("var1").unwrap();
        assert_eq!(var1_rules.len(), 1);
        assert_eq!(var1_rules[0].ship, "shooter1");
        assert_eq!(
            var1_rules[0].strategy,
            Some(ActionPlan::Shoot {
                target: vec!["client2".to_string()],
                id: 0
            })
        );
    }

    #[test]
    fn test_remove_client_not_present() {
        let mut rules = create_rules();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "client1".to_string(),
                strategy: Some(ActionPlan::Sail),
            }],
        );
        rules
            .cache
            .entry("var2".to_string())
            .or_default()
            .insert(("client2".to_string(), RatPubRegisterKind::Publish));

        let initial_store = rules.store.clone();
        let initial_cache = rules.cache.clone();

        rules.remove_client("client_not_present");

        // No changes should occur
        assert_eq!(rules.store, initial_store);
        assert_eq!(rules.cache, initial_cache);
    }
    #[test]
    fn test_resolve_cache_basic_pair() {
        let mut rules = Rules::new();
        rules.register(
            "position".to_string(),
            "ShipA".to_string(),
            RatPubRegisterKind::Publish,
        );
        rules.register(
            "position".to_string(),
            "ShipB".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        rules.resolve_cache();

        // Check the store
        let mut expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            let var_humans = vec![VariableHuman {
                ship: "ShipA".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["ShipB".to_string()],
                    id: 1,
                }),
            }];
            map.insert("position".to_string(), var_humans);
            map
        };

        let mut actual_store = rules.store.clone();

        // Sort both actual and expected for comparison
        if let Some(humans) = actual_store.get_mut("position") {
            sort_variable_humans(humans);
        }
        if let Some(humans) = expected_store.get_mut("position") {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store, expected_store);

        // Check the cache
        assert!(rules.cache.is_empty());
    }

    #[test]
    fn test_resolve_cache_multiple_pubs_subs() {
        let mut rules = Rules::new();
        rules.register(
            "velocity".to_string(),
            "ShipX".to_string(),
            RatPubRegisterKind::Publish,
        );
        rules.register(
            "velocity".to_string(),
            "ShipY".to_string(),
            RatPubRegisterKind::Publish,
        );
        rules.register(
            "velocity".to_string(),
            "ShipZ".to_string(),
            RatPubRegisterKind::Subscribe,
        );
        rules.register(
            "velocity".to_string(),
            "ShipW".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        rules.resolve_cache();

        // Check the store
        let expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            // Define the expected rules (order doesn't matter initially, will sort later)
            let var_humans = vec![
                // Publishers shooting at all subscribers (targets should be sorted by resolve_cache)
                VariableHuman {
                    ship: "ShipX".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipW".to_string(), "ShipZ".to_string()],
                        id: 1,
                    }),
                },
                VariableHuman {
                    ship: "ShipY".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipW".to_string(), "ShipZ".to_string()],
                        id: 1,
                    }),
                },
            ];
            map.insert("velocity".to_string(), var_humans);
            map
        };

        // Clone and sort the actual store for comparison
        let mut actual_store_sorted = rules.store.clone();
        if let Some(humans) = actual_store_sorted.get_mut("velocity") {
            sort_variable_humans(humans); // Sorts outer vec and inner targets
        }

        // Clone and sort the expected store for comparison
        let mut expected_store_sorted = expected_store.clone();
        if let Some(humans) = expected_store_sorted.get_mut("velocity") {
            sort_variable_humans(humans); // Sorts outer vec and inner targets
        }

        assert_eq!(actual_store_sorted, expected_store_sorted);

        // Check the cache
        assert!(rules.cache.is_empty());
    }

    #[test]
    fn test_resolve_cache_no_pair_pubs_only() {
        let mut rules = Rules::new();
        rules.register(
            "fuel".to_string(),
            "ShipA".to_string(),
            RatPubRegisterKind::Publish,
        );
        rules.register(
            "fuel".to_string(),
            "ShipC".to_string(),
            RatPubRegisterKind::Publish,
        );

        rules.resolve_cache();

        // Check the store - should be unchanged as 'fuel' was not in store and no pair formed
        assert!(rules.store.is_empty());

        // Check the cache - entries should still be there
        let expected_cache: HashMap<Variable, HashSet<(Client, RatPubRegisterKind)>> = {
            let mut map = HashMap::new();
            let mut set = HashSet::new();
            set.insert(("ShipA".to_string(), RatPubRegisterKind::Publish));
            set.insert(("ShipC".to_string(), RatPubRegisterKind::Publish));
            map.insert("fuel".to_string(), set);
            map
        };
        assert_eq!(rules.cache, expected_cache);
    }

    #[test]
    fn test_resolve_cache_no_pair_subs_only() {
        let mut rules = Rules::new();
        rules.register(
            "cargo".to_string(),
            "ShipB".to_string(),
            RatPubRegisterKind::Subscribe,
        );
        rules.register(
            "cargo".to_string(),
            "ShipD".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        rules.resolve_cache();

        // Check the store - should be unchanged as 'cargo' was not in store and no pair formed
        assert!(rules.store.is_empty());

        // Check the cache - entries should still be there
        let expected_cache: HashMap<Variable, HashSet<(Client, RatPubRegisterKind)>> = {
            let mut map = HashMap::new();
            let mut set = HashSet::new();
            set.insert(("ShipB".to_string(), RatPubRegisterKind::Subscribe));
            set.insert(("ShipD".to_string(), RatPubRegisterKind::Subscribe));
            map.insert("cargo".to_string(), set);
            map
        };
        assert_eq!(rules.cache, expected_cache);
    }

    #[test]
    fn test_resolve_cache_mixed_variables() {
        let mut rules = Rules::new();
        // Variable with a pair (should be resolved and added to store)
        rules.register(
            "shields".to_string(),
            "ShipF".to_string(),
            RatPubRegisterKind::Publish,
        );
        rules.register(
            "shields".to_string(),
            "ShipG".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        // Variable without a pair (should remain in cache)
        rules.register(
            "waypoints".to_string(),
            "ShipH".to_string(),
            RatPubRegisterKind::Publish,
        );

        rules.resolve_cache();

        // Check the store - only 'shields' should be present, generated from the pair
        let mut expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            let var_humans = vec![VariableHuman {
                ship: "ShipF".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["ShipG".to_string()],
                    id: 1,
                }),
            }];
            map.insert("shields".to_string(), var_humans);
            map
        };

        let mut actual_store = rules.store.clone();

        // Sort both actual and expected for comparison
        if let Some(humans) = actual_store.get_mut("shields") {
            sort_variable_humans(humans);
        }
        if let Some(humans) = expected_store.get_mut("shields") {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store, expected_store);

        // Check the cache - only 'waypoints' should remain
        let expected_cache: HashMap<Variable, HashSet<(Client, RatPubRegisterKind)>> = {
            let mut map = HashMap::new();
            let mut set = HashSet::new();
            set.insert(("ShipH".to_string(), RatPubRegisterKind::Publish));
            map.insert("waypoints".to_string(), set);
            map
        };
        assert_eq!(rules.cache, expected_cache);
    }

    #[test]
    fn test_resolve_cache_state_after_mixed_plus_new_subs() {
        let mut rules = Rules::new();

        // Set up the state expected after test_resolve_cache_mixed_variables
        // Store should have 'shields' rules
        let shields_rules = vec![VariableHuman {
            ship: "ShipF".to_string(),
            strategy: Some(ActionPlan::Shoot {
                target: vec!["ShipG".to_string()],
                id: 0,
            }),
        }];
        rules.insert("shields".to_string(), shields_rules.clone());

        // Cache should have 'waypoints' pub-only registration
        rules.register(
            "waypoints".to_string(),
            "ShipH".to_string(),
            RatPubRegisterKind::Publish,
        );

        // Add new cache entries: subs-only for a new topic 'status'
        rules.register(
            "status".to_string(),
            "ShipI".to_string(),
            RatPubRegisterKind::Subscribe,
        );
        rules.register(
            "status".to_string(),
            "ShipJ".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        // Clone initial states to compare against after resolution
        let initial_store = rules.store.clone();
        let initial_cache = rules.cache.clone();

        rules.resolve_cache();

        // Expected: Store remains unchanged ('status' cache didn't form pair, 'waypoints' still no sub)
        // We need to sort the initial store for comparison as insert order isn't guaranteed
        let mut initial_store_sorted = initial_store.clone();
        if let Some(humans) = initial_store_sorted.get_mut("shields") {
            sort_variable_humans(humans);
        }
        let mut actual_store_sorted = rules.store.clone();
        if let Some(humans) = actual_store_sorted.get_mut("shields") {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store_sorted, initial_store_sorted);

        // Expected: Cache remains unchanged ('status' and 'waypoints' entries persist)
        assert_eq!(rules.cache, initial_cache);
    }

    #[test]
    fn test_resolve_cache_state_after_prev_plus_new_pub() {
        let mut rules = Rules::new();

        // Set up the state expected after the previous test
        // Store should have 'shields' rules
        let shields_rules = vec![VariableHuman {
            ship: "ShipF".to_string(),
            strategy: Some(ActionPlan::Shoot {
                target: vec!["ShipG".to_string()],
                id: 0,
            }),
        }];
        rules.insert("shields".to_string(), shields_rules.clone());

        // Cache should have 'waypoints' pub-only registration
        rules.register(
            "waypoints".to_string(),
            "ShipH".to_string(),
            RatPubRegisterKind::Publish,
        );
        // Cache should have 'status' subs-only registrations
        rules.register(
            "status".to_string(),
            "ShipI".to_string(),
            RatPubRegisterKind::Subscribe,
        );
        rules.register(
            "status".to_string(),
            "ShipJ".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        // Add the new cache entry: publisher for 'status'
        rules.register(
            "status".to_string(),
            "ShipK".to_string(),
            RatPubRegisterKind::Publish,
        );

        rules.resolve_cache();

        // Check the store - Should contain original 'shields' rules + new 'status' rules from the pair
        let mut expected_store: HashMap<Variable, Vec<VariableHuman>> = HashMap::new();
        expected_store.insert("shields".to_string(), shields_rules); // Original shields rules

        // Expected 'status' rules (ShipK publishes, ShipI and ShipJ subscribe)
        let status_rules = vec![VariableHuman {
            ship: "ShipK".to_string(),
            strategy: Some(ActionPlan::Shoot {
                target: vec!["ShipI".to_string(), "ShipJ".to_string()],
                id: 1,
            }),
        }];
        expected_store.insert("status".to_string(), status_rules);

        let mut actual_store_sorted = rules.store.clone();
        if let Some(humans) = actual_store_sorted.get_mut("shields") {
            sort_variable_humans(humans);
        }
        if let Some(humans) = actual_store_sorted.get_mut("status") {
            sort_variable_humans(humans);
        }

        let mut expected_store_sorted = expected_store.clone();
        if let Some(humans) = expected_store_sorted.get_mut("shields") {
            sort_variable_humans(humans);
        }
        if let Some(humans) = expected_store_sorted.get_mut("status") {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store_sorted, expected_store_sorted);

        // Check the cache - 'waypoints' should remain, 'status' should be gone
        let expected_cache: HashMap<Variable, HashSet<(Client, RatPubRegisterKind)>> = {
            let mut map = HashMap::new();
            let mut set = HashSet::new();
            set.insert(("ShipH".to_string(), RatPubRegisterKind::Publish));
            map.insert("waypoints".to_string(), set);
            map
        };
        assert_eq!(rules.cache, expected_cache);
    }

    #[test]
    fn test_resolve_cache_empty_cache() {
        let mut rules = Rules::new();
        // Add some initial data to store to ensure it's not affected
        rules.insert(
            "initial".to_string(),
            vec![VariableHuman {
                ship: "ShipI".to_string(),
                strategy: Some(ActionPlan::Sail),
            }],
        );

        rules.resolve_cache();

        // Store should be unchanged
        let expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            map.insert(
                "initial".to_string(),
                vec![VariableHuman {
                    ship: "ShipI".to_string(),
                    strategy: Some(ActionPlan::Sail),
                }],
            );
            map
        };
        assert_eq!(rules.store, expected_store);

        // Cache should remain empty
        assert!(rules.cache.is_empty());
    }

    #[test]
    fn test_resolve_cache_store_with_rules_add_sub() {
        let mut rules = Rules::new();
        let var_name = "data_stream".to_string();

        // Initial Shoot/Catch rules in store
        rules.insert(
            var_name.clone(),
            vec![
                VariableHuman {
                    ship: "ShipA".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipB".to_string()],
                        id: 0,
                    }),
                },
                VariableHuman {
                    ship: "MonitorShip".to_string(),
                    strategy: Some(ActionPlan::Sail),
                }, // Existing Sail rule
            ],
        );

        // Add registration to cache: new subscriber
        rules.register(
            var_name.clone(),
            "ShipC".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        rules.resolve_cache();

        // Check the store - Should have updated Shoot for ShipA and new Catch for ShipC
        let expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            // Combined unique pubs: ShipA (from store)
            // Combined unique subs: ShipB (from store), ShipC (from cache)
            let var_humans = vec![
                VariableHuman {
                    ship: "MonitorShip".to_string(),
                    strategy: Some(ActionPlan::Sail),
                }, // Keep existing Sail rule
                VariableHuman {
                    ship: "ShipA".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipB".to_string(), "ShipC".to_string()],
                        id: 0,
                    }),
                }, // Updated target list
            ];
            map.insert(var_name.clone(), var_humans);
            map
        };

        let mut actual_store_sorted = rules.store.clone();
        if let Some(humans) = actual_store_sorted.get_mut(&var_name) {
            sort_variable_humans(humans);
        }

        let mut expected_store_sorted = expected_store.clone();
        if let Some(humans) = expected_store_sorted.get_mut(&var_name) {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store_sorted, expected_store_sorted);

        // Check the cache - entry for 'data_stream' should be removed
        assert!(!rules.cache.contains_key(&var_name));
        assert!(rules.cache.is_empty()); // Assuming no other cache entries
    }

    #[test]
    fn test_resolve_cache_store_with_rules_add_pub() {
        let mut rules = Rules::new();
        let var_name = "data_stream".to_string();

        // Initial Shoot/Catch rules in store
        rules.insert(
            var_name.clone(),
            vec![
                VariableHuman {
                    ship: "ShipA".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipB".to_string()],
                        id: 0,
                    }),
                },
                VariableHuman {
                    ship: "MonitorShip".to_string(),
                    strategy: Some(ActionPlan::Sail),
                }, // Existing Sail rule
            ],
        );

        // Add registration to cache: new subscriber
        rules.register(
            var_name.clone(),
            "ShipC".to_string(),
            RatPubRegisterKind::Publish,
        );

        rules.resolve_cache();

        let expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            let var_humans = vec![
                VariableHuman {
                    ship: "MonitorShip".to_string(),
                    strategy: Some(ActionPlan::Sail),
                }, // Keep existing Sail rule
                VariableHuman {
                    ship: "ShipA".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipB".to_string()],
                        id: 0,
                    }),
                }, // Existing Catch rule remains
                VariableHuman {
                    ship: "ShipC".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec!["ShipB".to_string()],
                        id: 0,
                    }),
                },
            ];
            map.insert(var_name.clone(), var_humans);
            map
        };

        let mut actual_store_sorted = rules.store.clone();
        if let Some(humans) = actual_store_sorted.get_mut(&var_name) {
            sort_variable_humans(humans);
        }

        let mut expected_store_sorted = expected_store.clone();
        if let Some(humans) = expected_store_sorted.get_mut(&var_name) {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store_sorted, expected_store_sorted);

        // Check the cache - entry for 'data_stream' should be removed
        assert!(!rules.cache.contains_key(&var_name));
        assert!(rules.cache.is_empty()); // Assuming no other cache entries
    }

    #[test]
    fn test_resolve_cache_store_with_rules_add_pub_sub() {
        let mut rules = Rules::new();
        let var_name = "telemetry".to_string();

        // Initial Shoot/Catch rules in store
        rules.insert(
            var_name.clone(),
            vec![VariableHuman {
                ship: "BaseStation".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["Rover1".to_string()],
                    id: 0,
                }),
            }],
        );

        // Add registrations to cache: new publisher and new subscriber
        rules.register(
            var_name.clone(),
            "BaseStationBackup".to_string(),
            RatPubRegisterKind::Publish,
        );
        rules.register(
            var_name.clone(),
            "Rover2".to_string(),
            RatPubRegisterKind::Subscribe,
        );

        rules.resolve_cache();

        // Check the store - Should update all Shoot/Catch rules based on all 4 ships
        let expected_store: HashMap<Variable, Vec<VariableHuman>> = {
            let mut map = HashMap::new();
            // Combined unique pubs: BaseStation (store), BaseStationBackup (cache)
            // Combined unique subs: Rover1 (store), Rover2 (cache)
            let all_subs = vec!["Rover1".to_string(), "Rover2".to_string()]; // Already sorted

            let var_humans = vec![
                // Shoot rules for all pubs targeting all subs
                VariableHuman {
                    ship: "BaseStation".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: all_subs.clone(),
                        id: 0,
                    }),
                },
                VariableHuman {
                    ship: "BaseStationBackup".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: all_subs.clone(),
                        id: 0,
                    }),
                },
            ];
            map.insert(var_name.clone(), var_humans);
            map
        };

        let mut actual_store_sorted = rules.store.clone();
        if let Some(humans) = actual_store_sorted.get_mut(&var_name) {
            sort_variable_humans(humans);
        }

        let mut expected_store_sorted = expected_store.clone();
        if let Some(humans) = expected_store_sorted.get_mut(&var_name) {
            sort_variable_humans(humans);
        }

        assert_eq!(actual_store_sorted, expected_store_sorted);

        // Check the cache - entry for 'telemetry' should be removed
        assert!(!rules.cache.contains_key(&var_name));
        assert!(rules.cache.is_empty()); // Assuming no other cache entries
    }
}
