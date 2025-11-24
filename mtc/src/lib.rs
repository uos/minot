use core::fmt;
use std::{
    collections::{HashMap, HashSet},
    path::PathBuf,
    str::FromStr,
};

use anyhow::anyhow;
use ariadne::{Color, Label, Report, ReportKind, Source};
use logos::{Lexer, Logos};
use rkyv::{Archive, Deserialize, Serialize};

use chumsky::{
    container::Seq,
    input::{Stream, ValueInput},
    prelude::*,
};

type Client = String;
type Variable = String;

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

pub const COMPARE_NODE_NAME: &str = "#minot-tui";

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

    fn new_id(&mut self) -> u32 {
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

                        // Build the new set of rules from the combined, unique, sorted client lists
                        generated_rules.extend(other_vh); // Keep the non-Shoot/Catch rules

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
                            // *** Removed Catch rule generation here ***
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
                            // *** Removed Catch rule generation here ***

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

                // Cache entry was removed at the start if processed.
                // If not processed (Case 3), it was inserted back.
                // This logic is handled by the `cache_processed` flag.
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

#[derive(Archive, Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
pub struct VariableHuman {
    pub ship: String,
    pub strategy: Option<ActionPlan>,
}

#[derive(Archive, Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
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

fn floating_millimeter<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let f: f64 = slice[..slice.len() - 2].parse().ok()?;
    Some(f)
}

fn floating_centimeter<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let f: f64 = slice[..slice.len() - 2].parse().ok()?;
    Some(f * 10.0)
}

fn floating_meter<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let f: f64 = slice[..slice.len() - 1].parse().ok()?;
    Some(f * 10.0 * 100.0)
}

fn integer_millimeter<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let f: i64 = slice[..slice.len() - 2].parse().ok()?;
    Some(f)
}

fn integer_centimeter<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let f: i64 = slice[..slice.len() - 2].parse().ok()?;
    Some(f * 10)
}

fn integer_meter<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let f: i64 = slice[..slice.len() - 1].parse().ok()?;
    Some(f * 10 * 100)
}

fn integer_second<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let f: i64 = slice[..slice.len() - 1].parse().ok()?;
    Some(f * 1000)
}

fn integer_minute<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let len = if slice.ends_with('s') { 4 } else { 3 };
    let f: i64 = slice[..slice.len() - len].parse().ok()?;
    Some(f * 60 * 1000)
}

fn integer_millisec<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let f: i64 = slice[..slice.len() - 2].parse().ok()?;
    Some(f)
}

fn integer_hour<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<i64> {
    let slice = lex.slice();
    let len = if slice.ends_with('s') { 5 } else { 4 };
    let f: i64 = slice[..slice.len() - len].parse().ok()?;
    Some(f * 60 * 60 * 1000)
}

fn floating_second<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let f: f64 = slice[..slice.len() - 1].parse().ok()?;
    Some(f * 1000.0)
}

fn floating_minute<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let len = if slice.ends_with('s') { 4 } else { 3 };
    let f: f64 = slice[..slice.len() - len].parse().ok()?;
    Some(f * 60.0 * 1000.0)
}

fn floating_millisec<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let f: f64 = slice[..slice.len() - 2].parse().ok()?;
    Some(f)
}

fn floating_hour<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<f64> {
    let slice = lex.slice();
    let len = if slice.ends_with('s') { 5 } else { 4 };
    let f: f64 = slice[..slice.len() - len].parse().ok()?;
    Some(f * 60.0 * 60.0 * 1000.0)
}

fn rm_first<'a>(lex: &mut Lexer<'a, Token<'a>>) -> &'a str {
    let slice = lex.slice();
    &slice[1..slice.len()]
}

fn rm_last<'a>(lex: &mut Lexer<'a, Token<'a>>) -> &'a str {
    let slice = lex.slice();
    &slice[0..slice.len() - 1]
}

fn rm_first_and_last<'a>(lex: &mut Lexer<'a, Token<'a>>) -> &'a str {
    let slice = lex.slice();
    &slice[1..slice.len() - 1]
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum SensorType {
    Lidar,
    Imu,
    Mixed,
    Any,
}

impl FromStr for SensorType {
    type Err = std::io::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(match s.to_lowercase().as_str() {
            "cloud" => Self::Lidar,
            "imu" => Self::Imu,
            "mixed" => Self::Mixed,
            _ => Self::Any,
        })
    }
}

pub const POINTCLOUD_ROS2_TYPE: &str = "sensor_msgs/msg/PointCloud2";
pub const IMU_ROS2_TYPE: &str = "sensor_msgs/msg/Imu";

impl SensorType {
    pub fn is(&self, query: &str) -> bool {
        match self {
            SensorType::Lidar => query == POINTCLOUD_ROS2_TYPE,
            SensorType::Imu => query == IMU_ROS2_TYPE,
            SensorType::Mixed => query == POINTCLOUD_ROS2_TYPE || query == IMU_ROS2_TYPE,
            SensorType::Any => true,
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
enum PlayType {
    SensorCount {
        sensors: Vec<String>,
    },
    UntilSensorCount {
        sending: Vec<String>,
        until_sensors: Vec<String>,
    },
}

#[derive(Logos, Debug, PartialEq, Clone)]
#[logos(skip r"[ \t\f]+")]
enum Token<'a> {
    Error,

    // -- Control --
    #[token("+")]
    OpPlus,

    #[token("-")]
    OpMinus,

    // #[token("<")]
    // OpInclude,
    #[regex(".", priority = 1)]
    Dot,

    #[token(",")]
    Comma,

    #[regex(r"(#.*)?\n")]
    NewLine,

    #[token("(")]
    BracketOpen,

    #[token(")")]
    BracketClose,

    #[token("{")]
    BlockStart,

    #[token("}")]
    BlockEnd,

    #[token("[")]
    LParen,

    #[token("]")]
    RParen,

    // -- Operators --
    #[token("==")]
    OpCompare,

    #[regex("<-|=")]
    OpAssignToLeft,

    #[regex("->", priority = 1000)]
    OpAssignToRight,

    #[token("..")]
    OpRange,

    #[regex(r"\.?\/+[^ \n]*")]
    Path(&'a str),

    #[regex(r#""[^"]+""#, rm_first_and_last)]
    String(&'a str),

    // #[token("::")]
    // EnumDivider,

    // -- Keywords --
    #[token("if")]
    KwIf,

    #[token("loop")]
    KwLoop,

    #[token("LOG")]
    KwLog,

    #[token("~")]
    DoNotCareOptim,

    // -- Embedded functions (Wind) --
    #[regex(r"pf!")]
    FnPlayFrames,

    #[token("reset!")]
    FnReset,

    #[token("rule!")]
    FnRule,

    // -- Expressions --
    #[token("true")]
    True,

    #[token("false")]
    False,

    #[regex(r"[+-]?\d+", |lex| lex.slice().parse().ok())]
    IntegerNumber(i64),

    #[regex(r"[+-]?(\d+)?\.\d*", |lex| lex.slice().parse().ok())]
    FloatingNumber(f64),

    #[regex(r"[+-]?(\d+)?\.\d*mm", floating_millimeter)]
    #[regex(r"[+-]?(\d+)?\.\d*m]", floating_meter)]
    #[regex(r"[+-]?(\d+)?\.\d*cm", floating_centimeter)]
    FloatingNumberMillimeter(f64),

    #[regex(r"[+-]?(\d+)?\.\d*s]", floating_second)]
    #[regex(r"[+-]?(\d+)?\.\d*ms]", floating_millisec)]
    #[regex(r"[+-]?(\d+)?\.\d*mins?]", floating_minute)]
    #[regex(r"[+-]?(\d+)?\.\d*hours?]", floating_hour)]
    FloatingNumberMillisecond(f64),

    #[regex(r"[+-]?(\d+cm)", integer_centimeter)]
    #[regex(r"[+-]?(\d+mm)", integer_millimeter)]
    #[regex(r"[+-]?(\d+m)", integer_meter)]
    IntegerNumberMillimeter(i64),

    #[regex(r"[+-]?(\d+s)", integer_second)]
    #[regex(r"[+-]?(\d+ms)", integer_millisec)]
    #[regex(r"[+-]?(\d+mins)", integer_minute)]
    #[regex(r"[+-]?(\d+hours)", integer_hour)]
    IntegerNumberMillisecond(i64),

    #[regex(r"[a-zA-Z_:]+\d*[a-zA-Z_:\d]*\.", rm_last)]
    VarNamespace(&'a str),

    #[regex(r"[a-zA-Z_:]+\d*[a-zA-Z_:\d]*")]
    Variable(&'a str),

    #[regex(r"_[a-zA-Z_:]+\d*[a-zA-Z_:\d]*", rm_first)]
    PredefVariable(&'a str),
}

impl fmt::Display for Token<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::VarNamespace(ns) => write!(f, "Namespace({:?})", ns),
            Self::Path(path) => write!(f, "Path({:?})", path),
            Self::String(string) => write!(f, "String({:?})", string),
            // Self::EnumDivider => write!(f, "::"),
            Self::FnReset => write!(f, "reset()"),
            Self::FnRule => write!(f, "rule()"),
            Self::Comma => write!(f, ","),
            Self::KwLog => write!(f, "LOG"),
            Self::OpPlus => write!(f, "+"),
            Self::Dot => write!(f, "."),
            Self::OpMinus => write!(f, "-"),
            Self::Error => write!(f, "<error>"),
            Self::NewLine => write!(f, "\\n"),
            Self::BracketOpen => write!(f, "["),
            Self::BracketClose => write!(f, "]"),
            Self::BlockStart => write!(f, "{{"),
            Self::BlockEnd => write!(f, "}}"),
            Self::LParen => write!(f, "("),
            Self::RParen => write!(f, ")"),
            Self::OpCompare => write!(f, "=="),
            Self::OpAssignToLeft => write!(f, "<-"),
            Self::OpAssignToRight => write!(f, "->"),
            Self::OpRange => write!(f, ".."),
            Self::KwIf => write!(f, "if"),
            Self::KwLoop => write!(f, "loop"),
            Self::DoNotCareOptim => write!(f, "~"),
            Self::FnPlayFrames => write!(f, "play_frames()"),
            Self::True => write!(f, "true"),
            Self::False => write!(f, "false"),
            Self::IntegerNumber(s) => write!(f, "{}", s),
            Self::FloatingNumber(s) => write!(f, "{}", s),
            Self::FloatingNumberMillimeter(s) => write!(f, "{}mm", s),
            Self::FloatingNumberMillisecond(s) => write!(f, "{}ms", s),
            Self::IntegerNumberMillimeter(s) => write!(f, "{}mm", s),
            Self::IntegerNumberMillisecond(s) => write!(f, "{}ms", s),
            Self::Variable(s) => write!(f, "{}", s),
            Self::PredefVariable(s) => write!(f, "_{}", s),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum NumVal {
    Floating(f64),
    Integer(i64),
}

impl PartialEq for NumVal {
    fn eq(&self, other: &Self) -> bool {
        let tolerance = 1e-6; // Adjust this value as needed
        match (self, other) {
            (NumVal::Floating(fl), NumVal::Floating(fr)) => (fl - fr).abs() < tolerance,
            (NumVal::Floating(_), NumVal::Integer(_)) => false,
            (NumVal::Integer(_), NumVal::Floating(_)) => false,
            (NumVal::Integer(il), NumVal::Integer(ir)) => il == ir,
        }
    }
}

impl Eq for NumVal {}

impl NumVal {
    pub fn integerize_unit_val(self) -> NumVal {
        if let NumVal::Floating(f) = self {
            NumVal::Integer(f.round() as i64)
        } else {
            self
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
    TimeMilliseconds,
    WayMillimeter,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Val {
    UnitedVal(UnitVal),
    NumVal(NumVal),
    StringVal(String),
    BoolVal(bool),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnitVal {
    pub val: u64,
    pub unit: Unit,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Rhs {
    Range { from: Option<Val>, to: Option<Val> },
    Var(Var),
    Path(String),
    // Expr() TODO evaluate so in the end it becomes atomic Rhs
    Val(Val),
    Array(Vec<Box<Self>>),
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Var {
    Log,
    Predef {
        name: String,
        namespace: Vec<String>,
    },
    User {
        name: String,
        namespace: Vec<String>,
    },
}

fn vec_prepend<T: Clone>(prepend: &[T], source: &[T]) -> Vec<T> {
    let mut out = prepend.to_vec();
    out.extend(source.iter().cloned());
    out
}

impl Var {
    pub fn add_namespace(self, ns: &Vec<String>) -> Self {
        match self {
            Var::User { name, namespace } => Var::User {
                name,
                namespace: vec_prepend(ns, &namespace),
            },
            Var::Log => Var::Log,
            Var::Predef { name, namespace } => Var::Predef {
                name,
                namespace: vec_prepend(ns, &namespace),
            },
        }
    }
}

impl FromStr for Var {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let splitted = s.split(".").collect::<Vec<_>>();

        let l = splitted.len();
        if l < 1 {
            return Err(anyhow!("Nothing to parse"));
        }

        let last = splitted[l - 1];

        Ok(if last == COMPARE_NODE_NAME {
            Self::Log
        } else {
            let mut v: Vec<String> = vec![];
            if l > 1 {
                v.extend(splitted.into_iter().take(l - 1).map(String::from));
            }
            if let Some(predef) = last.strip_prefix("_") {
                Self::Predef {
                    name: predef.to_owned(),
                    namespace: v,
                }
            } else {
                Self::User {
                    name: last.to_owned(),
                    namespace: v,
                }
            }
        })
    }
}

#[derive(Debug)]
pub struct Evaluated {
    pub rules: Rules,
    pub wind: Vec<WindFunction>,
    pub vars: VariableHistory,
}

impl Default for Evaluated {
    fn default() -> Self {
        Self::new()
    }
}

impl Evaluated {
    pub fn new() -> Self {
        Self {
            rules: Rules::new(),
            wind: vec![],
            vars: VariableHistory::new(vec![]),
        }
    }
}

#[derive(Debug)]
pub enum WindFunction {
    Reset(String),
    SendFrames(PlayKindUnitedPass3),
}

#[derive(Debug, Clone)]
pub struct Root<'a> {
    pub rules: RuleSexpr,
    pub wind: Vec<StatementKind<'a>>,
    pub vardefs: Vec<Statement>,
}

pub type PlayTimeMsRange = (u64, u64);

#[derive(Debug, PartialEq, Clone)]
pub enum PlayTrigger {
    DurationMs(u64),
    DurationRelFactor(f64),
    Variable(String),
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PlayMode {
    Dynamic,
    Fix,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum AbsTimeRange {
    Open,
    UpperOpen(u64),
    LowerOpen(u64),
    Closed((u64, u64)),
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PlayCount {
    TimeRelativeMs(u64),
    TimeRangeMs(AbsTimeRange),
    Amount(u64),
}

#[derive(Debug, PartialEq, Clone)]
pub enum SensorIdentification {
    Topic(String),    // no type given, compare by topics
    Type(SensorType), // type given but no topics, check for types
    TopicAndType { topic: String, msg_type: SensorType }, // both given but only use topics for collect until because it is more fine grained
}

#[derive(Debug, PartialEq, Clone)]
pub struct AnySensor {
    pub name: String, // custom name given by config
    pub id: SensorIdentification,
    pub short: Option<String>, // to be used while parsing to not write so much
}

#[derive(Debug, PartialEq, Clone)]
pub enum PlayKindUnited {
    SensorCount {
        sensors: Vec<String>,
        count: PlayCount,
        trigger: Option<PlayTrigger>,
        play_mode: PlayMode,
    },
    UntilSensorCount {
        sending: Vec<String>,
        until_sensors: Vec<String>,
        until_count: PlayCount,
        trigger: Option<PlayTrigger>,
        play_mode: PlayMode,
    },
}

#[derive(Debug, PartialEq, Clone)]
pub enum PlayKindUnitedPass3 {
    SensorCount {
        sensors: Vec<AnySensor>,
        count: PlayCount,
        trigger: Option<PlayTrigger>,
        play_mode: PlayMode,
    },
    UntilSensorCount {
        sending: Vec<AnySensor>,
        until_sensors: Vec<AnySensor>,
        until_count: PlayCount,
        trigger: Option<PlayTrigger>,
        play_mode: PlayMode,
    },
}

#[derive(Debug, Clone)]
pub enum StatementKind<'a> {
    Rule(Rule<'a>),
    VariableDef(Statement),
    Reset(String),
    SendFrames { kind: PlayKindUnited, at: String },
    VariableNamespaceBlock(Vec<Statement>),
}

#[derive(Debug, Clone)]
pub enum StatementKindPass1<'a> {
    Rule(Rule<'a>),
    VariableDef(Statement),
    Reset(String),
    Include {
        namespace: Vec<String>,
        path: String,
    },
    SendFrames(PlayKindUnited),
    VariableNamespaceBlock {
        ns: Vec<&'a str>,
        stmts: Vec<Box<StatementKindPass1<'a>>>,
    },
    Loop {
        count: usize,
        stmts: Vec<Box<StatementKindPass1<'a>>>,
    },
}

#[derive(Debug, Clone)]
pub enum StatementKindOwnedPass1 {
    Rule(RuleOwned),
    Include {
        namespace: Vec<String>,
        path: String,
    },
    VariableDef(Statement),
    VariableNamespaceBlock {
        ns: Vec<String>,
        stmts: Vec<Box<StatementKindOwnedPass1>>,
    },
    Reset(String),
    SendFrames(PlayKindUnited),
    Loop {
        count: usize,
        stmts: Vec<Box<StatementKindOwnedPass1>>,
    },
}

#[derive(Debug, Clone)]
pub enum StatementKindOwned {
    Rule(RuleOwned),
    VariableDef(Statement),
    Reset(String),
    SendFrames(PlayKindUnited),
}

#[derive(Debug, Clone)]
pub enum StatementKindOwnedPass3 {
    Rule(RuleOwned),
    VariableDef(Statement),
    Reset(String),
    SendFrames(PlayKindUnitedPass3),
}

fn do_pass1(
    pass1: Vec<StatementKindOwnedPass1>,
    current_parent: &std::path::Path,
) -> anyhow::Result<Vec<StatementKindOwned>> {
    let mut out = vec![];

    for stmt in pass1 {
        match stmt {
            StatementKindOwnedPass1::Rule(rule_owned) => {
                out.push(StatementKindOwned::Rule(rule_owned));
            }
            StatementKindOwnedPass1::VariableDef(statement) => {
                out.push(StatementKindOwned::VariableDef(statement));
            }
            StatementKindOwnedPass1::VariableNamespaceBlock { ns, stmts } => {
                let v = stmts.into_iter().map(|f| *f).collect::<Vec<_>>();

                let passed = do_pass1(v, current_parent)?;
                out.extend(
                    passed
                        .into_iter()
                        .map(|t| match t {
                            StatementKindOwned::VariableDef(statement) => Ok(
                                StatementKindOwned::VariableDef(statement.add_namespace(&ns)),
                            ),
                            _ => Err(anyhow!(
                                "Only variable statements allowed inside a namespace block."
                            )),
                        })
                        .try_fold(Vec::new(), |mut acc, res| match res {
                            Ok(ok) => {
                                acc.push(ok);
                                Ok(acc)
                            }
                            Err(e) => Err(e),
                        })?,
                );
            }
            StatementKindOwnedPass1::Reset(rs) => {
                out.push(StatementKindOwned::Reset(rs));
            }
            StatementKindOwnedPass1::SendFrames(kind) => {
                out.push(StatementKindOwned::SendFrames(kind));
            }
            StatementKindOwnedPass1::Loop { count, stmts } => {
                let v = stmts.into_iter().map(|f| *f).collect::<Vec<_>>();
                let passed = do_pass1(v, current_parent)?;

                // Repeat the statements `count` times
                for _ in 0..count {
                    out.extend(passed.clone());
                }
            }
            StatementKindOwnedPass1::Include { namespace, path } => {
                let to_parse = PathBuf::from_str(&path)?;
                let to_parse = if to_parse.is_relative() {
                    if to_parse.is_dir() {
                        return Err(anyhow!("File is a directory: {}", to_parse.display()));
                    } else {
                        current_parent.join(path).to_owned()
                    }
                } else {
                    to_parse
                };
                let nast = parse_to_ast(&to_parse)?;
                let nast = ast_add_namespace(&namespace, nast);
                out.extend(nast);
            }
        }
    }

    Ok(out)
}

fn parse_to_ast(path: &std::path::Path) -> anyhow::Result<Vec<StatementKindOwned>> {
    let file = std::fs::read_to_string(path);
    let file = match file {
        Ok(file) => Ok(file),
        Err(e) => match e.kind() {
            std::io::ErrorKind::NotFound => Err(anyhow!("Could not find file: {}", path.display())),
            std::io::ErrorKind::PermissionDenied => {
                Err(anyhow!("No permission to read file: {}", path.display()))
            }
            std::io::ErrorKind::IsADirectory => {
                Err(anyhow!("File is a directory: {}", path.display()))
            }
            _ => Err(anyhow!("Unexpected error for file: {}", path.display())),
        },
    }?;

    // parse file to be included recursively
    let token_iter = Token::lexer(&file).spanned().map(|(tok, span)| match tok {
        Ok(tok) => (tok, span.into()),
        Err(()) => (Token::Error, span.into()),
    });
    let token_stream =
        Stream::from_iter(token_iter).map((0..file.len()).into(), |(t, s): (_, _)| (t, s));
    match parser().parse(token_stream).into_result() {
        Ok(sexpr) => {
            let mut ast: Vec<StatementKindOwnedPass1> = Vec::new();
            for expr in sexpr {
                ast.push(expr.into());
            }

            let ast = do_pass1(
                ast,
                path.parent()
                    .ok_or(anyhow!("Can not get parent directory of file."))?,
            )?; // recursive call
            return Ok(ast);
        }
        Err(errs) => {
            for err in errs {
                Report::build(ReportKind::Error, (), err.span().start)
                    .with_code(3)
                    .with_message(err.to_string())
                    .with_label(
                        Label::new(err.span().into_range())
                            .with_message(err.reason().to_string())
                            .with_color(Color::Red),
                    )
                    .finish()
                    .eprint(Source::from(&file))
                    .unwrap();
            }
        }
    }

    Err(anyhow!("Could not parse ratslang code."))
}

impl<'a> From<StatementKindPass1<'a>> for StatementKindOwnedPass1 {
    fn from(value: StatementKindPass1<'a>) -> Self {
        match value {
            StatementKindPass1::VariableNamespaceBlock { ns, stmts } => {
                StatementKindOwnedPass1::VariableNamespaceBlock {
                    ns: ns.into_iter().map(String::from).collect::<Vec<_>>(),
                    stmts: stmts
                        .into_iter()
                        .map(|bstmt| Box::new(StatementKindOwnedPass1::from(*bstmt)))
                        .collect::<Vec<_>>(),
                }
            }
            StatementKindPass1::Rule(rule) => StatementKindOwnedPass1::Rule(RuleOwned::from(rule)),
            StatementKindPass1::VariableDef(statement) => {
                StatementKindOwnedPass1::VariableDef(statement)
            }
            StatementKindPass1::Reset(path) => StatementKindOwnedPass1::Reset(path),
            StatementKindPass1::SendFrames(kind) => StatementKindOwnedPass1::SendFrames(kind),
            StatementKindPass1::Include { namespace, path } => {
                StatementKindOwnedPass1::Include { namespace, path }
            }
            StatementKindPass1::Loop { count, stmts } => StatementKindOwnedPass1::Loop {
                count,
                stmts: stmts
                    .into_iter()
                    .map(|bstmt| Box::new(StatementKindOwnedPass1::from(*bstmt)))
                    .collect::<Vec<_>>(),
            },
        }
    }
}

// --- Coordinator Rules ---
#[derive(Debug, Clone)]
pub struct RuleSexpr {
    pub rules: Vec<RuleOwned>,
}

#[derive(Debug, Clone)]
pub struct Rule<'a> {
    pub variable: &'a str,
    pub stmts: Vec<Statement>,
}

#[derive(Debug, Clone)]
pub struct RuleOwned {
    pub variable: String,
    pub stmts: Vec<Statement>,
}

impl<'a> From<Rule<'a>> for RuleOwned {
    fn from(value: Rule<'a>) -> Self {
        Self {
            variable: value.variable.to_owned(),
            stmts: value.stmts,
        }
    }
}

#[derive(Debug, Clone)]
pub enum StatementPass1 {
    AssignLeft {
        lhs: Vec<Var>,
        rhs: Rhs,
    },
    AssignRight {
        lhs: Rhs,
        rhs: Vec<Var>,
    },
    Compare {
        rhs: Vec<Var>,
        lhs: Vec<Var>,
    },
    Include {
        namespace: Vec<String>,
        path: String,
    },
}

#[derive(Debug, Clone)]
pub enum Statement {
    AssignLeft { lhs: Vec<Var>, rhs: Rhs },
    AssignRight { lhs: Rhs, rhs: Vec<Var> },
    Compare { rhs: Vec<Var>, lhs: Vec<Var> },
}

impl Statement {
    fn add_namespace(self, ns: &Vec<String>) -> Self {
        match self {
            Statement::AssignLeft { lhs, rhs } => {
                let nlhs = lhs
                    .into_iter()
                    .map(|v| v.add_namespace(ns))
                    .collect::<Vec<_>>();
                Statement::AssignLeft { lhs: nlhs, rhs }
            }
            Statement::AssignRight { lhs, rhs } => {
                let nrhs = rhs
                    .into_iter()
                    .map(|v| v.add_namespace(ns))
                    .collect::<Vec<_>>();
                Statement::AssignRight { lhs, rhs: nrhs }
            }
            Statement::Compare { rhs, lhs } => Statement::Compare { rhs, lhs },
        }
    }
}

fn ast_add_namespace(ns: &Vec<String>, ast: Vec<StatementKindOwned>) -> Vec<StatementKindOwned> {
    ast.into_iter()
        .map(|stmt| match stmt {
            StatementKindOwned::VariableDef(statement) => {
                StatementKindOwned::VariableDef(statement.add_namespace(ns))
            }
            _ => stmt,
        })
        .collect()
}

#[derive(Clone, Copy, Debug)]
pub enum Operator {
    Right,   // "->"
    Left,    // "<-"
    Compare, // "=="
}

// TODO replace with rhs
// #[derive(Clone, Copy, Debug)]
// pub enum Expr<'a> {
//     Var(&'a str),
//     Predef(&'a str),
//     Path(&'a str),
//     Log, // represents the LOG keyword.
// }

// This function signature looks complicated, but don't fear! We're just saying that this function is generic over
// inputs that:
//     - Can have tokens pulled out of them by-value, by cloning (`ValueInput`)
//     - Gives us access to slices of the original input (`SliceInput`)
//     - Produces tokens of type `Token`, the type we defined above (`Token = Token<'a>`)
//     - Produces spans of type `SimpleSpan`, a built-in span type provided by chumsky (`Span = SimpleSpan`)
// The function then returns a parser that:
//     - Has an input type of type `I`, the one we declared as a type parameter
//     - Produces an `SExpr` as its output
//     - Uses `Rich`, a built-in error type provided by chumsky, for error generation
fn parser<'a, I>()
-> impl Parser<'a, I, Vec<StatementKindPass1<'a>>, extra::Err<Rich<'a, Token<'a>>>>
where
    I: ValueInput<'a, Token = Token<'a>, Span = SimpleSpan>,
{
    let time = select! {
        Token::IntegerNumberMillisecond(ms) => NumVal::Integer(ms),
        Token::FloatingNumberMillisecond(ms) => NumVal::Floating(ms).integerize_unit_val(),
    }
    .labelled("time");

    let way = select! {
        Token::IntegerNumberMillimeter(millis) => NumVal::Integer(millis),
        Token::FloatingNumberMillimeter(millis) => NumVal::Floating(millis).integerize_unit_val(),
    }
    .labelled("way");

    let timerange = time
        .or_not()
        .then_ignore(just(Token::OpRange))
        .then(time)
        .try_map(|(from, to), span| {
            Ok(Rhs::Range {
                from: {
                    if let Some(from) = from {
                        Some(Val::UnitedVal(UnitVal {
                            val: {
                                match from {
                                    NumVal::Floating(_) => unreachable!("integerised before"),
                                    NumVal::Integer(i) => {
                                        if i < 0 {
                                            return Err(Rich::custom(
                                                span,
                                                "Negative Integer Number for Timespan.",
                                            ));
                                        } else {
                                            i as u64
                                        }
                                    }
                                }
                            },
                            unit: Unit::TimeMilliseconds,
                        }))
                    } else {
                        None
                    }
                },
                to: Some(Val::UnitedVal(UnitVal {
                    val: match to {
                        NumVal::Floating(_) => unreachable!("integerised before"),
                        NumVal::Integer(i) => {
                            if i < 0 {
                                return Err(Rich::custom(
                                    span,
                                    "Negative Integer Number for Timespan.",
                                ));
                            } else {
                                i as u64
                            }
                        }
                    },
                    unit: Unit::TimeMilliseconds,
                })),
            })
        })
        .labelled("time range");

    let timerange_open_upper = time
        .then_ignore(just(Token::OpRange))
        .then_ignore(time.not())
        .try_map(|from, span| {
            Ok(Rhs::Range {
                from: Some(Val::UnitedVal(UnitVal {
                    val: match from {
                        NumVal::Floating(_) => unreachable!("integerised before"),
                        NumVal::Integer(i) => {
                            if i < 0 {
                                return Err(Rich::custom(
                                    span,
                                    "Negative Integer Number for Timespan.",
                                ));
                            } else {
                                i as u64
                            }
                        }
                    },
                    unit: Unit::TimeMilliseconds,
                })),
                to: None,
            })
        })
        .labelled("upper open time range");

    let timerange_open = time
        .not()
        .then_ignore(just(Token::OpRange))
        .then_ignore(time.not())
        .map(|_| Rhs::Range {
            from: None,
            to: None,
        })
        .labelled("open not-time range");

    let wayrange = way
        .then_ignore(just(Token::OpRange))
        .then(way)
        .try_map(|(from, to), span| {
            Ok(Rhs::Range {
                from: Some(Val::UnitedVal(UnitVal {
                    val: match from {
                        NumVal::Floating(_) => unreachable!("integerised before"),
                        NumVal::Integer(i) => {
                            if i < 0 {
                                return Err(Rich::custom(
                                    span,
                                    "Negative Integer Number for Wayspan.",
                                ));
                            } else {
                                i as u64
                            }
                        }
                    },
                    unit: Unit::WayMillimeter,
                })),
                to: Some(Val::UnitedVal(UnitVal {
                    val: match to {
                        NumVal::Floating(_) => unreachable!("integerised before"),
                        NumVal::Integer(i) => {
                            if i < 0 {
                                return Err(Rich::custom(
                                    span,
                                    "Negative Integer Number for Wayspan.",
                                ));
                            } else {
                                i as u64
                            }
                        }
                    },
                    unit: Unit::WayMillimeter,
                })),
            })
        })
        .labelled("way range");

    let numvalue = select! {
            Token::IntegerNumber(i) => Rhs::Val(Val::NumVal(NumVal::Integer(i))),
            Token::FloatingNumber(f) => Rhs::Val(Val::NumVal(NumVal::Floating(f))),
    }
    .labelled("number value");

    let value = select! {
            Token::IntegerNumber(i) => Rhs::Val(Val::NumVal(NumVal::Integer(i))),
            Token::FloatingNumber(f) => Rhs::Val(Val::NumVal(NumVal::Floating(f))),
            Token::String(s) => Rhs::Val(Val::StringVal(s.to_owned())),
            Token::True => Rhs::Val(Val::BoolVal(true)),
            Token::False => Rhs::Val(Val::BoolVal(false)),
    }
    .labelled("value");

    let numberrange = numvalue
        .then_ignore(just(Token::OpRange))
        .then(value)
        .try_map(|(from, to), span| match (&from, &to) {
            (
                Rhs::Val(Val::NumVal(NumVal::Floating(_))),
                Rhs::Val(Val::NumVal(NumVal::Integer(_))),
            )
            | (
                Rhs::Val(Val::NumVal(NumVal::Integer(_))),
                Rhs::Val(Val::NumVal(NumVal::Floating(_))),
            ) => Err(Rich::custom(span, "Cannot span between float and integer.")),
            (_, _) => Ok((from, to)),
        })
        .map(|(from, to)| {
            let from = match from {
                Rhs::Val(Val::NumVal(nv)) => nv,
                _ => unreachable!(),
            };
            let to = match to {
                Rhs::Val(Val::NumVal(nv)) => nv,
                _ => unreachable!(),
            };

            Rhs::Range {
                from: Some(Val::NumVal(from)),
                to: Some(Val::NumVal(to)),
            }
        })
        .labelled("number range");

    let variable = select! {
            Token::VarNamespace(ns) => ns.to_owned(),
        }
        .repeated()
        .collect::<Vec<_>>()
        .then(select! {
                Token::Variable(var) => Rhs::Var(Var::User { name: var.to_owned(), namespace: vec![] }),
                Token::PredefVariable(var) => Rhs::Var(Var::Predef { name: var.to_owned(), namespace: vec![] }),
            })
        .map(|(ns, var)| {
                let var = match var {
                    Rhs::Var(var) => match var {
                        Var::User { name, namespace: _} => {
                            Var::User { name, namespace: ns }
                        },
                        Var::Predef { name, namespace: _ } => {
                            Var::Predef { name, namespace: ns }
                        },
                        _ => unreachable!(),
                    },
                    _ => unreachable!(),
                };

                Rhs::Var(var)
            });

    let term = select! {
        Token::KwLog => Rhs::Var(Var::Log), // TODO maybe not rhs that can be assigned
        Token::Path(p) => Rhs::Path(p.to_owned()),
    }
    .or(variable)
    .labelled("term");

    let range = choice((numberrange, timerange.clone(), wayrange));

    let rhs_array = recursive(|rhs_array| {
        choice((term, range.clone(), value, rhs_array))
            .separated_by(just(Token::Comma).then_ignore(just(Token::NewLine).repeated()))
            .at_least(1)
            .collect::<Vec<_>>()
            .delimited_by(just(Token::LParen), just(Token::RParen))
            .map(|arr: Vec<Rhs>| Rhs::Array(arr.into_iter().map(Box::new).collect::<Vec<_>>()))
    });

    let rhs = choice((term, range, value, rhs_array));

    // parse a comma-separated list of terms (the left-hand side).
    let comma = just(Token::Comma).then_ignore(just(Token::NewLine).repeated());
    let multi_rhs = rhs
        .clone()
        .separated_by(comma)
        .at_least(1)
        .collect::<Vec<_>>()
        .labelled("rule term");

    let op = select! {
        Token::OpAssignToRight => Operator::Right,
        Token::OpAssignToLeft => Operator::Left,
        Token::OpCompare => Operator::Compare,
    }
    .labelled("operator");

    // a statement is: lhs, then an operator, then a single term (the right-hand side).
    let statement = multi_rhs
        .clone()
        .then(op)
        .then(multi_rhs)
        .then_ignore(just(Token::NewLine).repeated())
        .try_map(|((lhs, op), rhs), span| {
            Ok(StatementKindPass1::VariableDef(match op {
                Operator::Right => {
                    if lhs.len() > 1 {
                        return Err(Rich::custom(
                            span,
                            "Can only assign one term but to multiple variables.",
                        ));
                    }
                    if let Some(lhs) = lhs.first() {
                        let mut vars: Vec<Var> = Vec::new();
                        for r in rhs.iter() {
                            match r {
                                Rhs::Var(var) => {
                                    vars.push(var.clone());
                                }
                                _ => {
                                    return Err(Rich::custom(
                                        span,
                                        "Can only assign to variables.",
                                    ));
                                }
                            }
                        }

                        Statement::AssignRight {
                            lhs: lhs.clone(),
                            rhs: vars,
                        }
                    } else {
                        return Err(Rich::custom(span, "Missing left operand."));
                    }
                }
                Operator::Left => {
                    if rhs.len() > 1 {
                        return Err(Rich::custom(
                            span,
                            "Can only assign one term but to multiple variables.",
                        ));
                    }
                    if let Some(rhs) = rhs.first() {
                        let mut vars: Vec<Var> = Vec::new();
                        for l in lhs.iter() {
                            match l {
                                Rhs::Var(var) => {
                                    vars.push(var.clone());
                                }
                                _ => {
                                    return Err(Rich::custom(
                                        span,
                                        "Can only assign to variables.",
                                    ));
                                }
                            }
                        }

                        Statement::AssignLeft {
                            lhs: vars,
                            rhs: rhs.clone(),
                        }
                    } else {
                        return Err(Rich::custom(span, "Missing right operand."));
                    }
                }
                Operator::Compare => {
                    let mut lvars: Vec<Var> = Vec::new();
                    for l in lhs.iter() {
                        match l {
                            Rhs::Var(var) => {
                                lvars.push(var.clone());
                            }
                            _ => {
                                return Err(Rich::custom(span, "Can only compare variables."));
                            }
                        }
                    }
                    let mut rvars: Vec<Var> = Vec::new();
                    for r in rhs.iter() {
                        match r {
                            Rhs::Var(var) => {
                                rvars.push(var.clone());
                            }
                            _ => {
                                return Err(Rich::custom(span, "Can only compare variables."));
                            }
                        }
                    }

                    Statement::Compare {
                        rhs: rvars,
                        lhs: lvars,
                    }
                }
            }))
        })
        .labelled("statement");

    let rule_header = just(Token::FnRule)
        .ignore_then(just(Token::BracketOpen))
        .ignore_then(select! { Token::Variable(v) => v }.labelled("rule name"))
        .then_ignore(just(Token::NewLine).or_not())
        .labelled("rule header");

    let rule = rule_header
        .then_ignore(just(Token::NewLine).repeated())
        .then(statement.clone().repeated().collect())
        .then_ignore(just(Token::NewLine).repeated())
        .then_ignore(just(Token::BracketClose))
        .map(|(variable, stmts): (&str, Vec<StatementKindPass1>)| {
            let stmts = stmts
                .into_iter()
                .map(|stmtk| match stmtk {
                    StatementKindPass1::VariableDef(var) => var,
                    _ => unreachable!(),
                })
                .collect::<Vec<_>>();
            StatementKindPass1::Rule(Rule { variable, stmts })
        })
        .labelled("rule");

    let wind_reset_fn = just(Token::FnReset)
        .ignore_then(
            select! {
                Token::Path(p) => p, // prefixed with /, resolve
                Token::Variable(v) =>  v, // not prefixed, so implicit ./
            }
            .labelled("bagfile"),
        )
        .map(|expr| StatementKindPass1::Reset(expr.to_owned()))
        .labelled("reset");

    let str_only = select! {
            Token::String(s) => s.to_owned(),
    }
    .or(variable.try_map(|v, span| match v {
        Rhs::Var(Var::User {
            name,
            namespace: ns,
        }) => {
            if !ns.is_empty() {
                return Err(Rich::custom(span, "Found namespace in implicit string."));
            }
            Ok(name)
        }
        _ => Err(Rich::custom(span, "Expected implicit string.")),
    }));
    let str_array = str_only
        .separated_by(just(Token::Comma).then_ignore(just(Token::NewLine).repeated()))
        .at_least(1)
        .collect::<Vec<_>>()
        .delimited_by(just(Token::LParen), just(Token::RParen));

    let str_only_as_vec = str_only.map(|single| vec![single]).labelled("string");
    let one_or_more_str = choice((str_only_as_vec, str_array)).labelled("at least one string");
    let playfn_with_conditions = just(Token::FnPlayFrames)
        .labelled("play_frames")
        .ignore_then(one_or_more_str.clone())
        .then((just(Token::Comma).ignore_then(one_or_more_str)).or_not())
        .map(
            |(play_sensors, stop_sensors): (Vec<String>, Option<Vec<String>>)| match stop_sensors {
                Some(stop_sensors) => PlayType::UntilSensorCount {
                    sending: play_sensors
                        .into_iter()
                        .map(|el| el.to_owned())
                        .collect::<Vec<_>>(),
                    until_sensors: stop_sensors
                        .into_iter()
                        .map(|el| el.to_owned())
                        .collect::<Vec<_>>(),
                },
                None => PlayType::SensorCount {
                    sensors: play_sensors
                        .into_iter()
                        .map(|el| el.to_owned())
                        .collect::<Vec<_>>(),
                },
            },
        );

    let wind_play_frames_fn = playfn_with_conditions
        .boxed()
        .then(
            choice((
                timerange_open,
                timerange_open_upper,
                timerange.clone(),
                time.map(|t| {
                    Rhs::Val(Val::UnitedVal(UnitVal {
                        val: match t {
                            NumVal::Integer(i) => i as u64,
                            _ => unreachable!(),
                        },
                        unit: Unit::TimeMilliseconds,
                    }))
                }),
                select! {
                    Token::IntegerNumber(i) => Rhs::Val(Val::NumVal(NumVal::Integer(i))), // (until) number of frames
                },
            ))
            .labelled("int number for count, relative time from now or absolute timespan")
            .boxed(),
        )
        .then(
            choice((
                variable.try_map(|v, span| match v {
                    Rhs::Var(var) => match var {
                        Var::Log => Err(Rich::custom(span, "Can not trigger on Log.")),
                        Var::Predef {
                            name: _,
                            namespace: _,
                        } => Err(Rich::custom(
                            span,
                            "Did not expect a predef variable as a variable name.",
                        )),
                        Var::User { name, namespace } => {
                            if !namespace.is_empty() {
                                return Err(Rich::custom(
                                    span,
                                    "Rat names should not have a namespace.",
                                )); // TODO needs fix when using topics with namespaces
                            }

                            Ok(Rhs::Var(Var::User {
                                name,
                                namespace: vec![],
                            }))
                        }
                    },
                    _ => unreachable!(),
                }),
                time.map(|t| {
                    Rhs::Val(Val::UnitedVal(UnitVal {
                        val: match t {
                            NumVal::Integer(i) => i as u64,
                            _ => unreachable!(),
                        },
                        unit: Unit::TimeMilliseconds,
                    }))
                }),
                select! {
                    Token::FloatingNumber(f) => Rhs::Val(Val::NumVal(NumVal::Floating(f))),
                },
            ))
            .boxed()
            .or_not()
            .labelled("trigger"),
        )
        .then(
            variable
                .try_map(|v, span| {
                    let errmsg = "Only f or d allowed. f for fixed, d for dynamic.";
                    match v {
                        Rhs::Var(var) => match var {
                            Var::Log => Err(Rich::custom(span, errmsg)),
                            Var::Predef { name, namespace } => {
                                if !namespace.is_empty() {
                                    Err(Rich::custom(span, errmsg))
                                } else {
                                    if name.as_str() != "f" && name.as_str() != "d" {
                                        return Err(Rich::custom(span, errmsg));
                                    }

                                    Ok(name)
                                }
                            }
                            Var::User { name, namespace } => {
                                if !namespace.is_empty() {
                                    Err(Rich::custom(span, errmsg))
                                } else {
                                    if name.as_str() != "f" && name.as_str() != "d" {
                                        return Err(Rich::custom(span, errmsg));
                                    }

                                    Ok(name)
                                }
                            }
                        },
                        _ => unreachable!(),
                    }
                })
                .labelled("play mode [f|d]. f = fixed, d = dynamic")
                .or_not(),
        )
        .map(
            |(((pt, arg), trigger), mode): (((PlayType, Rhs), Option<Rhs>), Option<String>)| {
                let trigger = trigger.map(|trigger| match trigger {
                    Rhs::Val(Val::UnitedVal(UnitVal {
                        val,
                        unit: Unit::TimeMilliseconds,
                    })) => PlayTrigger::DurationMs(val),
                    Rhs::Var(Var::User { name, namespace: _ }) => PlayTrigger::Variable(name),
                    Rhs::Val(Val::NumVal(NumVal::Floating(f))) => PlayTrigger::DurationRelFactor(f),
                    _ => unreachable!(),
                });

                let play_mode = mode.map(|mode| match mode.as_str() {
                    "f" => PlayMode::Fix,
                    "d" => PlayMode::Dynamic,
                    _ => unreachable!(),
                });
                let default_play_mode = match trigger.as_ref() {
                    Some(PlayTrigger::Variable(_)) => PlayMode::Dynamic,
                    _ => PlayMode::Fix,
                };
                let play_mode = play_mode.unwrap_or(default_play_mode);

                let pku = match pt {
                    PlayType::SensorCount { sensors } => match arg {
                        Rhs::Range {
                            from:
                                Some(Val::UnitedVal(UnitVal {
                                    val: from_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                            to:
                                Some(Val::UnitedVal(UnitVal {
                                    val: to_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                        } => PlayKindUnited::SensorCount {
                            sensors,
                            count: PlayCount::TimeRangeMs(AbsTimeRange::Closed((from_val, to_val))),
                            trigger,
                            play_mode,
                        },
                        Rhs::Range {
                            from: None,
                            to:
                                Some(Val::UnitedVal(UnitVal {
                                    val: to_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                        } => PlayKindUnited::SensorCount {
                            sensors,
                            count: PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(to_val)),
                            trigger,
                            play_mode,
                        },
                        Rhs::Range {
                            from:
                                Some(Val::UnitedVal(UnitVal {
                                    val: from_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                            to: None,
                        } => PlayKindUnited::SensorCount {
                            sensors,
                            count: PlayCount::TimeRangeMs(AbsTimeRange::UpperOpen(from_val)),
                            trigger,
                            play_mode,
                        },
                        Rhs::Range {
                            from: None,
                            to: None,
                        } => PlayKindUnited::SensorCount {
                            sensors,
                            count: PlayCount::TimeRangeMs(AbsTimeRange::Open),
                            trigger,
                            play_mode,
                        },
                        Rhs::Val(Val::NumVal(NumVal::Integer(i))) => PlayKindUnited::SensorCount {
                            sensors,
                            count: PlayCount::Amount(i as u64),
                            trigger,
                            play_mode,
                        },
                        Rhs::Val(Val::UnitedVal(UnitVal {
                            val: ms,
                            unit: Unit::TimeMilliseconds,
                        })) => PlayKindUnited::SensorCount {
                            sensors,
                            count: PlayCount::TimeRelativeMs(ms),
                            trigger,
                            play_mode,
                        },
                        _ => {
                            unreachable!()
                        }
                    },
                    PlayType::UntilSensorCount {
                        sending,
                        until_sensors,
                    } => match arg {
                        Rhs::Range {
                            from:
                                Some(Val::UnitedVal(UnitVal {
                                    val: from_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                            to:
                                Some(Val::UnitedVal(UnitVal {
                                    val: to_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                        } => PlayKindUnited::UntilSensorCount {
                            sending,
                            until_sensors,
                            until_count: PlayCount::TimeRangeMs(AbsTimeRange::Closed((
                                from_val, to_val,
                            ))),
                            trigger,
                            play_mode,
                        },
                        Rhs::Range {
                            from: None,
                            to:
                                Some(Val::UnitedVal(UnitVal {
                                    val: to_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                        } => PlayKindUnited::UntilSensorCount {
                            sending,
                            until_sensors,
                            until_count: PlayCount::TimeRangeMs(AbsTimeRange::LowerOpen(to_val)),
                            trigger,
                            play_mode,
                        },
                        Rhs::Range {
                            from:
                                Some(Val::UnitedVal(UnitVal {
                                    val: from_val,
                                    unit: Unit::TimeMilliseconds,
                                })),
                            to: None,
                        } => PlayKindUnited::UntilSensorCount {
                            sending,
                            until_sensors,
                            until_count: PlayCount::TimeRangeMs(AbsTimeRange::UpperOpen(from_val)),
                            trigger,
                            play_mode,
                        },
                        Rhs::Val(Val::NumVal(NumVal::Integer(i))) => {
                            PlayKindUnited::UntilSensorCount {
                                sending,
                                until_sensors,
                                until_count: PlayCount::Amount(i as u64),
                                trigger,
                                play_mode,
                            }
                        }
                        Rhs::Val(Val::UnitedVal(UnitVal {
                            val: ms,
                            unit: Unit::TimeMilliseconds,
                        })) => PlayKindUnited::UntilSensorCount {
                            sending,
                            until_sensors,
                            until_count: PlayCount::TimeRelativeMs(ms),
                            trigger,
                            play_mode,
                        },
                        _ => {
                            unreachable!()
                        }
                    },
                };

                StatementKindPass1::SendFrames(pku)
            },
        )
        .labelled("play_frames");

    let include = just(Token::OpAssignToLeft)
        .ignore_then(select! {
            Token::Path(p) => p.to_owned(),
        })
        .then_ignore(just(Token::NewLine))
        .map(|path| StatementKindPass1::Include {
            namespace: vec![],
            path,
        });

    let namespace_block = recursive(|nsb| {
        select! {
            Token::VarNamespace(ns) => ns,
        }
        .repeated()
        .at_least(1)
        .collect()
        .then(
            choice((statement.clone(), include.clone(), nsb))
                .repeated()
                .collect::<Vec<_>>()
                .delimited_by(
                    just(Token::BlockStart).then_ignore(just(Token::NewLine).repeated()),
                    just(Token::BlockEnd).then_ignore(just(Token::NewLine).repeated()),
                ),
        )
        .map(|(ns, stmts): (Vec<&'_ str>, Vec<StatementKindPass1>)| {
            StatementKindPass1::VariableNamespaceBlock {
                ns,
                stmts: stmts.into_iter().map(Box::new).collect::<Vec<_>>(),
            }
        })
    });

    let loop_stmt = recursive(|loop_stmt_rec| {
        just(Token::KwLoop)
            .ignore_then(select! {
                Token::IntegerNumber(n) => n as usize,
            })
            .then(
                choice((statement.clone(), include.clone(), loop_stmt_rec))
                    .repeated()
                    .collect::<Vec<_>>()
                    .delimited_by(
                        just(Token::BlockStart).then_ignore(just(Token::NewLine).repeated()),
                        just(Token::BlockEnd).then_ignore(just(Token::NewLine).repeated()),
                    ),
            )
            .map(
                |(count, stmts): (usize, Vec<StatementKindPass1>)| StatementKindPass1::Loop {
                    count,
                    stmts: stmts.into_iter().map(Box::new).collect::<Vec<_>>(),
                },
            )
    });

    let one_of_wind_fns = choice((
        statement,
        wind_play_frames_fn,
        wind_reset_fn,
        rule,
        namespace_block,
        loop_stmt,
        include,
    ));
    let newlines = just(Token::NewLine).repeated();

    newlines
        .clone()
        .ignore_then((one_of_wind_fns.then_ignore(newlines)).repeated().collect())
        .then_ignore(end())
}

impl RuleSexpr {
    fn eval(&self) -> anyhow::Result<Rules> {
        let mut rules = Rules::new();
        for rule in self.rules.iter() {
            for stmt in rule.stmts.iter() {
                match &stmt {
                    Statement::AssignLeft { lhs, rhs } => {
                        let plan = ActionPlan::Shoot {
                            target: lhs
                                .iter()
                                .map(|part| match part {
                                    Var::User { name, namespace: _ } => name.clone(),
                                    Var::Log => COMPARE_NODE_NAME.to_owned(),
                                    Var::Predef {
                                        name: _,
                                        namespace: _,
                                    } => unreachable!(),
                                })
                                .collect(),
                            id: rules.new_id(),
                        };

                        let ship = match rhs {
                            Rhs::Var(var) => match var {
                                Var::User { name, namespace: _ } => name.clone(),
                                Var::Log => COMPARE_NODE_NAME.to_owned(),
                                Var::Predef {
                                    name: _,
                                    namespace: _,
                                } => unreachable!(),
                            },
                            _ => continue,
                        };
                        let varh = VariableHuman {
                            ship,
                            strategy: Some(plan.clone()),
                        };
                        rules.insert(rule.variable.to_owned(), vec![varh]);
                    }
                    Statement::AssignRight { lhs, rhs } => {
                        let plan = ActionPlan::Shoot {
                            target: rhs
                                .iter()
                                .map(|part| match part {
                                    Var::User { name, namespace: _ } => name.clone(),
                                    Var::Log => COMPARE_NODE_NAME.to_owned(),
                                    Var::Predef {
                                        name: _,
                                        namespace: _,
                                    } => unreachable!(),
                                })
                                .collect(),
                            id: rules.new_id(),
                        };

                        let ship = match lhs {
                            Rhs::Var(var) => match var {
                                Var::User { name, namespace: _ } => name.clone(),
                                Var::Log => COMPARE_NODE_NAME.to_owned(),
                                Var::Predef {
                                    name: _,
                                    namespace: _,
                                } => unreachable!(),
                            },
                            _ => continue,
                        };
                        let varh = VariableHuman {
                            ship,
                            strategy: Some(plan.clone()),
                        };
                        rules.insert(rule.variable.to_owned(), vec![varh]);
                    }
                    Statement::Compare { rhs, lhs } => {
                        let plan = ActionPlan::Shoot {
                            target: vec![COMPARE_NODE_NAME.to_owned()],
                            id: rules.new_id(),
                        };

                        let mut merged: Vec<String> = rhs
                            .iter()
                            .map(|var| match var {
                                Var::User { name, namespace: _ } => name.clone(),
                                Var::Log => COMPARE_NODE_NAME.to_owned(),
                                _ => unreachable!(),
                            })
                            .collect();

                        merged.extend(lhs.iter().map(|var| match var {
                            Var::User { name, namespace: _ } => name.clone(),
                            Var::Log => COMPARE_NODE_NAME.to_owned(),
                            _ => unreachable!(),
                        }));

                        merged.iter().for_each(|ex| {
                            let varh = VariableHuman {
                                ship: ex.clone(),
                                strategy: Some(plan.clone()),
                            };
                            rules.insert(rule.variable.to_owned(), vec![varh]);
                        });
                    }
                };
            }
        }
        Ok(rules)
    }
}

pub fn compile_file(
    path: &std::path::PathBuf,
    start_line: Option<usize>,
    end_line: Option<usize>,
) -> anyhow::Result<Evaluated> {
    compile_file_with_state(path, start_line, end_line, None, std::io::stderr(), true)
}

pub fn compile_file_with_state(
    path: &std::path::PathBuf,
    start_line: Option<usize>,
    end_line: Option<usize>,
    var_state: Option<HashMap<Var, Rhs>>,
    out: impl std::io::Write,
    rich_out: bool,
) -> anyhow::Result<Evaluated> {
    let file = std::fs::read_to_string(path.clone());
    let file = match file {
        Ok(file) => Ok(file),
        Err(e) => match e.kind() {
            std::io::ErrorKind::NotFound => Err(anyhow!("Could not find file: {}", path.display())),
            std::io::ErrorKind::PermissionDenied => {
                Err(anyhow!("No permission to read file: {}", path.display()))
            }
            std::io::ErrorKind::IsADirectory => {
                Err(anyhow!("File is a directory: {}", path.display()))
            }
            _ => Err(anyhow!("Unexpected error for file: {}", path.display())),
        },
    }?;
    let start_line = start_line.map(|l| l.saturating_sub(1)).unwrap_or_default();
    let lines = file.lines();
    let end_line = end_line.unwrap_or(lines.clone().count()).saturating_sub(1);
    let selected_lines = lines
        .skip(start_line)
        .take(end_line + 1 - start_line)
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"; // append newline as funny hack to fix one-line problems

    let dir = path.parent().ok_or(anyhow!(
        "Could not get parent directory of source code file."
    ))?;
    compile_code_with_state(&selected_lines, dir, var_state, out, rich_out)
}

#[derive(Debug)]
pub struct VariableHistory {
    ast: Vec<StatementKindOwned>,
    pub var_cache: HashMap<Var, Rhs>,
}

fn is_sub_namespace(sub: &[String], super_namespace: &[String]) -> bool {
    if sub.len() > super_namespace.len() {
        return false;
    }

    if sub.is_empty() {
        return false; // Empty is not a sub-namespace
    }

    if super_namespace.is_empty() {
        return false;
    }

    for i in 0..sub.len() {
        if sub[i] != super_namespace[i] {
            return false;
        }
    }

    true
}

fn remove_prefix_from_target(prefix_definer: &[String], target_vec: &Vec<String>) -> Vec<String> {
    let prefix_len = prefix_definer.len();
    let target_len = target_vec.len();
    let compare_len = std::cmp::min(prefix_len, target_len);

    let mut common_prefix_len = 0;
    for i in 0..compare_len {
        if prefix_definer[i] == target_vec[i] {
            common_prefix_len += 1;
        } else {
            break;
        }
    }

    target_vec[common_prefix_len..].to_vec()
}

fn truncate_namespace_var(ns: &[String], var: &Var) -> Var {
    match var {
        Var::User { name, namespace } => Var::User {
            name: name.clone(),
            namespace: { remove_prefix_from_target(ns, namespace) },
        },
        Var::Log => Var::Log,
        Var::Predef { name, namespace } => Var::Predef {
            name: name.clone(),
            namespace: { remove_prefix_from_target(ns, namespace) },
        },
    }
}

fn truncate_namespace_rhs(ns: &[String], rhs: &Rhs) -> Rhs {
    match rhs {
        Rhs::Array(items) => Rhs::Array(
            items
                .iter()
                .map(|arr_item| Box::new(truncate_namespace_rhs(ns, arr_item)))
                .collect::<Vec<_>>(),
        ),
        Rhs::Range { from: _, to: _ } => rhs.clone(),
        Rhs::Var(var) => Rhs::Var(truncate_namespace_var(ns, var)),
        Rhs::Path(_) => rhs.clone(),
        Rhs::Val(_) => rhs.clone(),
    }
}

fn truncate_namespace_stmt(ns: &[String], stmt: &Statement) -> Statement {
    match stmt {
        Statement::AssignLeft { lhs, rhs } => {
            let l = lhs
                .iter()
                .filter(|v| match v {
                    Var::Log => false,
                    Var::Predef {
                        name: _,
                        namespace: var_ns,
                    } => is_sub_namespace(ns, var_ns),
                    Var::User {
                        name: _,
                        namespace: var_ns,
                    } => is_sub_namespace(ns, var_ns),
                })
                .map(|l| truncate_namespace_var(ns, l))
                .collect::<Vec<_>>();
            let r = truncate_namespace_rhs(ns, rhs);

            Statement::AssignLeft { lhs: l, rhs: r }
        }
        Statement::AssignRight { lhs, rhs } => {
            let r = rhs
                .iter()
                .filter(|v| match v {
                    Var::Log => false,
                    Var::Predef {
                        name: _,
                        namespace: var_ns,
                    } => is_sub_namespace(ns, var_ns),
                    Var::User {
                        name: _,
                        namespace: var_ns,
                    } => is_sub_namespace(ns, var_ns),
                })
                .map(|l| truncate_namespace_var(ns, l))
                .collect::<Vec<_>>();
            let l = truncate_namespace_rhs(ns, lhs);

            Statement::AssignRight { lhs: l, rhs: r }
        }
        Statement::Compare { rhs, lhs } => Statement::Compare {
            rhs: rhs.to_vec(),
            lhs: lhs.to_vec(),
        },
    }
}

fn truncate_namespace_rule(ns: &[String], rule: &RuleOwned) -> RuleOwned {
    let stmts = rule
        .stmts
        .iter()
        .map(|stmt| truncate_namespace_stmt(ns, stmt))
        .collect::<Vec<_>>();
    RuleOwned {
        variable: rule.variable.clone(),
        stmts,
    }
}

fn truncate_namespace(ns: &[String], stmt: &StatementKindOwned) -> StatementKindOwned {
    match stmt {
        StatementKindOwned::Rule(rule_owned) => {
            StatementKindOwned::Rule(truncate_namespace_rule(ns, rule_owned))
        }
        StatementKindOwned::VariableDef(statement) => {
            StatementKindOwned::VariableDef(truncate_namespace_stmt(ns, statement))
        }
        StatementKindOwned::Reset(path) => StatementKindOwned::Reset(path.clone()),
        StatementKindOwned::SendFrames(kind) => StatementKindOwned::SendFrames(kind.clone()),
    }
}

fn stmt_in_ns(namespace: &[String], stmt: &Statement) -> bool {
    match stmt {
        Statement::AssignLeft { lhs, rhs: _ } => lhs.iter().any(|r| match r {
            Var::Log => false,
            Var::Predef {
                name: _,
                namespace: var_ns,
            } => is_sub_namespace(namespace, var_ns),
            Var::User {
                name: _,
                namespace: var_ns,
            } => is_sub_namespace(namespace, var_ns),
        }),
        Statement::AssignRight { lhs: _, rhs } => rhs.iter().any(|r| match r {
            Var::Log => false,
            Var::Predef {
                name: _,
                namespace: var_ns,
            } => is_sub_namespace(namespace, var_ns),
            Var::User {
                name: _,
                namespace: var_ns,
            } => is_sub_namespace(namespace, var_ns),
        }),

        Statement::Compare { rhs: _, lhs: _ } => false,
    }
}

impl VariableHistory {
    pub fn new(ast: Vec<StatementKindOwned>) -> Self {
        VariableHistory {
            ast,
            var_cache: HashMap::new(),
        }
    }

    pub fn with_state(mut self, state: HashMap<Var, Rhs>) -> Self {
        self.var_cache = state;
        self
    }

    pub fn populate_cache(&mut self) {
        self.ast.iter().for_each(|f| {
            if let StatementKindOwned::VariableDef(statement) = f {
                match statement {
                    Statement::AssignLeft { lhs, rhs } => {
                        for l in lhs {
                            self.var_cache.insert(l.clone(), rhs.clone());
                        }
                    }
                    Statement::AssignRight { lhs, rhs } => {
                        for r in rhs {
                            self.var_cache.insert(r.clone(), lhs.clone());
                        }
                    }
                    Statement::Compare { rhs: _, lhs: _ } => {}
                }
            }
        });
    }

    fn filter_by_namespace(
        &self,
        namespace: &[String],
        up_to: Option<usize>,
        trunc_ns: bool,
    ) -> Vec<StatementKindOwned> {
        let mut vars_in_ns = vec![];

        // use previous cached vars as well
        for (var, rhs) in self.var_cache.iter() {
            let mut wvar = var.clone();
            if is_sub_namespace(
                namespace,
                match var {
                    Var::Log => unreachable!(),
                    Var::Predef { name: _, namespace } => namespace,
                    Var::User { name: _, namespace } => namespace,
                },
            ) {
                if trunc_ns {
                    wvar = truncate_namespace_var(namespace, &wvar);
                }

                vars_in_ns.push(StatementKindOwned::VariableDef(Statement::AssignLeft {
                    lhs: vec![wvar],
                    rhs: rhs.clone(),
                }));
            }
        }

        for stmt in self.ast.iter().take(up_to.unwrap_or(usize::MAX)) {
            match &stmt {
                StatementKindOwned::Rule(rule_owned) => {
                    let filtered_rules = rule_owned
                        .stmts
                        .iter()
                        .filter(|stmt| stmt_in_ns(namespace, stmt))
                        .map(|s| {
                            trunc_ns
                                .then_some(truncate_namespace_stmt(namespace, s))
                                .unwrap_or(s.clone())
                        })
                        .collect::<Vec<_>>();

                    if !filtered_rules.is_empty() {
                        let rule = RuleOwned {
                            variable: rule_owned.variable.clone(),
                            stmts: filtered_rules,
                        };
                        vars_in_ns.push(StatementKindOwned::Rule(rule));
                    }
                }
                StatementKindOwned::VariableDef(rstmt) => {
                    if stmt_in_ns(namespace, rstmt) {
                        let rhs = trunc_ns
                            .then_some(truncate_namespace(namespace, stmt))
                            .unwrap_or(stmt.clone());
                        vars_in_ns.push(rhs);
                    }
                }
                StatementKindOwned::Reset(_) => {}
                StatementKindOwned::SendFrames(_) => {}
            }
        }

        vars_in_ns
    }

    fn resolve_recursive(&self, var: &Var, up_to: Option<usize>) -> anyhow::Result<Option<Rhs>> {
        // use previous state if already resolved there
        if let Some(cache_hit) = self.var_cache.get(var) {
            return Ok(Some(cache_hit.clone()));
        }

        let mut val = None;
        for (i, stmt) in self
            .ast
            .iter()
            .enumerate()
            .take(up_to.unwrap_or(usize::MAX))
        {
            match stmt {
                StatementKindOwned::Rule(rule) => {
                    for rstmt in rule.stmts.iter() {
                        match rstmt {
                            Statement::AssignLeft { lhs, rhs } => {
                                if lhs.contains(var) {
                                    val = Some(match rhs {
                                        Rhs::Var(var) => {
                                            match self.resolve_recursive(var, Some(i))? {
                                                None => Rhs::Val(Val::StringVal(match var {
                                                    Var::User { name, namespace: _ } => {
                                                        name.clone()
                                                    }
                                                    Var::Log => {
                                                        return Err(anyhow!(
                                                            "Log can not be assigned."
                                                        ));
                                                    }
                                                    Var::Predef {
                                                        name: _,
                                                        namespace: _,
                                                    } => {
                                                        return Err(anyhow!(
                                                            "Predef variables can not be assigned."
                                                        ));
                                                    }
                                                })),
                                                Some(rhs) => rhs,
                                            }
                                        }
                                        _ => rhs.clone(),
                                    });
                                }
                            }
                            Statement::AssignRight { lhs, rhs } => {
                                if rhs.contains(var) {
                                    val = Some(match lhs {
                                        Rhs::Var(var) => {
                                            match self.resolve_recursive(var, Some(i))? {
                                                None => Rhs::Val(Val::StringVal(match var {
                                                    Var::User { name, namespace: _ } => {
                                                        name.clone()
                                                    }
                                                    Var::Log => {
                                                        return Err(anyhow!(
                                                            "Log can not be assigned."
                                                        ));
                                                    }
                                                    Var::Predef {
                                                        name: _,
                                                        namespace: _,
                                                    } => {
                                                        return Err(anyhow!(
                                                            "Predef variables can not be assigned."
                                                        ));
                                                    }
                                                })),
                                                Some(lhs) => lhs,
                                            }
                                        }
                                        _ => lhs.clone(),
                                    });
                                }
                            }
                            Statement::Compare { rhs: _, lhs: _ } => {}
                        }
                    }
                }
                StatementKindOwned::VariableDef(statement) => match statement {
                    Statement::AssignLeft { lhs, rhs } => {
                        if lhs.contains(var) {
                            val = Some(match rhs {
                                Rhs::Var(var) => match self.resolve_recursive(var, Some(i))? {
                                    None => Rhs::Val(Val::StringVal(match var {
                                        Var::User { name, namespace: _ } => name.clone(),
                                        Var::Log => {
                                            return Err(anyhow!("Log can not be assigned."));
                                        }
                                        Var::Predef {
                                            name: _,
                                            namespace: _,
                                        } => {
                                            return Err(anyhow!(
                                                "Predef variables can not be assigned."
                                            ));
                                        }
                                    })),
                                    Some(rhs) => rhs,
                                },
                                _ => rhs.clone(),
                            });
                        }
                    }
                    Statement::AssignRight { lhs, rhs } => {
                        if rhs.contains(var) {
                            val = Some(match lhs {
                                Rhs::Var(var) => match self.resolve_recursive(var, Some(i))? {
                                    None => Rhs::Val(Val::StringVal(match var {
                                        Var::User { name, namespace: _ } => name.clone(),
                                        Var::Log => {
                                            return Err(anyhow!("Log can not be assigned."));
                                        }
                                        Var::Predef {
                                            name: _,
                                            namespace: _,
                                        } => {
                                            return Err(anyhow!(
                                                "Predef variables can not be assigned."
                                            ));
                                        }
                                    })),
                                    Some(lhs) => lhs,
                                },
                                _ => lhs.clone(),
                            });
                        }
                    }
                    Statement::Compare { rhs: _, lhs: _ } => {}
                },
                StatementKindOwned::Reset(_) => {}
                StatementKindOwned::SendFrames(_) => {}
            }
        }
        Ok(val)
    }

    fn prepend_ns(var: &Var, ns: &[String]) -> Var {
        match var {
            Var::Log => Var::Log,
            Var::Predef {
                name,
                namespace: var_ns,
            } => {
                let v = vec_prepend(ns, var_ns);

                Var::Predef {
                    name: name.clone(),
                    namespace: v,
                }
            }
            Var::User {
                name,
                namespace: var_ns,
            } => {
                let v = vec_prepend(ns, var_ns);

                Var::User {
                    name: name.clone(),
                    namespace: v,
                }
            }
        }
    }

    pub fn resolve_ns(&self, namespace: &[&str]) -> Vec<(Var, Rhs)> {
        let owned_ns = namespace
            .iter()
            .map(|s| (*s).to_owned())
            .collect::<Vec<_>>();
        self.filter_by_namespace(&owned_ns, None, true)
            .iter()
            .flat_map(|stmt| match stmt {
                StatementKindOwned::Rule(rule_owned) => rule_owned
                    .stmts
                    .iter()
                    .flat_map(|st| match st {
                        Statement::AssignLeft { lhs, rhs: _ } => Some(
                            lhs.iter()
                                .map(|lh| {
                                    (
                                        lh,
                                        self.resolve_var(&Self::prepend_ns(lh, &owned_ns))
                                            .expect("Should have failed in Pass 2"),
                                    )
                                })
                                .collect::<Vec<_>>(),
                        ),
                        Statement::AssignRight { lhs: _, rhs } => Some(
                            rhs.iter()
                                .map(|lh| {
                                    (
                                        lh,
                                        self.resolve_var(&Self::prepend_ns(lh, &owned_ns))
                                            .expect("Should have failed in Pass 2"),
                                    )
                                })
                                .collect::<Vec<_>>(),
                        ),
                        Statement::Compare { rhs: _, lhs: _ } => None,
                    })
                    .flatten()
                    .collect::<Vec<_>>(),
                StatementKindOwned::VariableDef(statement) => match statement {
                    Statement::AssignLeft { lhs, rhs: _ } => lhs
                        .iter()
                        .map(|lh| {
                            (
                                lh,
                                self.resolve_var(&Self::prepend_ns(lh, &owned_ns))
                                    .expect("Should have failed in Pass 2"),
                            )
                        })
                        .collect::<Vec<_>>(),

                    Statement::AssignRight { lhs: _, rhs } => rhs
                        .iter()
                        .map(|lh| {
                            (
                                lh,
                                self.resolve_var(&Self::prepend_ns(lh, &owned_ns))
                                    .expect("Should have failed in Pass 2"),
                            )
                        })
                        .collect::<Vec<_>>(),

                    Statement::Compare { rhs: _, lhs: _ } => vec![],
                },
                StatementKindOwned::Reset(_) => vec![],
                StatementKindOwned::SendFrames(_) => vec![],
            })
            .fold(Vec::new(), |mut acc, res| match res {
                (var, Some(rhs)) => {
                    acc.push((truncate_namespace_var(&owned_ns, var), rhs));
                    acc
                }
                (_, None) => acc, // skip if not set to anything
            })
    }

    pub fn filter_ns(&self, namespace: &[&str]) -> Self {
        let owned_ns = namespace
            .iter()
            .map(|s| (*s).to_owned())
            .collect::<Vec<_>>();
        let filtered = self.filter_by_namespace(&owned_ns, None, true);
        Self::new(filtered)
    }

    pub fn resolve(&self, var: &str) -> anyhow::Result<Option<Rhs>> {
        self.resolve_var(&Var::from_str(var)?)
    }

    fn resolve_var(&self, var: &Var) -> anyhow::Result<Option<Rhs>> {
        self.resolve_recursive(var, None)
    }
}

fn resolve_rhs_recursive(
    rhs: &mut Rhs,
    var_history: &VariableHistory,
    i: usize,
) -> anyhow::Result<()> {
    match rhs {
        Rhs::Array(items) => {
            for rhs_item in items.iter_mut() {
                resolve_rhs_recursive(&mut *rhs_item, var_history, i)?;
            }
        }
        Rhs::Var(var) => {
            // implicit strings when mentioned but not found in context earlier
            *rhs = match var_history.resolve_recursive(var, Some(i))? {
                None => Rhs::Val(Val::StringVal(match var {
                    Var::User { name, namespace: _ } => name.clone(),
                    Var::Log => {
                        return Err(anyhow!("Log can not be assigned."));
                    }
                    Var::Predef {
                        name: _,
                        namespace: _,
                    } => {
                        return Err(anyhow!("Predef variables can not be assigned."));
                    }
                })),
                Some(rhs) => rhs,
            };
        }
        _ => {}
    };

    Ok(())
}

fn resolve_stmt(
    stmt: &mut Statement,
    var_history: &VariableHistory,
    i: usize,
) -> anyhow::Result<()> {
    match stmt {
        Statement::AssignLeft { lhs: _, rhs } => {
            resolve_rhs_recursive(rhs, var_history, i)?;
        }
        Statement::AssignRight { lhs, rhs: _ } => {
            resolve_rhs_recursive(lhs, var_history, i)?;
        }
        Statement::Compare { rhs: _, lhs: _ } => {} // keep compare since only applicable to rules that are parsed as variables
    }

    Ok(())
}

pub fn compile_code(source_code_raw: &str) -> anyhow::Result<Evaluated> {
    let currdir = std::env::current_dir()?;
    compile_code_with_state(
        &(source_code_raw.to_owned() + "\n"), // hack to fix one-line token problems
        &currdir,
        None,
        std::io::stderr(),
        true,
    )
}

fn resolve_struct_in_ns(user_ns: &Vec<(Var, Rhs)>, var: &str) -> Option<AnySensor> {
    // var is the first namespace for the vars, so gather all and find _name, _type and _short in them.
    let name = var.to_owned();
    let mut short = None;
    let mut topic: Option<String> = None;
    let mut msg_type = None;
    let mut short_lookup: HashMap<String, String> = HashMap::new();

    for (v, rhs) in user_ns.iter() {
        if let Var::Predef { name, namespace } = v {
            if (*name).as_str() == "short" {
                let val = match rhs {
                    Rhs::Val(Val::StringVal(s)) => s.clone(),
                    _ => {
                        return None;
                    }
                };
                short_lookup.insert(val, namespace.first()?.clone());
            }
        }
    }
    for (v, rhs) in user_ns.iter() {
        match v {
            Var::Log => {}
            Var::Predef {
                name: predef_var_name,
                namespace,
            } => {
                let found_by_name = namespace
                    .first()
                    .map(|s| {
                        let long_name = (*s).as_str() == var;
                        if long_name {
                            return true;
                        }
                        let found_by_short = short_lookup.get(var);
                        if let Some(short_name) = found_by_short {
                            (*short_name).as_str() == (*s).as_str()
                        } else {
                            false
                        }
                    })
                    .unwrap_or_default();

                if !found_by_name {
                    continue;
                }

                match predef_var_name.as_str() {
                    "short" => match rhs {
                        Rhs::Val(Val::StringVal(s)) => {
                            short = Some((*s).clone());
                        }
                        _ => {
                            return None;
                        }
                    },
                    "topic" => match rhs {
                        Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => {
                            topic = Some((*s).clone());
                        }
                        _ => {
                            return None;
                        }
                    },
                    "type" => match rhs {
                        Rhs::Val(Val::StringVal(s)) => {
                            let typed = SensorType::from_str(s.as_str())
                                .expect("No error in impl, falls back to any");
                            msg_type = Some(typed);
                        }
                        _ => {
                            return None;
                        }
                    },
                    _ => {}
                }
            }
            Var::User {
                name: _,
                namespace: _,
            } => {}
        }
    }

    let id = match (topic, msg_type) {
        (Some(t), None) => SensorIdentification::Topic(t),
        (None, Some(t)) => SensorIdentification::Type(t),
        (Some(topic), Some(msg_type)) => SensorIdentification::TopicAndType { topic, msg_type },
        _ => {
            return None;
        }
    };

    Some(AnySensor { name, id, short })
}

fn resolve_sensors(
    user_ns: &Vec<(Var, Rhs)>,
    sensors: &[String],
) -> anyhow::Result<Vec<AnySensor>> {
    let mut mapped = vec![];
    for sensor in sensors {
        let any = resolve_struct_in_ns(user_ns, sensor)
            .ok_or(anyhow!("Could not find sensor: {}", &sensor))?;
        mapped.push(any);
    }
    Ok(mapped)
}

pub fn compile_code_with_state(
    source_code_raw: &str,
    source_code_parent_dir: &std::path::Path,
    var_state: Option<HashMap<Var, Rhs>>,
    mut out: impl std::io::Write,
    rich_out: bool,
) -> anyhow::Result<Evaluated> {
    let token_iter = Token::lexer(source_code_raw)
        .spanned()
        // Convert logos errors into tokens. We want parsing to be recoverable and not fail at the lexing stage, so
        // we have a dedicated `Token::Error` variant that represents a token error that was previously encountered
        .map(|(tok, span)| match tok {
            // Turn the `Range<usize>` spans logos gives us into chumsky's `SimpleSpan` via `Into`, because it's easier
            // to work with
            Ok(tok) => (tok, span.into()),
            Err(()) => (Token::Error, span.into()),
        });

    // Turn the token iterator into a stream that chumsky can use for things like backtracking
    let token_stream = Stream::from_iter(token_iter)
        // Tell chumsky to split the (Token, SimpleSpan) stream into its parts so that it can handle the spans for us
        // This involves giving chumsky an 'end of input' span: we just use a zero-width span at the end of the string
        .map((0..source_code_raw.len()).into(), |(t, s): (_, _)| (t, s));

    // Parse the token stream with our chumsky parser
    match parser().parse(token_stream).into_result() {
        Ok(sexpr) => {
            let owned = sexpr
                .into_iter()
                .map(|kind| kind.into())
                .collect::<Vec<StatementKindOwnedPass1>>();

            // Pass 1 -- parsing included files and expanding namespaces
            let mut ast = do_pass1(owned, source_code_parent_dir)?; // could print parse errors from included files

            let mut rules = Vec::new();
            let mut winds = Vec::new();
            // let mut ast: Vec<StatementKindOwnedPass1> = Vec::new();
            for expr in &ast {
                // clone for ease of use, but normally just use AST
                match expr {
                    StatementKindOwned::Rule(rule) => {
                        rules.push(rule.clone());
                    }
                    StatementKindOwned::Reset(_) => winds.push(expr.clone()),
                    StatementKindOwned::SendFrames(_) => winds.push(expr.clone()),
                    StatementKindOwned::VariableDef(_) => {}
                }
                // ast.push(expr.into());
            }
            let rules = RuleSexpr { rules };
            let rules = rules.eval()?;

            // Pass 2 -- needs entire ast for resolving. TODO could still be combined to Pass 1 with a subset of the ast to t-1.
            // Resolve all variables here, else implicit strings in nested assigns are handled as variables.
            // Use the repl state in case the user already run some variables from before which we want to build upon
            let var_history =
                VariableHistory::new(ast.clone()).with_state(var_state.clone().unwrap_or_default());
            for (i, node) in ast.iter_mut().enumerate() {
                match node {
                    StatementKindOwned::Rule(rule_owned) => {
                        for stmt in rule_owned.stmts.iter_mut() {
                            resolve_stmt(stmt, &var_history, i)?;
                        }
                    }
                    StatementKindOwned::VariableDef(statement) => {
                        resolve_stmt(statement, &var_history, i)?;
                    }
                    StatementKindOwned::Reset(_) => {}
                    StatementKindOwned::SendFrames(_) => {}
                }
            }

            // Ext. Pass 3 -- resolve the wind fns to anysensor to check if short or long form etc.
            let vars = VariableHistory::new(ast.clone()).with_state(var_state.unwrap_or_default());
            let bag_ns = vars.resolve_ns(&["_bag"]);
            let mut wind = Vec::new();
            for node in ast.into_iter() {
                match node {
                    StatementKindOwned::SendFrames(sf) => match sf {
                        PlayKindUnited::SensorCount {
                            sensors,
                            count,
                            trigger,
                            play_mode,
                        } => {
                            let sensors = resolve_sensors(&bag_ns, &sensors)?;
                            wind.push(WindFunction::SendFrames(PlayKindUnitedPass3::SensorCount {
                                sensors,
                                count,
                                trigger,
                                play_mode,
                            }));
                        }
                        PlayKindUnited::UntilSensorCount {
                            sending,
                            until_sensors,
                            until_count,
                            trigger,
                            play_mode,
                        } => {
                            let until_sensors = resolve_sensors(&bag_ns, &until_sensors)?;
                            let sending = resolve_sensors(&bag_ns, &sending)?;
                            wind.push(WindFunction::SendFrames(
                                PlayKindUnitedPass3::UntilSensorCount {
                                    sending,
                                    until_sensors,
                                    until_count,
                                    trigger,
                                    play_mode,
                                },
                            ));
                        }
                    },
                    StatementKindOwned::Reset(p) => {
                        wind.push(WindFunction::Reset(p));
                    }
                    _ => {}
                };
            }

            return Ok(Evaluated { rules, wind, vars });
        }
        Err(errs) => {
            let src = Source::from(source_code_raw);
            for err in errs {
                if rich_out {
                    let e = Report::build(ReportKind::Error, (), err.span().start)
                        .with_code(3)
                        .with_message(err.to_string())
                        .with_label(
                            Label::new(err.span().into_range())
                                .with_message(err.reason().to_string())
                                .with_color(Color::Red),
                        )
                        .finish();

                    e.write(src.clone(), &mut out).unwrap();
                } else {
                    let msg = format!(
                        "<{}-{}>  {}",
                        err.span().start,
                        err.span().end,
                        err.reason(),
                    );
                    out.write_all(msg.as_bytes()).unwrap();
                }
            }
        }
    }

    Err(anyhow!("Could not parse ratslang code."))
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

    #[test]
    fn test_minimal_rules() {
        const SRC: &str = r"
        rule!(var1
            testRat1 -> testRat2
        )
        
        rule!(var3
            testRat1, LOG <- testRat2
        )

        rule!(var4
            testRat1 == testRat2
        )
        ";

        let mut rules = Rules::new();
        rules.new_id();
        rules.new_id();
        rules.new_id();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "testRat1".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["testRat2".to_string()],
                    id: 1,
                }),
            }],
        );

        rules.insert(
            "var3".to_string(),
            vec![VariableHuman {
                ship: "testRat2".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["testRat1".to_string(), COMPARE_NODE_NAME.to_string()],
                    id: 2,
                }),
            }],
        );

        rules.insert(
            "var4".to_string(),
            vec![
                VariableHuman {
                    ship: "testRat2".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec![COMPARE_NODE_NAME.to_string()],
                        id: 3,
                    }),
                },
                VariableHuman {
                    ship: "testRat1".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec![COMPARE_NODE_NAME.to_string()],
                        id: 3,
                    }),
                },
            ],
        );

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.rules.store.is_empty());
        let eval = eval.rules;
        assert_eq!(eval, rules);
    }

    #[test]
    fn empty() {
        const SRC: &str = r"";
        let eval = compile_code(SRC);
        assert!(eval.is_ok());

        const SRC0: &str = r"
        ";
        let eval = compile_code(SRC0);
        assert!(eval.is_ok());

        const SRC1: &str = r"# blub";
        let eval = compile_code(SRC1);
        assert!(eval.is_ok());
    }

    #[test]
    fn minimal_wind() {
        const SRC: &str = r#"
        reset! ./bag_file_name

        _bag.{
            lidar.{
                _short = l
                _type = Pointcloud2
            }
            imu.{
                _short = i
                _type = Imu
            }
        }

        pf! l 10s var1
        pf! i, l 5 var1
        pf! l 10 var1

        rule!(var1
            testRat1 -> testRat2
        )
        pf! l 10s var1
        "#;

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.rules.raw().is_empty());
        assert!(!eval.wind.is_empty());
    }

    #[test]
    fn single_line_rule() {
        const SRC: &str = r"
        rule!(var1 testRat1 -> testRat2)
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.rules.raw().is_empty());
    }

    #[test]
    fn comments() {
        const SRC: &str = r"
        # bla,.
        # ups /// \masd
        a = 1 # yo
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(eval.rules.raw().is_empty());
        assert!(eval.wind.is_empty());
    }

    #[test]
    fn var_declare() {
        const SRC: &str = r"
         a <- 7
         b <- a
         a <- 6
         _l <- /cloud
        ";

        // let mut a = Token::lexer(SRC);
        // let b = a.next();
        // dbg!(b);

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let res = eval.vars.resolve("_l");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Path("/cloud".to_owned()));

        let res = eval.vars.resolve("a");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(6))));

        let res = eval.vars.resolve("b");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(7))));
    }

    #[test]
    fn var_declare_nested() {
        const SRC: &str = r"
         a.b.c = 7
         a.b = a.b.c
         a.b.c = 6
         wind._l = /cloud
        ";

        // let mut a = Token::lexer(SRC);
        // let b = a.next();
        // dbg!(b);

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let res = eval.vars.resolve("wind._l");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Path("/cloud".to_owned()));

        let res = eval.vars.resolve("a.b.c");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(6))));

        let res = eval.vars.resolve("a.b");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(7))));
    }

    #[test]
    fn include() {
        const SRC: &str = r"
        # like assigning a file to the current namespace
        <- ./../mt/test.mt

        # including in namespace blocks will prepend the namespaces to the included AST
        # (excluding rules, they are always global)
        c.{
            <- ./../mt/test.mt
        }
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let res = eval.vars.resolve("test");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(4))));

        let res = eval.vars.resolve("c.test");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(4))));
    }

    #[test]
    fn var_declare_blocks() {
        const SRC: &str = r"
        a.{
            b = 7
        }

        c.{
            t = 5

            d.{
                1 -> o
            }
            i = 95
        }
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let res = eval.vars.resolve("a.b");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(7))));

        let res = eval.vars.resolve("c.t");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(5))));

        let res = eval.vars.resolve("c.d.o");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(1))));

        let res = eval.vars.resolve("c.i");
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(95))));
    }

    #[test]
    fn in_ns() {
        // other, me
        let res = is_sub_namespace(&["a".to_owned()], &["a".to_owned()]); // true
        assert!(res);
        let res = is_sub_namespace(&["a".to_owned(), "b".to_owned()], &["a".to_owned()]); // a.b is deeper than a, so a.b can not be a sub namespace. false
        assert!(!res);
        let res = is_sub_namespace(&["a".to_owned()], &["a".to_owned(), "b".to_owned()]); // a is in a.b, true
        assert!(res);
        let res = is_sub_namespace(&[], &["a".to_owned()]); // empty is not in a.b, false
        assert!(!res);
    }

    #[test]
    fn resolve_ns() {
        const SRC: &str = r"
        rule! (var1
            testRat1 -> testRat2
            testRat1 -> testRat2, a.c
            a.b <- 5
        )

        _bag.{
            lidar.{
                short = l
                type = Pointcloud2
            }
            imu.{
                _short = i
                _type = Imu
            }
        }
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        let e = eval.vars.resolve_ns(&["a"]);
        assert_eq!(e.len(), 2);
        let (var, rhs) = &e[1];
        assert_eq!(*var, Var::from_str("b").unwrap());
        assert_eq!(*rhs, Rhs::Val(Val::NumVal(NumVal::Integer(5))));

        let e = eval.vars.resolve_ns(&["_bag"]);
        assert_eq!(e.len(), 4);
    }

    #[test]
    fn strings() {
        const SRC: &str = r#"
        a = "bla with space"
        b = Blub
        c = Lidar::Ouster
            
        "#;

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let e = eval.vars.resolve("a");
        assert!(e.is_ok());
        let e = e.unwrap();
        assert!(e.is_some());
        let rhs = e.unwrap();
        assert_eq!(rhs, Rhs::Val(Val::StringVal("bla with space".to_owned())));

        let e = eval.vars.resolve("b");
        assert!(e.is_ok());
        let e = e.unwrap();
        assert!(e.is_some());
        let rhs = e.unwrap();
        assert_eq!(rhs, Rhs::Val(Val::StringVal("Blub".to_owned())));

        let e = eval.vars.resolve("c");
        assert!(e.is_ok());
        let e = e.unwrap();
        assert!(e.is_some());
        let rhs = e.unwrap();
        assert_eq!(rhs, Rhs::Val(Val::StringVal("Lidar::Ouster".to_owned())));
    }

    #[test]
    fn arrays() {
        const SRC: &str = r#"
        a = [ "bla with space", Blub ]
        "#;

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let e = eval.vars.resolve("a");
        assert!(e.is_ok());
        let e = e.unwrap();
        assert!(e.is_some());
        let rhs = e.unwrap();
        assert_eq!(
            rhs,
            Rhs::Array(vec![
                Box::new(Rhs::Val(Val::StringVal("bla with space".to_owned()))),
                Box::new(Rhs::Val(Val::StringVal("Blub".to_owned()))),
            ])
        );
    }

    // #[test]
    // fn enums() {
    //     // in your code
    //     #[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
    //     enum Lidar {
    //         Ouster,
    //         Sick,
    //     }

    //     #[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
    //     enum Sensor {
    //         Lidar(Lidar),
    //         Imu,
    //     }

    //     // for rl
    //     #[derive(Debug)]
    //     enum Enums {
    //         Sensor(Sensor),
    //     }
    //     let a = Enums::Sensor(Sensor::Lidar(Lidar::Ouster));
    //     let b = format!("{:?}", a);
    //     let r = convert_to_path(&b);
    //     println!("{:?}", r);
    //     const SRC: &str = "a = Sensor::Lidar";

    // TODO Give Enum to parser function that has all possible variants as trait. so fn variants(self--impl display) -> [impl FromStr]. Then call that in parser and pass type through to AST. If parser finds Sensor type, that has the same name as one of the enums in display. so it calls the parse function on that variant.
    #[test]
    fn vec_mat_vals() {
        const SRC: &str = r"
        vec = [3, 2, 4]

        mat = [ [3, 1, 1],
                [3, 2, 4] ]
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        let vec = eval.vars.resolve("vec").unwrap().unwrap();
        assert_eq!(
            vec,
            Rhs::Array(vec![
                Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(3)))),
                Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(2)))),
                Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(4)))),
            ])
        );
        let mat = eval.vars.resolve("mat").unwrap().unwrap();
        assert_eq!(
            mat,
            Rhs::Array(vec![
                Box::new(Rhs::Array(vec![
                    Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(3)))),
                    Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(1)))),
                    Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(1)))),
                ])),
                Box::new(Rhs::Array(vec![
                    Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(3)))),
                    Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(2)))),
                    Box::new(Rhs::Val(Val::NumVal(NumVal::Integer(4)))),
                ])),
            ])
        );
    }

    //     #[test]
    //     fn resolve_struct() {
    //         const SRC: &str = r"
    // _bag.{
    // 	lidar.{
    // 		_short = l
    // 		_topic = /ouster/points
    // 		_type = Pointcloud2
    // 	}
    // }

    //         ";

    //         let eval = compile_code(SRC);
    //         assert!(eval.is_ok());
    //         let eval = eval.unwrap();

    //         resolve_struct_in_ns(user_ns, var)

    //     }
    #[test]
    fn advanced_wind() {
        const SRC: &str = r"
        reset! ./bag_file_name

        _bag.{
            lidar.{
                _short = l
                _type = Pointcloud2
            }
            imu.{
                _short = i
                _type = Imu
            }
        }

        # send all lidar frames between the absolute 10s and 20s of the bagfile immediately
        pf! l 10s..20s

        # send all lidar frames between the absolute 10s and 20s of the bagfile when reaching var1
        pf! l 10s..20s var1

        # send all lidar frames between the absolute 10s and 20s of the bagfile but slow down time so it sends it over 1 minute
        pf! l 10s..20s 2mins

        # send imu frames until 2 lidar frames are encountered
        pf! i,l 2

        # send imu frames until 2 lidar frames are encountered but spread over 8s real time
        pf! i,l 2 8s

        # send 10 lidar frames but spread them out over 20s
        pf! l 10 20s

        # send the next(!) 10 seconds of lidar frames every time it reaches var1
        pf! l 10s var1

        # fixed var
        pf! l 10s..20s var1 f

        # dynamic var
        pf! l 10s..20s var1 d

        # implicit dynamic var
        pf! l 10s..20s var1

        # relative play time
        pf! l 10s..20s 1.

        pf! l, i 2 1.
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.wind.is_empty());
    }

    #[test]
    fn test_loop_basic() {
        const SRC: &str = r"
        loop 3 {
            x = 1
            y = 2
        }
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        // The loop should expand to 3 copies of x=1 and y=2
        // So we should have x=1 defined 3 times (last one wins)
        let x = eval.vars.resolve("x").unwrap().unwrap();
        assert_eq!(x, Rhs::Val(Val::NumVal(NumVal::Integer(1))));
        let y = eval.vars.resolve("y").unwrap().unwrap();
        assert_eq!(y, Rhs::Val(Val::NumVal(NumVal::Integer(2))));
    }

    #[test]
    fn test_loop_nested() {
        const SRC: &str = r"
        loop 2 {
            a = 10
            loop 2 {
                b = 20
            }
        }
        ";

        let eval = compile_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let a = eval.vars.resolve("a").unwrap().unwrap();
        assert_eq!(a, Rhs::Val(Val::NumVal(NumVal::Integer(10))));
        let b = eval.vars.resolve("b").unwrap().unwrap();
        assert_eq!(b, Rhs::Val(Val::NumVal(NumVal::Integer(20))));
    }

    #[test]
    fn test_loop_token() {
        // Test that "loop" is recognized as a keyword token, not a variable
        let mut lex = Token::lexer("loop 2");
        assert_eq!(lex.next(), Some(Ok(Token::KwLoop)));
        assert_eq!(lex.next(), Some(Ok(Token::IntegerNumber(2))));
    }
}
