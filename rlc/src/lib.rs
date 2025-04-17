use core::fmt;

use anyhow::anyhow;
use ariadne::{Color, Label, Report, ReportKind, Source};
use logos::{Lexer, Logos};
use serde::{Deserialize, Serialize};

use chumsky::{
    input::{Stream, ValueInput},
    prelude::*,
};

#[derive(PartialEq, Eq, Debug)]
pub struct Rules(std::collections::HashMap<String, Vec<VariableHuman>>);

pub const COMPARE_NODE_NAME: &'static str = "#compare";

impl Rules {
    pub fn new() -> Self {
        Self(std::collections::HashMap::new())
    }

    pub fn insert(&mut self, variable: String, strategies: Vec<VariableHuman>) {
        match self.0.get_mut(&variable) {
            Some(el) => {
                el.extend(strategies);
            }
            None => {
                self.0.insert(variable, strategies);
            }
        }
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn raw(&self) -> &std::collections::HashMap<String, Vec<VariableHuman>> {
        &self.0
    }
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
pub struct VariableHuman {
    pub ship: String,
    pub strategy: Option<ActionPlan>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub enum ActionPlan {
    #[default]
    Sail,
    Shoot {
        target: Vec<String>,
    },
    Catch {
        source: String,
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

// TODO use Result in this callback for feedback
fn play_frames_args<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<PlayType> {
    let chars = lex.slice()["play_frames ".len()..]
        .chars()
        .collect::<Vec<_>>();

    match chars.len() {
        1 => {
            let sensor = match chars[0] {
                'm' | 'M' => Some(SensorType::Mixed),
                'l' | 'L' => Some(SensorType::Lidar),
                'i' | 'I' => Some(SensorType::Imu),
                _ => None,
            }
            .ok_or_else(|| format!("Invalid sensor character: '{}'", chars[0]))
            .ok()?;
            Some(PlayType::SensorCount { sensor })
        }
        2 => {
            let sending = match chars[0] {
                'm' | 'M' => Some(SensorType::Mixed),
                'l' | 'L' => Some(SensorType::Lidar),
                'i' | 'I' => Some(SensorType::Imu),
                _ => None,
            }
            .ok_or_else(|| format!("Invalid sensor character: '{}'", chars[0]))
            .ok()?;
            let until_sensor = match chars[1] {
                'l' | 'L' => Some(SensorType::Lidar),
                'i' | 'I' => Some(SensorType::Imu),
                _ => None,
            }
            .ok_or_else(|| format!("Invalid sensor character: '{}'", chars[1]))
            .ok()?;
            Some(PlayType::UntilSensorCount {
                sending,
                until_sensor,
            })
        }
        _ => Err("Input must be one or two characters long".to_string()).ok()?,
    }
}

#[derive(Debug, PartialEq, Clone, Copy, Default)]
pub enum SensorType {
    Lidar,
    Imu,
    #[default]
    Mixed,
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum PlayType {
    SensorCount {
        sensor: SensorType,
    },
    UntilSensorCount {
        sending: SensorType,
        until_sensor: SensorType,
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

    #[regex(".", priority = 1)]
    Dot,

    #[token(",")]
    Comma,

    #[regex(r"(#.*)?\n")]
    NewLine,

    #[token("(")]
    RuleDefinitionOpen,

    #[token(")")]
    RuleDefinitionClose,

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

    #[token("<-")]
    OpAssignToLeft,

    #[regex("->", priority = 1000)]
    OpAssignToRight,

    #[token("..")]
    OpRange,

    #[regex(r"\.?\/+[^ \n]*")]
    Path(&'a str),

    // -- Keywords --
    #[token("if")]
    KwIf,

    #[token("LOG")]
    KwLog,

    #[token("~")]
    DoNotCareOptim,

    // -- Embedded functions (Wind) --
    #[regex(r"play_frames [m|l|i][l|i]?", play_frames_args)]
    FnPlayFrames(PlayType),

    #[token("reset")]
    FnReset,

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

    #[regex(r"[a-zA-Z_]+\d*[a-zA-Z_\d]*")]
    Variable(&'a str),

    #[regex(r"_[a-zA-Z_]+\d*[a-zA-Z_\d]*", rm_first)]
    PredefVariable(&'a str),
}

impl fmt::Display for Token<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::Path(path) => write!(f, "Path({:?})", path),
            Self::FnReset => write!(f, "reset()"),
            Self::Comma => write!(f, ","),
            Self::KwLog => write!(f, "LOG"),
            Self::OpPlus => write!(f, "+"),
            Self::Dot => write!(f, "."),
            Self::OpMinus => write!(f, "-"),
            Self::Error => write!(f, "<error>"),
            Self::NewLine => write!(f, "\\n"),
            Self::RuleDefinitionOpen => write!(f, "["),
            Self::RuleDefinitionClose => write!(f, "]"),
            Self::BlockStart => write!(f, "{}", "{"),
            Self::BlockEnd => write!(f, "{}", "}"),
            Self::LParen => write!(f, "("),
            Self::RParen => write!(f, ")"),
            Self::OpCompare => write!(f, "=="),
            Self::OpAssignToLeft => write!(f, "<-"),
            Self::OpAssignToRight => write!(f, "->"),
            Self::OpRange => write!(f, ".."),
            Self::KwIf => write!(f, "if"),
            Self::DoNotCareOptim => write!(f, "~"),
            Self::FnPlayFrames(pt) => write!(f, "play_frames({:?})", pt),
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
    Time,
    Way,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Val {
    UnitedVal(UnitVal),
    NumVal(NumVal),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnitVal {
    pub val: u64,
    pub unit: Unit,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Rhs {
    Range { from: Val, to: Val },
    Var(Var),
    Path(String),
    // Expr() TODO evaluate so in the end it becomes atomic Rhs
    Val(Val),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Var {
    User(String),
    Log,
    Predef(String),
}

#[derive(Debug)]
pub struct Evaluated {
    pub rules: Rules,
    pub wind: Vec<WindFunctions>,
    pub vars: VariableHistory,
}

#[derive(Debug)]
pub enum WindFunctions {
    Reset(String),
    SendFrames { kind: PlayKindUnited, at: String },
}

#[derive(Debug, Clone)]
pub struct Root<'a> {
    pub rules: RuleSexpr<'a>,
    pub wind: Vec<StatementKind<'a>>,
    pub vardefs: Vec<Statement>,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PlayKindUnit {
    TimeMs(u64),
    Count(u64),
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PlayKindUnited {
    SensorCount {
        sensor: SensorType,
        count: usize,
    },
    UntilSensorCount {
        sending: SensorType,
        until_sensor: SensorType,
        until_count: usize,
    },
    UntilTime {
        sending: SensorType,
        duration: std::time::Duration,
    },
}

#[derive(Debug, Clone)]
pub enum StatementKind<'a> {
    Rule(Rule<'a>),
    VariableDef(Statement),
    Reset(String),
    SendFrames { kind: PlayKindUnited, at: String },
}

#[derive(Debug, Clone)]
pub enum StatementKindOwned {
    Rule(RuleOwned),
    VariableDef(Statement),
    Reset(String),
    SendFrames { kind: PlayKindUnited, at: String },
}

impl<'a> From<StatementKind<'a>> for StatementKindOwned {
    fn from(value: StatementKind<'a>) -> Self {
        match value {
            StatementKind::Rule(rule) => StatementKindOwned::Rule(RuleOwned::from(rule)),
            StatementKind::VariableDef(statement) => StatementKindOwned::VariableDef(statement),
            StatementKind::Reset(path) => StatementKindOwned::Reset(path),
            StatementKind::SendFrames { kind, at } => StatementKindOwned::SendFrames { kind, at },
        }
    }
}

// --- Coordinator Rules ---
#[derive(Debug, Clone)]
pub struct RuleSexpr<'a> {
    pub rules: Vec<Rule<'a>>,
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
pub enum Statement {
    // A statement is a left-hand side (one or more terms)
    // an operator ("->", "<-" or "==") and a right-hand side term.
    AssignLeft { lhs: Vec<Var>, rhs: Rhs },
    AssignRight { lhs: Rhs, rhs: Vec<Var> },
    Compare { rhs: Vec<Var>, lhs: Vec<Var> },
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
fn parser<'a, I>() -> impl Parser<'a, I, Vec<StatementKind<'a>>, extra::Err<Rich<'a, Token<'a>>>>
where
    I: ValueInput<'a, Token = Token<'a>, Span = SimpleSpan>,
{
    recursive(|_sexpr| {
        let time = select! {
            Token::IntegerNumberMillisecond(ms) => NumVal::Integer(ms as i64),
            Token::FloatingNumberMillisecond(ms) => NumVal::Floating(ms).integerize_unit_val(),
        }
        .labelled("time");

        let way = select! {
            Token::IntegerNumberMillimeter(millis) => NumVal::Integer(millis as i64),
            Token::FloatingNumberMillimeter(millis) => NumVal::Floating(millis).integerize_unit_val(),
        }.labelled("way");

        let timerange = time
            .then_ignore(just(Token::OpRange))
            .then(time)
            .try_map(|(from, to), span| {
                Ok(Rhs::Range {
                    from: Val::UnitedVal(UnitVal {
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
                        unit: Unit::Time,
                    }),
                    to: Val::UnitedVal(UnitVal {
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
                        unit: Unit::Time,
                    }),
                })
            })
            .labelled("time range");

        let wayrange = way
            .then_ignore(just(Token::OpRange))
            .then(way)
            .try_map(|(from, to), span| {
                Ok(Rhs::Range {
                    from: Val::UnitedVal(UnitVal {
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
                        unit: Unit::Way,
                    }),
                    to: Val::UnitedVal(UnitVal {
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
                        unit: Unit::Way,
                    }),
                })
            })
            .labelled("way range");

        let value = select! {
                Token::IntegerNumber(i) => Rhs::Val(Val::NumVal(NumVal::Integer(i))),
                Token::FloatingNumber(f) => Rhs::Val(Val::NumVal(NumVal::Floating(f))),
        }
        .labelled("number value");

        let numberrange = value
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
                ) => {
                    return Err(Rich::custom(span, "Cannot span between float and integer."));
                }
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
                    from: Val::NumVal(from),
                    to: Val::NumVal(to),
                }
            })
            .labelled("number range");

        // TODO parse recursive of other variable that can hold a value
        let term = select! {
            Token::Variable(v) => Rhs::Var(Var::User(v.to_owned())),
            Token::KwLog => Rhs::Var(Var::Log), // TODO maybe not rhs that can be assigned
            Token::Path(p) => Rhs::Path(p.to_owned()),
            Token::PredefVariable(pd) => Rhs::Var(Var::Predef(pd.to_owned())),
        }
        .labelled("term");

        let range = choice((numberrange, timerange, wayrange));
        let rhs = choice((term, range, value));

        // parse a comma-separated list of terms (the left-hand side).
        let comma = just(Token::Comma);
        let multi_rhs = rhs
            .clone()
            .separated_by(comma)
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
            .then_ignore(just(Token::NewLine).or_not())
            .try_map(|((lhs, op), rhs), span| {
                Ok(StatementKind::VariableDef(match op {
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

        // a rule header is a Variable enclosed in [ and ]
        let rule_header = just(Token::RuleDefinitionOpen)
            .ignore_then(select! { Token::Variable(v) => v }.labelled("rule name"))
            .then_ignore(just(Token::NewLine).or_not())
            .labelled("rule header");

        // a complete rule: a header, then optionally some newlines, then the statement.
        let rule = rule_header
            .then_ignore(just(Token::NewLine).repeated())
            .then(statement.clone().repeated().collect())
            .then_ignore(just(Token::NewLine).repeated())
            .then_ignore(just(Token::RuleDefinitionClose))
            .map(|(variable, stmts): (&str, Vec<StatementKind>)| {
                let stmts = stmts
                    .into_iter()
                    .map(|stmtk| match stmtk {
                        StatementKind::VariableDef(var) => var,
                        _ => unreachable!(),
                    })
                    .collect::<Vec<_>>();
                StatementKind::Rule(Rule { variable, stmts })
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
            .map(|expr| StatementKind::Reset(expr.to_owned()))
            .labelled("reset");

        let wind_play_frames_fn = select! { Token::FnPlayFrames(pt) => StatementKind::SendFrames { kind: match pt {
                    PlayType::SensorCount { sensor } => PlayKindUnited::SensorCount { sensor: sensor, count: 0 },
                    PlayType::UntilSensorCount { sending, until_sensor } => PlayKindUnited::UntilSensorCount { sending: sending, until_sensor: until_sensor, until_count: 0 },
                }, at: "".to_owned() }}.labelled("send_frames")
            .then(select! {
                Token::IntegerNumberMillisecond(ms) => PlayKindUnit::TimeMs(ms as u64),
                Token::FloatingNumberMillisecond(ms) => PlayKindUnit::TimeMs(ms.round() as u64),
                Token::IntegerNumber(i) => PlayKindUnit::Count(i as u64),
            }.labelled("int number or timespan"))
            .then(select! {
                Token::Variable(v) => Var::User(v.to_owned()),
            }.labelled("var as trigger"))
            .try_map(|((f, arg), at), span| {
                    match (f, arg) {
                        (StatementKind::SendFrames{ kind: PlayKindUnited::SensorCount { sensor, count: _ }, at: _ }, PlayKindUnit::TimeMs(ms)) => {
                            Ok(StatementKind::SendFrames{ kind: PlayKindUnited::UntilTime { sending: sensor, duration: std::time::Duration::from_millis(ms) }, at: match at {
                                Var::User(v) => v,
                                _ => unreachable!(),
                            }})
                        },
                        (StatementKind::SendFrames{ kind: PlayKindUnited::SensorCount { sensor, count: _}, at: _}, PlayKindUnit::Count(count)) => {
                            Ok(StatementKind::SendFrames{ kind: PlayKindUnited::SensorCount { sensor, count: count as usize }, at: match at {
                                Var::User(v) => v,
                                _ => unreachable!(),
                            }})
                        },
                        (StatementKind::SendFrames{ kind: PlayKindUnited::UntilSensorCount { sending, until_sensor, until_count: _}, at: _ }, PlayKindUnit::Count(count)) => {
                            Ok(StatementKind::SendFrames{ kind: PlayKindUnited::UntilSensorCount { sending, until_sensor, until_count: count as usize }, at: match at {
                                Var::User(v) => v,
                                _ => unreachable!(),
                            }})
                        },
                        (StatementKind::Reset(_), _) => {
                            Err(Rich::custom(span, "Reset does not take arguments."))
                        },
                        (StatementKind::Rule(_), _) => {
                            Err(Rich::custom(span, "Rules do not take arguments."))
                        },
                        (StatementKind::SendFrames { kind: _, at: _}, _) => {
                            Err(Rich::custom(span, "Unexpected arguments to send_frames function."))
                        },
                        (StatementKind::VariableDef(_), _) => unreachable!("parsed variable definition"),
                    }
                }).labelled("play_frames");

        let one_of_wind_fns = choice((statement, wind_play_frames_fn, wind_reset_fn, rule));
        let newlines = just(Token::NewLine).repeated();
        let one_block = newlines
            .clone()
            .ignore_then((one_of_wind_fns.then_ignore(newlines)).repeated().collect())
            .then_ignore(end());
        one_block
    })
}

// impl<'a> Expr<'a> {
//     pub fn var_name(&self) -> Option<String> {
//         match self {
//             Expr::Var(variable) => Some((*variable).to_owned()),
//             Expr::Log => Some(COMPARE_NODE_NAME.to_owned()),
//             Expr::Predef(variable) => Some((*variable).to_owned()),
//             Expr::Path(_) => None,
//         }
//     }
// }

impl<'a> RuleSexpr<'a> {
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
                                    Var::User(v) => v.clone(),
                                    Var::Log => COMPARE_NODE_NAME.to_owned(),
                                    Var::Predef(_) => unreachable!(),
                                })
                                .collect(),
                        };

                        let varh = VariableHuman {
                            ship: match rhs {
                                Rhs::Var(var) => match var {
                                    Var::User(v) => v.clone(),
                                    Var::Log => COMPARE_NODE_NAME.to_owned(),
                                    Var::Predef(_) => unreachable!(),
                                },
                                _ => unreachable!(),
                            },
                            strategy: Some(plan.clone()),
                        };
                        rules.insert(rule.variable.to_owned(), vec![varh]);
                    }
                    Statement::AssignRight { lhs, rhs } => {
                        let plan = ActionPlan::Shoot {
                            target: rhs
                                .iter()
                                .map(|part| match part {
                                    Var::User(v) => v.clone(),
                                    Var::Log => COMPARE_NODE_NAME.to_owned(),
                                    Var::Predef(_) => unreachable!(),
                                })
                                .collect(),
                        };

                        let varh = VariableHuman {
                            ship: match lhs {
                                Rhs::Var(var) => match var {
                                    Var::User(v) => v.clone(),
                                    Var::Log => COMPARE_NODE_NAME.to_owned(),
                                    Var::Predef(_) => unreachable!(),
                                },
                                _ => unreachable!(),
                            },
                            strategy: Some(plan.clone()),
                        };
                        rules.insert(rule.variable.to_owned(), vec![varh]);
                    }
                    Statement::Compare { rhs, lhs } => {
                        let plan = ActionPlan::Shoot {
                            target: vec![COMPARE_NODE_NAME.to_owned()],
                        };

                        let mut merged: Vec<String> = rhs
                            .iter()
                            .map(|var| match var {
                                Var::User(v) => v.clone(),
                                Var::Log => COMPARE_NODE_NAME.to_owned(),
                                _ => unreachable!(),
                            })
                            .collect();

                        merged.extend(lhs.iter().map(|var| match var {
                            Var::User(v) => v.clone(),
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

pub fn evaluate_file(
    path: &std::path::PathBuf,
    start_line: Option<usize>,
    end_line: Option<usize>,
) -> anyhow::Result<Evaluated> {
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
    let start_line = start_line.unwrap_or_default();
    let lines = file.lines();
    let end_line = end_line.unwrap_or(lines.clone().count());
    let selected_lines = lines
        .skip(start_line)
        .take(end_line - start_line)
        .collect::<Vec<_>>()
        .join("\n");
    evaluate_code(&selected_lines)
}

#[derive(Debug)]
pub struct VariableHistory {
    ast: Vec<StatementKindOwned>,
}

impl VariableHistory {
    pub fn new(ast: Vec<StatementKindOwned>) -> Self {
        VariableHistory { ast }
    }

    fn resolve_recursive(&self, var: &Var, up_to: Option<usize>) -> anyhow::Result<Option<Rhs>> {
        let mut val = None;
        for (i, stmt) in self
            .ast
            .iter()
            .enumerate()
            .take(up_to.unwrap_or(usize::MAX))
        {
            match &stmt {
                &StatementKindOwned::Rule(rule) => {
                    for rstmt in rule.stmts.iter() {
                        match rstmt {
                            Statement::AssignLeft { lhs, rhs } => {
                                if lhs.contains(var) {
                                    val = Some(match rhs {
                                        Rhs::Var(var) => {
                                            match self.resolve_recursive(var, Some(i))? {
                                                None => {
                                                    return Err(anyhow!(
                                                        "Assigned variable undefined."
                                                    ));
                                                }
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
                                                None => {
                                                    return Err(anyhow!(
                                                        "Assigned variable undefined."
                                                    ));
                                                }
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
                &StatementKindOwned::VariableDef(statement) => match statement {
                    Statement::AssignLeft { lhs, rhs } => {
                        if lhs.contains(var) {
                            val = Some(match rhs {
                                Rhs::Var(var) => match self.resolve_recursive(&var, Some(i))? {
                                    None => {
                                        return Err(anyhow!("Assigned variable undefined."));
                                    }
                                    Some(rhs) => rhs,
                                },
                                _ => rhs.clone(),
                            });
                        }
                    }
                    Statement::AssignRight { lhs, rhs } => {
                        if rhs.contains(var) {
                            val = Some(match lhs {
                                Rhs::Var(var) => match self.resolve_recursive(&var, Some(i))? {
                                    None => {
                                        return Err(anyhow!("Assigned variable undefined."));
                                    }
                                    Some(lhs) => lhs,
                                },
                                _ => lhs.clone(),
                            });
                        }
                    }
                    Statement::Compare { rhs: _, lhs: _ } => {}
                },
                &StatementKindOwned::Reset(_) => {}
                &StatementKindOwned::SendFrames { kind: _, at: _ } => {}
            }
        }
        Ok(val)
    }

    pub fn resolve(&self, var: &Var) -> anyhow::Result<Option<Rhs>> {
        self.resolve_recursive(var, None)
    }
}

pub fn evaluate_code(source_code_raw: &str) -> anyhow::Result<Evaluated> {
    // Create a logos lexer over the source code
    let token_iter = Token::lexer(&source_code_raw)
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
            let mut rules = Vec::new();
            let mut winds = Vec::new();
            let mut ast: Vec<StatementKindOwned> = Vec::new();
            for expr in sexpr {
                // clone for ease of use, but normally just use AST
                match &expr {
                    StatementKind::Rule(rule) => {
                        rules.push(rule.clone());
                    }
                    StatementKind::Reset(_) => winds.push(expr.clone()),
                    StatementKind::SendFrames { kind: _, at: _ } => winds.push(expr.clone()),
                    StatementKind::VariableDef(_) => {}
                }
                ast.push(expr.into());
            }
            let rules = RuleSexpr { rules };
            let rules = rules.eval()?;
            let wind = winds
                .into_iter()
                .filter_map(|f| match f {
                    StatementKind::Reset(file) => Some(WindFunctions::Reset(file.to_owned())),
                    StatementKind::SendFrames { kind, at } => {
                        Some(WindFunctions::SendFrames {
                            kind,
                            at: at.to_owned(),
                        }) // owned to make easier to outside parser, TODO maybe passthrough lifetimes
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();

            return Ok(Evaluated {
                rules,
                wind,
                vars: VariableHistory { ast },
            });
        }
        // If parsing was unsuccessful, generate a nice user-friendly diagnostic with ariadne. You could also use
        // codespan, or whatever other diagnostic library you care about. You could even just display-print the errors
        // with Rust's built-in `Display` trait, but it's a little crude
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
                    .eprint(Source::from(source_code_raw))
                    .unwrap();
            }
        }
    }

    Err(anyhow!("Could not parse"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minimal_rules() {
        const SRC: &str = r"
        (var1
            testRat1 -> testRat2
        )
        
        (var3
            testRat1, LOG <- testRat2
        )

        (var4
            testRat1 == testRat2
        )
        ";

        let mut rules = Rules::new();
        rules.insert(
            "var1".to_string(),
            vec![VariableHuman {
                ship: "testRat1".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["testRat2".to_string()],
                }),
            }],
        );

        rules.insert(
            "var3".to_string(),
            vec![VariableHuman {
                ship: "testRat2".to_string(),
                strategy: Some(ActionPlan::Shoot {
                    target: vec!["testRat1".to_string(), COMPARE_NODE_NAME.to_string()],
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
                    }),
                },
                VariableHuman {
                    ship: "testRat1".to_string(),
                    strategy: Some(ActionPlan::Shoot {
                        target: vec![COMPARE_NODE_NAME.to_string()],
                    }),
                },
            ],
        );

        let eval = evaluate_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.rules.0.is_empty());
        let eval = eval.rules;
        assert_eq!(eval, rules);
    }

    #[test]
    fn minimal_wind() {
        const SRC: &str = r"
        play_frames l 10s var1

        play_frames il 5 var1
        reset bag_file_name
        play_frames l 10 var1
        play_frames l 10 var1

        (var1
            testRat1 -> testRat2
        )
        play_frames l 10s var1
        ";

        let eval = evaluate_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.rules.raw().is_empty());
        assert!(!eval.wind.is_empty());
    }

    #[test]
    fn single_line_rule() {
        const SRC: &str = r"
        (var1 testRat1 -> testRat2)
        ";

        let eval = evaluate_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(!eval.rules.raw().is_empty());
    }

    #[test]
    fn comments() {
        const SRC: &str = r"
        # bla,.
        # ups /// \masd
        play_frames l 10s var1 # yo
        ";

        let eval = evaluate_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert!(eval.rules.raw().is_empty());
        assert!(!eval.wind.is_empty());
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
        // let b = a.next();
        // let b = a.next();
        // let b = a.next();
        // // let b = a.next();
        // dbg!(b);

        // let mut winds = Vec::new();
        // winds.push(WindFunctions::SendFrames(()));

        let eval = evaluate_code(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();

        let res = eval.vars.resolve(&Var::Predef("l".to_owned()));
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Path("/cloud".to_owned()));

        let res = eval.vars.resolve(&Var::User("a".to_owned()));
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(6))));

        let res = eval.vars.resolve(&Var::User("b".to_owned()));
        assert!(res.is_ok());
        let res = res.unwrap();
        assert!(res.is_some());
        let res = res.unwrap();
        assert_eq!(res, Rhs::Val(Val::NumVal(NumVal::Integer(7))));
    }

    // // TODO impl
    // // #[test]
    // fn vec_mat_vals() {
    //     const SRC: &str = r"
    //     vec = [3, 2, 4]

    //     mat = [ [3, 1, 1]
    //             [3, 2, 4] ]
    //     ";

    //     // let eval = evaluate_code(SRC);
    //     // assert!(eval.is_ok());
    //     // let eval = eval.unwrap();
    //     // assert!(eval.rules.raw().is_empty());
    //     // assert!(!eval.wind.is_empty());
    // }

    // // TODO impl
    // // #[test]
    // fn configuration_blocks() {
    //     const SRC: &str = r"
    //     # simple
    //     foo.{
    //         blub <- 5cm..1.8m  # foo.blub <- 5cm..1cm
    //         2s -> foo       # 2s -> foo.foo
    //         bar = 3         # foo.bar <- 3
    //     }

    //     # recursive, nested
    //     preprocess.{

    //         res = 5cm

    //         prep.{
    //             bla = 32.345
    //         }
    //     }
    //     ";

    //     // let eval = evaluate_code(SRC);
    //     // assert!(eval.is_ok());
    //     // let eval = eval.unwrap();
    //     // assert!(eval.rules.raw().is_empty());
    //     // assert!(!eval.wind.is_empty());
    // }
}
