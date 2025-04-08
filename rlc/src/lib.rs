use anyhow::anyhow;
use ariadne::{Color, Label, Report, ReportKind, Source};
use core::fmt;
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

    // pub fn insert_strategie(&mut self, variable: String, strategie: VariableHuman) {
    //     match self.0.get_mut(&variable) {
    //         Some(el) => {
    //             el.push(strategie);
    //         }
    //         None => {
    //             self.0.insert(variable, vec![strategie]);
    //         }
    //     }
    // }

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
    let len = if slice.ends_with('s') { 4 } else { 3 };
    let f: i64 = slice[..slice.len() - len].parse().ok()?;
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
    let len = if slice.ends_with('s') { 4 } else { 3 };
    let f: f64 = slice[..slice.len() - len].parse().ok()?;
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

fn parse_sensor_type(c: char) -> Option<SensorType> {
    match c {
        'm' | 'M' => Some(SensorType::Mixed),
        'l' | 'L' => Some(SensorType::Lidar),
        'i' | 'I' => Some(SensorType::Imu),
        _ => None,
    }
}

// TODO use Result in this callback for feedback
fn play_frames_args<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<PlayType> {
    let chars = lex.slice()["play_frames ".len()..]
        .chars()
        .collect::<Vec<_>>();

    match chars.len() {
        1 => {
            let sensor = parse_sensor_type(chars[0])
                .ok_or_else(|| format!("Invalid sensor character: '{}'", chars[0]))
                .ok()?;
            Some(PlayType::SensorCount { sensor })
        }
        2 => {
            let sending = parse_sensor_type(chars[0])
                .ok_or_else(|| format!("Invalid sensor character: '{}'", chars[0]))
                .ok()?;
            let until_sensor = parse_sensor_type(chars[1])
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
enum SensorType {
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
    #[token("#")]
    CommentLineStart,

    #[token("+")]
    OpPlus,

    #[token("-")]
    OpMinus,

    #[regex(".", priority = 1)]
    Dot,

    #[token(",")]
    Comma,

    #[token("\n")]
    NewLine,

    #[token("[")]
    RuleDefinitionOpen,

    #[token("]")]
    RuleDefinitionClose,

    #[token("{")]
    BlockStart,

    #[token("}")]
    BlockEnd,

    #[token("(")]
    LParen,

    #[token(")")]
    RParen,

    // -- Operators --
    #[token("==")]
    OpCompare,

    #[token("<-")]
    OpAssignToLeft,

    #[token("->")]
    OpAssignToRight,

    #[token("..")]
    OpRange,

    // -- Keywords --
    #[token("if")]
    KwIf,

    #[token("LOG")]
    KwLog,

    #[token("~")]
    DoNotCareOptim,

    // -- Embedded functions (Wind) --
    #[regex(r"play_frames [m|l|i][l|i]", play_frames_args)]
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
            Self::FnReset => write!(f, "reset()"),
            Self::Comma => write!(f, ","),
            Self::KwLog => write!(f, "LOG"),
            Self::OpPlus => write!(f, "+"),
            Self::Dot => write!(f, "."),
            Self::OpMinus => write!(f, "-"),
            Self::Error => write!(f, "<error>"),
            Self::CommentLineStart => write!(f, "#"),
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

#[derive(Debug)]
pub struct Evaluated {
    pub rules: Option<Rules>,
    pub wind: Vec<WindSexpr>,
}

#[derive(Debug, Clone)]
pub struct Root<'a> {
    pub rules: Option<RuleSexpr<'a>>,
    pub wind: Vec<WindSexpr>,
}

// --- Wind commands ---
#[derive(Debug, Clone)]
pub struct WindSexpr {
    pub commands: Vec<WindCommand>,
}

#[derive(Debug, Clone)]
pub struct WindCommand {
    pub stmts: Vec<WindFn>,
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

#[derive(Debug, Clone, Copy)]
pub enum WindFn {
    Reset,
    SendFrames(PlayKindUnited),
}

// --- Coordinator Rules ---
#[derive(Debug, Clone)]
pub struct RuleSexpr<'a> {
    pub rules: Vec<Rule<'a>>,
}

#[derive(Debug, Clone)]
pub struct Rule<'a> {
    pub variable: &'a str,
    pub stmts: Vec<Statement<'a>>,
}

#[derive(Debug, Clone)]
pub enum Statement<'a> {
    // A statement is a left-hand side (one or more terms)
    // an operator ("->", "<-" or "==") and a right-hand side term.
    Assign {
        lhs: Vec<Expr<'a>>,
        op: Operator,
        rhs: Vec<Expr<'a>>,
    },
}

#[derive(Clone, Copy, Debug)]
pub enum Operator {
    Right,   // corresponds to "->"
    Left,    // corresponds to "<-"
    Compare, // corresponds to "=="
}

#[derive(Clone, Copy, Debug)]
pub enum Expr<'a> {
    Var(&'a str),
    Log, // represents the LOG keyword.
}

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
fn parser<'a, I>() -> impl Parser<'a, I, Root<'a>, extra::Err<Rich<'a, Token<'a>>>>
where
    I: ValueInput<'a, Token = Token<'a>, Span = SimpleSpan>,
{
    recursive(|_sexpr| {
        // parse a single “term” – either a Variable or the LOG keyword.
        let term = select! {
        Token::Variable(v) => Expr::Var(v),
        Token::KwLog => Expr::Log,
        }
        .labelled("term");
        // parse a comma-separated list of terms (the left-hand side).
        let comma = just(Token::Comma);
        let term = term.clone().separated_by(comma).collect().labelled("term");

        // parse one of the three operators.
        let op = select! {
            Token::OpAssignToRight => Operator::Right,
            Token::OpAssignToLeft => Operator::Left,
            Token::OpCompare => Operator::Compare,
        }
        .labelled("operator");

        // a statement is: lhs, then an operator, then a single term (the right-hand side).
        let statement = term
            .clone()
            .then(op)
            .then(term)
            .then_ignore(just(Token::NewLine))
            .map(|((lhs, op), rhs)| Statement::Assign { lhs, op, rhs })
            .labelled("statement");

        let rule_ending = just(Token::NewLine)
            .repeated()
            .ignore_then(just(Token::RuleDefinitionClose))
            .then_ignore(just(Token::RuleDefinitionOpen))
            .labelled("explicit rule end");

        // a rule header is a Variable enclosed in [ and ]
        let rule_header = just(Token::RuleDefinitionOpen)
            .ignore_then(select! { Token::Variable(v) => v }.labelled("rule name"))
            .then_ignore(just(Token::RuleDefinitionClose))
            .labelled("rule header");

        // a complete rule: a header, then optionally some newlines, then the statement.
        let rule = rule_header
            .then_ignore(just(Token::NewLine).repeated())
            .then(statement.repeated().collect())
            .then_ignore(just(Token::NewLine).repeated())
            .then_ignore(rule_ending.or_not())
            .map(|(variable, stmts)| Rule { variable, stmts })
            .labelled("rule");

        // the full SExpr contains one or more rules; make sure we consume all tokens.
        let rule_block = just(Token::NewLine)
            .repeated()
            .ignore_then(rule)
            .repeated()
            .collect()
            .then_ignore(end())
            .map(|rules| RuleSexpr { rules });

        let wind_block = just(Token::NewLine)
            .repeated()
            .ignore_then(just(Token::FnReset))
            .then_ignore(just(Token::NewLine).repeated())
            .map(|wind| WindSexpr { commands: vec![] });

        let one_block = rule_block.or_not().then(wind_block.repeated().collect());

        one_block.map(|(rule, wind)| Root { rules: rule, wind })
    })
}

impl<'a> Expr<'a> {
    pub fn var_name(&self) -> Option<String> {
        match self {
            Expr::Var(variable) => Some((*variable).to_owned()),
            Expr::Log => Some(COMPARE_NODE_NAME.to_owned()),
            // _ => None,
        }
    }
}

impl<'a> RuleSexpr<'a> {
    fn eval(&self) -> anyhow::Result<Rules> {
        let mut rules = Rules::new();
        for rule in self.rules.iter() {
            for stmt in rule.stmts.iter() {
                match &stmt {
                    Statement::Assign { lhs, op, rhs } => match op {
                        Operator::Right => {
                            let plan = ActionPlan::Shoot {
                                target: rhs.iter().flat_map(|part| part.var_name()).collect(),
                            };
                            lhs.iter().flat_map(|lh| lh.var_name()).for_each(|lh| {
                                let varh = VariableHuman {
                                    ship: lh,
                                    strategy: Some(plan.clone()),
                                };
                                rules.insert(rule.variable.to_owned(), vec![varh]);
                            });
                        }
                        Operator::Left => {
                            let plan = ActionPlan::Shoot {
                                target: lhs.iter().flat_map(|part| part.var_name()).collect(),
                            };
                            rhs.iter().flat_map(|rh| rh.var_name()).for_each(|rh| {
                                let varh = VariableHuman {
                                    ship: rh,
                                    strategy: Some(plan.clone()),
                                };
                                rules.insert(rule.variable.to_owned(), vec![varh]);
                            });
                        }
                        Operator::Compare => {
                            let plan = ActionPlan::Shoot {
                                target: vec![COMPARE_NODE_NAME.to_owned()],
                            };

                            let mut merged: Vec<Expr<'_>> = rhs.iter().cloned().collect();
                            merged.extend(lhs);

                            merged.iter().flat_map(|ex| ex.var_name()).for_each(|ex| {
                                let varh = VariableHuman {
                                    ship: ex,
                                    strategy: Some(plan.clone()),
                                };
                                rules.insert(rule.variable.to_owned(), vec![varh]);
                            });
                        }
                    },
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
    lines
        .skip(start_line)
        .take(end_line - start_line)
        .collect::<Vec<_>>()
        .join("\n");
    evaluate_code(&file)
}

pub fn evaluate_code(source_code_raw: &str) -> anyhow::Result<Evaluated> {
    // Create a logos lexer over the source code
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
            let evaled = match sexpr.rules {
                Some(rules) => Some(rules.eval()?),
                None => None,
            };

            // TODO evaluate or directly send wind

            return Ok(Evaluated {
                rules: evaled,
                wind: vec![],
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
    fn test_minimal() {
        const SRC: &str = r"
        [var1]
        testRat1 -> testRat2

        [var3]
        testRat1, LOG <- testRat2

        [var4]
        testRat1 == testRat2
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
        assert!(eval.rules.is_some());
        let eval = eval.rules.unwrap();
        assert_eq!(eval, rules);
    }
}
