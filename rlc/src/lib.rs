use core::fmt;
use serde::{Deserialize, Serialize};

#[derive(PartialEq, Eq, Debug)]
pub struct Rules(std::collections::HashMap<String, Vec<VariableHuman>>);

impl Rules {
    pub fn new() -> Self {
        Self(std::collections::HashMap::new())
    }

    pub fn insert(&mut self, variable: String, strategies: Vec<VariableHuman>) {
        self.0.insert(variable, strategies).unwrap();
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }

    pub fn insert_strategie(&mut self, variable: String, strategie: VariableHuman) {
        match self.0.get_mut(&variable) {
            Some(el) => {
                el.push(strategie);
            }
            None => {
                self.0.insert(variable, vec![strategie]);
            }
        }
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

use ariadne::{Color, Label, Report, ReportKind, Source};
use logos::{Lexer, Logos};

use chumsky::{
    input::{Stream, ValueInput},
    prelude::*,
};

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
    #[token("play_frames")]
    FnPlayFrames,

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

#[derive(Debug)]
enum SExpr<'a> {
    Float(f64),
    Integer(i64),
    FloatMilliseconds(f64),
    FloatMillimeter(f64),
    IntegerMilliseconds(i64),
    IntegerMillimeter(i64),
    Variable(&'a str),
    AssignLeft,
    AssignRight,
    FnPlayFrames,
    True,
    False,
    PredefVariable(&'a str),
    CommentLineStart,
    OpPlus,
    OpMinus,
    NewLine,
    RuleDefinitionOpen,
    RuleDefinitionClose,
    BlockStart,
    BlockEnd,
    LParen,
    RParen,
    OpCompare,
    OpAssignToLeft,
    OpAssignToRight,
    OpRange,
    If,
    DoNotCareOptim,
    Log,
}

impl fmt::Display for Token<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Self::KwLog => write!(f, "LOG"),
            Self::OpPlus => write!(f, "+"),
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
            Self::FnPlayFrames => write!(f, "play_frames"),
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
fn parser<'a, I>() -> impl Parser<'a, I, SExpr<'a>, extra::Err<Rich<'a, Token<'a>>>>
where
    I: ValueInput<'a, Token = Token<'a>, Span = SimpleSpan>,
{
    recursive(|sexpr| {
        let atom = select! {
            Token::FloatingNumber(x) => SExpr::Float(x),
            Token::IntegerNumber(x) => SExpr::Integer(x),
            Token::IntegerNumberMillimeter(x) => SExpr::IntegerMillimeter(x),
            Token::IntegerNumberMillisecond(x) => SExpr::IntegerMilliseconds(x),
            Token::FloatingNumberMillimeter(x) => SExpr::FloatMillimeter(x),
            Token::FloatingNumberMillisecond(x) => SExpr::FloatMilliseconds(x),
            Token::Variable(x) => SExpr::Variable(x),
            Token::PredefVariable(x) => SExpr::PredefVariable(x),
            Token::OpAssignToLeft => SExpr::AssignLeft,
            Token::OpAssignToRight => SExpr::AssignRight,
            Token::FnPlayFrames=> SExpr::FnPlayFrames,
            Token::True => SExpr::True,
            Token::False => SExpr::False,
            Token::CommentLineStart=> SExpr::CommentLineStart,
            Token::OpPlus => SExpr::OpPlus,
            Token::OpMinus => SExpr::OpMinus,
            Token::OpMinus => SExpr::OpMinus,
            Token::NewLine => SExpr::NewLine,
            Token::RuleDefinitionOpen => SExpr::RuleDefinitionOpen,
            Token::RuleDefinitionClose => SExpr::RuleDefinitionClose,
            Token::BlockStart => SExpr::BlockStart,
            Token::BlockEnd => SExpr::BlockEnd,
            Token::LParen => SExpr::LParen,
            Token::RParen => SExpr::RParen,
            Token::OpCompare => SExpr::OpCompare,
            Token::OpRange => SExpr::OpRange,
            Token::KwIf => SExpr::If,
            Token::DoNotCareOptim => SExpr::DoNotCareOptim,
            Token::KwLog => SExpr::Log,
        };

        atom
    })
}

impl<'a> SExpr<'a> {
    // Recursively evaluate an s-expression
    fn eval(&self) -> Result<f64, &'static str> {
        match self {
            Self::Float(x) => Ok(*x),
            Self::OpPlus => Err("Cannot evaluate operator '+'"),
            Self::OpMinus => Err("Cannot evaluate operator '-'"),
            _ => Ok(1.0), // Self::Mul => Err("Cannot evaluate operator '*'"),
                          // Self::Div => Err("Cannot evaluate operator '/'"),
                          // Self::List(list) => match &list[..] {
                          //     [Self::Add, tail @ ..] => tail.iter().map(SExpr::eval).sum(),
                          //     [Self::Mul, tail @ ..] => tail.iter().map(SExpr::eval).product(),
                          //     [Self::Sub, init, tail @ ..] => {
                          //         Ok(init.eval()? - tail.iter().map(SExpr::eval).sum::<Result<f64, _>>()?)
                          //     }
                          //     [Self::Div, init, tail @ ..] => {
                          //         Ok(init.eval()? / tail.iter().map(SExpr::eval).product::<Result<f64, _>>()?)
                          //     }
                          //     _ => Err("Cannot evaluate list"),
                          // },
        }
    }
}

pub fn evaluate_rules_file(path: &std::path::PathBuf) -> anyhow::Result<Rules> {
    evaluate_rules(&std::fs::read_to_string(path)?)
}

pub fn evaluate_rules(source_code_raw: &str) -> anyhow::Result<Rules> {
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
        // If parsing was successful, attempt to evaluate the s-expression
        Ok(sexpr) => match sexpr.eval() {
            Ok(out) => println!("Result = {}", out), // TODO evaluate to rules, instead. do not print, just return
            Err(err) => println!("Runtime error: {}", err),
        },
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

    todo!()
    // Ok(())
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

        const COMPARE_NODE_NAME: &'static str = "#compare";

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

        let eval = evaluate_rules(SRC);
        assert!(eval.is_ok());
        let eval = eval.unwrap();
        assert_eq!(eval, rules);
    }

    // #[test]
    // fn test_hello_world() {}
}
