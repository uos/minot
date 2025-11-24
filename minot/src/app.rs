use core::panic;
use std::{
    collections::HashMap,
    fmt::Display,
    path::{Path, PathBuf},
    str::FromStr,
    sync::{Arc, Mutex, RwLock},
    time::{Duration, Instant},
};

use anyhow::{Error, anyhow};
use log::{error, info, warn};
use mtc::{
    Evaluated, PlayKindUnitedPass3, PlayMode, PlayTrigger, Rules, VariableHistory, WindFunction,
};
use once_cell::sync::Lazy;
use ratatui::{
    layout::Constraint,
    style::Stylize,
    widgets::{Cell, Paragraph, Row, ScrollbarState, Table, TableState},
};
use regex::Regex;
use rust_decimal::{Decimal, prelude::FromPrimitive};
use sea::net::{PacketKind, WindAt};

// --- Add this static regex ---
// This regex matches lines like:
// #--- SOME_NAME
// #---SOME_NAME_123
// #---    SOME_NAME_WITH_SPACES (Note: my regex below does *not* allow spaces, only one optional space after #---)
// The regex: r"^#--- ?[a-zA-Z0-9_]+$"
// ^      - start of the line
// #---   - literal characters
//  ?     - an optional space
// [a-zA-Z0-9_]+ - one or more letters, numbers, or underscores for the name
// $      - end of the line
static START_MARKER_RE: Lazy<Regex> = Lazy::new(|| Regex::new(r"^#--- ?([a-zA-Z0-9_]+)$").unwrap());

/// Application result type.
pub type AppResult<T> = std::result::Result<T, Box<dyn core::error::Error>>;

#[derive(Debug)]
pub enum Mode {
    Wind { hide_compare: bool },
    Compare,
}

impl Default for Mode {
    fn default() -> Self {
        Self::Wind { hide_compare: true }
    }
}

impl Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Mode::Wind { hide_compare: _ } => f.write_str("Wind"),
            Mode::Compare => f.write_str("Compare"),
        }
    }
}

#[derive(Debug)]
pub enum LockedState {
    Locked,
    Unlocked,
}

impl Display for LockedState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            LockedState::Locked => f.write_str("🔒"),
            LockedState::Unlocked => f.write_str("🔓"),
        }
    }
}

#[derive(Debug)]
pub struct Tolerance {
    pub value: String,
    pub pot_cursor: u8,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self {
            value: "0.".to_owned(),
            pot_cursor: 0,
        }
    }
}

impl From<&Tolerance> for f64 {
    fn from(value: &Tolerance) -> Self {
        let v = if value.value.ends_with(".") {
            value.value.clone() + "0"
        } else {
            value.value.clone()
        };
        v.parse::<f64>()
            .expect("tolerance string value is not a float")
    }
}

impl Tolerance {
    pub fn change_tolerance_at_current_cursor(&mut self, direction: VerticalDirection) {
        let dig = self
            .value
            .chars()
            .skip(self.pot_cursor as usize)
            .take(1)
            .collect::<Vec<char>>()
            .first()
            .map(|c| c.to_digit(10).map(|el| el as i8));

        self.value = self
            .value
            .chars()
            .enumerate()
            .map(|(i, c)| {
                if self.pot_cursor as usize == i {
                    let ndig = match direction {
                        VerticalDirection::Up => {
                            if let Some(Some(dig)) = dig {
                                dig + 1
                            } else {
                                0
                            }
                        }
                        VerticalDirection::Down => {
                            if let Some(Some(dig)) = dig {
                                (dig - 1).max(0)
                            } else {
                                0
                            }
                        }
                        VerticalDirection::Row(ndig) => ndig as i8,
                    }
                    .max(0) as u32;
                    if ndig < 10 {
                        char::from_digit(ndig, 10).unwrap()
                    } else {
                        c
                    }
                } else {
                    c
                }
            })
            .collect();
    }

    pub fn scroll_cursor(&mut self, direction: HorizontalDirection) {
        let ndig = match direction {
            HorizontalDirection::Left => self.pot_cursor as i8 - 1,
            HorizontalDirection::Right => self.pot_cursor as i8 + 1,
            HorizontalDirection::Col(npot) => npot as i8,
        };

        // prepend
        let ndig = {
            if ndig < 0 {
                let l = ndig.abs();
                for _ in 0..l {
                    self.value = "0".to_owned() + &self.value;
                }
                0
            } else {
                ndig
            }
        } as u8;

        // skip cursor
        let ndig = if ndig > 0 {
            let at_cursor = self.value.chars().nth(ndig as usize);
            if let Some(at_cursor) = at_cursor {
                if at_cursor == '.' {
                    match direction {
                        HorizontalDirection::Left => ndig - 1,
                        HorizontalDirection::Right => ndig + 1,
                        HorizontalDirection::Col(_) => todo!(),
                    }
                } else {
                    ndig
                }
            } else {
                ndig
            }
        } else {
            ndig
        };

        // trim firsts when moving to right
        let ndig = if self.value.ends_with(".") && ndig > 0 && self.value.len() > 2 {
            let diff = (ndig as i8 - self.pot_cursor as i8).unsigned_abs() as usize;
            self.value = self.value[diff..].to_owned();
            0
        } else {
            ndig
        };

        // append
        if ndig as usize > self.value.len() - 1 {
            let diff = (ndig as usize).abs_diff(self.value.len() - 1);
            for _ in 0..diff {
                self.value += "0";
            }
        }

        if (ndig as usize) < self.value.len() - 1 {
            let diff = self.value.len() - 1 - ndig as usize;
            for _ in 0..diff {
                if !self.value.ends_with(".") {
                    self.value.pop();
                }
            }
        }

        self.pot_cursor = ndig as u8;
    }
}

impl Display for Tolerance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Tol ")?;
        let chars: Vec<char> = self.value.chars().collect();

        for (i, &c) in chars.iter().enumerate() {
            if i == self.pot_cursor as usize {
                if i == 0 {
                    f.write_fmt(format_args!("{}", c))?;
                } else if i == chars.len() - 1 {
                    f.write_fmt(format_args!(" {}", c))?;
                } else {
                    f.write_fmt(format_args!(" {} ", c))?;
                }
            } else {
                f.write_fmt(format_args!("{}", c))?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Default, Clone)]
pub enum MatSelectState {
    Focused,
    Diff,
    #[default]
    Hidden,
}

#[derive(Debug, Clone)]
pub struct HistoryEntry {
    pub mat: Matrix,
    pub client: String,
    pub state: MatSelectState,
}

#[derive(Debug, Default)]
pub struct History {
    pub history: Vec<(HistoryKey, Arc<Mutex<Vec<HistoryEntry>>>)>,
    pub cursor: Option<HistoryKey>,
}

impl History {
    pub fn diff_to_ref(&mut self) {
        if let Some(curr_buff_in) = self.current_buff() {
            let mut curr_buff = curr_buff_in.lock().unwrap();
            let curr_focus = curr_buff
                .iter()
                .position(|hentry| matches!(hentry.state, MatSelectState::Focused));
            let curr_diff = curr_buff
                .iter()
                .position(|hentry| matches!(hentry.state, MatSelectState::Diff));

            if let (Some(focus_idx), Some(diff_idx)) = (curr_focus, curr_diff) {
                curr_buff.get_mut(focus_idx).unwrap().state = MatSelectState::Diff;
                curr_buff.get_mut(diff_idx).unwrap().state = MatSelectState::Focused;
            }
        }
    }

    pub fn key_of_ref_by_key(&self, key: &HistoryKey) -> Option<MatHistIdx> {
        let (_, buff) = self.history.iter().find(|(k, _)| k == key)?;
        let buff = buff.lock().unwrap();
        let idx = buff
            .iter()
            .position(|hentry| matches!(hentry.state, MatSelectState::Focused))?;
        Some(MatHistIdx {
            history_key: key.clone(),
            buffer_idx: idx,
        })
    }

    pub fn key_of_diff_by_key(&self, key: &HistoryKey) -> Option<MatHistIdx> {
        let (_, buff) = self.history.iter().find(|(k, _)| k == key)?;
        let buff = buff.lock().unwrap();
        let idx = buff
            .iter()
            .position(|hentry| matches!(hentry.state, MatSelectState::Diff))?;
        Some(MatHistIdx {
            history_key: key.clone(),
            buffer_idx: idx,
        })
    }

    pub fn key_of_ref(&self) -> Option<MatHistIdx> {
        let cursor = self.cursor.as_ref()?;
        self.key_of_ref_by_key(cursor)
    }

    pub fn key_of_diff(&self) -> Option<MatHistIdx> {
        let cursor = self.cursor.as_ref()?;
        self.key_of_diff_by_key(cursor)
    }

    pub fn mat_with_key(&self, key: &MatHistIdx) -> Option<(Matrix, String)> {
        let entry = self
            .history
            .iter()
            .find(|(k, _)| *k == key.history_key)
            .map(|(_, v)| {
                let buf = v.lock().unwrap();
                buf.get(key.buffer_idx).cloned()
            })??;
        Some((entry.mat, entry.client))
    }

    pub fn add(&mut self, mat: Matrix, client: String, variable: String) {
        let key = if let Some(cur_var) = self.latest_var() {
            let changed = variable != cur_var;
            let call_id = if changed {
                let var_count = self
                    .history
                    .iter()
                    .filter(|(key, _)| *key.variable == variable)
                    .count();
                var_count + 1
            } else {
                self.cursor
                    .as_ref()
                    .expect("curr var without cursor")
                    .call_id
            };

            HistoryKey {
                variable: variable.clone(),
                call_id,
            }
        } else {
            let changed = self.history.is_empty();
            let call_id = if changed {
                1
            } else {
                unreachable!("no var but history wasn't empty");
            };

            HistoryKey {
                variable: variable.clone(),
                call_id,
            }
        };

        let mut is_new = false;
        let var_buffer = {
            if let Some(var_buffer) = self.buff_at(&key) {
                var_buffer
            } else {
                is_new = true;
                Arc::new(Mutex::new(Vec::new()))
            }
        };

        let change_buff = var_buffer.clone();

        if self.shows_ref_at(&key) {
            if self.shows_diff_at(&key) {
                let mut buffer = change_buff.lock().unwrap();
                // put to stack of clients
                buffer.push(HistoryEntry {
                    mat,
                    client,
                    state: MatSelectState::Hidden,
                });
            } else {
                let mut buffer = change_buff.lock().unwrap();
                buffer.push(HistoryEntry {
                    mat,
                    client,
                    state: MatSelectState::Diff,
                });
            }
        } else {
            let mut buffer = change_buff.lock().unwrap();
            buffer.push(HistoryEntry {
                mat,
                client,
                state: MatSelectState::Focused,
            });
        }

        if is_new {
            if self.history.is_empty() {
                self.cursor = Some(key.clone());
            }
            self.history.push((key, var_buffer));
        }
    }

    fn current_buff(&self) -> Option<Arc<Mutex<Vec<HistoryEntry>>>> {
        let curr_cursor = self.cursor.as_ref()?;
        self.buff_at(curr_cursor)
    }

    fn buff_at(&self, key: &HistoryKey) -> Option<Arc<Mutex<Vec<HistoryEntry>>>> {
        self.history
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
    }

    pub fn latest_var(&self) -> Option<String> {
        self.history.iter().last().map(|el| el.0.variable.clone())
    }

    fn shows_ref_at(&self, key: &HistoryKey) -> bool {
        if let Some(buff) = self.buff_at(key) {
            buff.lock()
                .unwrap()
                .iter()
                .any(|el| matches!(el.state, MatSelectState::Focused))
        } else {
            false
        }
    }

    pub fn shows_ref(&self) -> bool {
        if let Some(cursor) = self.cursor.as_ref() {
            self.shows_ref_at(cursor)
        } else {
            false
        }
    }

    fn shows_diff_at(&self, key: &HistoryKey) -> bool {
        if let Some(buff) = self.buff_at(key) {
            buff.lock()
                .unwrap()
                .iter()
                .any(|el| matches!(el.state, MatSelectState::Diff))
        } else {
            false
        }
    }

    pub fn shows_diff(&self) -> bool {
        if let Some(cursor) = self.cursor.as_ref() {
            self.shows_diff_at(cursor)
        } else {
            false
        }
    }

    pub fn diff_scroll(&mut self, direction: VerticalDirection) {
        if let Some(buff_ref) = self.current_buff().clone() {
            let mut current_buff = buff_ref.lock().unwrap();
            match direction {
                VerticalDirection::Up => {
                    if let Some(sel) = current_buff
                        .iter()
                        .position(|el| matches!(el.state, MatSelectState::Diff))
                    {
                        if sel + 1 < current_buff.len() {
                            current_buff[sel].state = MatSelectState::Hidden;
                            current_buff[sel + 1].state = MatSelectState::Diff;
                        }
                    }
                }
                VerticalDirection::Down => {
                    if let Some(sel) = current_buff
                        .iter()
                        .position(|el| matches!(el.state, MatSelectState::Diff))
                    {
                        if sel - 1 > 0 {
                            current_buff[sel].state = MatSelectState::Hidden;
                            current_buff[sel - 1].state = MatSelectState::Diff;
                        }
                    }
                }
                VerticalDirection::Row(idx) => {
                    if let Some(sel) = current_buff
                        .iter()
                        .position(|el| matches!(el.state, MatSelectState::Diff))
                    {
                        if idx < current_buff.len() {
                            current_buff[sel].state = MatSelectState::Hidden;
                            current_buff[idx].state = MatSelectState::Diff;
                        }
                    }
                }
            }
        }
    }

    pub fn render_var_history(&self) -> String {
        if let Some(curr_cursor) = self.cursor.as_ref() {
            if let Some(idx) = self.history.iter().position(|(k, _)| k == curr_cursor) {
                let (key, _) = &self.history[idx];
                let control_hints = {
                    let right_hint = if self.history.len() > idx + 1 {
                        "→".to_owned()
                    } else {
                        "".to_owned()
                    };

                    let left_hint = if idx == 0 {
                        "".to_owned()
                    } else {
                        "←".to_owned()
                    };

                    (left_hint, right_hint)
                };
                format!(
                    "{}{}[{}]{}",
                    control_hints.0, key.variable, key.call_id, control_hints.1
                )
            } else {
                "".to_owned()
            }
        } else {
            "".to_owned()
        }
    }

    pub fn scroll_history(&mut self, direction: HorizontalDirection) {
        if let Some(curr_cursor) = self.cursor.as_ref() {
            if let Some(idx) = self.history.iter().position(|(k, _)| k == curr_cursor) {
                let new_idx = match direction {
                    HorizontalDirection::Left => {
                        if idx == 0 {
                            idx
                        } else {
                            idx - 1
                        }
                    }
                    HorizontalDirection::Right => {
                        if idx == self.history.len() - 1 {
                            idx
                        } else {
                            idx + 1
                        }
                    }
                    HorizontalDirection::Col(idx) => idx,
                };

                self.cursor = Some(
                    self.history
                        .get(new_idx)
                        .expect("new history idx not checked correctly")
                        .clone()
                        .0,
                );

                // enable the first entries from the newly focused buffer if there are none in there
                let new_ref = self.key_of_ref();
                let new_diff = self.key_of_diff();

                match (new_ref, new_diff) {
                    (None, None) => {
                        let mut next_buff = self.history[new_idx].1.lock().unwrap();

                        if let Some(next_ref) = next_buff.first_mut() {
                            next_ref.state = MatSelectState::Focused;
                        }

                        if let Some(next_diff) = next_buff.get_mut(2) {
                            next_diff.state = MatSelectState::Diff;
                        }
                    }
                    (None, Some(_diff_idx)) => {
                        unreachable!("Diff exists but no ref");
                    }
                    (Some(_ref_idx), None) => {
                        let mut next_buff = self.history[new_idx].1.lock().unwrap();

                        // find next best non-focused hentry
                        if let Some(n) = next_buff
                            .iter_mut()
                            .find(|hentry| !matches!(hentry.state, MatSelectState::Focused))
                        {
                            n.state = MatSelectState::Diff;
                        }
                    }
                    (Some(_ref_idx), Some(_diff_idx)) => {}
                }
            }
        }
    }

    pub fn render_vs_overview(&self) -> String {
        if let Some(buff_ref) = self.current_buff().clone() {
            let current_buff = buff_ref.lock().unwrap();
            let current_focus = current_buff
                .iter()
                .find(|el| matches!(el.state, MatSelectState::Focused))
                .map(|sel| sel.client.clone())
                .unwrap_or("<none>".to_owned());
            let current_diff = current_buff
                .iter()
                .find(|el| matches!(el.state, MatSelectState::Diff))
                .map(|sel| sel.client.clone())
                .unwrap_or("<none>".to_owned());

            let control_hints = if current_diff != "<none>" {
                let current_diff_pos = current_buff
                    .iter()
                    .position(|h| matches!(h.state, MatSelectState::Diff))
                    .expect("just checked");

                let is_other_hidden_lower = current_buff
                    .iter()
                    .take(current_diff_pos)
                    .any(|hentry| matches!(hentry.state, MatSelectState::Hidden));

                let is_other_hidden_upper = current_buff
                    .iter()
                    .skip(current_diff_pos + 1)
                    .any(|hentry| matches!(hentry.state, MatSelectState::Hidden));

                let upper_hint = if is_other_hidden_upper {
                    "↑".to_owned()
                } else {
                    "".to_owned()
                };

                let lower_hint = if is_other_hidden_lower {
                    "↓".to_owned()
                } else {
                    "".to_owned()
                };

                (lower_hint, upper_hint)
            } else {
                ("".to_owned(), "".to_owned())
            };

            format!(
                "{}  {}{}{}",
                current_focus, control_hints.0, current_diff, control_hints.1
            )
        } else {
            "".to_owned()
        }
    }
}

impl Display for History {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(cursor) = self.cursor.as_ref() {
            f.write_str(&cursor.variable)
        } else {
            f.write_str("")
        }
    }
}

#[derive(Debug)]
pub struct Coordinator {
    state: String,
    locked: bool,
}

impl Default for Coordinator {
    fn default() -> Self {
        Self {
            state: "".to_owned(),
            locked: false,
        }
    }
}

impl Display for Coordinator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.locked {
            false => f.write_str(&self.state),
            true => f.write_fmt(format_args!("🔒{}", &self.state)),
        }
    }
}

#[derive(Debug, Default)]
pub enum WindSendState {
    Acked,
    Sent,
    #[default]
    NotRun,
}

#[derive(Debug, Default)]
pub enum WindCompile {
    Compiling,
    #[default]
    Done,
}

pub const WIND_POPUP_TITLE_NOERR: &str = "Wind Cursor to Line";
pub const POPUP_TITLE_ERR: &str = "Could not parse!";
pub const COMPARE_POPUP_TITLE_NOERR: &str = "Compare Cursor to row[:col]";

#[derive(Debug, Default)]
pub enum WindMode {
    Inactive, // shown but not changable
    #[default]
    Active,
    ActiveSelect,
}

#[derive(Debug)]
pub struct WindCursor {
    line_start: Option<u32>,
    line_end: Option<u32>,
    section_label: Option<String>,
    pub showing_popup: bool,
    mode: WindMode,
    pub popup_title: &'static str,
    pub popup_buffer: Vec<char>,
    pub(crate) wind_send_state: WindSendState,
    pub(crate) wind_work_queue: usize,
    pub(crate) compile_state: WindCompile,
    wind_file_path: PathBuf,
    bagfile: Arc<RwLock<bagread::Bagfile>>,
    pub(crate) variable_cache: HashMap<mtc::Var, mtc::Rhs>,
    dynamic_vars: HashMap<String, Vec<PlayKindUnitedPass3>>,
}

impl Default for WindCursor {
    fn default() -> Self {
        Self {
            line_start: None,
            line_end: None,
            wind_send_state: WindSendState::default(),
            showing_popup: false,
            popup_buffer: Vec::new(),
            popup_title: WIND_POPUP_TITLE_NOERR,
            mode: WindMode::default(),
            wind_file_path: PathBuf::from("./wind.rl"),
            bagfile: Arc::new(RwLock::new(bagread::Bagfile::default())),
            variable_cache: HashMap::new(),
            wind_work_queue: 0,
            compile_state: WindCompile::default(),
            dynamic_vars: HashMap::new(),
            section_label: None,
        }
    }
}

impl Display for WindCursor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(section_name) = self.section_label.as_ref() {
            f.write_str(&section_name).unwrap();
            f.write_str(" ").unwrap();
        }
        match self.mode {
            WindMode::Inactive => {
                f.write_str("W")?;
            }
            WindMode::Active => {
                f.write_str("W")?;
            }
            WindMode::ActiveSelect => {
                f.write_str("[W]")?;
            }
        }

        let lines = match (self.line_start, self.line_end) {
            (Some(line_start), Some(line_end)) => {
                if line_start == line_end {
                    format!("{}", line_start)
                } else {
                    format!("{}-{}", line_start, line_end)
                }
            }
            _ => "*".to_string(),
        };

        let queue_len = self.wind_work_queue;
        let queue_len_present = if queue_len == 0 {
            "".to_owned()
        } else {
            format!("+{}", queue_len)
        };

        let compile_state = match self.compile_state {
            WindCompile::Compiling => "💫",
            WindCompile::Done => "",
        }
        .to_owned();

        match self.wind_send_state {
            WindSendState::Acked => {
                f.write_fmt(format_args!("{compile_state}{lines}✅{queue_len_present}"))
            }
            WindSendState::Sent => {
                f.write_fmt(format_args!("{compile_state}{lines}🔄{queue_len_present}"))
            }
            WindSendState::NotRun => f.write_fmt(format_args!("{lines}")),
        }
    }
}

impl WindCursor {
    fn selected_lines(&self) -> Vec<u32> {
        match (self.line_start, self.line_end) {
            (Some(line_start), Some(line_end)) => (line_start..line_end + 1).collect(),
            _ => vec![],
        }
    }

    fn with_rats(mut self, wind_file_path: PathBuf) -> Self {
        self.wind_file_path = wind_file_path;
        self
    }
}

// -- Matrix to parse and diff ------------
use std::cmp::Ordering;

#[derive(Debug, Clone, Copy)]
pub struct TotalF64(f64);

impl Eq for TotalF64 {}

impl Display for TotalF64 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl From<f64> for TotalF64 {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

// impl ToString for TotalF64 {
//     fn to_string(&self) -> String {
//         format!("{}", self)
//     }
// }

impl From<TotalF64> for f64 {
    fn from(value: TotalF64) -> Self {
        value.0
    }
}

impl PartialEq for TotalF64 {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0 || (self.0.is_nan() && other.0.is_nan())
    }
}

impl PartialOrd for TotalF64 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for TotalF64 {
    fn cmp(&self, other: &Self) -> Ordering {
        // Define NaN comparison behavior here to handle it the way you prefer
        if self.0.is_nan() && other.0.is_nan() {
            Ordering::Equal
        } else if self.0.is_nan() {
            Ordering::Less // or Greater, depending on application needs
        } else if other.0.is_nan() {
            Ordering::Greater
        } else {
            self.0.partial_cmp(&other.0).unwrap()
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum CellDiff {
    Higher(f64),
    Lower(f64),
    WithinTolerance,
    OutOfBounds,
}

#[derive(Debug)]
pub struct DiffMatrix {
    data: Matrix,
    diff: Vec<CellDiff>,
    pub nrows: usize, // padded with ref
    pub ncols: usize,
    pub curr_offset_cols: usize,
    pub curr_offset_rows: usize,
    pub rect_width: usize,
    pub rect_height: usize,
    pub max_rows: usize,
    pub max_cols: usize,
}

impl From<DiffMatrix> for Matrix {
    fn from(value: DiffMatrix) -> Self {
        value.data
    }
}

const ROWS_COLS_CELL: &str = "↓ →";
const ROWS_COLS_CELL_LEN: u16 = 3;

impl DiffMatrix {
    pub fn new(
        reference: &Matrix,
        tolerance: Option<f64>,
        max_cols: usize,
        max_rows: usize,
        mat: Matrix,
    ) -> Self {
        let nrows = reference.nrows.max(mat.nrows);
        let ncols = reference.ncols.max(mat.ncols);
        let mut diff = vec![CellDiff::OutOfBounds; nrows * ncols];
        for col in 0..ncols {
            for row in 0..nrows {
                let idx = col * nrows + row;
                let ref_data = reference.get(row, col);
                let mat_data = mat.get(row, col);
                let cell_diff = match (ref_data, mat_data) {
                    (Some(l), Some(r)) => {
                        let l = l.0;
                        let r = r.0;

                        if let Some(tolerance) = tolerance {
                            if (l - r).abs() <= tolerance {
                                CellDiff::WithinTolerance
                            } else if l < r {
                                CellDiff::Higher(r - l - tolerance)
                            } else {
                                CellDiff::Lower(l - r - tolerance)
                            }
                        } else if l < r {
                            CellDiff::Higher(r - l)
                        } else {
                            CellDiff::Lower(l - r)
                        }
                    }
                    (_, _) => CellDiff::OutOfBounds,
                };
                diff[idx] = cell_diff;
            }
        }
        Self {
            data: mat,
            diff,
            nrows,
            ncols,
            curr_offset_cols: 0,
            curr_offset_rows: 0,
            max_rows: nrows.min(max_rows),
            max_cols: ncols.min(max_cols),
            rect_width: 0,
            rect_height: 0,
        }
    }

    pub fn update_max_view(&mut self, height: u16, width: u16) {
        let show_max_rows = height - 1 - 2; // minus header and padding
        let show_max_cols = width / 4; // TODO if a header can be printed, print only one and see how long it would look like as a string. divide by this to get cols
        self.max_rows = show_max_rows as usize;
        self.max_cols = show_max_cols as usize;
    }

    pub fn render(&self, precision: usize) -> anyhow::Result<Option<Table<'_>>> {
        let header_cols_nums: Vec<Cell> = (0..self.ncols)
            .skip(self.curr_offset_cols)
            .take(self.max_cols)
            .map(|n| Cell::from(n.to_string()))
            .collect();
        let mut header_cols = vec![Cell::from(ROWS_COLS_CELL)];
        header_cols.extend(header_cols_nums);

        let header_rows: Vec<Cell> = (0..self.nrows)
            .skip(self.curr_offset_rows)
            .take(self.max_rows)
            .map(|n| Cell::from(n.to_string()))
            .collect();

        let cols_max_len: u16 = self
            .ncols
            .to_string()
            .len()
            .try_into()
            .map_err(|e| anyhow!("Too many cols: {e}"))?;
        let rows_max_len: u16 = self
            .nrows
            .to_string()
            .len()
            .try_into()
            .map_err(|e| anyhow!("Too many rows: {e}"))?;

        let nrows = (self.nrows - self.curr_offset_rows).min(self.max_rows);
        let ncols = (self.ncols - self.curr_offset_cols).min(self.max_cols);

        let header_rows_cell_len = Constraint::Length(rows_max_len.max(ROWS_COLS_CELL_LEN));

        let mut max_data_len: u16 = 1;
        let mut rows: Vec<Row> = vec![Row::new(Vec::<Cell>::new()); nrows];
        for row in self.curr_offset_rows..(self.curr_offset_rows + nrows) {
            let mut nrow = vec![Cell::from("".to_owned()); ncols + 1];
            let row_header = unsafe { header_rows.get_unchecked(row - self.curr_offset_rows) };
            nrow[0] = row_header.clone();
            for col in self.curr_offset_cols..(self.curr_offset_cols + ncols) {
                let mat_cell = unsafe { self.diff.get_unchecked(col * self.nrows + row) };
                let str_cell = match mat_cell {
                    CellDiff::Higher(diff) => {
                        let d = format!("{:.1$}", diff, precision);
                        max_data_len = max_data_len.max(d.len() as u16);
                        Cell::from(d.blue())
                    }
                    CellDiff::Lower(diff) => {
                        let d = format!("{:.1$}", diff, precision);
                        max_data_len = max_data_len.max(d.len() as u16);
                        Cell::from(d.red())
                    }
                    CellDiff::WithinTolerance => Cell::from(""),
                    CellDiff::OutOfBounds => Cell::from("-".gray()),
                };
                unsafe { *nrow.get_unchecked_mut(col - self.curr_offset_cols + 1) = str_cell };
            }
            unsafe { *rows.get_unchecked_mut(row - self.curr_offset_rows) = Row::new(nrow) };
        }
        let data_cell_len = Constraint::Length(max_data_len.max(cols_max_len));
        let mut widths = vec![header_rows_cell_len];
        widths.extend(core::iter::repeat_n(data_cell_len, ncols));
        let table = Table::new(rows, widths).header(Row::new(header_cols).height(2));
        Ok(Some(table))
    }
}

#[derive(Debug, Clone)]
pub struct Matrix {
    data: Vec<TotalF64>,
    pub nrows: usize,
    pub ncols: usize,
    pub curr_offset_cols: usize,
    pub curr_offset_rows: usize,
    pub max_rows: usize,
    pub max_cols: usize,
}

impl Default for Matrix {
    fn default() -> Self {
        Self::new()
    }
}

impl Matrix {
    pub fn new() -> Self {
        Self {
            data: vec![],
            nrows: 0,
            ncols: 0,
            curr_offset_cols: 0,
            curr_offset_rows: 0,
            max_rows: 0,
            max_cols: 0,
        }
    }

    pub fn get(&self, row: usize, col: usize) -> Option<TotalF64> {
        if row < self.nrows && col < self.ncols {
            let index = col * self.nrows + row;
            self.data.get(index).cloned()
        } else {
            None
        }
    }

    pub fn update_max_view(&mut self, height: u16, width: u16) {
        let show_max_rows = height - 1 - 2; // minus header and padding
        let show_max_cols = width / 4; // TODO if a header can be printed, print only one and see how long it would look like as a string. divide by this to get cols
        self.max_rows = show_max_rows as usize;
        self.max_cols = show_max_cols as usize;
    }

    pub fn render(&self, precision: usize) -> anyhow::Result<Option<Table<'_>>> {
        let header_cols_nums: Vec<Cell> = (0..self.ncols)
            .skip(self.curr_offset_cols)
            .take(self.max_cols)
            .map(|n| Cell::from(n.to_string()))
            .collect();
        let mut header_cols = vec![Cell::from(ROWS_COLS_CELL)];
        header_cols.extend(header_cols_nums);

        let header_rows: Vec<Cell> = (0..self.nrows)
            .skip(self.curr_offset_rows)
            .take(self.max_rows)
            .map(|n| Cell::from(n.to_string()))
            .collect();

        let cols_max_len: u16 = self
            .ncols
            .to_string()
            .len()
            .try_into()
            .map_err(|e| anyhow!("Too many cols: {e}"))?;

        let rows_max_len: u16 = self
            .nrows
            .to_string()
            .len()
            .try_into()
            .map_err(|e| anyhow!("Too many rows: {e}"))?;

        let header_rows_cell_len = Constraint::Length(rows_max_len.max(ROWS_COLS_CELL_LEN));

        let nrows = (self.nrows - self.curr_offset_rows).min(self.max_rows);
        let ncols = (self.ncols - self.curr_offset_cols).min(self.max_cols);

        let mut max_data_len: u16 = 1;
        let mut rows: Vec<Row> = vec![Row::new(Vec::<Cell>::new()); nrows];
        for row in self.curr_offset_rows..(self.curr_offset_rows + nrows) {
            let mut nrow = vec![Cell::from("".to_owned()); ncols + 1];
            let row_header = unsafe { header_rows.get_unchecked(row - self.curr_offset_rows) };
            nrow[0] = row_header.clone();
            for col in self.curr_offset_cols..(self.curr_offset_cols + ncols) {
                unsafe {
                    let d = format!(
                        "{:.1$}",
                        self.data.get_unchecked(col * self.nrows + row).0,
                        precision
                    );
                    max_data_len = max_data_len.max(d.len() as u16);
                    *nrow.get_unchecked_mut(col - self.curr_offset_cols + 1) = Cell::from(d)
                };
            }
            unsafe { *rows.get_unchecked_mut(row - self.curr_offset_rows) = Row::new(nrow) };
        }

        let data_cell_len = Constraint::Length(max_data_len.max(cols_max_len));
        let mut widths = vec![header_rows_cell_len];
        widths.extend(core::iter::repeat_n(data_cell_len, ncols));
        let table = Table::new(rows, widths).header(Row::new(header_cols).height(2));
        Ok(Some(table))
    }
}

impl FromStr for Matrix {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let input = s.trim();

        if !input.starts_with("VecStorage") || !input.ends_with("}") {
            return Err(anyhow!("Not a VecStorage debug output."));
        }

        let data_start = "VecStorage { data: [".len();
        let data_end = input
            .rfind("]")
            .ok_or_else(|| anyhow!("Could not find end of data."))?;
        let data_str = &input[data_start..data_end];

        let rest_start = data_end + "], nrows: Dyn(".len();
        let rest_str = &input[rest_start..input.len() - 3]; // Exclude ending

        let mut dims_iter = rest_str.split("), ncols: Dyn(");
        let nrows_str = dims_iter.next().ok_or_else(|| anyhow!("Missing nrows"))?;
        let ncols_str = dims_iter.next().ok_or_else(|| anyhow!("Missing ncols"))?;

        let data = data_str
            .split(',')
            .map(str::trim)
            .map(f64::from_str)
            .map(|result| result.map(TotalF64::from))
            .collect::<Result<Vec<TotalF64>, _>>()?;

        let nrows = usize::from_str(nrows_str)?;
        let ncols = usize::from_str(ncols_str)?;

        Ok(Matrix {
            data,
            nrows,
            ncols,
            max_rows: nrows,
            max_cols: ncols,
            curr_offset_cols: 0,
            curr_offset_rows: 0,
        })
    }
}

// -- Client Compare inner windows --------

const ITEM_HEIGHT: usize = 1;

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct HistoryKey {
    variable: String,
    call_id: usize,
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct MatHistIdx {
    pub history_key: HistoryKey,
    pub buffer_idx: usize,
}

#[derive(Debug)]
pub struct ClientCompare {
    pub vertical_scroll_state: ScrollbarState,
    pub horizontal_scroll_state: ScrollbarState,
    pub left_state: TableState,
    pub right_state: TableState,
    pub left: Option<(MatHistIdx, Matrix)>,
    pub right: Option<(MatHistIdx, DiffMatrix)>,
    pub show_line_input_popup: bool,
    pub popup_buffer: Vec<char>,
    pub popup_title: &'static str,
    pub ref_width: u16,
    pub ref_height: u16,
    pub diff_width: u16,
    pub diff_height: u16,
}

impl ClientCompare {
    pub fn update_rects(
        &mut self,
        ref_width: u16,
        ref_height: u16,
        diff_width: u16,
        diff_height: u16,
    ) {
        self.ref_width = ref_width;
        self.ref_height = ref_height;
        self.diff_width = diff_width;
        self.diff_height = diff_height;
    }

    pub fn update_states(&mut self) {
        let max_nrows = self
            .left
            .as_ref()
            .map(|(_, m)| m.nrows)
            .unwrap_or_default()
            .max(
                self.right
                    .as_ref()
                    .map(|(_, m)| m.nrows)
                    .unwrap_or_default(),
            );
        let max_ncols = self
            .left
            .as_ref()
            .map(|(_, m)| m.ncols)
            .unwrap_or_default()
            .max(
                self.right
                    .as_ref()
                    .map(|(_, m)| m.ncols)
                    .unwrap_or_default(),
            );

        self.vertical_scroll_state = ScrollbarState::new((max_nrows + 1) * ITEM_HEIGHT);
        self.horizontal_scroll_state = ScrollbarState::new(max_ncols * ITEM_HEIGHT)
    }

    pub fn update_ref(&mut self, mat: Option<(Matrix, MatHistIdx)>, cols_layouting: bool) {
        if let Some((mut mat, idx)) = mat {
            if cols_layouting {
                mat.update_max_view(self.ref_height, self.ref_width);
            }
            self.left.replace((idx, mat));
        } else {
            self.left.take();
        }
    }

    pub fn update_diff(
        &mut self,
        mat: Option<(Matrix, MatHistIdx)>,
        tolerance: Option<f64>,
        cols_layouting: bool,
    ) {
        if let Some((mut mat, idx)) = mat {
            if cols_layouting {
                mat.update_max_view(self.diff_height, self.diff_width);
            }
            let (_, mat_ref) = self
                .left
                .as_ref()
                .expect("Wants to update diff but no mat ref present!");
            let right = DiffMatrix::new(mat_ref, tolerance, mat.max_cols, mat.max_rows, mat);
            self.right.replace((idx, right));
        } else {
            self.right.take();
        }
    }

    pub fn next_row(&mut self) {
        match (self.left.as_mut(), self.right.as_mut()) {
            (Some((_, l)), Some((_, r))) => {
                let i = match self.right_state.selected() {
                    Some(i) => {
                        if i + r.curr_offset_rows == r.nrows - 1 {
                            r.curr_offset_rows = 0;
                            0
                        } else if i == r.max_rows - 1 {
                            r.curr_offset_rows += 1;
                            i
                        } else {
                            i + 1
                        }
                    }
                    None => 0,
                };
                self.right_state.select(Some(i));

                let l_start = r.curr_offset_rows.min(l.nrows - 1);
                l.curr_offset_rows = l_start;
                let i_l = i.min((l.max_rows - 1).min(l.nrows - 1));
                self.left_state.select(Some(i_l));

                self.vertical_scroll_state = self
                    .vertical_scroll_state
                    .position((i + r.curr_offset_rows) * ITEM_HEIGHT);
            }
            (Some((_, l)), None) => {
                let i = match self.left_state.selected() {
                    Some(i) => {
                        if i + l.curr_offset_rows == l.nrows - 1 {
                            l.curr_offset_rows = 0;
                            0
                        } else if i == l.max_rows - 1 {
                            l.curr_offset_rows += 1;
                            i
                        } else {
                            i + 1
                        }
                    }
                    None => 0,
                };
                self.left_state.select(Some(i));
                self.vertical_scroll_state = self
                    .vertical_scroll_state
                    .position((i + l.curr_offset_rows) * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn previous_row(&mut self) {
        match (self.left.as_mut(), self.right.as_mut()) {
            (Some((_, l)), Some((_, r))) => {
                let i = match self.right_state.selected() {
                    Some(i) => {
                        // wrap around to end
                        if i == 0 && r.curr_offset_rows == 0 {
                            if r.nrows >= r.max_rows {
                                r.curr_offset_rows =
                                    (r.nrows as i32 - r.max_rows as i32).max(0) as usize;
                            }
                            (r.max_rows.min(r.nrows) as i32 - 1).max(0) as usize
                        } else if i == 0 {
                            r.curr_offset_rows -= 1;
                            i
                        } else {
                            i - 1
                        }
                    }
                    None => 0,
                };
                self.right_state.select(Some(i));

                let l_start = r.curr_offset_rows.min(l.nrows - 1);
                l.curr_offset_rows = l_start;
                let i_l = i.min((l.max_rows - 1).min(l.nrows - 1));
                self.left_state.select(Some(i_l));

                self.vertical_scroll_state = self
                    .vertical_scroll_state
                    .position((i + r.curr_offset_rows) * ITEM_HEIGHT);
            }
            (Some((_, l)), None) => {
                let i = match self.left_state.selected() {
                    Some(i) => {
                        if i == 0 && l.curr_offset_rows == 0 {
                            l.curr_offset_rows =
                                (l.nrows as i32 - l.max_rows as i32).max(0) as usize;
                            (l.max_rows.min(l.nrows) as i32 - 1).max(0) as usize
                        } else if i == 0 {
                            l.curr_offset_rows -= 1;
                            i
                        } else {
                            i - 1
                        }
                    }
                    None => 0,
                };
                self.left_state.select(Some(i));
                self.vertical_scroll_state = self
                    .vertical_scroll_state
                    .position((i + l.curr_offset_rows) * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn col(&mut self, i: usize) {
        match (self.left.as_mut(), self.right.as_mut()) {
            (Some((_, l)), None) => {
                if i >= l.ncols {
                    // avoid out of bounds jump
                    return;
                }
                l.curr_offset_cols = i;
                self.left_state.select_column(Some(1));
                self.horizontal_scroll_state =
                    self.horizontal_scroll_state.position(i * ITEM_HEIGHT);
            }
            (Some((_, l)), Some((_, r))) => {
                if i >= l.ncols {
                    // avoid out of bounds jump
                    return;
                }

                r.curr_offset_cols = i;
                self.right_state.select_column(Some(1));
                l.curr_offset_cols = i.min(l.ncols - 1);
                self.left_state.select_column(Some(1));

                self.horizontal_scroll_state =
                    self.horizontal_scroll_state.position(i * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn row(&mut self, i: usize) {
        match (self.left.as_mut(), self.right.as_mut()) {
            (Some((_, l)), None) => {
                if i >= l.nrows {
                    // avoid out of bounds jump
                    return;
                }
                l.curr_offset_rows = i;
                self.left_state.select(Some(1));
                self.vertical_scroll_state = self.vertical_scroll_state.position(i * ITEM_HEIGHT);
            }
            (Some((_, l)), Some((_, r))) => {
                if i >= l.nrows {
                    // avoid out of bounds jump
                    return;
                }

                r.curr_offset_rows = i;
                self.right_state.select(Some(1));
                l.curr_offset_rows = i.min(l.nrows - 1);
                self.left_state.select(Some(1));

                self.vertical_scroll_state = self.vertical_scroll_state.position(i * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn next_col(&mut self) {
        match (self.left.as_mut(), self.right.as_mut()) {
            (Some((_, l)), Some((_, r))) => {
                let i = match self.right_state.selected_column() {
                    Some(i) => {
                        if i + r.curr_offset_cols == r.ncols {
                            r.curr_offset_cols = 0;
                            1
                        } else if i == r.max_cols {
                            r.curr_offset_cols += 1;
                            i
                        } else {
                            i + 1
                        }
                    }
                    None => 1,
                };
                self.right_state.select_column(Some(i));

                let l_start = r.curr_offset_cols.min(l.ncols - 1);
                l.curr_offset_cols = l_start;
                let i_l = i.min((l.max_cols).min(l.ncols));
                self.left_state.select_column(Some(i_l));

                self.horizontal_scroll_state = self
                    .horizontal_scroll_state
                    .position((i + r.curr_offset_cols - 1) * ITEM_HEIGHT);
            }
            (Some((_, l)), None) => {
                let i = match self.left_state.selected_column() {
                    Some(i) => {
                        if i + l.curr_offset_cols == l.ncols {
                            l.curr_offset_cols = 0;
                            1
                        } else if i == l.max_cols {
                            l.curr_offset_cols += 1;
                            i
                        } else {
                            i + 1
                        }
                    }
                    None => 1,
                };
                self.left_state.select_column(Some(i));
                self.horizontal_scroll_state = self
                    .horizontal_scroll_state
                    .position((i + l.curr_offset_cols - 1) * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn previous_col(&mut self) {
        match (self.left.as_mut(), self.right.as_mut()) {
            (Some((_, l)), Some((_, r))) => {
                let i = match self.right_state.selected_column() {
                    Some(i) => {
                        // wrap around to end of cols
                        if i == 1 && r.curr_offset_cols == 0 {
                            if r.ncols >= r.max_cols {
                                r.curr_offset_cols = r.ncols - r.max_cols;
                            }
                            r.max_cols.min(r.ncols)
                        } else if i == 1 {
                            r.curr_offset_cols -= 1;
                            i
                        } else {
                            i - 1
                        }
                    }
                    None => 1,
                };
                self.right_state.select_column(Some(i));

                let l_start = r.curr_offset_cols.min(l.ncols - 1);
                l.curr_offset_cols = l_start;
                let i_l = i.min(l.max_cols.min(l.ncols));
                self.left_state.select_column(Some(i_l));

                self.horizontal_scroll_state = self
                    .horizontal_scroll_state
                    .position((i + r.curr_offset_cols - 1) * ITEM_HEIGHT);
            }
            (Some((_, l)), None) => {
                let i = match self.left_state.selected_column() {
                    Some(i) => {
                        if i == 1 && l.curr_offset_cols == 0 {
                            if l.ncols >= l.max_cols {
                                l.curr_offset_cols = l.ncols - l.max_cols;
                            }
                            l.max_cols.min(l.ncols)
                        } else if i == 1 {
                            l.curr_offset_cols -= 1;
                            i
                        } else {
                            i - 1
                        }
                    }
                    None => 1,
                };
                self.left_state.select_column(Some(i));
                self.horizontal_scroll_state = self
                    .horizontal_scroll_state
                    .position((i + l.curr_offset_cols - 1) * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }
}

impl Default for ClientCompare {
    fn default() -> Self {
        Self {
            left_state: TableState::default()
                .with_selected(0)
                .with_selected_column(1),
            right_state: TableState::default()
                .with_selected(0)
                .with_selected_column(1),
            horizontal_scroll_state: ScrollbarState::new(ITEM_HEIGHT),
            vertical_scroll_state: ScrollbarState::new(ITEM_HEIGHT),
            left: None,
            right: None,
            show_line_input_popup: false,
            popup_buffer: Vec::new(),
            popup_title: COMPARE_POPUP_TITLE_NOERR,
            ref_width: 0,
            ref_height: 0,
            diff_width: 0,
            diff_height: 0,
        }
    }
}

#[derive(Debug)]
pub struct InfoView {
    pub shown: bool,
    pub log: Vec<String>,
}

impl Default for InfoView {
    fn default() -> Self {
        Self {
            shown: true,
            log: Vec::new(),
        }
    }
}

pub(crate) struct ErrorWriter {}
impl std::io::Write for ErrorWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let s = String::from_utf8_lossy(buf);
        error!("{s}");
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl InfoView {
    pub fn toggle(&mut self) {
        self.shown = !self.shown;
    }

    pub fn render(&self) -> Paragraph<'_> {
        let text = self.log.join("\n");
        Paragraph::new(text).wrap(ratatui::widgets::Wrap { trim: true })
    }
}

#[derive(Debug)]
pub struct App {
    pub running: bool,
    pub var_locked: Option<LockedState>,
    pub info_view: Arc<RwLock<InfoView>>,
    pub coordinator: Coordinator,
    pub wind_worker_tx: tokio::sync::mpsc::UnboundedSender<Evaluated>,
    pub history: Arc<RwLock<History>>, // 0 var1; -1 var2 etc.
    pub wind_cursor: Arc<RwLock<WindCursor>>,
    pub mode: Mode,
    pub tolerance: Tolerance,
    pub compare: ClientCompare,
    pub send_coordinator: tokio::sync::mpsc::Sender<PacketKind>,
    pub rats_file: Option<PathBuf>,
}

pub enum HorizontalDirection {
    Left,
    Right,
    Col(usize),
}

pub enum VerticalDirection {
    Up,
    Down,
    Row(usize),
}

impl App {
    pub async fn new(
        sender: tokio::sync::mpsc::Sender<PacketKind>,
        mut receiver: tokio::sync::mpsc::Receiver<(String, String, String)>,
        mut dyn_wind_receiver: tokio::sync::mpsc::Receiver<String>,
        rats: Option<PathBuf>,
    ) -> Self {
        let history = Arc::new(RwLock::new(History::default()));
        let history_add = Arc::clone(&history);
        tokio::spawn(async move {
            while let Some((client_name, mat, var_name)) = receiver.recv().await {
                let mut hist = history_add.write().unwrap();
                let mat = Matrix::from_str(&mat).unwrap();
                hist.add(mat, client_name, var_name);
            }
        });
        let mut wind_cursor = WindCursor::default();

        if let Some(rl) = &rats {
            wind_cursor = wind_cursor.with_rats(rl.clone());
        }
        let wind_cursor = Arc::new(RwLock::new(wind_cursor));

        let wind_update = Arc::clone(&wind_cursor);
        let wind_answer_sender = sender.clone();
        let (wind_worker_tx, mut wind_worker_rx) =
            tokio::sync::mpsc::unbounded_channel::<Evaluated>();
        let wind_worker_dynvars = wind_worker_tx.clone();
        tokio::spawn(async move {
            let wind_worker_sender = wind_worker_dynvars.clone();
            while let Some(var) = dyn_wind_receiver.recv().await {
                let wd = {
                    let wc = wind_update.write().unwrap();
                    let v = wc.dynamic_vars.get(&var);
                    match v {
                        Some(wd) => wd.clone(), // probably very costly but the old data needs to stay there
                        None => {
                            error!(
                                "Received Dyn Wind request for {} but no data available.",
                                &var
                            );
                            continue;
                        }
                    }
                };

                App::wind_fire_at_current_cursor(
                    wind_answer_sender.clone(),
                    Arc::clone(&wind_update),
                    wind_worker_sender.clone(),
                    Some(wd),
                )
                .await;
            }
        });

        // let wind_cursor_states_worker = Arc::clone(&wind_cursor);
        let wind_cursor_worker = Arc::clone(&wind_cursor);
        let sender_worker = sender.clone();
        tokio::spawn(async move {
            let wind_cursor_outer = Arc::clone(&wind_cursor_worker);
            let sender_worker_inner = sender_worker.clone();
            while let Some(eval) = wind_worker_rx.recv().await {
                let send_coordinator = sender_worker_inner.clone();
                let wind_cursor_worker = Arc::clone(&wind_cursor_outer);
                {
                    // decrease the indicator counter in the TUI
                    let mut wc = wind_cursor_worker.write().unwrap();
                    wc.wind_work_queue = wc.wind_work_queue.saturating_sub(1);
                };
                let fun = async move {
                    for windfn in eval.wind.iter() {
                        match windfn {
                            mtc::WindFunction::Reset(path) => {
                                let bag_blocking = {
                                    let wc = wind_cursor_worker.read().unwrap();
                                    Arc::clone(&wc.bagfile)
                                };

                                let reset_path = PathBuf::from(path);
                                let abs_path = if reset_path.is_relative() {
                                    let rats_file =
                                        wind_cursor_worker.read().unwrap().wind_file_path.clone();
                                    rats_file
                                        .parent()
                                        .unwrap()
                                        .join(reset_path)
                                        .canonicalize()
                                        .unwrap()
                                } else {
                                    reset_path.canonicalize().unwrap()
                                };

                                match bag_blocking.write().unwrap().reset(Some(abs_path)) {
                                    Err(e) => {
                                        error!("Could not reset bagfile path {path:?}: {e}");

                                        return;
                                    }
                                    Ok(_) => {
                                        info!("read bag {path}");
                                    }
                                }
                            }
                            mtc::WindFunction::SendFrames(kind) => {
                                match kind {
                                    PlayKindUnitedPass3::SensorCount {
                                        sensors: _,
                                        count: _,
                                        trigger: Some(PlayTrigger::Variable(var)),
                                        play_mode: PlayMode::Dynamic,
                                    }
                                    | PlayKindUnitedPass3::UntilSensorCount {
                                        sending: _,
                                        until_sensors: _,
                                        until_count: _,
                                        trigger: Some(PlayTrigger::Variable(var)),
                                        play_mode: PlayMode::Dynamic,
                                    } => {
                                        {
                                            let mut wc = wind_cursor_worker.write().unwrap();
                                            let kind = match kind {
                                                PlayKindUnitedPass3::SensorCount {
                                                    sensors,
                                                    count,
                                                    trigger: _,
                                                    play_mode: _,
                                                } => PlayKindUnitedPass3::SensorCount {
                                                    sensors: sensors.clone(),
                                                    count: *count,
                                                    trigger: None,
                                                    play_mode: PlayMode::Fix,
                                                },
                                                PlayKindUnitedPass3::UntilSensorCount {
                                                    sending,
                                                    until_sensors,
                                                    until_count,
                                                    trigger: _,
                                                    play_mode: _,
                                                } => PlayKindUnitedPass3::UntilSensorCount {
                                                    sending: sending.clone(),
                                                    until_sensors: until_sensors.clone(),
                                                    until_count: *until_count,
                                                    trigger: None,
                                                    play_mode: PlayMode::Fix,
                                                },
                                            };
                                            if let Some(at_var) = wc.dynamic_vars.get_mut(var) {
                                                at_var.push(kind);
                                            } else {
                                                wc.dynamic_vars.insert(var.clone(), vec![kind]);
                                            }
                                        }

                                        let send = send_coordinator
                                            .send(PacketKind::WindDynamic(var.clone()))
                                            .await;
                                        match send {
                                            Ok(_) => {}
                                            Err(e) => {
                                                error!("Could not send Wind to coordinator: {e}");
                                                return;
                                            }
                                        }
                                        continue; // early exit from fn so dynamic vars do not iterate in bagfile
                                    }
                                    _ => {}
                                };

                                let bag_blocking = {
                                    let wc = wind_cursor_worker.read().unwrap();
                                    Arc::clone(&wc.bagfile)
                                };
                                let start_bag_read = Instant::now();
                                info!("iterating on bagfile...");
                                let (res_tx, res_rx) = tokio::sync::oneshot::channel();
                                let ka = kind.clone();
                                std::thread::spawn(move || {
                                    let r = bag_blocking.write().unwrap().next(&ka);
                                    res_tx.send(r).unwrap();
                                });

                                let bagmsgs = match res_rx.await {
                                    Ok(Ok(msgs)) => msgs,
                                    Err(e) => {
                                        error!("{e}");
                                        return;
                                    }
                                    Ok(Err(e)) => {
                                        error!("{e}");
                                        return;
                                    }
                                };
                                info!(
                                    "got {} msg{} in {:?}",
                                    bagmsgs.len(),
                                    if bagmsgs.len() > 1 { "s" } else { "" },
                                    start_bag_read.elapsed()
                                );

                                let mut wind_data = Vec::with_capacity(bagmsgs.len());
                                let mut start_time = None;
                                let mut end_time = None;
                                for (t, msg) in bagmsgs {
                                    if start_time.is_none() {
                                        start_time.replace(t);
                                    }

                                    end_time.replace(t);

                                    let diff = end_time.unwrap() - start_time.unwrap();
                                    wind_data.push((diff, msg));
                                }

                                if wind_data.is_empty() {
                                    warn!("query returned empty data");
                                    return;
                                }

                                let trigger = match &kind {
                                    mtc::PlayKindUnitedPass3::SensorCount {
                                        sensors: _,
                                        count: _,
                                        trigger,
                                        play_mode,
                                    }
                                    | mtc::PlayKindUnitedPass3::UntilSensorCount {
                                        sending: _,
                                        until_sensors: _,
                                        until_count: _,
                                        trigger,
                                        play_mode,
                                    } => trigger.as_ref().map(|trigger| match trigger {
                                        mtc::PlayTrigger::DurationRelFactor(factor) => {
                                            let original_duration_ns =
                                                end_time.unwrap() - start_time.unwrap();

                                            let total_nanos_decimal =
                                                Decimal::from(original_duration_ns);
                                            let scale_factor_decimal = Decimal::from_f64(*factor)
                                                .expect(
                                                    "Could not convert f64 scale factor to Decimal",
                                                );
                                            let scaled_nanos_decimal =
                                                total_nanos_decimal * scale_factor_decimal;

                                            // ignoring fractional nanoseconds here ;)
                                            let scaled_ns = scaled_nanos_decimal
                                                .round()
                                                .to_string()
                                                .parse::<u64>()
                                                .unwrap_or(u64::MAX);
                                            let scaled_duration = Duration::from_nanos(scaled_ns);
                                            let dur_ms = mtc::PlayTrigger::DurationMs(
                                                scaled_duration.as_millis() as u64,
                                            );
                                            (dur_ms, play_mode)
                                        }
                                        _ => (trigger.clone(), play_mode),
                                    }),
                                };

                                match &trigger {
                                    Some((
                                        mtc::PlayTrigger::DurationMs(target_duration_ms),
                                        PlayMode::Fix,
                                    )) => {
                                        let total_nanos_decimal =
                                            Decimal::from(end_time.unwrap() - start_time.unwrap());
                                        let target_dur_nanos_decimal = Decimal::from(
                                            std::time::Duration::from_millis(*target_duration_ms)
                                                .as_nanos(),
                                        );

                                        let scale_factor_decimal =
                                            if total_nanos_decimal == Decimal::from(0) {
                                                Decimal::from(1)
                                            } else {
                                                target_dur_nanos_decimal / total_nanos_decimal
                                            };

                                        let start_time = Instant::now();

                                        for (original_ts, wind) in wind_data {
                                            let original_ts_nanos_decimal =
                                                Decimal::from(original_ts);
                                            let scaled_ts =
                                                original_ts_nanos_decimal * scale_factor_decimal;

                                            // ignoring fractional nanoseconds here ;)
                                            let scaled_ns = scaled_ts
                                                .round()
                                                .to_string()
                                                .parse::<u64>()
                                                .unwrap_or(u64::MAX);
                                            let target_offset = Duration::from_nanos(scaled_ns);
                                            let target_wall_time = start_time + target_offset;
                                            let sleep_duration = target_wall_time
                                                .saturating_duration_since(Instant::now());
                                            tokio::time::sleep(sleep_duration).await;

                                            let send = send_coordinator
                                                .send(PacketKind::Wind(vec![WindAt {
                                                    data: wind,
                                                    at_var: None,
                                                }]))
                                                .await;
                                            match send {
                                                Ok(_) => {}
                                                Err(e) => {
                                                    error!(
                                                        "Could not send Wind to coordinator: {e}"
                                                    );
                                                    return;
                                                }
                                            }
                                        }
                                    }
                                    Some((mtc::PlayTrigger::Variable(var), PlayMode::Fix)) => {
                                        let wd = wind_data
                                            .into_iter()
                                            .map(|(_, wind)| WindAt {
                                                data: wind,
                                                at_var: Some(var.clone()),
                                            })
                                            .collect::<Vec<_>>();

                                        let send =
                                            send_coordinator.send(PacketKind::Wind(wd)).await;
                                        match send {
                                            Ok(_) => {}
                                            Err(e) => {
                                                error!("Could not send Wind to coordinator: {e}");
                                                return;
                                            }
                                        }
                                    }

                                    Some((mtc::PlayTrigger::Variable(_), PlayMode::Dynamic)) => {
                                        panic!("Dynamic variables should be resolved here");
                                    }
                                    Some((PlayTrigger::DurationRelFactor(_), _)) => {
                                        panic!("Must be converted to relative time before");
                                    }
                                    Some((PlayTrigger::DurationMs(_), PlayMode::Dynamic)) => {
                                        error!(
                                            "Combinating a fixed millisecond span dynamically is not supported."
                                        );
                                    }
                                    None => {
                                        // publish immediately
                                        let wd = wind_data
                                            .into_iter()
                                            .map(|(_, wind)| WindAt {
                                                data: wind,
                                                at_var: None,
                                            })
                                            .collect::<Vec<_>>();

                                        let send =
                                            send_coordinator.send(PacketKind::Wind(wd)).await;
                                        match send {
                                            Ok(_) => {}
                                            Err(e) => {
                                                error!("Could not send Wind to coordinator: {e}");
                                                return;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                };

                // run function and always close by setting the acked state to wind
                fun.await;
                {
                    let mut wind_cursor = wind_cursor_outer.write().unwrap();
                    wind_cursor.wind_send_state = WindSendState::Acked;
                }

                let wind_cursor_state_reset = Arc::clone(&wind_cursor_outer);
                tokio::spawn(async move {
                    // reset state after
                    tokio::time::sleep(Duration::from_secs(2)).await;
                    let mut wind_cursor = wind_cursor_state_reset.write().unwrap();
                    match wind_cursor.wind_send_state {
                        WindSendState::Acked => {
                            wind_cursor.wind_send_state = WindSendState::NotRun;
                        }
                        WindSendState::Sent => {}
                        WindSendState::NotRun => {}
                    };
                });
            }
        });

        tokio::task::yield_now().await;

        Self {
            running: true,
            coordinator: Coordinator::default(),
            history,
            wind_cursor,
            wind_worker_tx,
            mode: Mode::default(),
            tolerance: Tolerance::default(),
            compare: ClientCompare::default(),
            send_coordinator: sender,
            rats_file: rats,
            info_view: Arc::new(RwLock::new(InfoView::default())),
            var_locked: None,
        }
    }

    pub fn tick(&self) {}

    pub fn quit(&mut self) {
        self.running = false;
    }

    pub fn render_coordinator(&self) -> String {
        format!("{}", self.coordinator)
    }

    pub fn render_vs_overview(&self) -> String {
        self.history.read().unwrap().render_vs_overview()
    }

    pub fn render_history(&self) -> String {
        self.history
            .read()
            .unwrap()
            .render_var_history()
            .to_string()
    }

    pub fn render_tolerance(&self) -> String {
        format!("{}", self.tolerance)
    }

    pub fn render_mode(&self) -> String {
        format!("{}", self.mode)
    }

    pub fn render_wind_cursor(&self) -> String {
        format!("{}", self.wind_cursor.read().unwrap())
    }

    pub fn toggle_info_window(&mut self) {
        self.info_view.write().unwrap().toggle();
    }

    pub fn set_var_locked(&mut self) {
        self.var_locked = Some(LockedState::Locked);
    }

    pub fn render_var_locked(&mut self) -> String {
        if let Some(vl) = self.var_locked.as_ref() {
            format!("{}", vl)
        } else {
            "".to_owned()
        }
    }

    pub fn set_var_unlocked(&mut self) {
        self.var_locked = Some(LockedState::Unlocked);
    }

    pub fn focus_right_rat(&mut self) {
        self.history.write().unwrap().diff_to_ref()
    }

    pub fn change_diff_rat(&mut self, direction: VerticalDirection) {
        self.history.write().unwrap().diff_scroll(direction)
    }

    pub async fn send_lock_next(&mut self, unlock_first: bool) {
        self.send_coordinator
            .send(PacketKind::LockNext { unlock_first })
            .await
            .unwrap();
        info!("sent lock next");
    }

    pub async fn send_unlock(&mut self) {
        self.send_coordinator
            .send(PacketKind::Unlock)
            .await
            .unwrap();
        info!("sent unlock");
    }

    pub fn change_tolerance_at_current_cursor(&mut self, direction: VerticalDirection) {
        self.tolerance.change_tolerance_at_current_cursor(direction);
        if let Some((matidx, old_mat)) = self.compare.right.as_ref() {
            let curr_history = self.history.read().unwrap();
            if let Some((mut mat, _)) = curr_history.mat_with_key(matidx) {
                mat.max_rows = old_mat.max_rows;
                mat.max_cols = old_mat.max_cols;
                self.compare.update_diff(
                    Some((mat, matidx.clone())),
                    Some(f64::from(&self.tolerance)),
                    false,
                );
            }
        }
    }

    pub fn change_tolerance_cursor(&mut self, direction: HorizontalDirection) {
        self.tolerance.scroll_cursor(direction)
    }

    pub fn change_history(&mut self, direction: HorizontalDirection) {
        self.history.write().unwrap().scroll_history(direction)
    }

    pub fn change_cols_ref(&mut self, direction: VerticalDirection) {
        if let Some((_, reference)) = self.compare.left.as_mut() {
            match direction {
                VerticalDirection::Up => {
                    reference.max_cols += 1;
                }
                VerticalDirection::Down => {
                    reference.max_cols -= 1;
                }
                VerticalDirection::Row(_) => todo!(),
            }
        }
    }

    pub fn change_cols_diff(&mut self, direction: VerticalDirection) {
        if let Some((_, diff)) = self.compare.right.as_mut() {
            match direction {
                VerticalDirection::Up => {
                    diff.max_cols += 1;
                }
                VerticalDirection::Down => {
                    diff.max_cols -= 1;
                }
                VerticalDirection::Row(_) => todo!(),
            }
        }
    }

    pub fn scroll_compare(
        &mut self,
        horizontal_dir: Option<HorizontalDirection>,
        vertical_dir: Option<VerticalDirection>,
    ) {
        if let Some(horiz) = horizontal_dir {
            match horiz {
                HorizontalDirection::Left => self.compare.previous_col(),
                HorizontalDirection::Right => self.compare.next_col(),
                HorizontalDirection::Col(line) => self.compare.col(line),
            }
        }
        if let Some(vertical) = vertical_dir {
            match vertical {
                VerticalDirection::Up => self.compare.previous_row(),
                VerticalDirection::Down => self.compare.next_row(),
                VerticalDirection::Row(line) => self.compare.row(line),
            }
        }
    }

    pub fn wind_cursor(&mut self, direction: Option<VerticalDirection>) {
        let mut wind_cursor = self.wind_cursor.write().unwrap();
        match wind_cursor.mode {
            WindMode::Active => match direction {
                Some(VerticalDirection::Up) => {
                    wind_cursor.section_label = None;
                    wind_cursor.line_start = Some(wind_cursor.line_start.unwrap_or(0));
                    wind_cursor.line_start =
                        Some(wind_cursor.line_start.unwrap().saturating_sub(1).max(1));
                    if wind_cursor.line_start.unwrap() == 0 {
                        wind_cursor.line_start.replace(1);
                    }
                    wind_cursor.line_end = wind_cursor.line_start;
                }
                Some(VerticalDirection::Down) => {
                    wind_cursor.section_label = None;
                    wind_cursor.line_start = Some(wind_cursor.line_end.unwrap_or(0));
                    wind_cursor.line_start =
                        Some(wind_cursor.line_start.unwrap().saturating_add(1));
                    wind_cursor.line_end = wind_cursor.line_start;
                }
                Some(VerticalDirection::Row(line)) => {
                    let lineu32 = line
                        .clamp(0, u32::MAX as usize)
                        .try_into()
                        .expect("just clamped");
                    wind_cursor.section_label = None;
                    wind_cursor.line_start = Some(lineu32);
                    wind_cursor.line_end = wind_cursor.line_start;
                }
                None => {
                    wind_cursor.section_label = None;
                    wind_cursor.line_start = None;
                    wind_cursor.line_end = None;
                }
            },
            WindMode::ActiveSelect => match direction {
                Some(VerticalDirection::Up) => {
                    wind_cursor.section_label = None;
                    wind_cursor.line_end = Some(wind_cursor.line_end.unwrap_or(0));
                    if wind_cursor.line_end == wind_cursor.line_start {
                        wind_cursor.line_start =
                            Some(wind_cursor.line_start.unwrap().saturating_sub(1).max(1));
                        if wind_cursor.line_start.unwrap() == 0 {
                            wind_cursor.line_start.replace(1);
                        }
                    } else {
                        wind_cursor.line_end =
                            Some(wind_cursor.line_end.unwrap().saturating_sub(1));
                        if wind_cursor.line_end.unwrap() == 0 {
                            wind_cursor.line_end.replace(1);
                        }
                    }
                }
                Some(VerticalDirection::Down) => {
                    if wind_cursor.line_start.is_none() {
                        wind_cursor.line_start = Some(1);
                    }
                    wind_cursor.section_label = None;
                    wind_cursor.line_end = Some(wind_cursor.line_end.unwrap_or(0));
                    wind_cursor.line_end = Some(wind_cursor.line_end.unwrap().saturating_add(1));
                }
                Some(VerticalDirection::Row(line)) => {
                    let lineu32 = line
                        .clamp(0, u32::MAX as usize)
                        .try_into()
                        .expect("just clamped");
                    wind_cursor.line_end = Some(lineu32);
                    wind_cursor.section_label = None;
                }
                None => {
                    wind_cursor.section_label = None;
                    wind_cursor.line_start = None;
                    wind_cursor.line_end = None;
                }
            },
            _ => {}
        };
    }

    // fn timestamp_to_millis(t: &bagread::TimeMsg) -> u64 {
    //     t.sec as u64 * 1_000 + t.nanosec as u64 / 1_000_000
    // }

    // fn timestamp_to_millis_rpcl2(t: &ros_pointcloud2::ros::TimeMsg) -> u64 {
    //     t.sec as u64 * 1_000 + t.nanosec as u64 / 1_000_000
    // }

    pub async fn wind_fire_at_current_cursor(
        send_coordinator: tokio::sync::mpsc::Sender<PacketKind>,
        wind_cursor_states: Arc<RwLock<WindCursor>>,
        wind_queue: tokio::sync::mpsc::UnboundedSender<Evaluated>,
        state: Option<Vec<PlayKindUnitedPass3>>,
    ) {
        let state = state.map(|pkus| {
            let wf = pkus
                .into_iter()
                .map(WindFunction::SendFrames)
                .collect::<Vec<_>>();
            Evaluated {
                rules: Rules::new(),
                wind: wf,
                vars: VariableHistory::new(vec![]),
            }
        });

        // task to not block the runtime while compiling
        tokio::spawn(async move {
            let wind_cursor_outer = wind_cursor_states.clone();

            let eval = if let Some(state) = state {
                state
            } else {
                let wind_cursor_in_blocking = Arc::clone(&wind_cursor_outer);
                let (res_tx, res_rx) = tokio::sync::oneshot::channel();
                // thread to not block the UI
                std::thread::spawn(move || {
                    let start_compile = Instant::now();
                    let var_state = {
                        let mut wc = wind_cursor_in_blocking.write().unwrap();
                        wc.compile_state = WindCompile::Compiling;
                        wc.variable_cache.clone()
                    };

                    let lines = wind_cursor_in_blocking.read().unwrap().selected_lines();
                    let start_line = lines.first().map(|l| *l as usize);
                    let end_line = lines.last().map(|l| *l as usize);
                    let rats_file = wind_cursor_in_blocking
                        .read()
                        .unwrap()
                        .wind_file_path
                        .clone();
                    let eval = mtc::compile_file_with_state(
                        &rats_file,
                        start_line,
                        end_line,
                        Some(var_state),
                        ErrorWriter {},
                        false,
                    );

                    let mut eval = match eval {
                        Ok(eval) => eval,
                        Err(e) => {
                            res_tx.send(Err(e)).unwrap();
                            return;
                        }
                    };

                    {
                        let mut wc = wind_cursor_in_blocking.write().unwrap();
                        eval.vars.populate_cache();
                        wc.variable_cache = eval.vars.var_cache.clone();
                        wc.compile_state = WindCompile::Done;
                    }
                    info!("compiled in {:?}", start_compile.elapsed());
                    res_tx.send(Ok(eval)).unwrap();
                });

                match res_rx.await {
                    Ok(Ok(eval)) => eval,
                    Err(e) => {
                        error!("{e}");
                        return;
                    }
                    Ok(Err(e)) => {
                        error!("{e}");
                        return;
                    }
                }
            };

            let mut vars = 0;
            for (var, rules) in eval.rules.raw() {
                vars += 1;
                send_coordinator
                    .send(PacketKind::RuleAppend {
                        variable: var.clone(),
                        commands: rules.clone(),
                    })
                    .await
                    .unwrap();
            }

            if vars != 0 {
                info!("sent {vars} new rules");
            }

            {
                let mut wind_cursor = wind_cursor_outer.write().unwrap();
                match wind_cursor.wind_send_state {
                    WindSendState::Acked | WindSendState::Sent => {
                        wind_queue.send(eval).unwrap();
                        wind_cursor.wind_work_queue += 1;
                    }
                    WindSendState::NotRun => {
                        wind_queue.send(eval).unwrap();
                        wind_cursor.wind_work_queue += 1;
                        wind_cursor.wind_send_state = WindSendState::Sent;
                    }
                }
            }
        });

        tokio::task::yield_now().await; // needs to spawn the task
    }

    pub fn wind_toggle_select(&mut self) {
        let mut wind_cursor = self.wind_cursor.write().unwrap();
        wind_cursor.mode = match wind_cursor.mode {
            WindMode::Inactive => WindMode::Inactive,
            WindMode::Active => WindMode::ActiveSelect,
            WindMode::ActiveSelect => WindMode::Active,
        };
    }

    pub fn compare_toggle_popup(&mut self) {
        self.compare.show_line_input_popup = !self.compare.show_line_input_popup;
    }

    pub async fn clear_rules(&mut self) {
        self.send_coordinator
            .send(PacketKind::RulesClear)
            .await
            .unwrap();
        info!("sent clear rules");
    }

    pub fn wind_toggle_mode(&mut self) {
        {
            let mut wind_cursor = self.wind_cursor.write().unwrap();
            wind_cursor.mode = match wind_cursor.mode {
                WindMode::Inactive => WindMode::Active,
                WindMode::Active => WindMode::Inactive,
                WindMode::ActiveSelect => {
                    wind_cursor.popup_title = WIND_POPUP_TITLE_NOERR;
                    wind_cursor.popup_buffer.clear();
                    wind_cursor.showing_popup = false;
                    WindMode::Inactive
                }
            };
        }

        if self.wind_mode() {
            self.mode = Mode::Wind {
                hide_compare: false,
            };
        } else {
            self.mode = Mode::Compare;
        }
    }

    pub fn wind_toggle_zen(&mut self) {
        if self.wind_mode() {
            match self.mode {
                Mode::Wind { hide_compare } => {
                    self.mode = Mode::Wind {
                        hide_compare: !hide_compare,
                    }
                }
                _ => {}
            }
        }
    }

    pub fn wind_mode(&mut self) -> bool {
        let wind_cursor = self.wind_cursor.read().unwrap();
        match wind_cursor.mode {
            WindMode::Inactive => false,
            WindMode::Active => true,
            WindMode::ActiveSelect => true,
        }
    }

    fn find_section_bounds(
        &self,
        lines: &Vec<String>,
        start_marker_idx_0: usize,
    ) -> Option<((usize, usize), String)> {
        let start_marker_line = &lines[start_marker_idx_0];

        // Extract the name from the capture group
        let Some(caps) = START_MARKER_RE.captures(start_marker_line) else {
            eprintln!("Regex match failed internally. This is a bug.");
            return None;
        };
        let name = caps.get(1).map_or("", |m| m.as_str()).to_string();

        let section_start_0 = start_marker_idx_0 + 1;

        if section_start_0 >= lines.len() {
            return None;
        }
        let mut end_marker_idx_0: Option<usize> = None;
        for (i, line) in lines.iter().enumerate().skip(section_start_0) {
            if line.starts_with("#---") {
                end_marker_idx_0 = Some(i);
                break;
            }
        }

        let section_end_0 = match end_marker_idx_0 {
            Some(end_marker_idx) => end_marker_idx.saturating_sub(1),
            None => lines.len().saturating_sub(1),
        };

        if section_end_0 < section_start_0 {
            return None;
        }

        let cursor_line_start = section_start_0 + 1;
        let cursor_line_end = section_end_0 + 1;

        Some(((cursor_line_start, cursor_line_end), name))
    }

    fn find_next_section(
        &self,
        lines: &Vec<String>,
        start_line_1based: usize,
    ) -> Option<((usize, usize), String)> {
        let search_from_index_0 = start_line_1based.saturating_sub(1);

        for (i, line) in lines.iter().enumerate().skip(search_from_index_0) {
            if START_MARKER_RE.is_match(line) {
                return self.find_section_bounds(lines, i);
            }
        }

        None
    }

    fn find_previous_section(
        &self,
        lines: &Vec<String>,
        start_line_1based: usize,
    ) -> Option<((usize, usize), String)> {
        let current_section_marker_idx_0 = start_line_1based.saturating_sub(2);

        if start_line_1based <= 1 {
            return None;
        }

        for i in (0..current_section_marker_idx_0).rev() {
            if START_MARKER_RE.is_match(&lines[i]) {
                return self.find_section_bounds(lines, i);
            }
        }

        None
    }

    fn read_rl_file(path: &Path) -> Option<String> {
        let mut err = ErrorWriter {};
        let file = std::fs::read_to_string(path);
        let file = match file {
            Ok(file) => file,
            Err(e) => match e.kind() {
                std::io::ErrorKind::NotFound => {
                    std::io::Write::write(
                        &mut err,
                        format!("Could not find file: {}", path.display()).as_bytes(),
                    )
                    .unwrap();
                    return None;
                }
                std::io::ErrorKind::PermissionDenied => {
                    std::io::Write::write(
                        &mut err,
                        format!("No permission to read file: {}", path.display()).as_bytes(),
                    )
                    .unwrap();
                    return None;
                }
                std::io::ErrorKind::IsADirectory => {
                    std::io::Write::write(
                        &mut err,
                        format!("File is a directory: {}", path.display()).as_bytes(),
                    )
                    .unwrap();
                    return None;
                }
                _ => {
                    std::io::Write::write(
                        &mut err,
                        format!("Unexpected error for file: {}", path.display()).as_bytes(),
                    )
                    .unwrap();
                    return None;
                }
            },
        };

        Some(file)
    }

    pub fn select_next_label(&mut self) {
        let Some(file) = self
            .rats_file
            .as_ref()
            .and_then(|path| Self::read_rl_file(path))
            .and_then(|file| Some(file.lines().map(String::from).collect::<Vec<_>>()))
        else {
            return;
        };
        {}
        let mut cursor = self.wind_cursor.write().unwrap();
        let start_line = match (cursor.line_start, cursor.line_end) {
            (None, None) => 1,
            (None, Some(last)) => last + 1,
            (Some(first), None) => first + 1,
            (Some(_), Some(last)) => last + 1,
        };

        let found = Self::find_next_section(&self, &file, start_line as usize);

        let Some(((start_cursor, end_cursor), section_name)) =
            found.or_else(|| Self::find_next_section(&self, &file, 1))
        else {
            return;
        };

        cursor.line_start = Some(start_cursor as u32);
        cursor.line_end = Some(end_cursor as u32);
        cursor.section_label = Some(section_name);
    }

    fn find_last_section(&self, lines: &Vec<String>) -> Option<((usize, usize), String)> {
        for i in (0..lines.len()).rev() {
            if START_MARKER_RE.is_match(&lines[i]) {
                return self.find_section_bounds(lines, i);
            }
        }
        None
    }

    pub fn select_prev_label(&mut self) {
        let Some(file) = self
            .rats_file
            .as_ref()
            .and_then(|path| Self::read_rl_file(path))
            .and_then(|file| Some(file.lines().map(String::from).collect::<Vec<_>>()))
        else {
            return;
        };
        let mut cursor = self.wind_cursor.write().unwrap();
        let start_line = match (cursor.line_start, cursor.line_end) {
            (None, None) => 1,
            (None, Some(last)) => last,
            (Some(first), None) => first,
            (Some(first), Some(_)) => first,
        };

        let found = Self::find_previous_section(&self, &file, start_line as usize);

        let Some(((start_cursor, end_cursor), section_name)) =
            found.or_else(|| Self::find_last_section(&self, &file))
        else {
            return;
        };

        cursor.line_start = Some(start_cursor as u32);
        cursor.line_end = Some(end_cursor as u32);
        cursor.section_label = Some(section_name);
    }

    pub fn wind_toggle_popup(&mut self) {
        let mut wind_cursor = self.wind_cursor.write().unwrap();
        wind_cursor.showing_popup = !wind_cursor.showing_popup;
    }
}
