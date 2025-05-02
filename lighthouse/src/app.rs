use std::{
    collections::{HashMap, VecDeque},
    fmt::Display,
    i8, io,
    ops::DerefMut,
    path::PathBuf,
    str::FromStr,
    sync::{Arc, Mutex, RwLock},
    time::{Duration, Instant},
};

use anyhow::{Error, anyhow};
use bagread::PlayKindUnitedRich;
use chrono::{DateTime, Local};
use log::{error, info};
use ratatui::{
    layout::{Alignment, Constraint},
    style::Stylize,
    widgets::{Cell, Paragraph, Row, ScrollbarState, Table, TableState},
};
use rlc::{Evaluated, Rhs};
use sea::{WindData, net::PacketKind};

/// Application result type.
pub type AppResult<T> = std::result::Result<T, Box<dyn core::error::Error>>;

#[derive(Debug, Default)]
pub enum Mode {
    Wind,
    #[default]
    Visual,
}

impl Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Mode::Wind => f.write_str("Wind"),
            Mode::Visual => f.write_str("Visual"),
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
            let diff = (ndig as i8 - self.pot_cursor as i8).abs() as usize;
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
        return Some(MatHistIdx {
            history_key: key.clone(),
            buffer_idx: idx,
        });
    }

    pub fn key_of_diff_by_key(&self, key: &HistoryKey) -> Option<MatHistIdx> {
        let (_, buff) = self.history.iter().find(|(k, _)| k == key)?;
        let buff = buff.lock().unwrap();
        let idx = buff
            .iter()
            .position(|hentry| matches!(hentry.state, MatSelectState::Diff))?;
        return Some(MatHistIdx {
            history_key: key.clone(),
            buffer_idx: idx,
        });
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
                buf.iter().nth(key.buffer_idx).cloned()
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

            let key = HistoryKey {
                variable: variable.clone(),
                call_id,
            };
            key
        } else {
            let changed = self.history.is_empty();
            let call_id = if changed {
                1
            } else {
                unreachable!("no var but history wasn't empty");
            };

            let key = HistoryKey {
                variable: variable.clone(),
                call_id,
            };
            key
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
                .find(|el| matches!(el.state, MatSelectState::Focused))
                .is_some()
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
                .find(|el| matches!(el.state, MatSelectState::Diff))
                .is_some()
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

                        if let Some(next_diff) = next_buff.iter_mut().nth(2) {
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
        return Self {
            state: "".to_owned(),
            locked: false,
        };
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

pub const WIND_POPUP_TITLE_NOERR: &'static str = "WindCursor to Line:";
pub const POPUP_TITLE_ERR: &'static str = "Could not parse! Please correct:";

pub const COMPARE_POPUP_TITLE_NOERR: &'static str = "CompareCursor to row[:col]";

#[derive(Debug, Default)]
pub enum WindMode {
    #[default]
    // Inactive, // only if no wind connected. TODO get this info from coordinator. Set automatically to Active as soon as some wind is connected
    Inactive, // shown but not changable
    Active,
    ActiveSelect,
}

#[derive(Debug)]
pub struct WindCursor {
    line_start: u32,
    line_end: u32,
    pub showing_popup: bool,
    mode: WindMode,
    pub popup_title: &'static str,
    pub popup_buffer: Vec<char>,
    wind_send_state: WindSendState,
    wind_work_queue: VecDeque<Evaluated>,
    compile_state: WindCompile,
    wind_file_path: PathBuf,
    bagfile: bagread::Bagfile,
    variable_cache: HashMap<rlc::Var, rlc::Rhs>, // TODO use
}

impl Default for WindCursor {
    fn default() -> Self {
        Self {
            line_start: 1,
            line_end: 1,
            wind_send_state: WindSendState::default(),
            showing_popup: false,
            popup_buffer: Vec::new(),
            popup_title: WIND_POPUP_TITLE_NOERR,
            mode: WindMode::default(),
            wind_file_path: PathBuf::from("./wind.rl"),
            bagfile: bagread::Bagfile::default(),
            variable_cache: HashMap::new(),
            wind_work_queue: VecDeque::new(),
            compile_state: WindCompile::default(),
        }
    }
}

impl Display for WindCursor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.mode {
            WindMode::Inactive => {
                f.write_str("W")?;
            }
            WindMode::Active => {
                f.write_str("|W")?;
            }
            WindMode::ActiveSelect => {
                f.write_str("[W]")?;
            }
        }

        let lines = if self.line_start == self.line_end {
            format!("{}", self.line_start)
        } else {
            format!("{}-{}", self.line_start, self.line_end)
        };

        let queue_len = self.wind_work_queue.len();
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
        (self.line_start..self.line_end + 1).map(|i| i).collect()
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

const ROWS_COLS_CELL: &'static str = "↓ →";
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
                            } else {
                                if l < r {
                                    CellDiff::Higher(r - l - tolerance)
                                } else {
                                    CellDiff::Lower(l - r - tolerance)
                                }
                            }
                        } else {
                            if l < r {
                                CellDiff::Higher(r - l)
                            } else {
                                CellDiff::Lower(l - r)
                            }
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

    pub fn render(&self, precision: usize) -> anyhow::Result<Option<Table>> {
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

    pub fn render(&self, precision: usize) -> anyhow::Result<Option<Table>> {
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
            let right = DiffMatrix::new(&mat_ref, tolerance, mat.max_cols, mat.max_rows, mat);
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
                        } else {
                            if i == r.max_rows - 1 {
                                r.curr_offset_rows += 1;
                                i
                            } else {
                                i + 1
                            }
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
                        } else {
                            if i == l.max_rows - 1 {
                                l.curr_offset_rows += 1;
                                i
                            } else {
                                i + 1
                            }
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
                        } else {
                            if i == 0 {
                                r.curr_offset_rows -= 1;
                                i
                            } else {
                                i - 1
                            }
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
                        } else {
                            if i == 0 {
                                l.curr_offset_rows -= 1;
                                i
                            } else {
                                i - 1
                            }
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
                        } else {
                            if i == r.max_cols {
                                r.curr_offset_cols += 1;
                                i
                            } else {
                                i + 1
                            }
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
                        } else {
                            if i == l.max_cols {
                                l.curr_offset_cols += 1;
                                i
                            } else {
                                i + 1
                            }
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
                        } else {
                            if i == 1 {
                                r.curr_offset_cols -= 1;
                                i
                            } else {
                                i - 1
                            }
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
                        } else {
                            if i == 1 {
                                l.curr_offset_cols -= 1;
                                i
                            } else {
                                i - 1
                            }
                        }
                    }
                    None => 1,
                };
                // self.right_state.select_column(Some(i));
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
            horizontal_scroll_state: ScrollbarState::new(1 * ITEM_HEIGHT),
            vertical_scroll_state: ScrollbarState::new(1 * ITEM_HEIGHT),
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

#[derive(Debug, Default)]
pub struct InfoView {
    pub shown: bool,
    pub log: Vec<String>,
}

fn get_current_time() -> String {
    let now: DateTime<Local> = Local::now();
    let formatted_time = now.format("%H:%M:%S").to_string();
    formatted_time
}

pub(crate) const INFO_LINES: usize = 10;

impl std::io::Write for InfoView {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let s = str::from_utf8(buf).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let s = String::from(s);
        error!("{s}");
        // self.log.push(s);

        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

// TODO fix text not rendered coloured because info is lost with String. Needs "Line" from ratatui
impl InfoView {
    // pub fn log_info(&mut self, log: &str) {
    //     let txt = "INFO".blue();
    //     self.log
    //         .push(format!("[{txt}] {} -- {log}", get_current_time()));

    //     if self.log.len() > INFO_LINES {
    //         self.log.remove(1);
    //     }
    // }
    // pub fn log_error(&mut self, log: &str) {
    //     let txt = "ERROR".red();
    //     self.log
    //         .push(format!("[{txt}] {} - {log}", get_current_time()));

    //     if self.log.len() > INFO_LINES {
    //         self.log.remove(1);
    //     }
    // }
    // pub fn log_debug(&mut self, log: &str) {
    //     let txt = "DEBUG";
    //     self.log
    //         .push(format!("[{txt}] {} -- {log}", get_current_time()));

    //     if self.log.len() > INFO_LINES {
    //         self.log.remove(1);
    //     }
    // }

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
    pub info_view: Arc<RwLock<InfoView>>,
    pub coordinator: Coordinator,
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
        tokio::task::yield_now().await;

        let mut wind_cursor = WindCursor::default();

        if let Some(rl) = &rats {
            wind_cursor = wind_cursor.with_rats(rl.clone());
        }

        Self {
            running: true,
            coordinator: Coordinator::default(),
            history,
            wind_cursor: Arc::new(RwLock::new(wind_cursor)),
            mode: Mode::default(),
            tolerance: Tolerance::default(),
            compare: ClientCompare::default(),
            send_coordinator: sender,
            rats_file: rats,
            info_view: Arc::new(RwLock::new(InfoView::default())),
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
        format!("{}", self.history.read().unwrap().render_var_history())
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

    pub fn focus_right_rat(&mut self) {
        self.history.write().unwrap().diff_to_ref()
    }

    pub fn change_diff_rat(&mut self, direction: VerticalDirection) {
        self.history.write().unwrap().diff_scroll(direction)
    }

    pub async fn send_lock_next(&mut self) {
        self.send_coordinator
            .send(PacketKind::LockNext)
            .await
            .unwrap();
    }

    pub async fn send_unlock(&mut self) {
        self.send_coordinator
            .send(PacketKind::Unlock)
            .await
            .unwrap();
    }

    pub fn change_tolerance_at_current_cursor(&mut self, direction: VerticalDirection) {
        self.tolerance.change_tolerance_at_current_cursor(direction);
        if let Some((matidx, old_mat)) = self.compare.right.as_ref() {
            let curr_history = self.history.read().unwrap();
            if let Some((mut mat, _)) = curr_history.mat_with_key(&matidx) {
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

    pub fn wind_cursor(&mut self, direction: VerticalDirection) {
        let mut wind_cursor = self.wind_cursor.write().unwrap();
        match wind_cursor.mode {
            WindMode::Active => match direction {
                VerticalDirection::Up => {
                    wind_cursor.line_start = wind_cursor.line_start.saturating_sub(1);
                    if wind_cursor.line_start == 0 {
                        wind_cursor.line_start = 1;
                    }
                    wind_cursor.line_end = wind_cursor.line_start;
                }
                VerticalDirection::Down => {
                    wind_cursor.line_start = wind_cursor.line_start.saturating_add(1);
                    wind_cursor.line_end = wind_cursor.line_start;
                }
                VerticalDirection::Row(line) => {
                    wind_cursor.line_end = line
                        .clamp(0, u32::MAX as usize)
                        .try_into()
                        .expect("just clamped");
                    wind_cursor.line_end = wind_cursor.line_start;
                }
            },
            WindMode::ActiveSelect => match direction {
                VerticalDirection::Up => {
                    if wind_cursor.line_end == wind_cursor.line_start {
                        wind_cursor.line_start = wind_cursor.line_start.saturating_sub(1);
                        if wind_cursor.line_start == 0 {
                            wind_cursor.line_start = 1;
                        }
                    } else {
                        wind_cursor.line_end = wind_cursor.line_end.saturating_sub(1);
                        if wind_cursor.line_end == 0 {
                            wind_cursor.line_end = 1;
                        }
                    }
                }
                VerticalDirection::Down => {
                    wind_cursor.line_end = wind_cursor.line_end.saturating_add(1);
                }
                VerticalDirection::Row(line) => {
                    wind_cursor.line_end = line
                        .clamp(1, u32::MAX as usize)
                        .try_into()
                        .expect("just clamped");
                }
            },
            _ => {}
        };
    }

    fn topic_from_eval_or_default(
        eval: &rlc::Evaluated,
        path: &str,
        default: &str,
    ) -> anyhow::Result<String> {
        let topic = eval.vars.resolve(path)?;
        if let Some(topic) = topic {
            match topic {
                Rhs::Path(topic) | Rhs::Val(rlc::Val::StringVal(topic)) => Ok(topic),
                _ => {
                    return Err(anyhow!("Expected String or Path for sending lidar topic"));
                }
            }
        } else {
            Ok(default.to_owned())
        }
    }

    fn timestamp_to_millis(t: &bagread::TimeMsg) -> u64 {
        t.sec as u64 * 1_000 + t.nanosec as u64 / 1_000_000
    }

    fn timestamp_to_millis_rpcl2(t: &ros_pointcloud2::ros::TimeMsg) -> u64 {
        t.sec as u64 * 1_000 + t.nanosec as u64 / 1_000_000
    }

    pub async fn wind_fire_at_current_cursor(&mut self) {
        let send_coordinator = self.send_coordinator.clone();
        let info_view = self.info_view.clone();
        let wind_cursor_states = self.wind_cursor.clone();

        // task to not block the UI while compiling
        tokio::spawn(async move {
            let wind_cursor_outer = wind_cursor_states.clone();
            let info_view_outer = info_view.clone();

            let var_state = {
                let mut wc = wind_cursor_outer.write().unwrap();
                wc.compile_state = WindCompile::Compiling;
                wc.variable_cache.clone()
            };

            let lines = wind_cursor_outer.read().unwrap().selected_lines();
            let start_line = lines.first().map(|l| *l as usize);
            let end_line = lines.last().map(|l| *l as usize);
            let rats_file = wind_cursor_outer.read().unwrap().wind_file_path.clone();
            let eval = rlc::compile_file_with_state(
                &rats_file,
                start_line,
                end_line,
                Some(var_state),
                info_view_outer.write().unwrap().deref_mut(),
                false,
            );
            let mut eval = match eval {
                Ok(eval) => eval,
                Err(e) => {
                    error!("{e}");
                    return;
                }
            };

            {
                let mut wc = wind_cursor_outer.write().unwrap();
                eval.vars.populate_cache();
                wc.variable_cache = eval.vars.var_cache.clone();
                wc.compile_state = WindCompile::Done;
            }

            info!("compiled {start_line:?}-{end_line:?}");
            {
                let mut wind_cursor = wind_cursor_outer.write().unwrap();
                match wind_cursor.wind_send_state {
                    WindSendState::Acked | WindSendState::Sent => {
                        wind_cursor.wind_work_queue.push_back(eval);
                        return;
                    }
                    WindSendState::NotRun => {
                        wind_cursor.wind_work_queue.push_back(eval);
                        wind_cursor.wind_send_state = WindSendState::Sent;
                    }
                }
            }

            // task for doing the task on the queue
            tokio::spawn(async move {
                let wind_cursor = wind_cursor_states.clone();
                let fun = async move {
                    loop {
                        let eval = {
                            let mut wc = wind_cursor.write().unwrap();
                            wc.wind_work_queue.pop_front()
                        };

                        match eval {
                            Some(eval) => {
                                for windfn in eval.wind.iter() {
                                    match windfn {
                                        rlc::WindFunction::Reset(path) => {
                                            match wind_cursor
                                                .write()
                                                .unwrap()
                                                .bagfile
                                                .reset(Some(path))
                                            {
                                                Err(e) => {
                                                    error!(
                                                        "Could not reset bagfile path {path:?}: {e}"
                                                    );

                                                    return;
                                                }
                                                Ok(_) => {
                                                    info!("read bag {path}");
                                                }
                                            }
                                        }
                                        rlc::WindFunction::SendFrames(kind) => {
                                            let sending_lidar_topic =
                                                match Self::topic_from_eval_or_default(
                                                    &eval,
                                                    "_bag.lidar.topic",
                                                    "/cloud",
                                                ) {
                                                    Ok(topic) => topic,
                                                    Err(e) => {
                                                        error!("{e}");
                                                        return;
                                                    }
                                                };
                                            info!("lidar topic: {sending_lidar_topic}");
                                            let sending_imu_topic =
                                                match Self::topic_from_eval_or_default(
                                                    &eval,
                                                    "_bag.imu.topic",
                                                    "/imu",
                                                ) {
                                                    Ok(topic) => topic,
                                                    Err(e) => {
                                                        error!("{e}");
                                                        return;
                                                    }
                                                };

                                            info!("imu topic: {sending_imu_topic}");

                                            let trigger = match &kind {
                                                rlc::PlayKindUnited::SensorCount {
                                                    sensor: _,
                                                    count: _,
                                                    trigger,
                                                } => trigger,
                                                rlc::PlayKindUnited::UntilSensorCount {
                                                    sending: _,
                                                    until_sensor: _,
                                                    until_count: _,
                                                    trigger,
                                                } => trigger,
                                            };

                                            let bagmsgs =
                                                wind_cursor.write().unwrap().bagfile.next(
                                                    &PlayKindUnitedRich::with_topic(
                                                        kind.clone(),
                                                        &vec![
                                                            sending_lidar_topic.as_str(),
                                                            sending_imu_topic.as_str(),
                                                        ],
                                                    ),
                                                );

                                            let bagmsgs = match bagmsgs {
                                                Ok(msgs) => msgs,
                                                Err(e) => {
                                                    error!("{e}");
                                                    return;
                                                }
                                            };

                                            let mut wind_data = Vec::with_capacity(bagmsgs.len());
                                            let mut start_time = None;
                                            let mut end_time = None;
                                            for msg in bagmsgs {
                                                let wind = match msg {
                                                    bagread::BagMsg::Cloud(point_cloud2_msg) => {
                                                        let t = Self::timestamp_to_millis_rpcl2(
                                                            &point_cloud2_msg.header.stamp,
                                                        );
                                                        if start_time.is_none() {
                                                            start_time.replace(t);
                                                        }

                                                        end_time.replace(t);

                                                        (
                                                            end_time.unwrap() - start_time.unwrap(),
                                                            WindData::Pointcloud(point_cloud2_msg),
                                                        )
                                                    }
                                                    bagread::BagMsg::Imu(imu_msg) => {
                                                        let t = Self::timestamp_to_millis(
                                                            &imu_msg.header.stamp,
                                                        );
                                                        if start_time.is_none() {
                                                            start_time.replace(t);
                                                        }

                                                        end_time.replace(t);

                                                        (
                                            end_time.unwrap() - start_time.unwrap(),
                                            WindData::Imu(sea::ImuMsg {
                                                header: sea::Header {
                                                    seq: imu_msg.header.seq,
                                                    stamp: sea::TimeMsg {
                                                        sec: imu_msg.header.stamp.sec,
                                                        nanosec: imu_msg.header.stamp.nanosec,
                                                    },
                                                    frame_id: imu_msg.header.frame_id,
                                                },
                                                timestamp_sec: sea::TimeMsg {
                                                    sec: imu_msg.timestamp_sec.sec,
                                                    nanosec: imu_msg.timestamp_sec.nanosec,
                                                },
                                                orientation: imu_msg.orientation,
                                                orientation_covariance: imu_msg
                                                    .orientation_covariance,
                                                angular_velocity: imu_msg.angular_velocity,
                                                angular_velocity_covariance: imu_msg
                                                    .angular_velocity_covariance,
                                                linear_acceleration: imu_msg.linear_acceleration,
                                                linear_acceleration_covariance: imu_msg
                                                    .linear_acceleration_covariance,
                                            }),
                                        )
                                                    }
                                                };

                                                wind_data.push(wind);
                                            }

                                            if wind_data.is_empty() {
                                                return;
                                            }

                                            let var = match trigger {
                                                Some(rlc::PlayTrigger::Variable(var)) => {
                                                    Some(var.clone())
                                                }
                                                _ => None,
                                            };

                                            match trigger {
                                                Some(rlc::PlayTrigger::DurationMs(
                                                    target_duration_ms,
                                                )) => {
                                                    let original_duration_ms =
                                                        end_time.unwrap() - start_time.unwrap();
                                                    let scale = *target_duration_ms as f64
                                                        / original_duration_ms as f64;

                                                    let start_time = Instant::now();

                                                    for (original_ts, wind) in wind_data {
                                                        let scaled_ts = original_ts as f64 * scale;
                                                        let target_offset = Duration::from_secs_f64(
                                                            scaled_ts / 1000.0,
                                                        );
                                                        let target_wall_time =
                                                            start_time + target_offset;
                                                        let sleep_duration = target_wall_time
                                                            .saturating_duration_since(
                                                                Instant::now(),
                                                            );
                                                        tokio::time::sleep(sleep_duration).await;

                                                        let send = send_coordinator
                                                            .send(PacketKind::Wind {
                                                                data: wind,
                                                                at_var: None,
                                                            })
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
                                                // TODO this can be probably way more elegant. needs var as None in case None and Some(var) in case some var
                                                _ => {
                                                    for (_, wind) in wind_data {
                                                        let send = send_coordinator
                                                            .send(PacketKind::Wind {
                                                                data: wind,
                                                                at_var: var.clone(),
                                                            })
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
                                            }
                                        }
                                    }
                                }
                            }
                            None => break,
                        }
                    }
                };

                // run function and always close by setting the acked state to wind
                fun.await;
                {
                    let mut wind_cursor = wind_cursor_states.write().unwrap();
                    wind_cursor.wind_send_state = WindSendState::Acked;
                }

                // reset state after
                tokio::time::sleep(Duration::from_secs(2)).await;
                let mut wind_cursor = wind_cursor_states.write().unwrap();
                match wind_cursor.wind_send_state {
                    WindSendState::Acked => {
                        wind_cursor.wind_send_state = WindSendState::NotRun;
                    }
                    WindSendState::Sent => {}
                    WindSendState::NotRun => {}
                };
            });
        });

        tokio::task::yield_now().await; // needs to spawn the task
    }

    // TODO change display to show what is selected and that mode is active
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

    pub fn wind_toggle_mode(&mut self) {
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
        info!("{:?}", wind_cursor.mode);
    }

    pub fn wind_mode(&mut self) -> bool {
        let wind_cursor = self.wind_cursor.read().unwrap();
        match wind_cursor.mode {
            WindMode::Inactive => false,
            WindMode::Active => true,
            WindMode::ActiveSelect => true,
        }
    }

    pub fn wind_toggle_popup(&mut self) {
        let mut wind_cursor = self.wind_cursor.write().unwrap();
        wind_cursor.showing_popup = !wind_cursor.showing_popup;
    }
}
