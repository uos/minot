use core::panic;
use std::{
    error::{self},
    fmt::Display,
    str::FromStr,
};

use anyhow::{anyhow, Error};
use ratatui::{
    layout::Constraint,
    style::Stylize,
    widgets::{Cell, Row, ScrollbarState, Table, TableState},
};

/// Application result type.
pub type AppResult<T> = std::result::Result<T, Box<dyn error::Error>>;

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

#[derive(Debug, Default)]
pub struct Tolerance {
    pub value: f64,
    pub pot_cursor: u8,
}

impl Display for Tolerance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("tol ")?;
        // f.write_str(create_underlined_string(self.value, self.pot_cursor).as_str())
        let decimal_places = if self.pot_cursor == 0 {
            0
        } else {
            self.pot_cursor as usize
        };
        let formatted_tolerance = format!("{:.1$}", self.value, decimal_places);
        let chars: Vec<char> = formatted_tolerance.chars().collect();
        let cursor_index = if self.pot_cursor > 0 {
            formatted_tolerance.find('.').unwrap_or(0) + self.pot_cursor as usize
        } else {
            0
        };

        for (i, &c) in chars.iter().enumerate() {
            if i == cursor_index {
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

#[derive(Debug, Default)]
pub struct History {
    history: Vec<String>,
    pos: usize,
}

impl Display for History {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.history.get(self.pos) {
            Some(el) => f.write_str(el),
            None => f.write_str(""),
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

pub const WIND_POPUP_TITLE_NOERR: &'static str = "WindCursor to Line:";
pub const POPUP_TITLE_ERR: &'static str = "Could not parse! Please correct:";

pub const COMPARE_POPUP_TITLE_NOERR: &'static str = "CompareCursor to row[:col]";

#[derive(Debug, Default)]
pub enum WindMode {
    #[default]
    Inactive, // only if no wind connected. TODO get this info from coordinator. Set automatically to Active as soon as some wind is connected
    Active,
    NormalCursor,
    Select,
}

#[derive(Debug)]
pub struct WindCursor {
    line: u32,
    pub showing_popup: bool,
    mode: WindMode,
    pub popup_title: &'static str,
    pub popup_buffer: Vec<char>,
    wind_send_state: WindSendState,
}

impl Default for WindCursor {
    fn default() -> Self {
        Self {
            line: 0,
            wind_send_state: WindSendState::default(),
            showing_popup: false,
            popup_buffer: Vec::new(),
            popup_title: WIND_POPUP_TITLE_NOERR,
            mode: WindMode::default(),
        }
    }
}

impl Display for WindCursor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.mode {
            WindMode::Inactive => {
                f.write_str("")?;
            }
            WindMode::Active => {
                f.write_str("W")?;
            }
            WindMode::NormalCursor => {
                f.write_str("|W")?;
            }
            WindMode::Select => {
                f.write_str("[W]")?;
            }
        }
        match self.wind_send_state {
            WindSendState::Acked => f.write_fmt(format_args!("{}✅", &self.line)),
            WindSendState::Sent => f.write_fmt(format_args!("{}🔄", &self.line)),
            WindSendState::NotRun => f.write_fmt(format_args!("{}", &self.line)),
        }
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
}

impl From<DiffMatrix> for Matrix {
    fn from(value: DiffMatrix) -> Self {
        value.data
    }
}

const ROWS_COLS_CELL: &'static str = "↓ →";

impl DiffMatrix {
    pub fn new(reference: &Matrix, mat: Matrix, tolerance: Option<f64>) -> Self {
        let rows = reference.nrows.max(mat.nrows);
        let cols = reference.ncols.max(mat.ncols);
        let mut diff = vec![CellDiff::OutOfBounds; rows * cols];
        for col in 0..mat.ncols {
            for row in 0..mat.nrows {
                let cell_diff = match (reference.get(row, col), mat.get(row, col)) {
                    (None, None) => panic!("wtf"),
                    (None, Some(_)) => CellDiff::OutOfBounds,
                    (Some(_), None) => CellDiff::OutOfBounds,
                    (Some(l), Some(r)) => {
                        let l = l.0;
                        let r = r.0;

                        if let Some(tolerance) = tolerance {
                            if (l - r).abs() < tolerance {
                                CellDiff::WithinTolerance
                            } else {
                                if l < r {
                                    CellDiff::Higher(r - l)
                                } else {
                                    CellDiff::Lower(l - r)
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
                };
                let index = row + col * rows;
                diff[index] = cell_diff;
            }
        }
        Self { data: mat, diff }
    }

    pub fn render(&self, precision: usize) -> anyhow::Result<Option<Table>> {
        let header_cols_nums: Vec<Cell> = (0..self.data.ncols)
            .map(|n| Cell::from(n.to_string()))
            .collect();
        let rows_cols_cell_len = ROWS_COLS_CELL.len() as u16;
        let mut header_cols = vec![Cell::from(ROWS_COLS_CELL)];
        header_cols.extend(header_cols_nums);

        let header_rows: Vec<Cell> = (0..self.data.nrows)
            .map(|n| Cell::from(n.to_string()))
            .collect();

        let cols_max_len: u16 = self
            .data
            .ncols
            .to_string()
            .len()
            .try_into()
            .map_err(|e| anyhow!("Too many cols: {e}"))?;
        let rows_max_len: u16 = self
            .data
            .nrows
            .to_string()
            .len()
            .try_into()
            .map_err(|e| anyhow!("Too many rows: {e}"))?;
        let max_data_len: u16 = if let Some(max) = self.data.data.iter().max() {
            format!("{}", max)
                .len()
                .try_into()
                .map_err(|e| anyhow!("Too many digits in the data to show: {e}"))?
        } else {
            return Ok(None);
        };
        let data_cell_len = Constraint::Length(max_data_len.max(cols_max_len));
        let header_rows_cell_len = Constraint::Length(rows_max_len.max(rows_cols_cell_len));

        let mut rows: Vec<Row> = vec![Row::new(Vec::<Cell>::new()); self.data.nrows];
        for row in 0..self.data.nrows {
            let mut nrow = vec![Cell::from("".to_owned()); self.data.ncols + 1];
            let row_header = unsafe { header_rows.get_unchecked(row) };
            nrow[0] = row_header.clone();
            for col in 0..self.data.ncols {
                let mat_cell = unsafe { self.diff.get_unchecked(col * self.data.nrows + row) };
                let str_cell = match mat_cell {
                    CellDiff::Higher(diff) => Cell::from(format!("{:.1$}", diff, precision).blue()),
                    CellDiff::Lower(diff) => Cell::from(format!("{:.1$}", diff, precision).red()),
                    CellDiff::WithinTolerance => Cell::from(""),
                    CellDiff::OutOfBounds => Cell::from("-".gray()),
                };
                unsafe { *nrow.get_unchecked_mut(col + 1) = str_cell };
            }
            unsafe { *rows.get_unchecked_mut(row) = Row::new(nrow) };
        }

        let mut widths = vec![header_rows_cell_len];
        widths.extend(core::iter::repeat_n(data_cell_len, self.data.ncols));
        let table = Table::new(rows, widths).header(Row::new(header_cols));
        Ok(Some(table))
    }
}

#[derive(Debug)]
pub struct Matrix {
    data: Vec<TotalF64>,
    pub nrows: usize,
    pub ncols: usize,
}

impl Matrix {
    pub fn new() -> Self {
        Self {
            data: vec![],
            nrows: 0,
            ncols: 0,
        }
    }

    pub fn get(&self, row: usize, col: usize) -> Option<TotalF64> {
        if row < self.nrows && col < self.ncols {
            let index = col * self.nrows + row;
            Some(self.data[index])
        } else {
            None
        }
    }

    pub fn render(&self, precision: usize) -> anyhow::Result<Option<Table>> {
        let header_cols_nums: Vec<Cell> =
            (0..self.ncols).map(|n| Cell::from(n.to_string())).collect();
        let rows_cols_cell_len = ROWS_COLS_CELL.len() as u16;
        let mut header_cols = vec![Cell::from(ROWS_COLS_CELL)];
        header_cols.extend(header_cols_nums);

        let header_rows: Vec<Cell> = (0..self.nrows).map(|n| Cell::from(n.to_string())).collect();
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
        let max_data_len: u16 = if let Some(max) = self.data.iter().max() {
            format!("{}", max)
                .len()
                .try_into()
                .map_err(|e| anyhow!("Too many digits in the data to show: {e}"))?
        } else {
            return Ok(None);
        };
        let data_cell_len = Constraint::Length(max_data_len.max(cols_max_len));
        let header_rows_cell_len = Constraint::Length(rows_max_len.max(rows_cols_cell_len));

        let mut rows: Vec<Row> = vec![Row::new(Vec::<Cell>::new()); self.nrows];
        for row in 0..self.nrows {
            let mut nrow = vec![Cell::from("".to_owned()); self.ncols + 1];
            let row_header = unsafe { header_rows.get_unchecked(row) };
            nrow[0] = row_header.clone();
            for col in 0..self.ncols {
                unsafe {
                    *nrow.get_unchecked_mut(col + 1) = Cell::from(format!(
                        "{:.1$}",
                        self.data.get_unchecked(col * self.nrows + row).0,
                        precision
                    ))
                };
            }
            unsafe { *rows.get_unchecked_mut(row) = Row::new(nrow) };
        }

        let mut widths = vec![header_rows_cell_len];
        widths.extend(core::iter::repeat_n(data_cell_len, self.ncols));
        let table = Table::new(rows, widths).header(Row::new(header_cols));
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

        Ok(Matrix { data, nrows, ncols })
    }
}

// #[derive(Debug)]
// pub enum MatrixView {
//     Base { data: Matrix, client: String },
//     Diff { data: DiffMatrix, client: String },
// }

// impl MatrixView {
//     pub fn client_name(&self) -> String {
//         match self {
//             MatrixView::Base { data: _, client } => client.clone(),
//             MatrixView::Diff { data: _, client } => client.clone(),
//         }
//     }
//     fn render(&self, precision: usize) -> anyhow::Result<Option<Table>> {
//         match self {
//             MatrixView::Base { data, client: _ } => data.render(precision),
//             MatrixView::Diff { data, client: _ } => data.render(precision),
//         }
//     }
//     fn nrows(&self) -> usize {
//         match self {
//             MatrixView::Base { data, client: _ } => data.nrows,
//             MatrixView::Diff { data, client: _ } => data.data.nrows,
//         }
//     }
//     fn ncols(&self) -> usize {
//         match self {
//             MatrixView::Base { data, client: _ } => data.ncols,
//             MatrixView::Diff { data, client: _ } => data.data.ncols,
//         }
//     }
// }

// -- Client Compare inner windows --------

const ITEM_HEIGHT: usize = 4;

#[derive(Debug)]
pub struct ClientCompare {
    pub vertical_scroll_state: ScrollbarState,
    pub horizontal_scroll_state: ScrollbarState,
    pub left_state: TableState,
    pub right_state: TableState,
    pub left: Option<(String, Matrix)>,
    pub right: Option<(String, DiffMatrix)>,
    pub show_line_input_popup: bool,
    pub popup_buffer: Vec<char>,
    pub popup_title: &'static str,
}

impl ClientCompare {
    pub fn update(
        &mut self,
        mat_ref: (Matrix, String),
        mat_diff: Option<(Matrix, String)>,
        tolerance: Option<f64>,
    ) {
        self.left_state.select_cell(Some((0, 0)));
        self.right_state.select_cell(Some((0, 0)));
        let max_nrows = mat_ref
            .0
            .nrows
            .max(mat_diff.as_ref().map(|(m, _)| m.nrows).unwrap_or_default());
        let max_ncols = mat_ref
            .0
            .ncols
            .max(mat_diff.as_ref().map(|(m, _)| m.ncols).unwrap_or_default());

        self.vertical_scroll_state = ScrollbarState::new(max_nrows * ITEM_HEIGHT);
        self.horizontal_scroll_state = ScrollbarState::new(max_ncols * ITEM_HEIGHT);

        if let Some((rmat, rname)) = mat_diff {
            let right = DiffMatrix::new(&mat_ref.0, rmat, tolerance);
            self.right.replace((rname, right));
        }

        let left = mat_ref.0;
        self.left.replace((mat_ref.1, left));
    }

    pub fn next_row(&mut self) {
        match (self.left.as_ref(), self.right.as_ref()) {
            (Some((_, l)), Some((_, r))) => {
                let max_nrows = l.nrows.max(r.data.nrows);
                let i = match self.left_state.selected() {
                    Some(i) => {
                        if i >= max_nrows - 1 {
                            0
                        } else {
                            i + 1
                        }
                    }
                    None => 0,
                };
                self.left_state.select(Some(i));
                let i = match self.right_state.selected() {
                    Some(i) => {
                        if i >= max_nrows - 1 {
                            0
                        } else {
                            i + 1
                        }
                    }
                    None => 0,
                };
                self.right_state.select(Some(i));
                self.vertical_scroll_state = self.vertical_scroll_state.position(i * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn previous_row(&mut self) {
        match (self.left.as_ref(), self.right.as_ref()) {
            (Some((_, l)), Some((_, r))) => {
                let max_nrows = l.nrows.max(r.data.nrows);
                let i = match self.left_state.selected() {
                    Some(i) => {
                        if i == 0 {
                            max_nrows - 1
                        } else {
                            i - 1
                        }
                    }
                    None => 0,
                };
                self.left_state.select(Some(i));
                let i = match self.right_state.selected() {
                    Some(i) => {
                        if i == 0 {
                            max_nrows - 1
                        } else {
                            i - 1
                        }
                    }
                    None => 0,
                };
                self.right_state.select(Some(i));
                self.vertical_scroll_state = self.vertical_scroll_state.position(i * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn col(&mut self, i: usize) {
        match (self.left.as_ref(), self.right.as_ref()) {
            (Some((_, l)), Some((_, r))) => {
                let max_ncols = l.ncols.max(r.data.ncols);
                if i < max_ncols {
                    self.left_state.select_column(Some(i));
                    self.right_state.select_column(Some(i));
                    self.horizontal_scroll_state =
                        self.horizontal_scroll_state.position(i * ITEM_HEIGHT);
                }
            }
            (_, _) => {}
        };
    }

    pub fn row(&mut self, i: usize) {
        match (self.left.as_ref(), self.right.as_ref()) {
            (Some((_, l)), Some((_, r))) => {
                let max_nrows = l.nrows.max(r.data.nrows);
                if i < max_nrows {
                    self.left_state.select(Some(i));
                    self.right_state.select(Some(i));
                    self.horizontal_scroll_state =
                        self.vertical_scroll_state.position(i * ITEM_HEIGHT);
                }
            }
            (_, _) => {}
        };
    }

    pub fn next_col(&mut self) {
        match (self.left.as_ref(), self.right.as_ref()) {
            (Some((_, l)), Some((_, r))) => {
                let max_ncols = l.ncols.max(r.data.ncols);
                let i = match self.left_state.selected_column() {
                    Some(i) => {
                        if i >= max_ncols - 1 {
                            0
                        } else {
                            i + 1
                        }
                    }
                    None => 0,
                };
                self.left_state.select_column(Some(i));
                let i = match self.right_state.selected_column() {
                    Some(i) => {
                        if i >= max_ncols - 1 {
                            0
                        } else {
                            i + 1
                        }
                    }
                    None => 0,
                };
                self.right_state.select_column(Some(i));
                self.horizontal_scroll_state =
                    self.horizontal_scroll_state.position(i * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }

    pub fn previous_col(&mut self) {
        match (self.left.as_ref(), self.right.as_ref()) {
            (Some((_, l)), Some((_, r))) => {
                let max_ncols = l.ncols.max(r.data.ncols);
                let i = match self.left_state.selected_column() {
                    Some(i) => {
                        if i == 0 {
                            max_ncols - 1
                        } else {
                            i - 1
                        }
                    }
                    None => 0,
                };
                self.left_state.select_column(Some(i));
                let i = match self.right_state.selected_column() {
                    Some(i) => {
                        if i == 0 {
                            max_ncols - 1
                        } else {
                            i - 1
                        }
                    }
                    None => 0,
                };
                self.right_state.select_column(Some(i));
                self.horizontal_scroll_state =
                    self.horizontal_scroll_state.position(i * ITEM_HEIGHT);
            }
            (_, _) => {}
        };
    }
}

impl Default for ClientCompare {
    fn default() -> Self {
        Self {
            left_state: TableState::default().with_selected(0),
            right_state: TableState::default().with_selected(0),
            horizontal_scroll_state: ScrollbarState::new(1 * ITEM_HEIGHT),
            vertical_scroll_state: ScrollbarState::new(1 * ITEM_HEIGHT),
            left: None,
            right: None,
            show_line_input_popup: false,
            popup_buffer: Vec::new(),
            popup_title: COMPARE_POPUP_TITLE_NOERR,
        }
    }
}

/// Application.
#[derive(Debug)]
pub struct App {
    pub running: bool,
    pub coordinator: Coordinator,
    pub history: History, // 0 var1; -1 var2 etc.
    pub wind_cursor: WindCursor,
    pub mode: Mode,
    pub tolerance: Tolerance,
    pub compare: ClientCompare,
    pub clients: Vec<String>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            running: true,
            coordinator: Coordinator::default(),
            history: History::default(),
            wind_cursor: WindCursor::default(),
            mode: Mode::default(),
            tolerance: Tolerance::default(),
            compare: ClientCompare::default(),
            clients: Vec::new(),
        }
    }
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
    /// Constructs a new instance of [`App`].
    pub fn new() -> Self {
        Self::default()
    }

    /// Handles the tick event of the terminal.
    pub fn tick(&self) {}

    /// Set running to false to quit the application.
    pub fn quit(&mut self) {
        self.running = false;
    }

    pub fn render_coordinator(&self) -> String {
        format!("{}", self.coordinator)
    }

    pub fn render_vs_overview(&self) -> String {
        format!(
            "{} vs {}",
            self.compare
                .left
                .as_ref()
                .map(|name| name.0.clone())
                .unwrap_or("<none>".to_owned()),
            self.compare
                .right
                .as_ref()
                .map(|name| name.0.clone())
                .unwrap_or("<none>".to_owned()),
        )
    }

    pub fn render_history(&self) -> String {
        format!("{}", self.history)
    }

    pub fn render_tolerance(&self) -> String {
        format!("{}", self.tolerance)
    }

    pub fn render_mode(&self) -> String {
        format!("{}", self.mode)
    }

    pub fn render_wind_cursor(&self) -> String {
        format!("{}", self.wind_cursor)
    }

    pub fn focus_right_rat(&mut self) {
        todo!()
    }

    pub fn send_lock_next(&mut self) {
        todo!()
    }
    pub fn send_unlock(&mut self) {
        todo!()
    }
    pub fn change_tolerance_at_current_cursor(&mut self, direction: VerticalDirection) {
        todo!()
    }
    pub fn change_tolerance_cursor(&mut self, direction: HorizontalDirection) {
        todo!()
    }
    pub fn change_history(&mut self, direction: VerticalDirection) {
        todo!()
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

    pub fn wind_cursor(&mut self, direction: Option<VerticalDirection>, line: Option<usize>) {
        // TODO add cursor and use windmode to check if needing to select
        todo!()
    }
    pub fn wind_fire_at_current_cursor(&mut self) {
        todo!()
    }

    // TODO change display to show what is selected and that mode is active
    pub fn wind_toggle_select(&mut self) {
        self.wind_cursor.mode = match self.wind_cursor.mode {
            WindMode::Active => WindMode::Active,
            WindMode::NormalCursor => WindMode::Select,
            WindMode::Select => WindMode::NormalCursor,
            WindMode::Inactive => WindMode::Inactive,
        };
    }

    pub fn compare_toggle_popup(&mut self) {
        self.compare.show_line_input_popup = !self.compare.show_line_input_popup;
    }

    pub fn wind_toggle_mode(&mut self) {
        self.wind_cursor.mode = match self.wind_cursor.mode {
            WindMode::Active => WindMode::NormalCursor,
            WindMode::NormalCursor => WindMode::Active,
            WindMode::Select => {
                self.wind_cursor.popup_title = WIND_POPUP_TITLE_NOERR;
                self.wind_cursor.popup_buffer.clear();
                self.wind_cursor.showing_popup = false;
                WindMode::Active
            }
            WindMode::Inactive => WindMode::Inactive,
        };
    }

    pub fn wind_mode(&mut self) -> bool {
        match self.wind_cursor.mode {
            WindMode::Active => false,
            WindMode::NormalCursor => true,
            WindMode::Select => true,
            WindMode::Inactive => false,
        }
    }

    pub fn wind_toggle_popup(&mut self) {
        self.wind_cursor.showing_popup = !self.wind_cursor.showing_popup;
    }
}
