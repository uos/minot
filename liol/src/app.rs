use std::{
    collections::LinkedList,
    error,
    fmt::{Display, Write},
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

fn create_underlined_string(tolerance: f64, cursor: u8) -> String {
    let decimal_places = if cursor == 0 { 0 } else { cursor as usize };
    let formatted_tolerance = format!("{:.1$}", tolerance, decimal_places);
    let chars: Vec<char> = formatted_tolerance.chars().collect();
    let cursor_index = if cursor > 0 {
        formatted_tolerance.find('.').unwrap_or(0) + cursor as usize
    } else {
        0
    };

    let mut result = String::new();
    for (i, &c) in chars.iter().enumerate() {
        if i == cursor_index {
            if i == 0 {
                result.push_str(&format!("{} ", c));
            } else if i == chars.len() - 1 {
                result.push_str(&format!(" {}", c));
            } else {
                result.push_str(&format!(" {} ", c));
            }
        } else {
            result.push(c);
        }
    }

    result
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
pub const WIND_POPUP_TITLE_ERR: &'static str = "Could not parse! Please correct:";

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
#[derive(Debug)]
pub struct Matrix {
    data: Vec<f64>,
}

impl Default for Matrix {
    fn default() -> Self {
        Self { data: vec![0.05] }
    }
}

#[derive(Debug)]
pub enum MatrixView {
    Base {
        data: Matrix,
        client: String,
    },
    Diff {
        base: Matrix,
        data: Matrix,
        client: String,
    },
}

impl MatrixView {
    pub fn client_name(&self) -> String {
        match self {
            MatrixView::Base { data, client } => client.clone(),
            MatrixView::Diff { base, data, client } => client.clone(),
        }
    }
}

impl Display for MatrixView {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MatrixView::Base { data, client } => {
                for el in data.data.iter() {
                    f.write_str("Base\n")?;
                    f.write_str(&el.to_string())?;
                    f.write_str("\n")?;
                }
            }
            MatrixView::Diff { base, data, client } => {
                for el in data.data.iter() {
                    f.write_str("diff\n")?;
                    f.write_str(&el.to_string())?;
                    f.write_str("\n")?;
                }
            }
        }
        Ok(())
    }
}

// -- Client Compare inner windows --------
#[derive(Debug)]
pub struct ClientCompare {
    pub left: MatrixView,
    pub right: MatrixView,
}

impl Default for ClientCompare {
    fn default() -> Self {
        Self {
            left: MatrixView::Base {
                data: Matrix::default(),
                client: "<none>".to_owned(),
            },
            right: MatrixView::Diff {
                base: Matrix::default(),
                data: Matrix::default(),
                client: "<none>".to_owned(),
            },
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
    /// counter
    pub counter: u8,
}

impl Default for App {
    fn default() -> Self {
        Self {
            running: true,
            counter: 0,
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
}

pub enum VerticalDirection {
    Up,
    Down,
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
            self.compare.left.client_name(),
            self.compare.right.client_name()
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

    // TODO make as scrollable table horizontal and vertical.
    pub fn render_right_client(&self) -> String {
        format!("{}", self.compare.right)
    }

    // TODO make as scrollable table horizontal and vertical.
    pub fn render_left_client(&self) -> String {
        format!("{}", self.compare.left)
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
        todo!()
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
