//! The log pane: a scrollable, wrapping view of a [`LogBuffer`]'s entries.
//!
//! The pane knows nothing about which entries deserve to be on screen. The
//! caller filters — by target, by source tag, by whatever its own toggles
//! say — and hands over what is left, each entry paired with the index it has
//! in the caller's own list. Scroll positions are entry indices, so a
//! filtered-out entry must not renumber the rest.
//!
//! [`LogBuffer`]: crate::LogBuffer

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem},
};

use crate::buffer::LogEntry;
use crate::format::LogLevel;

/// Lines a page key moves.
const PAGE_LINES: usize = 20;
/// A subtle one-column inset, without putting a border around the pane.
const LEFT_GUTTER: usize = 1;
/// Below this pane width the header is compacted: the level shrinks to its
/// initial and wrapped text uses a two-column hanging indent. Wide panes keep
/// full alignment. Narrow panes preserve more message width.
const COMPACT_WIDTH: u16 = 72;
/// Columns the "↳ " marker on a wrapped line occupies.
const HANGING_INDENT: usize = 2;

/// The colour a level is drawn in.
pub fn level_color(level: LogLevel) -> Color {
    match level {
        LogLevel::Trace => Color::DarkGray,
        LogLevel::Debug => Color::Gray,
        LogLevel::Info => Color::Cyan,
        LogLevel::Warn => Color::Yellow,
        LogLevel::Error => Color::Red,
    }
}

/// Scroll state for the log pane.
///
/// The scroll position uses an *entry* index, so wrapping a long message keeps
/// the reader's place.
#[derive(Default)]
pub struct LogView {
    /// `None` follows the tail. `Some` holds a pinned entry index.
    scroll: Option<usize>,
}

impl LogView {
    pub fn following(&self) -> bool {
        self.scroll.is_none()
    }

    /// Leaving the tail pins the entry one above it, so the first key press
    /// stops the log from scrolling out from under the reader.
    pub fn scroll_up(&mut self, total: usize) {
        match self.scroll {
            None if total > 1 => self.scroll = Some(total.saturating_sub(2)),
            Some(position) if position > 0 => self.scroll = Some(position - 1),
            _ => {}
        }
    }

    /// Reaching the bottom resumes following the newest entry.
    pub fn scroll_down(&mut self, total: usize) {
        if let Some(position) = self.scroll {
            if position + 1 >= total.saturating_sub(1) {
                self.scroll = None;
            } else {
                self.scroll = Some(position + 1);
            }
        }
    }

    pub fn page_up(&mut self, total: usize) {
        for _ in 0..PAGE_LINES {
            self.scroll_up(total);
        }
    }

    pub fn page_down(&mut self, total: usize) {
        for _ in 0..PAGE_LINES {
            self.scroll_down(total);
        }
    }

    pub fn to_start(&mut self) {
        self.scroll = Some(0);
    }

    pub fn to_end(&mut self) {
        self.scroll = None;
    }
}

/// Split a message into chunks that fit the pane, never inside a character.
///
/// The first line has the header beside it and the rest sit under a shorter
/// indent, so the two widths are given separately. Long
/// single-token lines still have to break somewhere, so a word wider than the
/// pane uses a hard split to stay within its width.
pub fn wrap(message: &str, first_width: usize, rest_width: usize) -> Vec<String> {
    let first_width = first_width.max(1);
    let rest_width = rest_width.max(1);
    let mut lines: Vec<String> = Vec::new();
    let mut current = String::new();
    let mut current_width = 0;
    let mut width = first_width;

    for word in message.split_inclusive(' ') {
        // A trailing space is trimmed off a break, so it never has to fit.
        let fitted = word.trim_end().chars().count();
        if current_width + fitted > width && current_width > 0 {
            lines.push(current.trim_end().to_owned());
            current.clear();
            current_width = 0;
            width = rest_width;
        }
        if fitted > width {
            for character in word.chars() {
                if current_width >= width {
                    lines.push(std::mem::take(&mut current));
                    current_width = 0;
                    width = rest_width;
                }
                current.push(character);
                current_width += 1;
            }
            continue;
        }
        current.push_str(word);
        current_width += word.chars().count();
    }
    if !current.is_empty() || lines.is_empty() {
        lines.push(current.trim_end().to_owned());
    }
    lines
}

/// Draw the pane over an already-filtered set of entries.
///
/// `title` is the caller's, so it can say what its own filters are hiding.
pub fn render(
    frame: &mut ratatui::Frame,
    area: Rect,
    visible: &[(usize, &LogEntry)],
    view: &LogView,
    title: &str,
) {
    // A narrow pane cannot afford the full header: seven columns of level tag
    // and an indent that lines wrapped text up under the message would leave
    // the message itself with a third of the line.
    let compact = area.width < COMPACT_WIDTH;
    let stamp = |timestamp: f64| {
        if compact {
            format!("{timestamp:.1}s")
        } else {
            format!("{timestamp:.2}s")
        }
    };
    // A floor keeps the message column from shifting as the run passes 10s
    // and 100s. After that the column grows with the timestamps.
    let floor = if compact { 5 } else { 8 };
    let time_width = visible
        .iter()
        .map(|(_, entry)| stamp(entry.timestamp).len())
        .max()
        .unwrap_or(floor)
        .max(floor);
    let level_width = if compact { 3 } else { 7 };
    let height = area.height.saturating_sub(1) as usize;
    // One column goes to the selector, prepended when the items are built.
    let inner = (area.width as usize).saturating_sub(1);
    // "<gutter><time> <level> ".
    let header_width = LEFT_GUTTER + time_width + 1 + level_width + 1;
    let continuation_width = if compact {
        LEFT_GUTTER + HANGING_INDENT
    } else {
        header_width + HANGING_INDENT
    };
    let first_width = inner.saturating_sub(header_width);
    let rest_width = inner.saturating_sub(continuation_width);
    let left_gutter = " ".repeat(LEFT_GUTTER);
    let continuation = " ".repeat(continuation_width - LEFT_GUTTER - HANGING_INDENT);

    let mut lines: Vec<(usize, Line)> = Vec::new();
    for (index, entry) in visible {
        let message_color = match entry.level {
            LogLevel::Error | LogLevel::Warn => level_color(entry.level),
            _ => Color::White,
        };
        let time = format!("{:>time_width$}", stamp(entry.timestamp));
        let level = if compact {
            entry.level.badge()
        } else {
            entry.level.tag()
        };
        for (part, chunk) in wrap(&entry.message, first_width, rest_width)
            .into_iter()
            .enumerate()
        {
            let line = if part == 0 {
                Line::from(vec![
                    Span::raw(left_gutter.clone()),
                    Span::styled(time.clone(), Style::default().fg(Color::DarkGray)),
                    Span::raw(" "),
                    Span::styled(
                        level,
                        Style::default()
                            .fg(level_color(entry.level))
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw(" "),
                    Span::styled(chunk, Style::default().fg(message_color)),
                ])
            } else {
                Line::from(vec![
                    Span::raw(left_gutter.clone()),
                    Span::raw(continuation.clone()),
                    Span::styled("↳ ", Style::default().fg(Color::DarkGray)),
                    Span::styled(chunk, Style::default().fg(message_color)),
                ])
            };
            lines.push((*index, line));
        }
    }

    let total_lines = lines.len();
    let (start, following, selected) = match view.scroll {
        None => (total_lines.saturating_sub(height), true, None),
        Some(entry) => {
            let line = lines
                .iter()
                .position(|(index, _)| *index >= entry)
                .unwrap_or(0);
            (
                line.min(total_lines.saturating_sub(height)),
                false,
                Some(entry),
            )
        }
    };
    let end = (start + height).min(total_lines);

    let items = lines
        .get(start..end)
        .unwrap_or(&[])
        .iter()
        .map(|(index, line)| {
            let mut spans = Vec::with_capacity(line.spans.len() + 1);
            if selected == Some(*index) {
                spans.push(Span::styled(
                    "▶",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ));
                spans.extend(line.spans.iter().cloned());
                ListItem::new(Line::from(spans)).style(Style::default().bg(Color::DarkGray))
            } else {
                spans.push(Span::raw(" "));
                spans.extend(line.spans.iter().cloned());
                ListItem::new(Line::from(spans))
            }
        })
        .collect::<Vec<_>>();

    let position = if following {
        format!(" [AUTO {} entries] ", visible.len())
    } else {
        let furthest = total_lines.saturating_sub(height);
        let percent = if furthest > 0 {
            (start as f64 / furthest as f64) * 100.0
        } else {
            100.0
        };
        format!(" [line {}/{total_lines} {percent:.0}%] ", start + 1)
    };

    let block = Block::default()
        .borders(Borders::NONE)
        .title(Span::styled(
            title,
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        ))
        .title_bottom(Span::styled(
            position,
            Style::default().fg(if following {
                Color::Cyan
            } else {
                Color::Yellow
            }),
        ));
    frame.render_widget(List::new(items).block(block), area);
}
