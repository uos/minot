use ratatui::{
    Frame,
    layout::{Constraint, Layout, Margin},
    style::{Modifier, Style, palette::tailwind},
    text::Line,
    widgets::{
        Block, BorderType, Borders, HighlightSpacing, Padding, Paragraph, Scrollbar,
        ScrollbarOrientation,
    },
};
use tui_logger::TuiLoggerWidget;

use crate::app::App;

pub fn render(app: &mut App, frame: &mut Frame) {
    let outer = Block::bordered()
        .border_type(BorderType::Rounded)
        .title_top(app.render_coordinator())
        .title_top(Line::from(app.render_vs_overview()).centered())
        .title_top(Line::from(app.render_history()).right_aligned())
        .title_bottom(app.render_tolerance())
        .title_bottom(Line::from(app.render_mode()).centered())
        .title_bottom(Line::from(app.render_wind_cursor()).right_aligned());

    let inner_layout =
        Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)]).margin(2);

    let [client_base_area, client_diff_area] = inner_layout.areas(frame.area());
    frame.render_widget(outer, frame.area());

    app.compare.update_rects(
        client_base_area.width,
        client_base_area.height,
        client_diff_area.width,
        client_diff_area.height,
    );

    let selected_style = Style::default()
        .add_modifier(Modifier::BOLD)
        .bg(tailwind::GRAY.c500);

    let curr_history = app.history.read().unwrap();
    let history_left_key = curr_history.key_of_ref();
    let compare_left = app.compare.left.as_ref();

    let updated_ref = match (history_left_key, compare_left) {
        (None, None) => false,
        (None, Some(_)) => {
            app.compare.update_ref(None, false);
            true
        }
        (Some(should_idx), None) => {
            if let Some((mat, _var)) = curr_history.mat_with_key(&should_idx) {
                app.compare.update_ref(Some((mat, should_idx)), true);
                true
            } else {
                false
            }
        }
        (Some(should_idx), Some((is_idx, _))) => {
            if should_idx != *is_idx {
                if let Some((mat, _var)) = curr_history.mat_with_key(&should_idx) {
                    app.compare.update_ref(Some((mat, should_idx)), true);
                    true
                } else {
                    false
                }
            } else {
                false
            }
        }
    };

    let history_right_key = curr_history.key_of_diff();
    let compare_right = app.compare.right.as_ref();

    let updated_diff = match (history_right_key, compare_right) {
        (None, None) => false,
        (None, Some(_)) => {
            app.compare.update_diff(None, None, true);
            true
        }
        (Some(should_idx), None) => {
            if let Some((mat, _var)) = curr_history.mat_with_key(&should_idx) {
                app.compare.update_diff(
                    Some((mat, should_idx)),
                    Some(f64::from(&app.tolerance)),
                    true,
                );
                true
            } else {
                false
            }
        }
        (Some(should_idx), Some((is_idx, _))) => {
            if should_idx != *is_idx {
                if let Some((mat, _var)) = curr_history.mat_with_key(&should_idx) {
                    app.compare.update_diff(
                        Some((mat, should_idx)),
                        Some(f64::from(&app.tolerance)),
                        true,
                    );
                    true
                } else {
                    false
                }
            } else {
                false
            }
        }
    };

    if updated_diff || updated_ref {
        app.compare.update_states();
    }

    let precision = if app.tolerance.pot_cursor > 0 {
        app.tolerance.pot_cursor - 1
    } else {
        app.tolerance.pot_cursor
    } as usize;

    // render reference
    if curr_history.shows_ref() {
        if let Some((_key, lmat)) = app.compare.left.as_ref() {
            let ltable = lmat.render(precision).unwrap();
            if let Some(ltable) = ltable {
                let ltable = ltable
                    .cell_highlight_style(selected_style)
                    .highlight_spacing(HighlightSpacing::Always)
                    .block(
                        Block::new()
                            .border_type(BorderType::Plain)
                            .borders(Borders::RIGHT)
                            .padding(Padding::horizontal(1)),
                    );
                frame.render_stateful_widget(ltable, client_base_area, &mut app.compare.left_state);
            }
        };
    }

    // render diff
    if curr_history.shows_diff() {
        if let Some((_key, rmat)) = app.compare.right.as_ref() {
            let rtable = rmat.render(precision).unwrap();
            if let Some(rtable) = rtable {
                let rtable = rtable
                    .cell_highlight_style(selected_style)
                    .highlight_spacing(HighlightSpacing::Always)
                    .block(
                        Block::new()
                            .border_type(BorderType::Plain)
                            .borders(Borders::LEFT)
                            .padding(Padding::horizontal(2)),
                    );
                frame.render_stateful_widget(
                    rtable,
                    client_diff_area,
                    &mut app.compare.right_state,
                );
            }
        }
    }

    frame.render_stateful_widget(
        Scrollbar::default()
            .orientation(ScrollbarOrientation::VerticalRight)
            .begin_symbol(None)
            .end_symbol(None),
        frame.area().inner(Margin {
            vertical: 1,
            horizontal: 1,
        }),
        &mut app.compare.vertical_scroll_state,
    );

    frame.render_stateful_widget(
        Scrollbar::default()
            .orientation(ScrollbarOrientation::HorizontalBottom)
            .thumb_symbol("🬋")
            .begin_symbol(None)
            .end_symbol(None),
        frame.area().inner(Margin {
            vertical: 1,
            horizontal: 1,
        }),
        &mut app.compare.horizontal_scroll_state,
    );

    if app.wind_cursor.read().unwrap().showing_popup {
        let block = Block::bordered().title("Wind Cursor to Line:");
        let area = popup_area(frame.area(), 30, 3);
        let buffer = app
            .wind_cursor
            .read()
            .unwrap()
            .popup_buffer
            .iter()
            .collect::<String>();
        let content = Paragraph::new(buffer).block(block);
        frame.render_widget(ratatui::widgets::Clear, area); //this clears out the background
        frame.render_widget(content, area);
    }

    if app.info_view.read().unwrap().shown {
        let block = Block::bordered().title("Info");
        let area = frame.area().inner(Margin {
            horizontal: 10,
            vertical: 5,
        });

        let content = TuiLoggerWidget::default()
            .style_error(Style::default().fg(ratatui::style::Color::Red))
            .style_warn(Style::default().fg(ratatui::style::Color::Yellow))
            .style_info(Style::default().fg(ratatui::style::Color::Blue))
            .style_debug(Style::default().fg(ratatui::style::Color::Magenta))
            .style_trace(Style::default().fg(ratatui::style::Color::Gray))
            .output_line(false)
            .output_file(false)
            .output_timestamp(Some("%H:%M:%S".to_owned()))
            .output_target(false)
            .output_separator(' ')
            .block(block);
        frame.render_widget(ratatui::widgets::Clear, area); //this clears out the background
        frame.render_widget(content, area);
    }
}

fn popup_area(area: ratatui::layout::Rect, x: u16, y: u16) -> ratatui::layout::Rect {
    let vertical = Layout::vertical([Constraint::Length(y)]).flex(ratatui::layout::Flex::Center);
    let horizontal =
        Layout::horizontal([Constraint::Length(x)]).flex(ratatui::layout::Flex::Center);
    let [area] = vertical.areas(area);
    let [area] = horizontal.areas(area);
    area
}
