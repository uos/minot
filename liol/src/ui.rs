use core::panic;

use ratatui::{
    layout::{Constraint, Layout, Margin},
    style::{palette::tailwind, Modifier, Style},
    text::Line,
    widgets::{
        Block, BorderType, Borders, HighlightSpacing, Paragraph, Scrollbar, ScrollbarOrientation,
    },
    Frame,
};

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

    let selected_style = Style::default()
        .add_modifier(Modifier::BOLD)
        .bg(tailwind::GRAY.c500);

    if let Some((lname, lmat)) = app.compare.left.as_ref() {
        let ltable = lmat.render(app.tolerance.pot_cursor as usize).unwrap();
        if let Some(ltable) = ltable {
            let ltable = ltable
                .cell_highlight_style(selected_style)
                .highlight_spacing(HighlightSpacing::Always)
                .block(
                    Block::new()
                        .border_type(BorderType::Plain)
                        .borders(Borders::RIGHT),
                );
            frame.render_stateful_widget(ltable, client_base_area, &mut app.compare.left_state);
        }
    }

    if let Some((rname, rmat)) = app.compare.right.as_ref() {
        let rtable = rmat.render(app.tolerance.pot_cursor as usize).unwrap();
        if let Some(rtable) = rtable {
            let rtable = rtable
                .cell_highlight_style(selected_style)
                .highlight_spacing(HighlightSpacing::Always)
                .block(
                    Block::new()
                        .border_type(BorderType::Plain)
                        .borders(Borders::LEFT),
                );
            frame.render_stateful_widget(rtable, client_diff_area, &mut app.compare.right_state);
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
            .begin_symbol(None)
            .end_symbol(None),
        frame.area().inner(Margin {
            vertical: 1,
            horizontal: 1,
        }),
        &mut app.compare.horizontal_scroll_state,
    );

    if app.wind_cursor.showing_popup {
        let block = Block::bordered().title("Wind Cursor to Line:");
        let area = popup_area(frame.area(), 30, 3);
        let buffer = app.wind_cursor.popup_buffer.iter().collect::<String>();
        let content = Paragraph::new(buffer).block(block);
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
