use ratatui::{
    layout::{Constraint, Layout},
    text::Line,
    widgets::{Block, BorderType, Borders, Paragraph},
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

    let client_base_paragraph = Paragraph::new(app.render_left_client());
    let client_diff_paragraph = Paragraph::new(app.render_right_client());

    frame.render_widget(outer, frame.area());
    frame.render_widget(
        client_base_paragraph.block(
            Block::new()
                .border_type(BorderType::Plain)
                .borders(Borders::RIGHT),
        ),
        client_base_area,
    );
    frame.render_widget(
        client_diff_paragraph.block(
            Block::new()
                .border_type(BorderType::Plain)
                .borders(Borders::LEFT),
        ),
        client_diff_area,
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
