use crate::app::{
    App, AppResult, COMPARE_POPUP_TITLE_NOERR, POPUP_TITLE_ERR, WIND_POPUP_TITLE_NOERR,
};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

/// Handles the key events and updates the state of [`App`].
pub async fn handle_key_events(key_event: KeyEvent, app: &mut App) -> AppResult<()> {
    match key_event.code {
        // Exit application on `ESC` or `q`
        KeyCode::Esc | KeyCode::Char('q') => {
            if app.wind_mode() && app.wind_cursor.showing_popup {
                app.wind_cursor.showing_popup = false;
                app.wind_cursor.popup_title = WIND_POPUP_TITLE_NOERR;
                app.wind_cursor.popup_buffer.clear();
            } else {
                if app.compare.show_line_input_popup {
                    app.compare.show_line_input_popup = false;
                    app.compare.popup_title = COMPARE_POPUP_TITLE_NOERR;
                    app.compare.popup_buffer.clear();
                } else {
                    app.quit();
                }
            }
        }
        // Exit application on `Ctrl-C`
        KeyCode::Char('c') | KeyCode::Char('C') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.quit();
            }
        }
        KeyCode::Char('?') => {
            app.toggle_info_window();
        }
        KeyCode::Char('F') => {
            app.focus_right_rat();
        }
        KeyCode::Char('l') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.send_lock_next().await;
            } else {
                if app.wind_mode() {
                    app.wind_toggle_popup();
                } else {
                    app.scroll_compare(Some(crate::app::HorizontalDirection::Right), None);
                }
            }
        }
        KeyCode::Right => {
            app.scroll_compare(Some(crate::app::HorizontalDirection::Right), None);
        }
        KeyCode::Char('h') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.change_history(crate::app::HorizontalDirection::Left);
            } else {
                app.scroll_compare(Some(crate::app::HorizontalDirection::Left), None);
            }
        }
        KeyCode::Left => {
            app.scroll_compare(Some(crate::app::HorizontalDirection::Left), None);
        }
        KeyCode::Char('L') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.send_unlock().await;
            } else {
                app.compare_toggle_popup();
            }
        }
        // unlock_until_next
        KeyCode::Char('n') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.send_unlock().await;
                app.send_lock_next().await;
            }
        }
        KeyCode::Char('w') => {
            app.wind_toggle_mode();
        }
        KeyCode::Char('t') => {
            app.change_tolerance_at_current_cursor(crate::app::VerticalDirection::Down);
        }
        KeyCode::Char('T') => {
            app.change_tolerance_at_current_cursor(crate::app::VerticalDirection::Up);
        }
        KeyCode::Char('p') => {
            app.change_tolerance_cursor(crate::app::HorizontalDirection::Left);
        }

        // TODO non-freedom chars
        KeyCode::Char('ü') => {
            app.change_cols_ref(crate::app::VerticalDirection::Up);
        }
        KeyCode::Char('ä') => {
            app.change_cols_ref(crate::app::VerticalDirection::Down);
        }
        KeyCode::Char('+') => {
            app.change_cols_diff(crate::app::VerticalDirection::Up);
        }
        KeyCode::Char('#') => {
            app.change_cols_diff(crate::app::VerticalDirection::Down);
        }

        KeyCode::Char('P') => {
            app.change_tolerance_cursor(crate::app::HorizontalDirection::Right);
        }
        KeyCode::Char('k') => match app.wind_mode() {
            false => app.scroll_compare(None, Some(crate::app::VerticalDirection::Up)),
            true => {
                app.wind_cursor(crate::app::VerticalDirection::Up);
            }
        },
        KeyCode::Tab => {
            app.change_history(crate::app::HorizontalDirection::Right);
        }
        KeyCode::BackTab => {
            app.change_history(crate::app::HorizontalDirection::Left);
        }
        KeyCode::Char('j') => match app.wind_mode() {
            false => app.scroll_compare(None, Some(crate::app::VerticalDirection::Down)),
            true => {
                app.wind_cursor(crate::app::VerticalDirection::Down);
            }
        },
        KeyCode::Down => match app.wind_mode() {
            false => app.scroll_compare(None, Some(crate::app::VerticalDirection::Down)),
            true => {
                app.wind_cursor(crate::app::VerticalDirection::Down);
            }
        },
        KeyCode::Up => match app.wind_mode() {
            false => app.scroll_compare(None, Some(crate::app::VerticalDirection::Up)),
            true => {
                app.wind_cursor(crate::app::VerticalDirection::Up);
            }
        },
        KeyCode::Char('v') => {
            if app.wind_mode() {
                app.wind_toggle_select();
            }
        }
        KeyCode::Char(' ') => {
            app.wind_fire_at_current_cursor().await;
        }
        KeyCode::PageUp => {
            app.change_diff_rat(crate::app::VerticalDirection::Up);
        }
        KeyCode::PageDown => {
            app.change_diff_rat(crate::app::VerticalDirection::Down);
        }
        KeyCode::Enter => {
            if app.wind_mode() && app.wind_cursor.showing_popup {
                let buffer = app.wind_cursor.popup_buffer.clone();
                let s: String = buffer.into_iter().collect();

                let parsed = match s.parse::<usize>() {
                    Ok(val) => Ok(val),
                    Err(_) => Err(format!("Failed to parse '{}' into unsigned integer", s)),
                };

                match parsed {
                    Ok(p) => {
                        app.wind_cursor(crate::app::VerticalDirection::Row(p));
                        app.wind_cursor.popup_title = WIND_POPUP_TITLE_NOERR;
                        app.wind_cursor.showing_popup = false;
                        app.wind_cursor.popup_buffer.clear();
                    }
                    Err(_e) => {
                        app.wind_cursor.popup_title = POPUP_TITLE_ERR;
                    }
                }
            } else {
                if app.compare.show_line_input_popup {
                    let buffer = app.compare.popup_buffer.clone();
                    let s: String = buffer.into_iter().collect();

                    let (parsed_row, parsed_col) = if let Some(start_col) = s.rfind(':') {
                        let (row, col) = s.split_at(start_col);

                        let parsed_row = match row.parse::<usize>() {
                            Ok(val) => Ok(val),
                            Err(_) => Err(format!("Failed to parse '{}' into unsigned integer", s)),
                        };
                        let parsed_col = match col.parse::<usize>() {
                            Ok(val) => Ok(val),
                            Err(_) => Err(format!("Failed to parse '{}' into unsigned integer", s)),
                        };
                        (Some(parsed_row), Some(parsed_col))
                    } else {
                        let parsed_row = match s.parse::<usize>() {
                            Ok(val) => Ok(val),
                            Err(_) => Err(format!("Failed to parse '{}' into unsigned integer", s)),
                        };
                        (Some(parsed_row), None)
                    };

                    let succ = match (parsed_row, parsed_col) {
                        (None, None) => true,
                        (Some(Ok(row)), None) => {
                            app.scroll_compare(None, Some(crate::app::VerticalDirection::Row(row)));
                            true
                        }
                        (Some(Ok(row)), Some(Ok(col))) => {
                            app.scroll_compare(
                                Some(crate::app::HorizontalDirection::Col(col)),
                                Some(crate::app::VerticalDirection::Row(row)),
                            );
                            true
                        }
                        (_, _) => {
                            app.wind_cursor.popup_title = POPUP_TITLE_ERR;
                            false
                        }
                    };

                    if succ {
                        app.compare.popup_title = WIND_POPUP_TITLE_NOERR;
                        app.compare.show_line_input_popup = false;
                        app.compare.popup_buffer.clear();
                    }
                }
            }
        }
        // --- Entering into popup --------
        KeyCode::Backspace
        | KeyCode::Char(':')
        | KeyCode::Char('1')
        | KeyCode::Char('2')
        | KeyCode::Char('3')
        | KeyCode::Char('4')
        | KeyCode::Char('5')
        | KeyCode::Char('6')
        | KeyCode::Char('7')
        | KeyCode::Char('8')
        | KeyCode::Char('9')
        | KeyCode::Char('0') => {
            if app.wind_mode() && app.wind_cursor.showing_popup {
                match key_event.code {
                    KeyCode::Backspace => {
                        app.wind_cursor.popup_buffer.pop();
                    }
                    KeyCode::Char('1') => {
                        app.wind_cursor.popup_buffer.push('1');
                    }
                    KeyCode::Char('2') => {
                        app.wind_cursor.popup_buffer.push('2');
                    }
                    KeyCode::Char('3') => {
                        app.wind_cursor.popup_buffer.push('3');
                    }
                    KeyCode::Char('4') => {
                        app.wind_cursor.popup_buffer.push('4');
                    }
                    KeyCode::Char('5') => {
                        app.wind_cursor.popup_buffer.push('5');
                    }
                    KeyCode::Char('6') => {
                        app.wind_cursor.popup_buffer.push('6');
                    }
                    KeyCode::Char('7') => {
                        app.wind_cursor.popup_buffer.push('7');
                    }
                    KeyCode::Char('8') => {
                        app.wind_cursor.popup_buffer.push('8');
                    }
                    KeyCode::Char('9') => {
                        app.wind_cursor.popup_buffer.push('9');
                    }
                    KeyCode::Char('0') => {
                        app.wind_cursor.popup_buffer.push('0');
                    }
                    _ => {}
                }
            } else {
                if app.compare.show_line_input_popup {
                    match key_event.code {
                        KeyCode::Backspace => {
                            app.compare.popup_buffer.pop();
                        }
                        KeyCode::Char('1') => {
                            app.compare.popup_buffer.push('1');
                        }
                        KeyCode::Char('2') => {
                            app.compare.popup_buffer.push('2');
                        }
                        KeyCode::Char('3') => {
                            app.compare.popup_buffer.push('3');
                        }
                        KeyCode::Char('4') => {
                            app.compare.popup_buffer.push('4');
                        }
                        KeyCode::Char('5') => {
                            app.compare.popup_buffer.push('5');
                        }
                        KeyCode::Char('6') => {
                            app.compare.popup_buffer.push('6');
                        }
                        KeyCode::Char('7') => {
                            app.compare.popup_buffer.push('7');
                        }
                        KeyCode::Char('8') => {
                            app.compare.popup_buffer.push('8');
                        }
                        KeyCode::Char('9') => {
                            app.compare.popup_buffer.push('9');
                        }
                        KeyCode::Char('0') => {
                            app.compare.popup_buffer.push('0');
                        }
                        KeyCode::Char(':') => {
                            app.compare.popup_buffer.push(':');
                        }
                        _ => {}
                    }
                }
            }
        }
        _ => {}
    }
    Ok(())
}
