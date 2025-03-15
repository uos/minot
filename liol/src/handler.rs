use crate::app::{App, AppResult, WIND_POPUP_TITLE_ERR, WIND_POPUP_TITLE_NOERR};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

/// Handles the key events and updates the state of [`App`].
pub fn handle_key_events(key_event: KeyEvent, app: &mut App) -> AppResult<()> {
    match key_event.code {
        // Exit application on `ESC` or `q`
        KeyCode::Esc | KeyCode::Char('q') => {
            if app.wind_mode() && app.wind_cursor.showing_popup {
                app.wind_cursor.showing_popup = false;
                app.wind_cursor.popup_title = WIND_POPUP_TITLE_NOERR;
                app.wind_cursor.popup_buffer.clear();
            } else {
                app.quit();
            }
        }
        // Exit application on `Ctrl-C`
        KeyCode::Char('c') | KeyCode::Char('C') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.quit();
            }
        }
        KeyCode::Char('F') => {
            app.focus_right_rat();
        }
        KeyCode::Char('l') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.send_lock_next();
            } else {
                if app.wind_mode() {
                    app.wind_toggle_popup();
                } else {
                    app.scroll_compare(Some(crate::app::HorizontalDirection::Right), None);
                }
            }
        }
        KeyCode::Char('h') => {
            app.scroll_compare(Some(crate::app::HorizontalDirection::Left), None);
        }
        KeyCode::Char('L') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.send_unlock();
            }
        }
        // unlock_until_next
        KeyCode::Char('n') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.send_unlock();
                app.send_lock_next();
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
        KeyCode::Char('P') => {
            app.change_tolerance_cursor(crate::app::HorizontalDirection::Right);
        }
        KeyCode::Char('k') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.change_history(crate::app::VerticalDirection::Up);
            } else {
                match app.wind_mode() {
                    false => app.scroll_compare(None, Some(crate::app::VerticalDirection::Up)),
                    true => {
                        app.wind_cursor(Some(crate::app::VerticalDirection::Up), None);
                    }
                }
            }
        }
        KeyCode::PageUp => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.change_history(crate::app::VerticalDirection::Up);
            }
        }
        KeyCode::Char('j') => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.change_history(crate::app::VerticalDirection::Down);
            } else {
                match app.wind_mode() {
                    false => app.scroll_compare(None, Some(crate::app::VerticalDirection::Down)),
                    true => {
                        app.wind_cursor(Some(crate::app::VerticalDirection::Down), None);
                    }
                }
            }
        }
        KeyCode::PageDown => {
            if key_event.modifiers == KeyModifiers::CONTROL {
                app.change_history(crate::app::VerticalDirection::Down);
            }
        }
        KeyCode::Char('v') => {
            if app.wind_mode() {
                app.wind_toggle_select();
            }
        }
        KeyCode::Char(' ') => {
            app.wind_fire_at_current_cursor();
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
                        app.wind_cursor(None, Some(p));
                        app.wind_cursor.popup_title = WIND_POPUP_TITLE_NOERR;
                        app.wind_cursor.showing_popup = false;
                        app.wind_cursor.popup_buffer.clear();
                    }
                    Err(_e) => {
                        app.wind_cursor.popup_title = WIND_POPUP_TITLE_ERR;
                    }
                }
            }
        }
        // --- Entering into popup --------
        KeyCode::Backspace
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
            }
        }
        _ => {}
    }
    Ok(())
}
