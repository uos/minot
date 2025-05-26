# Keybindings

Keybindings for the Minot TUI. They cannot be changed after compile time for the moment.

---

## Basics

| Command           | Key                                   |
| :---------------- | :------------------------------------ |
| Quit              | <kbd>q</kbd>, <kbd>Esc</kbd>, <kbd>Ctrl</kbd>+<kbd>c</kbd> |
| Toggle Log Window | <kbd>?</kbd>                          |
| Switch Mode (Wind/Compare)         | <kbd>w</kbd>    |

---

## Wind Cursor

| Command                            | Key         |
| :--------------------------------- | :---------- |
| Move Cursor up                     | <kbd>k</kbd>, <kbd>↑</kbd> |
| Move Cursor down                   | <kbd>j</kbd>, <kbd>↓</kbd> |
| Toggle Goto Line Window (in wind mode)           | <kbd>g</kbd>     |
| Apply Goto Line Input              | <kbd>Enter</kbd> |
| Select All Lines                   | <kbd>*</kbd>    |
| Toggle Select Mode (in wind mode)                | <kbd>v</kbd>     |
| Compile and Run Selection (in wind mode)         | <kbd>Space</kbd>  |

---

## Rules

| Command                    | Key       |
| :------------------------- | :-------- |
| Clear (in compare mode)                     | <kbd>Space</kbd>  |

Adding new ones happens automatically when firing the `.rl` file if it contains some.

---

## Matrix Compare Window

| Command                             | Key                                   |
| :---------------------------------- | :------------------------------------ |
| Move Cursor Left (in compare mode)                   | <kbd>h</kbd>, <kbd>←</kbd>  |
| Move Cursor Up (in compare mode)                     | <kbd>k</kbd>, <kbd>↑</kbd> |
| Move Cursor Down (in compare mode)                   | <kbd>j</kbd>, <kbd>↓</kbd> |
| Move Cursor Right (in compare mode)                  | <kbd>l</kbd>, <kbd>→</kbd> |
| Toggle Cursor Popup (in compare mode)                | <kbd>g</kbd>         |
| Apply Current Input in Popup        | <kbd>Enter</kbd>                      |
| Scroll History Older                | <kbd>Shift</kbd>+<kbd>Tab</kbd>      |
| Scroll History Newer                | <kbd>Tab</kbd>                        |
| Swap Compare Buffers                | <kbd>Shift</kbd>+<kbd>f</kbd>        |
| Move Through Rats at Current Var    | <kbd>PageUp</kbd> / <kbd>PageDown</kbd> |
| Unlock Vars                         | <kbd>.</kbd>                         |
| Lock Next Var                       | <kbd>-</kbd>                         |
| Unlock Until Next Var (Step forward)| <kbd>,</kbd>                         |
| Increase Left Window Column Width   | <kbd>Ctrl</kbd>+<kbd>q</kbd>         |
| Decrease Left Window Column Width   | <kbd>Ctrl</kbd>+<kbd>a</kbd>         |
| Increase Right Window Column Width  | <kbd>Ctrl</kbd>+<kbd>w</kbd>         |
| Decrease Right Window Column Width  | <kbd>Ctrl</kbd>+<kbd>s</kbd>         |

---

## Difference Tolerance

| Command                             | Key                                   |
| :---------------------------------- | :------------------------------------ |
| Increase at Current Cursor          | <kbd>Shift</kbd>+<kbd>t</kbd>         |
| Decrease at Current Cursor          | <kbd>t</kbd>                          |
| Move Cursor Left                    | <kbd>p</kbd>                          |
| Move Cursor Right                   | <kbd>Shift</kbd>+<kbd>p</kbd>        |
