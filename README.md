# Lighthouse

A toolchain for building and testing distributed robot systems with ROS1 and ROS2.

## Use cases
* Debugging LIO/SLAM systems
* Synchronous Testing of Nodes
* ROS1 -> ROS2 migrations
* Language migrations
* Method evaluation (functional and performance)
* Fine-grained Bagfile publishing

## Syncing with C

```bash
cargo build
cd rat/examples
gcc rat2.c -o rat2 -L../../target/debug -lrat

RUST_LOG=debug ./rat2
```

## LH TUI Keybindings

### Basics

**Quit** = q *or* <kbd>Esc</kbd> *or* <kbd>Ctrl</kbd>+c

**Toggle Info Window** = ?

### Wind Cursor

**Toggle Mode** = w

**Toggle Popup** (in wind mode) = l

**Apply Current Input in Popup** = <kbd>Enter</kbd>

<br>

**Move Cursor** (in wind mode)
- **up** = k *or* <kbd>↑</kbd>
- **down** = j *or* <kbd>↓</kbd>

<br>

**Toggle Select Mode** (in wind mode) = v

**Compile and Run Wind Selection** (in wind mode) = <kbd>Space</kbd>

### Matrix Compare Window

**Move Cursor** (*not* in wind mode)
- **left** = h *or* <kbd>←</kbd>
- **up** = k *or* <kbd>↑</kbd>
- **down** = j *or* <kbd>↓</kbd>
- **right** = l *or* <kbd>→</kbd> 

<br>


**Toggle Cursor Popup** = <kbd>Shift</kbd>+l

**Apply Current Input in Popup** = <kbd>Enter</kbd>

**Scroll History**
- **older** = <kbd>Shift</kbd>+<kbd>Tab</kbd>
- **newer** = <kbd>Tab</kbd>

**Swap Compare Buffers** = <kbd>Shift</kbd>+f

**Move Through Rats at Current Var** = <kbd>PageUp</kbd> / <kbd>PageDown</kbd>

<br>


**unlock next var** = <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+l

**lock next var** = <kbd>Ctrl</kbd>+l

**unlock until next var** = <kbd>Ctrl</kbd>+n

#### Difference Tolerance

**Tolerance at Current Cursor**
- **increase** = <kbd>Shift</kbd>+t
- **decrease** = t
<br>


**Move Tolerance Cursor**
- **left** = p
- **right** = <kbd>Shift</kbd>+p
<br>



**Left Window Column Width**
- **increase** = <kbd>Ctrl</kbd>+q
- **decrease** = <kbd>Ctrl</kbd>+a
<br>


**Right Window Column Width**
- **increase** = <kbd>Ctrl</kbd>+w
- **decrease** = <kbd>Ctrl</kbd>+s

