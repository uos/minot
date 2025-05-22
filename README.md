# minot

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

## Minot TUI Keybindings

### Basics

**Quit** = q *or* <kbd>Esc</kbd> *or* <kbd>Ctrl</kbd>+c

**Toggle Info Window** = ?

### Wind Cursor

**Toggle Mode** = w

**Toggle Manual Enter** (in wind mode) = g

**Apply Current Input in Popup** = <kbd>Enter</kbd>

<br>

**Move Cursor** (in wind mode)
- **up** = k *or* <kbd>↑</kbd>
- **down** = j *or* <kbd>↓</kbd>

<br>

**Select Entire File** = *

**Toggle Select Mode** (in wind mode) = v

**Compile and Run Wind Selection** (in wind mode) = <kbd>Space</kbd>

### Rules

**Clear** = <kbd>Space</kbd> (in compare mode)

Adding new ones happens automatically when firing the .rl file if it contains some.

### Matrix Compare Window

**Move Cursor** (in compare mode)
- **left** = h *or* <kbd>←</kbd>
- **up** = k *or* <kbd>↑</kbd>
- **down** = j *or* <kbd>↓</kbd>
- **right** = l *or* <kbd>→</kbd> 

<br>


**Toggle Cursor Popup** = g (in compare mode)

**Apply Current Input in Popup** = <kbd>Enter</kbd>

**Scroll History**
- **older** = <kbd>Shift</kbd>+<kbd>Tab</kbd>
- **newer** = <kbd>Tab</kbd>

**Swap Compare Buffers** = <kbd>Shift</kbd>+f

**Move Through Rats at Current Var** = <kbd>PageUp</kbd> / <kbd>PageDown</kbd>

<br>


**Unlock Next Var** = <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+l

**Lock Next Var** = <kbd>Ctrl</kbd>+l

**Unlock Until Next Var** = <kbd>Ctrl</kbd>+n

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

