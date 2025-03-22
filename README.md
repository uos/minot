# liolink

## TODOs
- use offsets for only printing matrix that fits on screen in rendering
- load msgs from rosbag in tui
- impl rats lang bare bones
- impl wind for ros1 and ros2
- [later] only show scrollbars when size of content out of frame
- c version copies matrix because mapping to nalgebra did not work, maybe try mapping again

## Rats

Using C
```bash
cargo build
cd rat/examples
gcc rat2.c -o rat2 -L../../target/debug -lrat

RUST_LOG=debug ./rat2
```

## LH Keybindings

**History**

Go Right (newer): <Tab>
Go Left (older): <Shift>+<Tab>


**Tolerance**

Cursor to Left: p
Cursor to Right: P

Decrease at current Cursor: t
Increase at current Cursor: T


**Controller**

Lock all Rats for next action: 
Unlock all Rats to continue:

Go one step blocking: n

**Navigation**

Scrolling Matrices: h,j,k,l | left,down,up,right
Swap Focus with Diff Matrix: <shift>+f

Next Rat to Diff: <PageUp>, <PageDown>
