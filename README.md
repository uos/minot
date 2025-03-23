# liolink

## TODOs
- load msgs from rosbag in tui
- impl rats lang bare bones
- impl wind for ros1 and ros2

** Nice to haves**
- Python bindings to C version. enables flexible client to manually "expect" values dynamically
- c version copies matrix because mapping to nalgebra did not work, maybe try mapping again
- only show scrollbars when size of content out of frame
- sometimes in big mat, data len is one bigger at some cols -- right now workaround by manual setting max_cols for matrix

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

- Go Right (newer): <Tab>
- Go Left (older): <Shift>+<Tab>


**Tolerance**

- Cursor to Left: p
- Cursor to Right: P

- Decrease at current Cursor: t
- Increase at current Cursor: T


**Controller**

- Lock all Rats for next action: 
- Unlock all Rats to continue:

- Go one step blocking: n

**Navigation**

- Scrolling Matrices: h,j,k,l | left,down,up,right
- Swap Focus with Diff Matrix: <shift>+f

- Next Rat to Diff: <PageUp>, <PageDown>

- Inc/Dec shown columns for Ref: ü,ä
- Inc/Dec shown columns for Diff: +,#
