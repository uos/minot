# Lighthouse

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

**Wind**

- Toggle wind mode: w
- Cursor up/down: k/j
- toggle select (in wind mode): v
