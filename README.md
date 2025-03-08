# liolink

## TODOs
- when rat2.c is initialized first, it never finds the coordinator
- c version copies matrix because mapping to nalgebra did not work, maybe try mapping again

## Rats

Using C
```bash
cargo build
cd rat/examples
gcc rat2.c -o rat2 -L../../target/debug -lrat

RUST_LOG=debug ./rat2
```
