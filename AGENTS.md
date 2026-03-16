# Minot Project Overview

This document provides an overview of the Minot project structure and development guidelines for AI agents working with this codebase.

## Project Structure

The Minot project is organized into several components:

1. **Core Components**:
   - `minot` - Main executable and core logic
   - `mt_*` - Various specialized libraries (mt_pubsub, mt_rat, mt_net, etc.)

2. **Key Libraries**:
   - `mt_pubsub` - Publish/Subscribe implementation
   - `mt_rat` - Variable sharing library
   - `mt_net` - Networking layer
   - `mt_coord` - Coordination layer
   - `mt_sea` - Network communication layer

## Essential Commands

- **Build command**: `cargo build --release`
- **Test command**: `cargo test`
- **Run command**: `minot tui <file.mt>`

## Code Organization

The codebase is organized as a collection of Rust crates with specific responsibilities:

1. **minot** - Main executable crate
2. **mt_pubsub** - Publish/subscribe functionality
3. **mt_rat** - Variable sharing implementation
4. **mt_net** - Networking layer
5. **mt_coord** - Coordination layer
6. **mt_sea** - Network communication
7. **mt_service** - Service handling
8. **mt_bagread** - Bag file reading
9. **mt_wind** - Windowing system
10. **mt_scope** - Scope management
11. **mt_action** - Action implementation
12. **mt_mtc** - Core language processing

## Naming Conventions

- Library names follow the pattern `mt_*` to indicate they are Minot-specific
- All libraries use snake_case for function names
- Constants are in SCREAMING_SNAKE_CASE
- Public API functions are documented with rustdoc

## Testing Approach

Tests are implemented using Rust's built-in testing framework:
- Unit tests are colocated with the code in each module
- Integration tests are in the `tests/` directory of each crate
- All tests can be run with `cargo test` in the workspace root

## Important Gotchas

1. The project uses Rust's ownership model extensively
2. Message passing uses the actor model via channels
3. All network communication is handled through the `mt_net` layer
4. The system is designed to be used in a distributed fashion
5. C code generation is supported through the `mt` code generator