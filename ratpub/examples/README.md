# Ratpub Examples

This directory contains examples for using the `ratpub` library - Minot's native Publish/Subscribe system.

## Available Examples

- **pub.rs** - Publisher example that publishes messages to topics
- **sub.rs** - Subscriber example that receives messages from topics
- **smol.rs** - Example using the smol async runtime (requires `smol` feature)

## Running the Examples

### Basic Usage

First, start the coordinator:
```bash
cargo run --bin minot-coord
```

Then run the publisher and subscriber in separate terminals:
```bash
# Terminal 2
cargo run --example pub

# Terminal 3
cargo run --example sub
```

### Using Domain IDs for Network Isolation

If you have multiple Minot networks on the same physical network, use the `MINOT_DOMAIN_ID` environment variable to prevent them from connecting to each other:

```bash
# Network 1 (domain 0 - default)
minot-coord
cargo run --example pub
cargo run --example sub

# Network 2 (domain 1) - in different terminals
MINOT_DOMAIN_ID=1 minot-coord
MINOT_DOMAIN_ID=1 cargo run --example pub
MINOT_DOMAIN_ID=1 cargo run --example sub
```

**Important:** All nodes in the same network must use the same domain ID. Nodes with different domain IDs will not discover or communicate with each other.

Domain IDs range from 0-99, where:
- Domain 0 (default) uses port 6594
- Domain 1 uses port 6595
- Domain 2 uses port 6596
- ...and so on

## Notes

- The coordinator must be running before starting publishers or subscribers
- All nodes automatically discover each other via UDP broadcast when using the same domain ID
- When running with a non-default domain ID, you'll see a log message like: "Using domain ID 1 (port 6595)"
- For debugging, run with `RUST_LOG=debug` to see detailed connection information
