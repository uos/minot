# Minot — VS Code Extension

Minot VS Code brings the Minot development and debugging tools into the editor. It provides controls to run the Minot coordinator, compile and execute Minot query scripts, and interact with the variable-sharing/locking controls from the status bar.

**Repository**: https://uos.github.io/minot/

## Features

- **Start/stop the Minot Coordinator**: spawn the `minot serve` process from your workspace or PATH and stream logs into an Output channel.
- **Compile & Execute**: send the current selection (or current line) of a Minot script to the running Coordinator to compile and execute.
- **Status bar controls and Hotkeys**: quick access buttons for Run Selection, Step, and the Lock toggle.
- **Automatic version check**: the extension verifies a compatible `minot` binary is available and respects the `engines.minot` requirement declared in the packaged `package.json`.

## Requirements

- `minot` CLI: a compatible `minot` executable must be available in your `PATH` or in the workspace root. The extension will call `minot --version` to detect the installed version.

If `minot` is missing the extension offers an installer and a link to the installation guide.

## Quick Start / Usage

1. Open the Minot workspace (the workspace root is used to locate a local `minot` binary when present).
2. Open a `.mt` script or any editor window:
	- Use the status bar `Run Selection` to compile & execute the selection or current line.
	- Use the lock toggle to `🔓` lock/unlock Minot loops. Use Step to advance the loop.

Commands provided by the extension (command IDs):

- `minot.compileAndExecute` — Compile and execute the current selection or line.
- `minot.restartServer` — Restart the running Minot server process.
- `minot.sendUnlock` — Send an Unlock command to the running Coordinator.
- `minot.sendLockNext` — Send a Lock command.
- `minot.sendLockNextPrevious` — Step once in your current loop.
- `minot.clearRules` — Clear all rules in the Coordinator.
- `minot.toggleLock` — Toggle between lock/unlock (status bar shortcut).

Keyboard Shortcuts:

- Run selection: suggested `Ctrl+Shift+Enter`
- Step: suggested `Ctrl+,`
- Unlock: suggested `Ctrl+.`
- Lock: suggested `Ctrl+,`

## Extension Settings

Currently none. Will be added with future releases.

## Release Notes

### 0.1.0

- Initial release: server management, compile & execute, lock controls. Minot compatible version: 0.1.0-rc.6

## Contributing

Contributions are welcome. Please open issues and pull requests on the main repository. For publishing changes, ensure `vscode/package.json` contains the correct `version` and `publisher` fields.
