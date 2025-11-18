import * as vscode from 'vscode';
import * as cp from 'child_process';
import * as fs from 'fs';
import * as path from 'path';

// Global variables for managing the server
let serverProcess: cp.ChildProcess | null = null;
let outputChannel: vscode.OutputChannel;
let isLocked = false; // Track current lock state
let lockToggleButton: vscode.StatusBarItem | null = null;

// Minimum required compatibility: currently we require the 0.1.x family
// and we treat any prerelease (rc, alpha, beta, etc.) as incompatible.
// Assumption: for now we demand major=0 and minor=1 (i.e. 0.1.*) and
// no prerelease. This follows the user's note that 0.1.0 -> 0.2.0 can be breaking
// and that every rc/alpha/beta is breaking.

// The required Minot version string read from package.json (engines.minot)
// NOTE: no fallback is allowed — engines.minot in the packaged extension is the
// single source of truth. If it's missing or unparsable we abort activation.
let requiredMinotVersionRaw = '';

function loadRequiredMinotVersion(extensionPath: string) {
  try {
    const pkgPath = path.join(extensionPath, 'package.json');
    const raw = fs.readFileSync(pkgPath, 'utf8');
    const pkg = JSON.parse(raw);
    // Strict: only accept engines.minot as the single source of truth.
    if (pkg.engines && typeof pkg.engines.minot === 'string') {
      requiredMinotVersionRaw = pkg.engines.minot;
    } else {
      throw new Error('package.json must include engines.minot as the required Minot version');
    }
  } catch (e) {
    // Bubble up the error so the caller can decide — we don't use any fallback.
    throw e;
  }
}

async function ensureMinotCompatible(workspaceRoot?: string): Promise<string> {
  const installUrl = 'https://uos.github.io/minot/installation.html';

  // Try workspace-local binary first (./minot), fallback to global 'minot'
  const tryCmds = ['minot --version'];

  let lastErr: any = null;
  for (const cmd of tryCmds) {
    try {
      const out = await new Promise<string>((resolve, reject) => {
        cp.exec(cmd, { cwd: workspaceRoot }, (err, stdout, stderr) => {
          if (err) {
            reject(err);
            return;
          }
          resolve(stdout || stderr || '');
        });
      });

      const version = extractVersion(out);
      if (!version) {
        // Could not parse version string
        vscode.window.showErrorMessage(
          `Unknown Minot version: ${out.trim()}. Please install a compatible version (${installUrl}).`,
        );
        throw new Error('Could not parse minot version');
      }

      // Parse required version (from package.json) into a semver object
      const required = parseSemverString(requiredMinotVersionRaw);
      if (!required) {
        // Required must be present and parseable according to the single-source rule
        vscode.window.showErrorMessage(
          `Extension configuration error: engines.minot is missing or invalid in package.json. Please check the extension package.`,
        );
        throw new Error('Missing or invalid engines.minot in package.json');
      }

      // Check compatibility: installed versions must NOT be prereleases and must
      // satisfy >= required and < compatibility upper bound.
      if (!isCompatibleVersion(version, required)) {
        vscode.window.showErrorMessage(
          `Incompatible Minot version detected: ${version.raw}. Required: >=${requiredMinotVersionRaw} and <${compatUpperBoundString(required)}. Pre-release versions are only allowed when exactly matching the required prerelease. Please install a compatible version.`,
        );
        throw new Error('Incompatible minot version');
      }

      // compatible — return the executable path (strip the " --version" suffix)
      logToChannel(`Found compatible Minot version: ${version.raw}`);
      return cmd.replace(/\s+--version\s*$/, '');
    } catch (err: any) {
      lastErr = err;
      // Try next candidate (global) unless this was a parse/compat failure which we want to stop on
      // If err.code === 'ENOENT' keep trying (maybe global exists). If parse/compat error, break.
      if (err && err.code === 'ENOENT') {
        // Not found here, continue to next candidate
        continue;
      }
      // If we got an error from parsing or compatibility, rethrow to stop.
      if (err && err.message && /Incompatible|parse/i.test(err.message)) {
        throw err;
      }
      // Otherwise continue to try the next command
    }
  }

  // If we reach here, no minot binary found or all attempts failed
  logToChannel('Minot executable not found on PATH or workspace.', 'ERROR');
  
  const choice = await vscode.window.showErrorMessage(
    'Minot binary not found. Would you like to install it now?',
    'Run Install (no ROS)',
    'Open Installation Guide',
    'Cancel'
  );
  
  if (choice === 'Run Install (no ROS)') {
    // Create a new terminal and run the install script
    const terminal = vscode.window.createTerminal('Minot Installer');
    terminal.show();
    terminal.sendText('curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh');
    logToChannel('Started Minot installation in terminal. Please follow the prompts.');
    vscode.window.showInformationMessage(
      'Minot installation started in terminal. After installation completes, restart the extension or reload the window.'
    );
  } else if (choice === 'Open Installation Guide') {
    vscode.env.openExternal(vscode.Uri.parse(installUrl));
  }
  
  throw lastErr || new Error('Minot executable not found');
}

function extractVersion(output: string): { major: number; minor: number; patch: number; prerelease: string | null; raw: string } | null {
  if (!output) {
    return null;
  }
  // Typical output: "minot 0.1.0-rc.5" or similar
  const m = output.match(/(\d+)\.(\d+)\.(\d+)(?:-([^+\s]+))?(?:\+\S+)?/);
  if (!m) {
    return null;
  }
  const major = parseInt(m[1], 10);
  const minor = parseInt(m[2], 10);
  const patch = parseInt(m[3], 10);
  const prerelease = m[4] || null;
  return { major, minor, patch, prerelease, raw: m[0] };
}

/**
 * Parse a semver string like "0.1.0", "^0.1.0", ">=0.1.0" or "0.1.0-rc.5".
 * Returns the numeric parts and optional prerelease.
 */
function parseSemverString(s: string): { major: number; minor: number; patch: number; prerelease: string | null } | null {
  if (!s || typeof s !== 'string') {
    return null;
  }
  // Find first semver-looking substring
  const m = s.match(/(\d+)\.(\d+)\.(\d+)(?:-([^+\s]+))?/);
  if (!m) {
    return null;
  }
  return { major: parseInt(m[1], 10), minor: parseInt(m[2], 10), patch: parseInt(m[3], 10), prerelease: m[4] || null };
}

function compareSemver(a: { major: number; minor: number; patch: number }, b: { major: number; minor: number; patch: number }): number {
  if (a.major !== b.major) {
    return a.major < b.major ? -1 : 1;
  }
  if (a.minor !== b.minor) {
    return a.minor < b.minor ? -1 : 1;
  }
  if (a.patch !== b.patch) {
    return a.patch < b.patch ? -1 : 1;
  }
  return 0;
}

function compatUpperBoundString(required: { major: number; minor: number; patch: number }) {
  // Rust-like compatibility: for 0.y.z, a change to 0.(y+1).0 can be breaking.
  if (required.major === 0) {
    return `${required.major}.${required.minor + 1}.0`;
  }
  return `${required.major + 1}.0.0`;
}

function isCompatibleVersion(
  v: { major: number; minor: number; patch: number; prerelease: string | null; raw: string },
  required?: { major: number; minor: number; patch: number; prerelease: string | null } | null,
): boolean {
  // We require a provided required version (single source of truth)
  if (!required) {
    return false;
  }

  // If the installed version is a prerelease, only allow it when the required
  // version is the same prerelease (exact match of major/minor/patch and prerelease).
  if (v.prerelease) {
    if (
      required.prerelease &&
      v.prerelease === required.prerelease &&
      v.major === required.major &&
      v.minor === required.minor &&
      v.patch === required.patch
    ) {
      return true;
    }
    return false;
  }

  // Installed is a stable release: must be >= required and < upper bound.
  if (compareSemver(v, required) < 0) {
    return false;
  }

  const upper = required.major === 0 ? { major: 0, minor: required.minor + 1, patch: 0 } : { major: required.major + 1, minor: 0, patch: 0 };
  if (compareSemver(v, upper) >= 0) {
    return false;
  }

  return true;
}

export function activate(context: vscode.ExtensionContext) {
  outputChannel = vscode.window.createOutputChannel('Minot');
  outputChannel.show(true);
  
  try {
    loadRequiredMinotVersion(context.extensionPath);
  } catch (e: any) {
    const msg = `Extension configuration error: ${e?.message || String(e)}. The extension requires engines.minot in its package.json.`;
    logToChannel(msg, 'ERROR');
    vscode.window.showErrorMessage(msg);
    return;
  }
  logToChannel('Minot extension activated.');

  context.subscriptions.push(
    vscode.commands.registerCommand(
      'minot.compileAndExecute',
      compileAndExecute,
    ),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.restartServer', () => {
      stopServer();
      startServer();
    }),
  );

  // Internal commands (called by the toggle button)
  context.subscriptions.push(
    vscode.commands.registerCommand('minot.sendUnlock', async () => {
      await ensureServerRunningAndSend({ action: 'SendUnlock' });
      updateLockState(false);
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.sendLockNext', async () => {
      await ensureServerRunningAndSend({ action: 'SendLockNext', previous: false });
      updateLockState(true);
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.sendLockNextPrevious', async () => {
      await ensureServerRunningAndSend({ action: 'SendLockNext', previous: true });
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.clearRules', async () => {
      await ensureServerRunningAndSend({ action: 'ClearRules' });
    }),
  );

  // Toggle command (dispatches to lock or unlock based on current state)
  context.subscriptions.push(
    vscode.commands.registerCommand('minot.toggleLock', async () => {
      if (isLocked) {
        await vscode.commands.executeCommand('minot.sendUnlock');
      } else {
        await vscode.commands.executeCommand('minot.sendLockNext');
      }
    }),
  );

  // Create a single toggle button for lock/unlock
  lockToggleButton = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
  updateLockButtonUI();
  lockToggleButton.show();
  context.subscriptions.push(lockToggleButton);

  // Run button (no keybinding in text, only in tooltip)
  const sbRun = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 99);
  sbRun.text = 'Run Selection';
  sbRun.tooltip = 'Compile and Execute selection (Ctrl+Shift+Enter)';
  sbRun.command = 'minot.compileAndExecute';
  sbRun.show();
  context.subscriptions.push(sbRun);

  // Step button
  const sbStep = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 98);
  sbStep.text = 'Step';
  sbStep.tooltip = 'Step once in a Minot loop (Ctrl+,)';
  sbStep.command = 'minot.sendLockNextPrevious';
  sbStep.show();
  context.subscriptions.push(sbStep);

  context.subscriptions.push({
    dispose: () => {
      stopServer();
    },
  });
}

function updateLockState(locked: boolean) {
  isLocked = locked;
  updateLockButtonUI();
}

function updateLockButtonUI() {
  if (!lockToggleButton) {
    return;
  }
  
  if (isLocked) {
    lockToggleButton.text = '🔒 Locked';
    lockToggleButton.tooltip = 'Unlock Minot loops (Ctrl+.)';
    lockToggleButton.command = 'minot.sendUnlock';
  } else {
    lockToggleButton.text = '🔓 Unlocked';
    lockToggleButton.tooltip = 'Lock Minot loops (Ctrl+-)';
    lockToggleButton.command = 'minot.sendLockNext';
  }
}

/**
 * This method is called when your extension is deactivated
 */
export function deactivate() {
  stopServer();
}

/**
 * Kills the server process if it's running.
 */
function stopServer() {
  if (serverProcess) {
    logToChannel('Stopping Minot server...');
    serverProcess.kill();
    serverProcess = null;
  }
}

async function startServer(): Promise<void> {
  logToChannel('Starting Minot server...');
  let hasInitialized = false;
  // TODO remove workspace, expect to be in PATH
  const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
  if (!workspaceRoot) {
    logToChannel(
      'No workspace open. Cannot find Minot executable relative to root.',
      'ERROR',
    );
    throw new Error('No workspace open');
  }

  // Ensure minot binary exists and is compatible before spawning and get the exact executable to use
  const minotExec = await ensureMinotCompatible(workspaceRoot);

  return new Promise((resolve, reject) => {

    // Spawn a fresh process and keep a strong local reference to avoid races
    const proc = cp.spawn(minotExec, ['serve'], {
      cwd: workspaceRoot,
    });

    if (!proc) {
      logToChannel('Failed to spawn server process.', 'ERROR');
      reject(new Error('Failed to spawn server process'));
      return;
    }

    // Publish this as the current active server process
    serverProcess = proc;

    const installUrl = 'https://uos.github.io/minot/installation.html';
    const onError = async (err: any) => {
      const code = err && (err.code || err.errno);
      if (code === 'ENOENT') {
        logToChannel(
          `Minot executable not found (ENOENT). Offering installation options.`,
          'ERROR',
        );
        const choice = await vscode.window.showErrorMessage(
          'Minot binary not found. Would you like to install it now?',
          'Run Install Script',
          'Open Installation Guide',
          'Cancel'
        );
        
        if (choice === 'Run Install Script') {
          const terminal = vscode.window.createTerminal('Minot Installer');
          terminal.show();
          terminal.sendText('curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh');
          logToChannel('Started Minot installation in terminal. Please follow the prompts.');
          vscode.window.showInformationMessage(
            'Minot installation started in terminal. After installation completes, restart the extension or reload the window.'
          );
        } else if (choice === 'Open Installation Guide') {
          vscode.env.openExternal(vscode.Uri.parse(installUrl));
        }
      } else {
        logToChannel(`Error spawning Minot: ${err}`, 'ERROR');
        vscode.window.showErrorMessage(
          `Error starting Minot: ${err?.message || String(err)}`,
        );
      }
      cleanup();
      finishReject(err || new Error('Error spawning Minot'));
    };

    const onClose = (code: number | null) => {
      logToChannel(`Minot server process exited with code ${code}.`, 'INFO');
      cleanup();
      if (!hasInitialized) {
        finishReject(new Error(`Server exited before initialization (code ${code})`));
        return;
      }
      // Only clear the global if this is still the active process
      if (serverProcess === proc) {
        serverProcess = null;
      }
    };

    const onStdout = (data: any) => {
      data
        .toString()
        .split('\n')
        .filter(Boolean) // Remove empty lines
        .forEach((line: string) => {
          let message: any;
          try {
            message = JSON.parse(line);
          } catch (e) {
            logToChannel(`Failed to parse server JSON: ${line}`, 'ERROR');
            return; // Skip this malformed line
          }

          if (message.type === 'ServerReady' && !hasInitialized) {
            hasInitialized = true;
            logToChannel('Server reported ready. Sending "Init" message...');

            // Initialize with an empty/newline file as before
            const initMsg = {
              type: 'Init',
              file_content: '\n',
              file_path: vscode.window.activeTextEditor
                ? vscode.window.activeTextEditor.document.uri.fsPath
                : '',
            };
            // Write via the process instance we just spawned to avoid race conditions
            if (proc && proc.stdin && !proc.killed) {
              try {
                proc.stdin.write(JSON.stringify(initMsg) + '\n');
              } catch (e) {
                logToChannel(`Failed to write Init to server stdin: ${e}`, 'ERROR');
              }
            } else {
              logToChannel('Tried to write Init, but server is not running.', 'ERROR');
            }

            // Resolve only after ServerReady was received and Init sent
            finishResolve();
          } else {
            handleServerMessage(message);
          }
        });
    };

    const cleanup = () => {
      // Remove listeners from the specific process instance to avoid tampering with a new process
      proc.removeListener('error', onError);
      proc.removeListener('close', onClose);
      proc.stdout?.removeListener('data', onStdout);
    };

    const originalResolve = resolve;
    const originalReject = reject;

    const finishResolve = (value?: void | PromiseLike<void>) => {
      // don't remove stdout listener here — keep listening for server messages
      originalResolve(value as any);
    };
    const finishReject = (err?: any) => {
      cleanup();
      originalReject(err);
    };

    // error / close listeners
    proc.on('error', onError);
    proc.on('close', onClose);
    proc.stdout?.on('data', onStdout);
  });
}

/**
 * Gets the current file's content and path, then either
 * starts the server (if not running) or sends a command.
 */
async function compileAndExecute() {
  const editor = vscode.window.activeTextEditor;
  if (!editor) {
    vscode.window.showErrorMessage('No active editor open.');
    return;
  }

  const selection = editor.selection;
  let fileContent: string;

  if (selection.isEmpty) {
    const currentLine = editor.selection.active.line;
    fileContent = editor.document.lineAt(currentLine).text;
    logToChannel(`No selection. Sending current line (${currentLine + 1}).`);
  } else {
    const startLine = selection.start.line;
    const endLine = selection.end.line;

    let lines: string[] = [];
    for (let i = startLine; i <= endLine; i++) {
      lines.push(editor.document.lineAt(i).text);
    }
    fileContent = lines.join('\n');
  }

  const sendCompileExecute = () => {
    const commandMsg = {
      type: 'Command',
      command: {
        action: 'CompileExecute',
        file_content: fileContent,
        file_path: editor.document.uri.fsPath,
      },
    };
    writeToProcess(JSON.stringify(commandMsg));
  };

  if (!serverProcess) {
    try {
      await startServer();
      sendCompileExecute();
    } catch (err: any) {
      logToChannel(`Failed to start server: ${err}`, 'ERROR');
      vscode.window.showErrorMessage(
        `Failed to start Minot: ${err?.message || String(err)}`,
      );
    }
    return;
  }

  // Server already running
  sendCompileExecute();
}

/**
 * Spawns the Rust server, sets up listeners, and sends
 * the initial "Init" message.
 */
async function startServerAndInit(
  fileContent: string,
  filePath: string,
): Promise<void> {
  logToChannel('Starting Minot server...');
  let hasInitialized = false;

  const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
  if (!workspaceRoot) {
    logToChannel(
      'No workspace open. Cannot find Minot executable relative to root.',
      'ERROR',
    );
    return;
  }

  // Return a promise that resolves only after ServerReady is received and Init was sent
  return new Promise<void>(async (resolve, reject) => {
    let proc: cp.ChildProcess | null = null;

    try {
      // get the exact executable path to use for spawning
      const minotExec = await ensureMinotCompatible(workspaceRoot);
      proc = cp.spawn(minotExec, ['serve'], { cwd: workspaceRoot });
      serverProcess = proc;
    } catch (err) {
      logToChannel(`Minot compatibility check or spawn failed: ${err}`, 'ERROR');
      return reject(err);
    }

    if (!proc) {
      logToChannel('Failed to spawn server process.', 'ERROR');
      return reject(new Error('Failed to spawn server process'));
    }

    const installUrl = 'https://uos.github.io/minot/installation.html';

    const onError = async (err: any) => {
      const code = err && (err.code || err.errno);
      if (code === 'ENOENT') {
        logToChannel(
          `Minot executable not found (ENOENT). Offering installation options.`,
          'ERROR',
        );
        const choice = await vscode.window.showErrorMessage(
          'Minot binary not found. Would you like to install it now?',
          'Run Install Script',
          'Open Installation Guide',
          'Cancel'
        );
        
        if (choice === 'Run Install Script') {
          const terminal = vscode.window.createTerminal('Minot Installer');
          terminal.show();
          terminal.sendText('curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh');
          logToChannel('Started Minot installation in terminal. Please follow the prompts.');
          vscode.window.showInformationMessage(
            'Minot installation started in terminal. After installation completes, restart the extension or reload the window.'
          );
        } else if (choice === 'Open Installation Guide') {
          vscode.env.openExternal(vscode.Uri.parse(installUrl));
        }
      } else {
        logToChannel(`Error spawning Minot: ${err}`, 'ERROR');
        vscode.window.showErrorMessage(
          `Error starting Minot: ${err?.message || String(err)}`,
        );
      }
      cleanup();
      stopServer();
      return reject(err || new Error('Error spawning Minot'));
    };

    const onClose = (code: number | null) => {
      logToChannel(`Minot server process exited with code ${code}.`, 'INFO');
      cleanup();
      if (!hasInitialized) {
        stopServer();
        return reject(new Error(`Server exited before initialization (code ${code})`));
      }
      if (serverProcess === proc) {
        serverProcess = null;
      }
    };

    const onStdout = (data: any) => {
      data
        .toString()
        .split('\n')
        .filter(Boolean) // Remove empty lines
        .forEach((line: string) => {
          let message: any;
          try {
            message = JSON.parse(line);
          } catch (e) {
            logToChannel(`Failed to parse server JSON: ${line}`, 'ERROR');
            return; // Skip this malformed line
          }

          if (message.type === 'ServerReady' && !hasInitialized) {
            hasInitialized = true;
            logToChannel('Server reported ready. Sending "Init" message...');

            // Important: do NOT send the entire file content as Init. Send a
            // minimal newline so the server initializes quickly; actual commands
            // (like CompileExecute) will carry the real file content.
            const initMsg = {
              type: 'Init',
              file_content: fileContent,
              file_path: filePath,
            };
            // Write Init to the specific process instance to avoid races
            if (proc && proc.stdin && !proc.killed) {
              try {
                proc.stdin.write(JSON.stringify(initMsg) + '\n');
              } catch (e) {
                logToChannel(`Failed to write Init to server stdin: ${e}`, 'ERROR');
              }
            } else {
              logToChannel('Tried to write Init, but server is not running.', 'ERROR');
            }

            // Resolve only after ServerReady was received and Init sent
            return resolve();
          } else {
            handleServerMessage(message);
          }
        });
    };

    const onStderr = (data: any) => {
      logToChannel(`[SERVER-STDERR] ${data.toString()}`, 'ERROR');
    };

    const cleanup = () => {
      proc?.removeListener('error', onError);
      proc?.removeListener('close', onClose);
      proc?.stdout?.removeListener('data', onStdout);
      proc?.stderr?.removeListener('data', onStderr);
    };

    proc.on('error', onError);
    proc.on('close', onClose);
    proc.stdout?.on('data', onStdout);
    proc.stderr?.on('data', onStderr);
  });
}

/**
 * Ensure the server is running (start it if necessary) and send a Command message.
 * commandObj should be the inner command payload, e.g. { action: 'SendUnlock' } or
 * { action: 'SendLockNext', previous: true }
 */
async function ensureServerRunningAndSend(commandObj: any) {
  const editor = vscode.window.activeTextEditor;
  if (!serverProcess) {
    // Do not initialize the server with the entire file content. Send a minimal
    // init (single newline) to establish the connection, then send the command.
    const filePath = editor ? editor.document.uri.fsPath : '';
    try {
      // Start server with a minimal init (newline) — don't send full file here
      await startServerAndInit('\n', filePath);
    } catch (e: any) {
      logToChannel(`Failed to start server for command ${commandObj?.action}: ${e}`, 'ERROR');
      vscode.window.showErrorMessage(`Failed to start Minot for command ${commandObj?.action}: ${e?.message || String(e)}`);
      return;
    }
  }

  const commandMsg = { type: 'Command', command: commandObj };
  writeToProcess(JSON.stringify(commandMsg));
}

function handleServerMessage(message: any) {
  if (message.type === 'LogRecord') {
    const level = message.level || 'INFO';
    const msg = message.message || 'No message';
    logToChannel(`[${level}] ${msg}`, level.toUpperCase());
  } else if (message.type === 'CommandResponse') {
    const status = message.status?.toUpperCase() || 'UNKNOWN';
    if (status === 'ERROR' && message.message) {
      vscode.window.showErrorMessage(`Minot Error: ${message.message}`);
    }
  } else {
    logToChannel(`[SERVER] ${JSON.stringify(message)}`);
  }
}

function writeToProcess(message: string) {
  if (serverProcess && serverProcess.stdin) {
    try {
      serverProcess.stdin.write(message + '\n');
    } catch (e) {
      logToChannel(`Failed to write to server stdin: ${e}`, 'ERROR');
      stopServer();
    }
  } else {
    logToChannel('Tried to write, but server is not running.', 'ERROR');
  }
}

function logToChannel(message: string, level: string = 'INFO') {
  const timestamp = new Date().toLocaleTimeString();
  outputChannel.appendLine(`[${timestamp} ${level}] ${message}`);

  if (level === 'ERROR' || level === 'error') {
    vscode.window.showErrorMessage(message);
  }
}