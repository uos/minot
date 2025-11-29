import * as vscode from 'vscode';
import * as cp from 'child_process';
import * as fs from 'fs';
import * as path from 'path';

let serverProcess: cp.ChildProcess | null = null;
let outputChannel: vscode.OutputChannel;
let isLocked = false;
let lockToggleButton: vscode.StatusBarItem | null = null;
let sbRun: vscode.StatusBarItem | null = null;
let sbStep: vscode.StatusBarItem | null = null;

function updateAllTooltips() {
  updateLockButtonUI();
  updateRunButtonTooltip();
  updateStepButtonTooltip();
}

function updateRunButtonTooltip() {
  if (!sbRun) {
    return;
  }
  sbRun.tooltip = 'Compile and Execute selection';
}

function updateStepButtonTooltip() {
  if (!sbStep) {
    return;
  }
  sbStep.tooltip = 'Step once in a Minot loop';
}

let requiredMinotVersionRaw = '';

function loadRequiredMinotVersion(extensionPath: string) {
  try {
    const pkgPath = path.join(extensionPath, 'package.json');
    const raw = fs.readFileSync(pkgPath, 'utf8');
    const pkg = JSON.parse(raw);
    if (pkg.engines && typeof pkg.engines.minot === 'string') {
      requiredMinotVersionRaw = pkg.engines.minot;
    } else {
      throw new Error('package.json must include engines.minot as the required Minot version');
    }
  } catch (e) {
    throw e;
  }
}

async function ensureMinotCompatible(workspaceRoot?: string): Promise<string> {
  const installUrl = 'https://uos.github.io/minot/installation.html';

  const tryCmds = ['./minot --version', 'minot --version'];

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
        throw new Error('Could not parse minot version');
      }

      const required = parseSemverString(requiredMinotVersionRaw);
      if (!required) {
        throw new Error('Missing or invalid engines.minot in package.json');
      }

      if (!isCompatibleVersion(version, required)) {
        logToChannel(`Incompatible Minot version detected: ${version.raw}. Required: >=${requiredMinotVersionRaw} and <${compatUpperBoundString(required)}. Pre-release versions are only allowed when exactly matching the required prerelease. Please install a compatible version.`, 'ERROR');
        throw new Error('Incompatible minot version');
      }

      logToChannel(`Found compatible Minot version: ${version.raw}`);
      return cmd.replace(/\s+--version\s*$/, '');
    } catch (err: any) {
      lastErr = err;
      if (err && err.code === 'ENOENT') {
        continue;
      }
      if (err && err.message && /Incompatible|parse/i.test(err.message)) {
        throw err;
      }
    }
  }

  logToChannel('Minot executable not found on PATH or workspace.', 'ERROR');
  
  const choice = await vscode.window.showErrorMessage(
    'Minot binary not found. Would you like to install it now? (ROS support is installed automatically if your shell is sourced with ROS)',
    'Run Install',
    'Open Installation Guide',
    'Cancel'
  );

  if (choice === 'Run Install') {
    const terminal = vscode.window.createTerminal('Minot Installer');
    terminal.show();
    const requiredVer = requiredMinotVersionRaw || '';
    let installCmd = 'curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh';
    if (requiredVer) {
      const escaped = requiredVer.replace(/'/g, "'\\''");
      installCmd = `curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh -s -- --version '${escaped}'`;
    }
    terminal.sendText(installCmd);
    logToChannel('Started Minot installation in terminal. Please follow the prompts.');
    vscode.window.showInformationMessage(
      'Minot installation started in terminal. If your shell is sourced with ROS, ROS support will be installed automatically. After installation completes, restart the extension or reload the window.'
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
      'minot.run',
      compileAndExecute,
    ),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.restart', () => {
      stopServer();
      startServer();
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.stop', () => {
      stopServer();
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.unlock', async () => {
      await ensureServerRunningAndSend({ action: 'SendUnlock' });
      updateLockState(false);
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.lock', async () => {
      await ensureServerRunningAndSend({ action: 'SendLockNext', previous: false });
      updateLockState(true);
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.step', async () => {
      await ensureServerRunningAndSend({ action: 'SendLockNext', previous: true });
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.clearRules', async () => {
      await ensureServerRunningAndSend({ action: 'ClearRules' });
    }),
  );

  context.subscriptions.push(
    vscode.commands.registerCommand('minot.toggleLock', async () => {
      if (isLocked) {
        await vscode.commands.executeCommand('minot.unlock');
      } else {
        await vscode.commands.executeCommand('minot.lock');
      }
    }),
  );

  lockToggleButton = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
  lockToggleButton.show();
  context.subscriptions.push(lockToggleButton);

  sbRun = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 99);
  sbRun.text = 'Run Selection';
  sbRun.command = 'minot.run';
  sbRun.show();
  context.subscriptions.push(sbRun);

  sbStep = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 98);
  sbStep.text = 'Step';
  sbStep.command = 'minot.step';
  sbStep.show();
  context.subscriptions.push(sbStep);

  updateAllTooltips();

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
    lockToggleButton.text = '🔒 Unlock';
    lockToggleButton.tooltip = 'Unlock Minot loop';
    lockToggleButton.command = 'minot.unlock';
  } else {
    lockToggleButton.text = '🔓 Lock';
    lockToggleButton.tooltip = 'Lock next Minot loop';
    lockToggleButton.command = 'minot.lock';
  }
}

export function deactivate() {
  stopServer();
}

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
  // Workspace is optional — prefer PATH/global minot when no workspace is open
  const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;

  // Ensure minot binary exists and is compatible before spawning and get the exact executable to use
  const minotExec = await ensureMinotCompatible(workspaceRoot);

  return new Promise((resolve, reject) => {

    // Spawn a fresh process and keep a strong local reference to avoid races
    const proc = cp.spawn(minotExec, ['serve'], workspaceRoot ? { cwd: workspaceRoot } : undefined);

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
            'Minot binary not found. Would you like to install it now? (ROS support is installed automatically if your shell is sourced with ROS)',
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
            'Minot installation started in terminal. If your shell is sourced with ROS, ROS support will be installed automatically. After installation completes, restart the extension or reload the window.'
          );
        } else if (choice === 'Open Installation Guide') {
          vscode.env.openExternal(vscode.Uri.parse(installUrl));
        }
      } else {
        logToChannel(`Error spawning Minot: ${err}`, 'ERROR');
        // Quiet: avoid popup; log only
        logToChannel(`Error starting Minot: ${err?.message || String(err)}`);
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
      // Quiet: avoid popup
      logToChannel(`Failed to start Minot: ${err?.message || String(err)}`, 'ERROR');
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

  // Workspace is optional — prefer PATH/global minot when no workspace is open
  const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;

  // Return a promise that resolves only after ServerReady is received and Init was sent
  return new Promise<void>(async (resolve, reject) => {
    let proc: cp.ChildProcess | null = null;

    try {
      // get the exact executable path to use for spawning
      const minotExec = await ensureMinotCompatible(workspaceRoot);
      proc = cp.spawn(minotExec, ['serve'], workspaceRoot ? { cwd: workspaceRoot } : undefined);
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
          'Minot binary not found. Would you like to install it now? (ROS support is installed automatically if your shell is sourced with ROS)',
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
            'Minot installation started in terminal. If your shell is sourced with ROS, ROS support will be installed automatically. After installation completes, restart the extension or reload the window.'
          );
        } else if (choice === 'Open Installation Guide') {
          vscode.env.openExternal(vscode.Uri.parse(installUrl));
        }
      } else {
        logToChannel(`Error spawning Minot: ${err}`, 'ERROR');
        logToChannel(`Error starting Minot: ${err?.message || String(err)}`);
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
      // Quiet: avoid popup
      logToChannel(`Failed to start Minot for command ${commandObj?.action}: ${e?.message || String(e)}`, 'ERROR');
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
      logToChannel(`Minot Error: ${message.message}`, 'ERROR');
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
  // Avoid duplicated log level tags if message already contains one like "[INFO] ..."
  const cleaned = message.replace(/^\s*\[(INFO|ERROR|WARN|WARNING|DEBUG|TRACE)\]\s*/i, '');
  outputChannel.appendLine(`[${timestamp} ${level}] ${cleaned}`);
}