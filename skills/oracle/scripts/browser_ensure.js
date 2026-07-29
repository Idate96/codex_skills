#!/usr/bin/env node

const { execFileSync, spawn } = require("node:child_process");
const { closeSync, existsSync, mkdirSync, openSync } = require("node:fs");
const path = require("node:path");
const CDP = require("/home/lorenzo/oracle/node_modules/chrome-remote-interface");

const IN_CONTAINER = existsSync("/.dockerenv");
const DEFAULT_PROFILE =
  process.env.ORACLE_BROWSER_PROFILE ||
  (IN_CONTAINER
    ? "/home/lorenzo/.oracle/browser-profile-moleworks-ros-container-gpt56"
    : "/home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts");
const DEFAULT_PORT =
  process.env.ORACLE_BROWSER_PORT || (IN_CONTAINER ? "9223" : "9222");
const DEFAULT_CHROME =
  process.env.ORACLE_BROWSER_CHROME_PATH || "/opt/google/chrome/chrome";

function argument(name, fallback) {
  const index = process.argv.indexOf(name);
  return index === -1 ? fallback : process.argv[index + 1];
}

function chromeProcesses() {
  return execFileSync("ps", ["-eo", "pid=,args="], { encoding: "utf8" })
    .split("\n")
    .filter((line) => line.includes("--remote-debugging-port="));
}

async function devToolsAvailable(port) {
  try {
    await CDP.List({ port });
    return true;
  } catch {
    return false;
  }
}

async function waitForDevTools(port) {
  const deadline = Date.now() + 30_000;
  while (Date.now() < deadline) {
    if (await devToolsAvailable(port)) {
      return;
    }
    await new Promise((resolve) => setTimeout(resolve, 250));
  }
  throw new Error(`Chrome DevTools did not become available on port ${port}`);
}

async function main() {
  const profile = path.resolve(argument("--profile", DEFAULT_PROFILE));
  const port = Number(argument("--port", DEFAULT_PORT));
  const chromePath = argument("--chrome-path", DEFAULT_CHROME);
  if (!profile || !Number.isInteger(port) || port < 1 || port > 65535) {
    throw new Error(
      "usage: browser_ensure.js [--profile PATH] [--port PORT] [--chrome-path PATH]",
    );
  }
  if (!existsSync(chromePath)) {
    throw new Error(`Chrome is not installed at ${chromePath}; rebuild the full image`);
  }

  const portToken = `--remote-debugging-port=${port}`;
  const profileToken = `--user-data-dir=${profile}`;
  const processes = chromeProcesses();
  const exactProcess = processes.find(
    (line) => line.includes(portToken) && line.includes(profileToken),
  );
  const otherProcess = processes.find((line) => line.includes(portToken));

  if (otherProcess && !exactProcess) {
    throw new Error(`DevTools port ${port} belongs to a different Chrome profile`);
  }
  if (exactProcess) {
    await waitForDevTools(port);
    console.log(`ORACLE_BROWSER_READY profile=${profile} port=${port} reused=true`);
    return;
  }
  if (await devToolsAvailable(port)) {
    throw new Error(`DevTools port ${port} is active but its Chrome process is not visible`);
  }

  mkdirSync(profile, { recursive: true });
  const logDir = path.join(path.dirname(profile), "logs");
  mkdirSync(logDir, { recursive: true });
  const logPath = path.join(logDir, `chrome-${port}.log`);
  const logFd = openSync(logPath, "a");
  try {
    const child = spawn(
      chromePath,
      [
        portToken,
        profileToken,
        "--no-first-run",
        "--no-default-browser-check",
        "https://chatgpt.com",
      ],
      {
        detached: true,
        stdio: ["ignore", logFd, logFd],
      },
    );
    child.unref();
  } finally {
    closeSync(logFd);
  }

  await waitForDevTools(port);
  console.log(`ORACLE_BROWSER_READY profile=${profile} port=${port} reused=false`);
}

main().catch((error) => {
  console.error(`ORACLE_BROWSER_START_FAILED: ${error.message}`);
  process.exit(1);
});
