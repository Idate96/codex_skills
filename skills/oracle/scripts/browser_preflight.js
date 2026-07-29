#!/usr/bin/env node

const { execFileSync } = require("node:child_process");
const { existsSync } = require("node:fs");
const CDP = require("/home/lorenzo/oracle/node_modules/chrome-remote-interface");

const IN_CONTAINER = existsSync("/.dockerenv");
const DEFAULT_PROFILE =
  process.env.ORACLE_BROWSER_PROFILE ||
  (IN_CONTAINER
    ? "/home/lorenzo/.oracle/browser-profile-moleworks-ros-container-gpt56"
    : "/home/lorenzo/.oracle/browser-profile-real-google-profile1-current-experts");
const DEFAULT_PORT =
  process.env.ORACLE_BROWSER_PORT || (IN_CONTAINER ? "9223" : "9222");

function argument(name, fallback) {
  const index = process.argv.indexOf(name);
  return index === -1 ? fallback : process.argv[index + 1];
}

async function main() {
  const profile = argument("--profile", DEFAULT_PROFILE);
  const port = Number(argument("--port", DEFAULT_PORT));
  if (!profile || !Number.isInteger(port)) {
    throw new Error("usage: browser_preflight.js [--profile PATH] [--port PORT]");
  }

  const processes = execFileSync("ps", ["-eo", "args="], { encoding: "utf8" });
  const chrome = processes
    .split("\n")
    .find(
      (line) =>
        line.startsWith("/opt/google/chrome/chrome ") &&
        line.includes(`--remote-debugging-port=${port}`),
    );
  if (!chrome) {
    throw new Error(`no Chrome DevTools process is listening on port ${port}`);
  }
  const chromeArguments = chrome.trim().split(/\s+/);
  if (!chromeArguments.includes(`--user-data-dir=${profile}`)) {
    throw new Error(`Chrome on port ${port} is not using the required profile: ${profile}`);
  }
  if (/--user-data-dir=\/tmp\/oracle-(browser|reattach)-/.test(chrome)) {
    throw new Error("refusing a temporary Oracle Chrome profile");
  }

  const targets = await CDP.List({ port });
  const target = targets.find(
    (item) => item.type === "page" && item.url.startsWith("https://chatgpt.com/"),
  );
  if (!target) {
    throw new Error("no ChatGPT page is available through DevTools");
  }

  const client = await CDP({ target, port });
  try {
    const result = await client.Runtime.evaluate({
      expression: `(() => ({
        text: document.body?.innerText || "",
        hasComposer: Boolean(document.querySelector('[contenteditable="true"], textarea'))
      }))()`,
      returnByValue: true,
    });
    const { text, hasComposer } = result.result.value;
    const loginGate = /^(Log in|Sign up(?: for free)?|Welcome back)$/im.test(text);
    const signedInMarker = /(ChatGPT Pro|Chat history|Library|Projects)/i.test(text);
    if (loginGate || !signedInMarker || !hasComposer) {
      throw new Error("ChatGPT is not verifiably signed in with an available composer");
    }
  } finally {
    await client.close();
  }

  console.log(
    `ORACLE_BROWSER_PREFLIGHT_OK profile=${profile} port=${port} title=${target.title}`,
  );
}

main().catch((error) => {
  console.error(`ORACLE_BROWSER_PREFLIGHT_FAILED: ${error.message}`);
  process.exit(1);
});
