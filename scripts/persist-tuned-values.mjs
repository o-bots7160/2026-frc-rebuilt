#!/usr/bin/env node
/**
 * persist-tuned-values.mjs
 *
 * Connects to a WPILib NetworkTables 4.1 server, reads tuned values from
 * SmartDashboard, compares them against the subsystem JSON config file, and
 * optionally persists changes back to disk.
 *
 * Checks two NT key namespaces (prefers System 2):
 *
 *   System 2 (preferred): /AdvantageKit/NetworkInputs/SmartDashboard/<ClassPrefix>/<field>
 *     Created lazily by AbstractConfig.readTunableNumber via AdvantageKit's
 *     LoggedNetworkNumber. This is the namespace Elastic / Shuffleboard edits live.
 *
 *   System 1 (fallback):  /SmartDashboard/SubsystemsConfig/<json/path>
 *     Seeded once at startup by ConfigurationLoader.iterateFields.
 *     Mirrors the JSON structure exactly.
 *
 * Usage: node persist-tuned-values.mjs [options]
 */

import { decodeMulti } from "@msgpack/msgpack";
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";
import WebSocket from "ws";

const __dirname = dirname(fileURLToPath(import.meta.url));

// ─── CLI ───────────────────────────────────────────────────────────────────────

const HELP = `
persist-tuned-values — persist NetworkTables tunables back to JSON config

Usage: node persist-tuned-values.mjs [options]

Options:
  --team <number>      Team number for robot address (default: localhost)
  --config <path>      Path to subsystems JSON file
                       (default: ../src/main/deploy/subsystems.json)
  --subsystem <name>   Limit to a single subsystem key (e.g. turretSubsystem)
  --dry-run            Show changes without writing to disk
  --watch, -w          Stay connected and monitor for changes live
  --debug              Show all NT topics, key mappings, and match details
  --help, -h           Show this help message

Examples:
  node persist-tuned-values.mjs --config ../src/main/deploy/subsystems-sim.json
  node persist-tuned-values.mjs --team 7160 --subsystem turretSubsystem
  node persist-tuned-values.mjs --config ../src/main/deploy/subsystems-sim.json --watch
  npm run persist:sim -- --watch --subsystem turretSubsystem
`.trim();

function parseArgs(argv) {
  const args = { dryRun: false, watch: false, debug: false };
  for (let i = 2; i < argv.length; i++) {
    switch (argv[i]) {
      case "--help":
      case "-h":
        console.log(HELP);
        process.exit(0);
        break;
      case "--team":
        args.team = parseInt(argv[++i], 10);
        if (Number.isNaN(args.team)) {
          console.error("Error: --team requires a numeric value");
          process.exit(1);
        }
        break;
      case "--config":
        args.configPath = argv[++i];
        break;
      case "--subsystem":
        args.subsystem = argv[++i];
        break;
      case "--dry-run":
        args.dryRun = true;
        break;
      case "--watch":
      case "-w":
        args.watch = true;
        break;
      case "--debug":
        args.debug = true;
        break;
      default:
        console.error(`Unknown option: ${argv[i]}\nRun with --help for usage.`);
        process.exit(1);
    }
  }
  return args;
}

// ─── NT4 Client ────────────────────────────────────────────────────────────────

const NT4_SUBPROTOCOL = "v4.1.networktables.first.wpi.edu";
const CONNECT_TIMEOUT_MS = 4000;
const SETTLE_QUIET_MS = 500;
const SETTLE_MAX_MS = 3000;

/**
 * Lightweight NT4 WebSocket client. Subscribes to SmartDashboard and
 * AdvantageKit NetworkInputs topics and stores the latest value for each topic.
 */
class NT4Client {
  constructor(address) {
    this.address = address;
    /** Map of topic ID → { name, type } */
    this.topics = new Map();
    /** Map of topic name → latest value */
    this.values = new Map();
    this.ws = null;
    this._connected = false;
  }

  /**
   * Opens the WebSocket, subscribes to NT topics, and waits for the
   * initial burst of values to settle before resolving.
   *
   * @param {Object} opts
   * @param {Function} [opts.onClose] Called when the connection drops.
   * @returns {Promise<void>}
   */
  connect({ onClose } = {}) {
    return new Promise((res, rej) => {
      const url = `ws://${this.address}:5810/nt/${NT4_SUBPROTOCOL}`;
      this.ws = new WebSocket(url, [NT4_SUBPROTOCOL]);

      let quietTimer = null;
      let settled = false;
      const connectTimeout = setTimeout(() => {
        if (!settled) {
          settled = true;
          try { this.ws.terminate(); } catch { /* ignore */ }
          rej(new Error(`Connection to ${this.address} timed out`));
        }
      }, CONNECT_TIMEOUT_MS);

      const settle = () => {
        if (settled) return;
        settled = true;
        clearTimeout(connectTimeout);
        if (quietTimer) clearTimeout(quietTimer);
        res();
      };

      const resetQuiet = () => {
        if (settled) return;
        if (quietTimer) clearTimeout(quietTimer);
        quietTimer = setTimeout(settle, SETTLE_QUIET_MS);
      };

      this.ws.on("open", () => {
        this._connected = true;
        const subscribeMsg = [
          {
            method: "subscribe",
            params: {
              topics: ["/SmartDashboard/"],
              subuid: 1,
              options: { prefix: true, all: true, periodic: 0.1 },
            },
          },
          {
            method: "subscribe",
            params: {
              topics: ["/AdvantageKit/NetworkInputs/SmartDashboard/"],
              subuid: 2,
              options: { prefix: true, all: true, periodic: 0.1 },
            },
          },
        ];
        this.ws.send(JSON.stringify(subscribeMsg));
        // Max-wait: settle after SETTLE_MAX_MS even if messages keep arriving
        setTimeout(settle, SETTLE_MAX_MS);
      });

      this.ws.on("message", (data, isBinary) => {
        if (isBinary) {
          this._handleBinary(data);
        } else {
          this._handleText(data.toString());
        }
        resetQuiet();
      });

      this.ws.on("close", () => {
        this._connected = false;
        settle();
        if (onClose) onClose();
      });

      this.ws.on("error", (err) => {
        this._connected = false;
        if (!settled) {
          settled = true;
          clearTimeout(connectTimeout);
          rej(new Error(`NT4 connection error (${this.address}): ${err.message}`));
        }
      });
    });
  }

  isConnected() {
    return this._connected;
  }

  close() {
    if (this.ws) {
      try { this.ws.close(); } catch { /* ignore */ }
      this.ws = null;
      this._connected = false;
    }
  }

  /** Handle JSON text frames (announce / unannounce). */
  _handleText(raw) {
    let messages;
    try {
      messages = JSON.parse(raw);
    } catch {
      return;
    }
    if (!Array.isArray(messages)) return;

    for (const msg of messages) {
      if (msg.method === "announce" && msg.params) {
        const { name, id, type } = msg.params;
        this.topics.set(id, { name, type });
      } else if (msg.method === "unannounce" && msg.params) {
        this.topics.delete(msg.params.id);
      }
    }
  }

  /** Handle MessagePack binary frames (value updates). */
  _handleBinary(data) {
    try {
      for (const entry of decodeMulti(data)) {
        if (!Array.isArray(entry) || entry.length < 4) continue;
        const [topicId, , , value] = entry;
        const topic = this.topics.get(topicId);
        if (topic) {
          this.values.set(topic.name, value);
        }
      }
    } catch {
      // Malformed frame — skip
    }
  }
}

// ─── Address resolution ────────────────────────────────────────────────────────

function resolveAddresses(team) {
  if (!team) return ["localhost"];
  const upper = Math.floor(team / 100);
  const lower = team % 100;
  return [
    `roboRIO-${team}-FRC.local`,
    `10.${upper}.${lower}.2`,
    "172.22.11.2",
  ];
}

/**
 * Tries each address in order until one connects successfully.
 *
 * @param {string[]} addresses
 * @param {Object} [opts] Forwarded to NT4Client.connect().
 * @returns {Promise<NT4Client>}
 */
async function connectWithRetry(addresses, opts = {}) {
  for (const addr of addresses) {
    try {
      console.log(`  Trying ${addr}…`);
      const client = new NT4Client(addr);
      await client.connect(opts);
      console.log(`  Connected to ${addr}`);
      return client;
    } catch (err) {
      console.error(`  Failed (${addr}): ${err.message}`);
    }
  }
  throw new Error(
    `Could not connect to any NT server. Tried: ${addresses.join(", ")}`
  );
}

// ─── JSON helpers ──────────────────────────────────────────────────────────────

function loadJson(filePath) {
  return JSON.parse(readFileSync(filePath, "utf-8"));
}

function saveJson(filePath, obj) {
  writeFileSync(filePath, JSON.stringify(obj, null, 2) + "\n", "utf-8");
}

/** Deep-get using a dot-separated path. */
function deepGet(obj, path) {
  let cur = obj;
  for (const key of path.split(".")) {
    if (cur == null || typeof cur !== "object") return undefined;
    cur = cur[key];
  }
  return cur;
}

/** Deep-set using a dot-separated path, creating intermediate objects. */
function deepSet(obj, path, value) {
  const parts = path.split(".");
  let cur = obj;
  for (let i = 0; i < parts.length - 1; i++) {
    if (cur[parts[i]] == null || typeof cur[parts[i]] !== "object") {
      cur[parts[i]] = {};
    }
    cur = cur[parts[i]];
  }
  cur[parts[parts.length - 1]] = value;
}

// ─── Namespace derivation ──────────────────────────────────────────────────────

/**
 * Derives the System 2 NT key prefix from a JSON field name.
 *
 * Convention (mirrors AbstractConfig.computeDefaultDashboardPrefix):
 *   1. Upper-case first letter (JSON field → Java class name basis).
 *   2. Strip trailing "Config" if present (class name → dashboard prefix).
 *
 * Examples:
 *   turretSubsystem   → TurretSubsystem
 *   motorConfig         → Motor
 *   shooterSubsystem   → ShooterSubsystem
 */
function deriveSystem2Prefix(jsonFieldName) {
  let prefix = jsonFieldName.charAt(0).toUpperCase() + jsonFieldName.slice(1);
  if (prefix.endsWith("Config")) {
    prefix = prefix.slice(0, -"Config".length);
  }
  return prefix;
}

// ─── Change detection ──────────────────────────────────────────────────────────

/**
 * Walks the JSON config tree and compares every scalar leaf against the
 * corresponding NT value. Returns an array of detected changes.
 *
 * For each scalar field the function builds two candidate NT keys:
 *   System 2: /AdvantageKit/NetworkInputs/SmartDashboard/<ClassPrefix>[/<nestedKey>...]/<fieldName>    (Elastic-editable)
 *   System 1: /SmartDashboard/SubsystemsConfig/<json/path> (startup seed)
 *
 * Nested config objects (pid, feedforward, motionProfile, etc.) extend the
 * System 2 prefix with the JSON key name, matching Java's
 * initializeNestedDashboardPrefixes behavior.
 *
 * System 2 is preferred when both keys exist.
 *
 * @param {Object} json         Parsed subsystems JSON.
 * @param {Map}    ntValues     NT4Client.values map.
 * @param {string} [subsystem]  Optional filter for a single subsystem key.
 * @returns {{ jsonPath: string, jsonValue: any, ntValue: any, ntKey: string, system: number }[]}
 */
function detectChanges(json, ntValues, subsystem, debug = false) {
  const changes = [];
  const keys = subsystem ? [subsystem] : Object.keys(json);

  for (const subsystemKey of keys) {
    const subsystemObj = json[subsystemKey];
    if (subsystemObj == null || typeof subsystemObj !== "object") continue;

    const sys2Prefix = deriveSystem2Prefix(subsystemKey);
    if (debug) {
      console.log(`\n  [debug] Subsystem: ${subsystemKey}`);
      console.log(`  [debug]   System 2 prefix: ${sys2Prefix}`);
      console.log(`  [debug]   System 1 base:   SubsystemsConfig/${subsystemKey}`);
    }
    walkObject(
      subsystemObj,
      subsystemKey,
      sys2Prefix,
      `SubsystemsConfig/${subsystemKey}`,
      ntValues,
      changes,
      debug,
    );
  }
  return changes;
}

/**
 * Recursively walks a config object. For every scalar leaf, checks NT for a
 * changed value. For nested objects, recurses with the parent System 2 prefix
 * extended by the JSON key name (matching Java's initializeNestedDashboardPrefixes).
 */
function walkObject(obj, jsonPathPrefix, sys2Prefix, sys1Path, ntValues, changes, debug = false) {
  for (const [key, value] of Object.entries(obj)) {
    const jsonPath = `${jsonPathPrefix}.${key}`;
    const sys1Key = `/SmartDashboard/${sys1Path}/${key}`;

    if (value != null && typeof value === "object" && !Array.isArray(value)) {
      // Nested config — extend the parent System 2 prefix with the JSON key
      const nestedPrefix = `${sys2Prefix}/${key}`;
      if (debug) {
        console.log(`  [debug]   Nesting into "${key}" → sys2 prefix: ${nestedPrefix}`);
      }
      walkObject(
        value,
        jsonPath,
        nestedPrefix,
        `${sys1Path}/${key}`,
        ntValues,
        changes,
        debug,
      );
    } else if (
      typeof value === "number" ||
      typeof value === "boolean" ||
      typeof value === "string"
    ) {
      // Scalar leaf — try both namespaces
      const sys2Key = `/AdvantageKit/NetworkInputs/SmartDashboard/${sys2Prefix}/${key}`;

      const hasSys2 = ntValues.has(sys2Key);
      const hasSys1 = ntValues.has(sys1Key);

      let ntValue;
      let ntKey;
      let system;

      if (hasSys2) {
        ntValue = ntValues.get(sys2Key);
        ntKey = sys2Key;
        system = 2;
      } else if (hasSys1) {
        ntValue = ntValues.get(sys1Key);
        ntKey = sys1Key;
        system = 1;
      }

      if (debug) {
        const fmt = (v) => typeof v === "number" ? Number(v.toFixed(6)) : JSON.stringify(v);
        const sys2Status = hasSys2 ? `✓ val=${fmt(ntValues.get(sys2Key))}` : "✗ not found";
        const sys1Status = hasSys1 ? `✓ val=${fmt(ntValues.get(sys1Key))}` : "✗ not found";
        const match = ntValue !== undefined
          ? (valuesEqual(value, ntValue) ? "MATCH" : "CHANGED")
          : "NO NT KEY";
        console.log(`  [debug]   ${jsonPath} (json=${fmt(value)})`);
        console.log(`  [debug]     sys2: ${sys2Key}  ${sys2Status}`);
        console.log(`  [debug]     sys1: ${sys1Key}  ${sys1Status}`);
        console.log(`  [debug]     result: ${match}`);
      }

      if (ntValue !== undefined && !valuesEqual(value, ntValue)) {
        changes.push({ jsonPath, jsonValue: value, ntValue, ntKey, system });
      }
    }
  }
}

/**
 * Compares a JSON value to an NT value with floating-point tolerance.
 */
function valuesEqual(jsonVal, ntVal) {
  if (typeof jsonVal === "number" && typeof ntVal === "number") {
    return Math.abs(jsonVal - ntVal) < 1e-9;
  }
  return jsonVal === ntVal;
}

// ─── Debug helpers ─────────────────────────────────────────────────────────────

/**
 * Prints a full dump of all NT topics and values received from the server,
 * filtered to SmartDashboard entries.
 */
function printDebugSummary(client) {
  const topicCount = client.topics.size;
  const valueCount = client.values.size;
  console.log(`\n  [debug] NT4 received ${topicCount} topic(s), ${valueCount} value(s)`);

  // Group values by prefix for readability
  const sorted = [...client.values.entries()].sort(([a], [b]) =>
    a.localeCompare(b),
  );
  console.log(`  [debug] All NT values:`);
  for (const [key, val] of sorted) {
    const fmt =
      typeof val === "number" ? Number(val.toFixed(6)) : JSON.stringify(val);
    console.log(`    ${key} = ${fmt}`);
  }
  console.log();
}

// ─── Display & persistence ─────────────────────────────────────────────────────

function printChanges(changes) {
  if (changes.length === 0) {
    console.log("\n  No changes detected.\n");
    return;
  }

  console.log(`\n  ${changes.length} change(s) detected:\n`);
  for (const c of changes) {
    const fmt = (v) =>
      typeof v === "number" ? Number(v.toFixed(6)) : JSON.stringify(v);
    const source = c.system === 2 ? "(Tunable)" : "(Seed)";
    console.log(`    ${c.jsonPath}: ${fmt(c.jsonValue)} → ${fmt(c.ntValue)}  ${source}`);
  }
  console.log();
}

function persistChanges(configPath, json, changes) {
  for (const c of changes) {
    deepSet(json, c.jsonPath, c.ntValue);
  }
  saveJson(configPath, json);
  console.log(`  ✓ Wrote ${changes.length} change(s) to ${configPath}\n`);
}

// ─── One-shot mode ─────────────────────────────────────────────────────────────

async function runOnce(addresses, configPath, json, args) {
  const client = await connectWithRetry(addresses);

  if (args.debug) {
    printDebugSummary(client);
  }

  const changes = detectChanges(json, client.values, args.subsystem, args.debug);

  printChanges(changes);

  if (changes.length > 0 && !args.dryRun) {
    persistChanges(configPath, json, changes);
  } else if (changes.length > 0) {
    console.log("  (dry run — no changes written)\n");
  }

  client.close();
}

// ─── Watch mode ────────────────────────────────────────────────────────────────

const POLL_INTERVAL_MS = 2000;

async function runWatchMode(addresses, configPath, json, args) {
  let client = null;
  let reconnecting = false;

  const scheduleReconnect = () => {
    if (reconnecting) return;
    reconnecting = true;
    console.log("\n  Connection lost. Reconnecting in 3 s…");
    setTimeout(async () => {
      try {
        client = await connectWithRetry(addresses, {
          onClose: scheduleReconnect,
        });
        reconnecting = false;
        console.log("  Reconnected — monitoring resumed.\n");
      } catch (err) {
        reconnecting = false;
        console.error(`  ${err.message}`);
        scheduleReconnect();
      }
    }, 3000);
  };

  // Initial connection
  try {
    client = await connectWithRetry(addresses, {
      onClose: scheduleReconnect,
    });
  } catch (err) {
    console.error(err.message);
    process.exit(1);
  }

  if (args.debug) {
    printDebugSummary(client);
  }

  console.log(
    "\n  Watch mode active. Press Enter to persist changes, Ctrl+C to exit.\n",
  );

  // Readline for Enter-to-persist
  const rl = createInterface({ input: process.stdin, output: process.stdout });

  rl.on("line", () => {
    if (!client || !client.isConnected()) {
      console.log("  Not connected.\n");
      return;
    }

    // Re-read JSON from disk in case it was edited externally
    let freshJson;
    try {
      freshJson = loadJson(configPath);
    } catch (err) {
      console.error(`  Error reloading config: ${err.message}`);
      return;
    }

    const changes = detectChanges(freshJson, client.values, args.subsystem, args.debug);
    if (changes.length === 0) {
      console.log("  No changes to persist.\n");
      return;
    }

    printChanges(changes);

    if (args.dryRun) {
      console.log("  (dry run — no changes written)\n");
    } else {
      persistChanges(configPath, freshJson, changes);
      // Update in-memory copy so the next poll sees the new baseline
      json = freshJson;
    }
  });

  // Periodic change scanner
  let lastSummary = "";

  const poll = () => {
    if (!client || !client.isConnected()) return;

    let freshJson;
    try {
      freshJson = loadJson(configPath);
    } catch {
      return;
    }

    const changes = detectChanges(freshJson, client.values, args.subsystem);

    // Build a summary string to avoid reprinting the same output
    const summary = changes
      .map((c) => `${c.jsonPath}=${c.ntValue}`)
      .sort()
      .join("|");

    if (summary !== lastSummary) {
      lastSummary = summary;
      if (changes.length > 0) {
        printChanges(changes);
        console.log("  Press Enter to persist, or keep tuning.\n");
      } else {
        console.log("  All values match JSON config.\n");
      }
    }
  };

  const pollInterval = setInterval(poll, POLL_INTERVAL_MS);

  // Graceful shutdown
  const shutdown = () => {
    console.log("\n  Shutting down…");
    clearInterval(pollInterval);
    rl.close();
    if (client) client.close();
    process.exit(0);
  };

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);
}

// ─── Main ──────────────────────────────────────────────────────────────────────

async function main() {
  const args = parseArgs(process.argv);

  const configPath = resolve(
    __dirname,
    args.configPath || "../src/main/deploy/subsystems.json",
  );

  let json;
  try {
    json = loadJson(configPath);
  } catch (err) {
    console.error(`Error loading config: ${err.message}`);
    process.exit(1);
  }

  // Resolve subsystem name case-insensitively against JSON keys so users can
  // pass either the JSON key (turretSubsystem) or the NT-style name
  // (TurretSubsystem) and it just works.
  if (args.subsystem) {
    const match = Object.keys(json).find(
      (k) => k.toLowerCase() === args.subsystem.toLowerCase(),
    );
    if (!match) {
      console.error(
        `Error: subsystem "${args.subsystem}" not found in config.\n` +
          `Valid keys: ${Object.keys(json).join(", ")}`,
      );
      process.exit(1);
    }
    if (match !== args.subsystem) {
      console.log(`  Resolved subsystem "${args.subsystem}" → "${match}"`);
      args.subsystem = match;
    }
  }

  console.log(`Config:     ${configPath}`);
  console.log(`Subsystem:  ${args.subsystem || "(all)"}`);
  console.log(`Dry run:    ${args.dryRun}`);
  console.log(`Watch mode: ${args.watch}`);
  console.log(`Debug:      ${args.debug}\n`);

  const addresses = resolveAddresses(args.team);

  if (args.watch) {
    await runWatchMode(addresses, configPath, json, args);
  } else {
    await runOnce(addresses, configPath, json, args);
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
