#!/usr/bin/env node
//
// Fetch the mermaid bundle that `doc/*.html` needs, into `doc/assets/`.
//
//   node scripts/fetch-mermaid.js
//
// The bundle is **not committed**: it is 3.4 MB of vendor build output, and
// this script reproduces it exactly rather than the repository carrying it.
// "Exactly" is the condition for leaving it out, so both halves are pinned:
//
//   - the version is an exact release, never a range. `mermaid@11` would drift
//     to whatever is newest and the checksum below would start failing for a
//     reason that is not a problem
//   - the result is checked against a recorded SHA-256. Registry tarballs are
//     immutable, so a run either produces the same bytes or fails loudly
//
// Nothing here is automatic. `md2html.js` points at this script when the asset
// is missing rather than reaching for the network on its own: a documentation
// build that quietly downloads things is a documentation build that fails on
// the machine without a route to the registry.

"use strict";

const { execFileSync, execSync } = require("child_process");
const crypto = require("crypto");
const fs = require("fs");
const os = require("os");
const path = require("path");

/** Exact release. Bump deliberately, and update SHA256 in the same commit. */
const VERSION = "11.16.0";

/** SHA-256 of `package/dist/mermaid.min.js` from that release's tarball. */
const SHA256 = "74d7c46dabca328c2294733910a8aa1ed0c37451776e8d5295da38a2b758fb9b";

const ASSETS = path.join(__dirname, "..", "doc", "assets");
// On Windows npm is a .cmd shim, and Node has refused to spawn those without a
// shell since 18.20 (the fix for CVE-2024-27980), so `spawnSync npm.cmd` fails
// with EINVAL. Hence `shell: true` there. Every argument passed below is a
// literal with no spaces or shell metacharacters, which is what makes that
// safe. See also doc/handover.md §6 on bare `npm` under PowerShell.
const WINDOWS = process.platform === "win32";
const NPM = WINDOWS ? "npm.cmd" : "npm";

function sha256(file) {
  return crypto.createHash("sha256").update(fs.readFileSync(file)).digest("hex");
}

function main() {
  const target = path.join(ASSETS, "mermaid.min.js");

  if (fs.existsSync(target)) {
    const have = sha256(target);
    if (have === SHA256) {
      console.log(`mermaid ${VERSION} already present and matching — nothing to do`);
      return;
    }
    console.log(`replacing an existing bundle whose checksum does not match (${have.slice(0, 12)}…)`);
  }

  const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "mermaid-fetch-"));
  try {
    console.log(`fetching mermaid@${VERSION} from the npm registry…`);
    const args = ["pack", `mermaid@${VERSION}`, "--loglevel", "warn"];
    const opts = { cwd: tmp, stdio: ["ignore", "inherit", "inherit"] };
    if (WINDOWS) {
      // A single command string rather than an args array: Node deprecates the
      // latter alongside `shell: true` (DEP0190), because it concatenates
      // without escaping. Every token here is a constant declared in this
      // file, so concatenation is precisely what is wanted and there is
      // nothing that could need escaping.
      execSync([NPM, ...args].join(" "), opts);
    } else {
      execFileSync(NPM, args, opts);
    }

    const tarball = path.join(tmp, `mermaid-${VERSION}.tgz`);
    if (!fs.existsSync(tarball)) {
      throw new Error(`npm pack produced no ${path.basename(tarball)}`);
    }
    // bsdtar on Windows 10+, GNU tar elsewhere. Both read a gzipped tar.
    execFileSync("tar", ["-xzf", tarball], { cwd: tmp, stdio: "inherit" });

    // The UMD build, deliberately: it is self-contained (the ESM build splits
    // diagram types into chunks that would each need shipping) and loads from
    // a plain <script src>, so the page needs no module plumbing.
    const built = path.join(tmp, "package", "dist", "mermaid.min.js");
    const got = sha256(built);
    if (got !== SHA256) {
      throw new Error(
        `checksum mismatch for mermaid@${VERSION}\n` +
          `  expected ${SHA256}\n` +
          `  got      ${got}\n` +
          "Refusing to install it. Either the pin is wrong or the artifact is not " +
          "what this repository was verified against.",
      );
    }

    fs.mkdirSync(ASSETS, { recursive: true });
    fs.copyFileSync(built, target);

    const license = path.join(tmp, "package", "LICENSE");
    if (fs.existsSync(license)) {
      fs.copyFileSync(license, path.join(ASSETS, "mermaid-LICENSE.txt"));
    }

    const mb = (fs.statSync(target).size / 1024 / 1024).toFixed(2);
    console.log(`installed doc/assets/mermaid.min.js (${mb} MB, sha256 verified)`);
  } finally {
    fs.rmSync(tmp, { recursive: true, force: true });
  }
}

main();
