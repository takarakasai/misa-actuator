#!/usr/bin/env node
//
// Generate a standalone HTML view of a markdown document.
//
//   node scripts/md2html.js doc/architecture.md
//
// **The markdown is the master.** The HTML this writes is a build artifact and
// carries a "do not edit" banner: any hand edit is lost the next time this runs.
// Edit the `.md`, re-run this, commit both.
//
// Zero dependencies, on purpose. This workspace's only Node install is the GUI
// front end, and a doc build has no business dragging a markdown toolchain into
// it. The cost is that only the subset of markdown these docs actually use is
// supported: ATX headings, paragraphs, `-`/`1.` lists, tables with alignment,
// fenced code, blockquotes, `---` rules, and inline code / bold / links.
//
// Mermaid blocks become `<pre class="mermaid">`, rendered at view time by
// `doc/assets/mermaid.min.js` — vendored locally rather than pulled from a CDN
// so the page works offline and survives a proxy that blocks one.
//
// Links are emitted unchanged. A link to another `.md` file therefore still
// points at the markdown, which is correct inside the repo but shows raw text
// in a browser.

"use strict";

const fs = require("fs");
const path = require("path");

// ---------------------------------------------------------------------------
// Inline
// ---------------------------------------------------------------------------

function escapeHtml(s) {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

/**
 * Inline markdown → HTML.
 *
 * Code spans are pulled out first so that `**` inside them stays literal, and
 * everything is escaped before the link/bold passes so document text can never
 * inject markup.
 */
function inline(src) {
  const codes = [];
  // Private-use sentinels, not a padded word. These documents write
  // `code`直後に日本語 with no space, so a placeholder carrying spaces would
  // open a gap in the output that is not in the source.
  let s = src.replace(/`([^`]+)`/g, (_, code) => {
    codes.push(code);
    return `\uE000${codes.length - 1}\uE001`;
  });

  s = escapeHtml(s);
  s = s.replace(/\[([^\]]+)\]\(([^)\s]+)\)/g, (_, text, href) => {
    const external = /^https?:/.test(href);
    const attrs = external ? ' target="_blank" rel="noreferrer"' : "";
    return `<a href="${href}"${attrs}>${text}</a>`;
  });
  s = s.replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");

  return s.replace(
    /\uE000(\d+)\uE001/g,
    (_, i) => `<code>${escapeHtml(codes[Number(i)])}</code>`,
  );
}

/**
 * Join a soft-wrapped line onto the previous one.
 *
 * Markdown joins wrapped lines with a space, which is right for English and
 * wrong for Japanese — these docs wrap mid-sentence and a space would open a
 * visible gap. Join without one when both sides of the break are CJK.
 */
function joinWrapped(a, b) {
  if (!a) return b;
  const cjk = /[　-鿿＀-￯]/;
  return cjk.test(a.slice(-1)) && cjk.test(b.slice(0, 1)) ? a + b : `${a} ${b}`;
}

function slug(text) {
  return text
    .replace(/[`*]/g, "")
    .trim()
    .replace(/\s+/g, "-")
    .replace(/[^\w　-鿿＀-￯.-]/g, "");
}

// ---------------------------------------------------------------------------
// Block
// ---------------------------------------------------------------------------

function isTableDelimiter(line) {
  return /^\|?\s*:?-{2,}:?\s*(\|\s*:?-{2,}:?\s*)*\|?$/.test(line.trim());
}

function splitRow(line) {
  return line
    .trim()
    .replace(/^\|/, "")
    .replace(/\|$/, "")
    .split("|")
    .map((c) => c.trim());
}

function alignments(delim) {
  return splitRow(delim).map((c) => {
    const left = c.startsWith(":");
    const right = c.endsWith(":");
    if (left && right) return "center";
    if (right) return "right";
    return "left";
  });
}

function convert(md) {
  const lines = md.replace(/\r\n/g, "\n").split("\n");
  const out = [];
  const toc = [];
  let i = 0;

  const blockStart = (l) =>
    l.trim() === "" ||
    /^#{1,6} /.test(l) ||
    /^```/.test(l) ||
    /^(-{3,}|\*{3,})\s*$/.test(l) ||
    /^[-*] /.test(l) ||
    /^\d+\. /.test(l) ||
    /^> /.test(l) ||
    l.startsWith("|");

  while (i < lines.length) {
    const line = lines[i];

    if (line.trim() === "") {
      i++;
      continue;
    }

    // Fenced code / mermaid
    if (/^```/.test(line)) {
      const lang = line.slice(3).trim();
      const body = [];
      i++;
      while (i < lines.length && !/^```/.test(lines[i])) body.push(lines[i++]);
      i++; // closing fence
      const text = escapeHtml(body.join("\n"));
      // Mermaid reads textContent, so the escaping above is what gets the
      // literal `<br/>` in node labels through to the renderer intact.
      out.push(
        lang === "mermaid"
          ? `<pre class="mermaid">${text}</pre>`
          : `<pre><code${lang ? ` class="language-${lang}"` : ""}>${text}</code></pre>`,
      );
      continue;
    }

    // Heading
    const h = line.match(/^(#{1,6}) (.*)$/);
    if (h) {
      const level = h[1].length;
      const id = slug(h[2]);
      if (level === 2 || level === 3) toc.push({ level, id, text: h[2] });
      out.push(`<h${level} id="${id}">${inline(h[2])}</h${level}>`);
      i++;
      continue;
    }

    // Horizontal rule
    if (/^(-{3,}|\*{3,})\s*$/.test(line)) {
      out.push("<hr>");
      i++;
      continue;
    }

    // Table
    if (line.startsWith("|") && i + 1 < lines.length && isTableDelimiter(lines[i + 1])) {
      const header = splitRow(line);
      const align = alignments(lines[i + 1]);
      i += 2;
      const rows = [];
      while (i < lines.length && lines[i].startsWith("|")) rows.push(splitRow(lines[i++]));

      const th = header
        .map((c, n) => `<th style="text-align:${align[n] || "left"}">${inline(c)}</th>`)
        .join("");
      const body = rows
        .map(
          (r) =>
            "<tr>" +
            r
              .map(
                (c, n) =>
                  `<td style="text-align:${align[n] || "left"}">${inline(c)}</td>`,
              )
              .join("") +
            "</tr>",
        )
        .join("\n");
      out.push(
        `<div class="table-wrap"><table>\n<thead><tr>${th}</tr></thead>\n<tbody>\n${body}\n</tbody>\n</table></div>`,
      );
      continue;
    }

    // Blockquote
    if (/^> /.test(line)) {
      const body = [];
      while (i < lines.length && /^>/.test(lines[i])) {
        body.push(lines[i].replace(/^>\s?/, ""));
        i++;
      }
      out.push(`<blockquote>${convert(body.join("\n"))}</blockquote>`);
      continue;
    }

    // Lists
    const bullet = line.match(/^([-*]) (.*)$/);
    const numbered = line.match(/^\d+\. (.*)$/);
    if (bullet || numbered) {
      const ordered = Boolean(numbered);
      const items = [];
      while (i < lines.length) {
        const m = ordered ? lines[i].match(/^\d+\. (.*)$/) : lines[i].match(/^[-*] (.*)$/);
        if (!m) break;
        let text = m[1];
        i++;
        // Continuation: indented, and not the start of a sibling item.
        while (
          i < lines.length &&
          lines[i].trim() !== "" &&
          /^\s+/.test(lines[i]) &&
          !/^\s*[-*] /.test(lines[i]) &&
          !/^\s*\d+\. /.test(lines[i])
        ) {
          text = joinWrapped(text, lines[i].trim());
          i++;
        }
        items.push(`<li>${inline(text)}</li>`);
      }
      const tag = ordered ? "ol" : "ul";
      out.push(`<${tag}>\n${items.join("\n")}\n</${tag}>`);
      continue;
    }

    // Paragraph
    let text = line.trim();
    i++;
    while (i < lines.length && !blockStart(lines[i])) {
      text = joinWrapped(text, lines[i].trim());
      i++;
    }
    out.push(`<p>${inline(text)}</p>`);
  }

  return { html: out.join("\n\n"), toc };
}

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

const CSS = `
:root {
  --fg: #1a1a1a; --muted: #666; --bg: #fff; --panel: #f6f7f9;
  --border: #d8dbe0; --link: #0b5fbf; --accent: #b45309;
}
@media (prefers-color-scheme: dark) {
  :root {
    --fg: #e6e6e6; --muted: #9aa0a6; --bg: #14161a; --panel: #1c1f24;
    --border: #2f343b; --link: #6ea8fe; --accent: #e0a458;
  }
}
* { box-sizing: border-box; }
body {
  margin: 0; padding: 0 1.5rem 6rem; background: var(--bg); color: var(--fg);
  font-family: "Segoe UI", "Yu Gothic UI", Meiryo, system-ui, sans-serif;
  line-height: 1.85; font-size: 16px;
}
main { max-width: 62rem; margin: 0 auto; }
h1 { font-size: 2rem; margin: 2.5rem 0 .5rem; }
h2 {
  font-size: 1.5rem; margin: 3rem 0 1rem; padding-bottom: .35rem;
  border-bottom: 2px solid var(--border);
}
h3 { font-size: 1.15rem; margin: 2.2rem 0 .75rem; }
p, li { overflow-wrap: anywhere; }
a { color: var(--link); }
hr { border: 0; border-top: 1px solid var(--border); margin: 2.5rem 0; }
code {
  background: var(--panel); border: 1px solid var(--border); border-radius: 4px;
  padding: .08em .35em; font-size: .88em;
  font-family: "Cascadia Mono", Consolas, "Courier New", monospace;
}
pre {
  background: var(--panel); border: 1px solid var(--border); border-radius: 8px;
  padding: 1rem; overflow-x: auto; line-height: 1.55;
}
pre code { background: none; border: 0; padding: 0; font-size: .85em; }
blockquote {
  margin: 1.5rem 0; padding: .25rem 1.25rem; border-left: 4px solid var(--accent);
  color: var(--muted);
}
.table-wrap { overflow-x: auto; margin: 1.25rem 0; }
table { border-collapse: collapse; width: 100%; font-size: .93em; }
th, td { border: 1px solid var(--border); padding: .45rem .7rem; vertical-align: top; }
th { background: var(--panel); }
pre.mermaid {
  background: transparent; border: 1px dashed var(--border); text-align: center;
  padding: 1.25rem;
}
pre.mermaid svg { max-width: 100%; height: auto; }
.banner {
  max-width: 62rem; margin: 1.5rem auto 0; padding: .7rem 1rem;
  background: var(--panel); border: 1px solid var(--border);
  border-left: 4px solid var(--accent); border-radius: 6px;
  font-size: .87em; color: var(--muted);
}
.toc {
  background: var(--panel); border: 1px solid var(--border); border-radius: 8px;
  padding: 1rem 1.5rem; margin: 2rem 0;
}
.toc ul { list-style: none; padding-left: 0; margin: .3rem 0; }
.toc li.lvl3 { padding-left: 1.5rem; font-size: .93em; }
.toc a { text-decoration: none; }
.toc a:hover { text-decoration: underline; }
@media print {
  body { font-size: 11pt; }
  .banner, .toc { display: none; }
  pre, .table-wrap, pre.mermaid { break-inside: avoid; }
}
`;

function page({ title, bodyHtml, toc, source, mermaidHref, generatedAt }) {
  const tocHtml = toc.length
    ? `<nav class="toc"><strong>目次</strong><ul>${toc
        .map(
          (t) =>
            `<li class="lvl${t.level}"><a href="#${t.id}">${inline(t.text)}</a></li>`,
        )
        .join("")}</ul></nav>`
    : "";

  return `<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>${escapeHtml(title)}</title>
<!--
  GENERATED FILE — DO NOT EDIT.

  Source : ${source}
  Command: node scripts/md2html.js ${source}
  Built  : ${generatedAt}

  Every edit made here is destroyed the next time the command above runs.
  Change the markdown instead.
-->
<style>${CSS}</style>
</head>
<body>
<div class="banner">
  <strong>自動生成ファイル。直接編集しないこと。</strong>
  マスタは <code>${escapeHtml(source)}</code> で、この HTML は
  <code>node scripts/md2html.js ${escapeHtml(source)}</code> の出力です。
  ここへの編集は次回の生成で消えます。
</div>
<main>
${tocHtml}
${bodyHtml}
</main>
<script src="${mermaidHref}"></script>
<script>
  // Vendored locally (doc/assets/), so this page renders with no network.
  if (window.mermaid) {
    const dark = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
    window.mermaid.initialize({
      startOnLoad: true,
      theme: dark ? "dark" : "default",
      securityLevel: "strict",
      flowchart: { htmlLabels: true, useMaxWidth: true },
    });
  } else {
    for (const el of document.querySelectorAll("pre.mermaid")) {
      el.insertAdjacentHTML(
        "beforebegin",
        '<p style="color:#b45309">mermaid.min.js が読み込めませんでした。図はソースのまま表示されます。</p>'
      );
    }
  }
</script>
</body>
</html>
`;
}

// ---------------------------------------------------------------------------

function build(mdPath) {
  const abs = path.resolve(mdPath);
  const md = fs.readFileSync(abs, "utf8");
  const { html, toc } = convert(md);

  const outPath = abs.replace(/\.md$/i, ".html");
  const titleMatch = md.match(/^#\s+(.*)$/m);
  const title = titleMatch ? titleMatch[1].trim() : path.basename(abs, ".md");

  // Not fetched automatically: a doc build that reaches for the network on its
  // own is one that fails on the machine without a route to the registry.
  const assets = path.join(path.dirname(abs), "assets", "mermaid.min.js");
  if (!fs.existsSync(assets)) {
    console.warn(
      `warning: ${path.relative(process.cwd(), assets).replace(/\\/g, "/")} is missing, ` +
        "so the diagrams will render as plain text.\n" +
        "         Run: node scripts/fetch-mermaid.js",
    );
  }

  fs.writeFileSync(
    outPath,
    page({
      title,
      bodyHtml: html,
      toc,
      source: path.relative(process.cwd(), abs).replace(/\\/g, "/"),
      mermaidHref: "assets/mermaid.min.js",
      generatedAt: new Date().toISOString().replace("T", " ").slice(0, 16) + " UTC",
    }),
    "utf8",
  );

  console.log(`${path.relative(process.cwd(), abs)} -> ${path.relative(process.cwd(), outPath)}`);
}

const inputs = process.argv.slice(2);
if (inputs.length === 0) {
  console.error("usage: node scripts/md2html.js <file.md> [file.md ...]");
  process.exit(2);
}
for (const f of inputs) build(f);
