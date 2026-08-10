# doc/assets

Generated / fetched files for the HTML view of the documents. **Nothing here is
committed except this file.**

## mermaid.min.js

Renders the diagrams in [`../architecture.html`](../architecture.html).

```
node scripts/fetch-mermaid.js
```

| | |
|---|---|
| version | **11.16.0**, pinned exactly in `scripts/fetch-mermaid.js` |
| source | npm registry (`npm pack mermaid@11.16.0`), `package/dist/mermaid.min.js` |
| build | UMD — self-contained, loads from a plain `<script src>` |
| size | 3.4 MB |
| sha256 | `74d7c46dabca328c2294733910a8aa1ed0c37451776e8d5295da38a2b758fb9b` |
| licence | MIT. The fetch also writes `mermaid-LICENSE.txt` next to it |

Not committed because it is vendor build output that the command above
reproduces byte for byte: the version is an exact release rather than a range,
registry tarballs are immutable, and the script verifies the checksum and
refuses to install anything else. A 3.4 MB binary in the history for something
regenerable in seconds is not a trade worth making.

**Vendored locally rather than loaded from a CDN.** The page then renders with
no network at all, which matters for the same reason `webviewInstallMode` does:
offline distribution, and proxies that block script CDNs.

### Bumping it

Change `VERSION` and `SHA256` in `scripts/fetch-mermaid.js` **in the same
commit**, update the table above, then re-run the fetch and
`node scripts/md2html.js doc/architecture.md`. Check the diagrams still render:
a mermaid major release has broken flowchart syntax before.
