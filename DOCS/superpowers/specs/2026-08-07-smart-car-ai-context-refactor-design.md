# Smart_Car AI Context Refactor Design

## Goal

Build a durable engineering knowledge system that lets a new Codex session
understand the Smart_Car project from a small, authoritative reading set while
preserving detailed module, history, debug, and third-party material.

## Scope Boundary

- This task may add, modify, move, or organize Markdown only.
- Source, generated source, Swift, Python, CMake, CubeMX, IOC, GPIO, protocol
  implementation, hardware definitions, and business behavior are immutable.
- The audit covers every Markdown file under the workspace, including nested
  repositories, extracted archives, vendor material, and third-party SDKs.
- Physical reorganization is limited to root Smart_Car-owned central
  documentation. Nested Git repositories, module-local README files, vendor
  material, extracted archives, and third-party documentation remain in place.
- Existing user changes are preserved. No Git commit, staging, cleanup, build,
  flash, monitor, device, server, or runtime action is part of this task.

## Considered Approaches

### 1. Move every Markdown file into one `docs/` tree

This gives a visually uniform tree, but breaks module-local discovery, nested
repository ownership, third-party links, and historical evidence paths. It is
rejected because the repository is a workspace containing multiple ownership
boundaries rather than a single flat project.

### 2. Add indexes only and leave all existing documents authoritative

This minimizes churn, but does not solve contradictory protocol documents,
stale status records, or the risk that Codex selects an old design as current
truth. It is rejected because navigation alone is not context engineering.

### 3. Authoritative layer plus history and reference layers

This is the selected design. A compact root and `.codex/` layer holds stable
facts, rules, current status, decisions, and navigation. Canonical module and
architecture documents live under `docs/`. Superseded Smart_Car-owned central
documents move under `docs/history/`; compatibility stubs remain only where an
old path is an important entry point. Module-local and external material stays
in place and is indexed with an explicit trust classification.

## Knowledge Layers

| Layer | Purpose | Typical files | Update cadence | Authority |
| --- | --- | --- | --- | --- |
| Boot | Minimal session startup | `.codex/BOOT.md`, `.codex/INDEX.md` | Rare | Navigation only |
| Stable memory | Confirmed, slow-changing facts | `.codex/MEMORY.md`, `.codex/RULES.md` | Rare | High |
| Current state | Version, status, risk, TODO | `PROJECT_STATUS.md` | Frequent | High, date-bound |
| Decisions | Accepted design choices and evidence | `DECISION_LOG.md` | Append-oriented | High when confirmed |
| Canonical engineering | Architecture, protocol, modules, code map | `docs/**` | As implementation changes | High |
| Local reference | Module README and tool documentation | Existing module paths | With owning module | Medium to high |
| History | Superseded plans, audits, debug records | `docs/history/**` | Append-only | Historical only |
| External | Vendor, SDK, nested repositories, extracted files | Existing external paths | External ownership | Low for Smart_Car truth |

## Fact Model

Every material claim must be identified as one of:

- `CONFIRMED`: verified in current source, current hardware definition, or a
  named current authoritative document.
- `PLANNED`: intended architecture with no implementation evidence.
- `RESERVED`: hardware or software capability kept for later use.
- `PAUSED`: intentionally not active, such as BMI323 runtime use.
- `DEPRECATED`: retained only for history or migration context.
- `UNVERIFIED`: plausible or previously reported but not proven in the current
  audit.

Build evidence, host tests, device logs, and full hardware/integration behavior
remain separate evidence levels. A successful build is never recorded as live
vehicle acceptance.

## Canonical Structure

```text
README.md
PROJECT_STATUS.md
DECISION_LOG.md
DOCUMENT_AUDIT_REPORT.md
DOCUMENT_INDEX.md
MODULE_INDEX.md
CHANGELOG.md
AI_CONTEXT_REFACTOR_REPORT.md
.codex/
  BOOT.md
  MEMORY.md
  RULES.md
  WORKFLOW.md
  INDEX.md
docs/
  README.md
  architecture/
  hardware/
  stm32/
  esp32s3/
  app/
  protocol/
  imu/
  radar/
  motor/
  ros2/
  debug/
  history/
  code_map.md
```

On this case-insensitive macOS filesystem, the existing `DOCS/` directory and
requested `docs/` spelling resolve to the same directory. Canonical links use
lowercase `docs/`; the physical directory is not case-renamed during this task
because it contains a non-Markdown `.DS_Store`, and changing that artifact is
outside the authorized scope.

## Source-of-Truth Precedence

1. Current source and current hardware configuration for implemented behavior.
2. Frozen protocol and confirmed hardware documents.
3. `.codex/MEMORY.md`, current status, decisions, and canonical module docs.
4. Module-local README files.
5. Historical records, plans, audit snapshots, and debug logs.
6. Third-party, vendor, extracted, and nested-project material for reference
   only.

When evidence conflicts, the lower-ranked document is not silently rewritten
as fact. The conflict is recorded in the audit or status database and marked
for human confirmation when current evidence is insufficient.

## Verification Design

- Inventory every Markdown/README path and record purpose, module, state, trust,
  and disposition.
- Verify every required deliverable exists and contains its required sections.
- Resolve local Markdown links and report all remaining broken targets.
- Validate Mermaid fence balance and required diagram presence.
- Scan canonical files for stale protocol markers and unresolved placeholders.
- Compare the final changed-file set against the Markdown-only allowlist.
- Do not run compilation or runtime tests because no executable artifact is
  modified.

## Acceptance

A new engineer must be able to understand the project within 30 minutes from
`README.md`, `.codex/BOOT.md`, `.codex/MEMORY.md`, `PROJECT_STATUS.md`, and
`docs/architecture/system.md`. A new Codex session must be ready to work after
reading `.codex/BOOT.md`, `.codex/MEMORY.md`, `.codex/RULES.md`, and
`.codex/INDEX.md`, while being directed to module-specific evidence before any
change.
