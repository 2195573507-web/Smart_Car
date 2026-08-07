# Smart_Car AI Context Refactor Plan

## Objective

Complete the documentation-only AI engineering knowledge-base refactor defined
in the attached task specification. Preserve every non-Markdown artifact and
all unrelated user changes.

## Boundaries

- Allowed: add, edit, move, and organize Markdown.
- Forbidden: any non-Markdown edit; build, flash, monitor, runtime, device,
  server, CubeMX, GPIO, protocol implementation, or business-logic change.
- Audit scope: the full workspace.
- Move scope: root Smart_Car-owned central Markdown only.
- Nested Git repositories, third-party trees, extracted archives, vendor
  materials, and module-local README files remain at their current paths.

## Phases

| Phase | Status | Deliverables |
| --- | --- | --- |
| 1. Baseline and inventory | Complete | Complete Markdown inventory, ownership classes, immutable boundary |
| 2. Core context | Complete | `README.md`, `.codex/*`, status, decisions |
| 3. Canonical architecture | Complete | System, communication, and data-flow documents with Mermaid |
| 4. Module documentation | Complete | Required module pages with uniform sections |
| 5. Reorganization | Complete | Deprecated protocol documents archived; local/external docs retained |
| 6. Mapping and indexes | Complete | Code map, document index, module index |
| 7. Audit and reports | Complete | Audit report, changelog, final refactor report |
| 8. Verification | Complete | Required-file, section, link, Mermaid, placeholder, and boundary checks |
| 9. Completion audit | Complete | Requirement-by-requirement evidence matrix |

## Design Decisions

| Decision | Reason |
| --- | --- |
| Use authoritative/history/reference layers | Prevent stale material from competing with current truth |
| Keep nested repositories and third-party docs in place | Preserve ownership, local links, and upstream provenance |
| Keep module README files beside source | Preserve developer discovery at the modification boundary |
| Use lowercase `docs/` in canonical links | Match requested structure while respecting case-insensitive filesystem identity |
| Classify claims by evidence state | Separate confirmed behavior from plans and historical reports |
| Do not compile | Markdown-only changes cannot establish firmware or hardware behavior |

## Errors

| Error | Attempts | Resolution |
| --- | --- | --- |
| New goal creation rejected because `/goal` already created one | 1 | Reused the active user goal |
| Two exploratory zsh commands emitted `unknown file attribute` | 2 | Avoided `find` expressions whose parentheses were interpreted by zsh; use `rg --files` or quoted predicates |
| Some broad source searches included vendor CMSIS trees | 1 | Narrow subsequent searches to project-owned source directories and explicit files |

## Completion Rule

Do not mark complete until every named artifact and content requirement from the
task specification has authoritative current-state evidence and no changed
non-Markdown path is attributable to this task.
