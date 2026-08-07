# Codex Workflow

## Analysis

1. Define the requested outcome and hard exclusions.
2. Locate the current source entry point, public headers, task/callback path,
   and owning document.
3. Search consumers and producers before changing an interface.
4. Identify conflicts and label evidence rather than guessing.

## Implementation

1. Read the relevant module page and decision history.
2. Keep the change inside the allowed boundary.
3. Preserve transport/parser/task separation and existing error semantics.
4. Update canonical documentation only when implementation behavior actually
   changed or the task is documentation-only.

## Verification Ladder

| Level | Evidence | Does not prove |
| --- | --- | --- |
| Static | source search, link checks, Markdown structure | compiler or hardware |
| Build | compile/link/package result | flash, device, transport, vehicle |
| Device | reset/log/bus capture | full integration unless all links are exercised |
| Integration | App-S3-STM chain, radar, controlled vehicle | claims outside the tested setup |

## Standard Report

```text
Modified files:
Reason:
Ownership/impact:
Risks:
Verification:
Unverified or blocked:
```

## Documentation Refactor Workflow

Inventory first, classify second, establish authority third, archive only
project-owned central records fourth, then verify links and the Markdown-only
boundary. Never move a nested-repository or vendor document merely to make the
root tree look uniform.
