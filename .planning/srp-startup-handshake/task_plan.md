# Task Plan: CM7 Fault Hardening and S3-led Startup Sync

## Goal

Implement the approved CM7 PendSV/fail-closed hardening and SRP v4 startup
handshake with S3 as the active synchronizer, while preserving unrelated dirty
worktree changes.

## Phases

- [x] Audit current fault, startup, transport, and protocol boundaries
- [x] Add debug-only PendSV validation and atomic fault stop (existing worktree scope)
- [x] Add SRP sync IDs, STM WAIT_FOR_HOST/HOST_SYNCED gate, and timeout reset
- [x] Add S3 retry state, transport-level motion guard, and app timeout status
- [x] Build CM7 Debug and ESP-IDF targets with isolated sdkconfig
- [x] Record bench-only validation procedures and remaining hardware evidence

## Verification boundary

Builds and host/static checks do not prove flashing, UART captures, debugger
fault injection, PWM electrical behavior, or vehicle motion. Those remain
explicit bench acceptance steps.
