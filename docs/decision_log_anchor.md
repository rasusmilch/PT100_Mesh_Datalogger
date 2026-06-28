# Project Decision Log Anchor

Suggested path: `docs/decision_log_anchor.md`

## Decision 1 — Use nested monthly PTLOG directories for new daily logs

Status: settled
Reference: PR #368, Task 1 / SD FAT16-safe path work

### Decision

New daily PTLOG files use the nested monthly FAT 8.3 layout:

* `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`

`RRR` is a decimal same-day revision from `000` through `999`; the extension is revision metadata, not file-type metadata. PTLOG file identity is the first-line ASCII magic `#PT100_LOG_V1`.

Old root-level and long-name `.ptlog` files are not supported by firmware scanning, stats, or reclaim. They may remain on a card, but firmware ignores them rather than migrating or renaming them.

### Rationale

The leading SD failure theory is FAT root-directory entry exhaustion from many long daily filenames. Moving new files into monthly directories reduces root-directory pressure while preserving readable date-based filenames.

### Evidence or context references

* SD/FRAM failure analysis.
* PR #368: `Move PTLOG writes to FAT16-safe log directories`.
* Current `sd_ptlog_paths` helpers and daily PTLOG path behavior.

### Consequences

* New log files are no longer expected at SD root.
* Tools and firmware code must not assume root-only PTLOG layout.
* Firmware retention and stats support only compact nested monthly paths.

### Rejected alternatives

* Keep all daily PTLOG files at root.
* Use the superseded YYMMDDRR.PTL design from closed PR #372.
* Immediately migrate to FAT32 solely to avoid FAT16 root limits.

### Follow-up tasks or validation needed

* Verify all host tools can discover nested `/logs/YYYY-MM/` PTLOG files.
* Hardware validate long-running nested daily file creation on target SD media.

---

## Decision 2 — Remove legacy root PTLOG filename compatibility

Status: settled
Reference: PR #368, PR #369

### Decision

The firmware shall ignore root-level PTLOG-looking files and old long-name `.ptlog` files for scanning, stats, and reclaim.

### Rationale

Existing SD cards and field data may already contain root-level or long-name PTLOG files. They are not migrated, renamed, parsed, counted, or reclaimed automatically by firmware.

### Evidence or context references

* Task 2B-3A-F1 supersedes earlier compatibility decisions and forces the compact nested layout only.

### Consequences

* Firmware scan, stats, and reclaim must ignore root-level PTLOG-looking files and old long-name `.ptlog` files.
* Tests should preserve firmware ignore behavior without requiring obsolete `legacy_root` compatibility fields if those fields are later removed.
* Host tools may keep explicit import or inspection support for historical files only when that support remains separate from firmware scan/reclaim policy.

### Rejected alternatives

* Migrate or rewrite root PTLOGs automatically.
* Delete all root PTLOGs as a one-time cleanup.
* Continue parsing old long `.ptlog` names in firmware.

### Follow-up tasks or validation needed

* Keep host-tool historical-file support clearly separated from firmware scan/stats/reclaim behavior.
* Remove obsolete `legacy_root` firmware API fields only in a scoped compatibility cleanup.

---

## Decision 3 — Replace generic scratch-owner direction with narrow static SD/PTLOG path workspace

Status: accepted; supersedes Task 2B-3B-DOC1/DOC2 scratch-owner direction
Reference: Task 2B-3B-DOC3; PR #376 reviewed as abandoned context

### Decision

The generic PSRAM-backed storage scratch owner model is abandoned for now. PR #376, `Introduce PSRAM-backed storage scratch owner and integrate into PTLOG paths/logger`, is not the intended architecture and must not be merged as-is. Its inspected head was previously recorded as `0819853d39c3d464de3bb391a4243271c43ca28d`; the work is preserved for reference only on branch `archive/scratch-owner-pr376-0819853`.

The next implementation shall use a simpler, dedicated static SD/PTLOG path workspace. SD/PTLOG path, filename, month-directory, candidate-path, and related path-workflow buffers that are currently local stack arrays should be moved to centrally owned static storage where practical. Prefer PSRAM-backed allocation/storage on the target when safe and available. Largish reusable buffers should also be centrally owned/static rather than repeatedly placed on FreeRTOS task stacks. Do not increase FreeRTOS task stack sizes to hide storage/path stack pressure.

The immediate goal is only to remove file/path/candidate buffers from FreeRTOS task stacks. Do not create a generic allocator, slot pool, borrow/release API, concurrency framework, broad allocator, or generalized scratch mechanism. A future implementation may use a dedicated typed SD/PTLOG path workspace object, but it must keep ownership, lifetime, serialization/locking assumptions, required capacity, fail-closed behavior, and diagnostics explicit. Existing SD/runtime serialization assumptions may be used only when documented in that implementation.

Suggested next implementation name: `2B-3B-1R — Replace SD/PTLOG stack path buffers with static PSRAM-backed path workspace`.

### Rationale

Prior stack pressure issues still make path-buffer growth risky, but PR #376 showed that a generic scratch-owner model is broader than the near-term storage problem. The safer embedded path is a narrow, typed workspace for SD/PTLOG path workflows, preserving deterministic behavior and avoiding a new allocator/concurrency abstraction before the project has proved it needs one.

### Consequences

* Future SD/PTLOG path-buffer work must inventory current stack path/candidate buffers and move only the scoped path-workflow buffers off task stacks.
* Large per-entry arrays, large candidate arrays, recursive traversal, generic allocators, slot pools, broad borrow/release APIs, and unnecessary concurrency frameworks remain prohibited.
* The path workspace remains separate from PTLOG scan facts, reclaim thresholds, CSV tail repair, file-buffer sizing, SD format changes, reclaim policy, display wording, ntfy behavior, FRAM semantics, retry policy, and host-tool ingestion changes.
* Future code review must verify stack, static, heap, and PSRAM placement and document task-safety/ownership assumptions.

### Validation needed

* Future code review must verify actual stack usage and allocation placement for storage/path changes.
* Firmware build validation remains required for embedded changes when available.
* When possible, target stack high-water, heap, and PSRAM validation should be reported for storage traversal paths.

### Exclusions

* This decision does not implement the path workspace.
* This decision does not remove existing runtime tail repair/resume scanning.
* This decision does not approve reclaim thresholds, display progress, create/open retry policy, ntfy/display changes, FRAM semantic changes, host-tool ingestion changes, or SD format changes.

---

## Decision 4 — Keep FAT16 / 16 KiB allocation unit format for now

Status: settled for current implementation; may be revisited with evidence
Reference: PR #368 and SD/FRAM failure analysis

### Decision

Keep the current FAT-compatible SD format behavior, including 16 KiB allocation units. Do not migrate to FAT32 or alter allocation settings without a separate decision and validation plan.

### Rationale

The immediate failure can be addressed by moving long filenames out of the FAT root and by adding retention controls. A filesystem migration increases risk and validation burden.

### Evidence or context references

* PR #368 explicitly avoided FAT32 migration.
* PR #369 preserved FAT format behavior.
* SD/FRAM analysis ranks FAT root-directory entry exhaustion as the leading theory, not media failure or a format choice alone.

### Consequences

* Firmware must continue to respect FAT16-like constraints.
* Directory-entry pressure remains a design concern.
* Future retention policy must consider file count and long-filename entry usage.

### Rejected alternatives

* Immediate FAT32 migration.
* Change allocation unit size during SD reclaim/path work.
* Increase open-file count as part of unrelated SD fixes.

### Follow-up tasks or validation needed

* Hardware validate on actual target SD media.
* Revisit FAT32 only if monthly directories and retention thresholds do not resolve field behavior.

---

## Decision 5 — Use bounded PTLOG traversal, not unbounded recursion

Status: settled
Reference: PR #368, PR #369

### Decision

Automatic PTLOG discovery shall be bounded to approved locations:

* `/sdcard`
* `/sdcard/logs`
* `/sdcard/logs/YYYY-MM`

The scanner shall not traverse arbitrary nested directories.

### Rationale

Automatic deletion on embedded firmware must fail closed. Unbounded recursion risks deleting unintended files, walking host-created directories, exhausting stack/heap, or hanging on malformed media.

### Evidence or context references

* `SdPtlogFindOldestCandidate()` scans `/logs` and valid month directories only; root files are ignored.
* System directories and non-month directories are ignored.
* PTLOG candidates must parse and must be regular files.

### Consequences

* Unknown directories under `/logs` are ignored.
* Host-created system directories are ignored.
* Future retention tasks should extend the same scanner rather than duplicating traversal logic.

### Rejected alternatives

* Recursive walk of the entire SD card.
* Deleting any file ending in `.ptlog` anywhere on the card.
* Trusting directory enumeration order.

### Follow-up tasks or validation needed

* Add diagnostics if a directory cannot be opened but scanning continues.
* Confirm host tools use equivalent discovery rules where relevant.

---

## Decision 6 — Reclaim deletes one oldest eligible PTLOG candidate per pass

Status: settled
Reference: PR #369, Task 2A

### Decision

`SdLoggerReclaimSpaceLocked()` uses `SdPtlogFindOldestCandidate()` and deletes one oldest eligible PTLOG candidate per loop pass.

### Rationale

One-candidate-per-pass reclaim is simpler, bounded, and compatible with low-memory embedded constraints. It avoids building large arrays of candidates and aligns with the scanner’s “find oldest candidate, delete, rescan” model.

### Evidence or context references

* PR #369 replaced root-only date-group deletion with bounded candidate traversal.
* Current code recomputes free bytes after each successful delete.
* Delete count increments only after successful `unlink()`.

### Consequences

* Reclaim may recover space more gradually than old whole-date-group deletion.
* Reclaim behavior is deterministic by parsed date and revision.
* Root and nested monthly PTLOGs are treated through the same mechanism.

### Rejected alternatives

* Delete all PTLOGs for a date group.
* Preload all candidates into memory and sort.
* Delete all old files in one pass.
* Keep root-only reclaim.

### Follow-up tasks or validation needed

* Hardware validate reclaim on FAT16 SD media.
* Evaluate whether one-file-per-pass is fast enough in real low-space conditions.

---

## Decision 6 — Current-date protection is the Task 2A guard; current-open path tracking deferred

Status: provisional
Reference: PR #369, Task 2A

### Decision

For Task 2A, reclaim passes `NULL` for current open path and relies on `logger->current_date` to protect the current day’s PTLOG files.

### Rationale

`sd_logger_t` does not currently track the exact open PTLOG path. Adding that field would expand scope into state lifecycle, close/reset behavior, and tests. Current-date protection is conservative and sufficient for the approved Task 2A policy.

### Evidence or context references

* PR #369 implementation comment documents broad current-date protection.
* Current product policy does not permit automatic deletion of current-date files.

### Consequences

* All current-date PTLOG files, including same-date revisions, are protected.
* Same-day old revisions cannot be reclaimed automatically under current policy.
* If future policy allows same-day revision deletion, current-open path tracking becomes more important.

### Rejected alternatives

* Add `current_path` to `sd_logger_t` during Task 2A.
* Allow deletion of same-day old revisions.
* Rely only on file handle state without a path guard.

### Follow-up tasks or validation needed

* Decide whether to add a current-open PTLOG path field.
* Decide whether same-day old revisions may ever be deleted automatically.

---

## Decision 7 — Reclaim remains byte-space-triggered only until threshold policy is approved

Status: settled for Task 2A; broader retention policy remains provisional
Reference: PR #369, SD/FRAM failure analysis

### Decision

Task 2A preserves the existing byte-space trigger. File-count, root-entry, per-directory count, and estimated directory-entry thresholds are deferred.

### Rationale

Task 2A was scoped to wire the scanner into reclaim, not to decide retention policy. Directory-entry thresholds require product decisions and careful validation.

### Evidence or context references

* PR #369 explicitly did not add file-count thresholds or directory-entry thresholds.
* SD/FRAM analysis identifies directory-entry-aware retention as future required work.

### Consequences

* Current reclaim still will not run if free bytes are high but directory entries are exhausted.
* Create/open failures from directory-entry pressure remain possible until later tasks.

### Rejected alternatives

* Add thresholds in Task 2A.
* Treat free-byte reclaim as the final retention model.
* Infer thresholds without user/product decision.

### Follow-up tasks or validation needed

* Task 2B: add file-count, root-entry, per-directory, and/or directory-entry-aware triggers.
* Decide exact threshold values.

---

## Decision 8 — Daily create/open reclaim retry is deferred

Status: provisional
Reference: SD/FRAM failure analysis, PR #368, PR #369

### Decision

Do not yet reclaim and retry when daily PTLOG `fopen` create/open fails. This is deferred to a later task.

### Rationale

Create/open failure retry requires policy decisions: which errno values qualify, how many files to delete, whether to retry once, and how to report status if retry fails.

### Evidence or context references

* PR #368 added diagnostics but no create/open retry.
* PR #369 updated reclaim selection but did not touch daily open retry behavior.
* SD/FRAM analysis recommends reclaim-on-create-failure as a later fix group.

### Consequences

* A card with high free bytes but directory-entry exhaustion may still fail daily file create/open.
* Current improvements reduce future root pressure but do not fully address create/open recovery.

### Rejected alternatives

* Add create/open retry in PR #368.
* Add create/open retry in Task 2A.
* Retry indefinitely.

### Follow-up tasks or validation needed

* Task 2C: capture errno, classify create/open failure, reclaim one approved candidate, retry once, and report specific diagnostic status.

---

## Decision 9 — Historical FRAM overrun must be separated from active/current FRAM pressure

Status: settled conceptually; implementation pending
Reference: SD/FRAM failure analysis

### Decision

The project model shall distinguish historical FRAM data-loss evidence from current active FRAM pressure or active overwriting.

### Rationale

A cumulative overrun total is useful audit data, but it should not necessarily behave as an active system error after SD recovery and FRAM drain.

### Evidence or context references

* SD/FRAM analysis identifies stale cumulative FRAM overrun state as a likely cause of misleading `fram overrun - active` alerts after recovery.
* Repeated stale `fram_overrun - active` ntfy after SD recovery and FRAM drain is a known deferred defect.

### Consequences

* Future FRAM work should introduce or clarify fields for current pressure, session overrun, lifetime overrun, unacknowledged historical loss, and notification state.
* ntfy/display behavior should not spam active alerts for stale historical state.
* Static SD/PTLOG path-workspace planning and implementation must not mix in FRAM active-vs-historical behavior changes.

### Rejected alternatives

* Keep deriving active overrun solely from cumulative total greater than acknowledged total.
* Hide historical data loss entirely.
* Treat historical and active states as the same condition.

### Follow-up tasks or validation needed

* Implement FRAM active-vs-historical overrun model in a later dedicated task.
* Update ntfy and display/status wording accordingly in that dedicated FRAM/status scope.
* Add tests for post-recovery stale-overrun scenarios.

---

## Decision 10 — `SDOUT` is overloaded and must not be treated as precise out-of-space terminology

Status: settled as deprecated terminology; replacement taxonomy pending
Reference: SD/FRAM failure analysis

### Decision

`SDOUT` shall not be treated as a precise “out of byte space” status. It is an overloaded attention code and should be replaced or supplemented by more specific SD states in a later task.

### Rationale

The incident involved SD access/create failure while free bytes were available. `SDOUT` does not distinguish card missing, mount failure, I/O error, byte-space pressure, or create/directory-entry failure.

### Evidence or context references

* SD/FRAM analysis notes `SDOUT` conflates multiple SD conditions.

### Consequences

* Future status/display work should split SD states.
* Tasks should not preserve `SDOUT` wording as design authority.

### Rejected alternatives

* Continue using `SDOUT` for all SD failures without clarification.
* Treat `SDOUT` as confirmed byte-space exhaustion.

### Follow-up tasks or validation needed

* Add or refine SD status codes such as missing, mount failure, I/O error, byte full, create/directory failure, and degraded/backoff.
* Update display and ntfy behavior in a dedicated task.

---

## Decision 11 — Host tests do not prove hardware SD/FAT behavior

Status: settled
Reference: PR #368, PR #369 reviews

### Decision

Host tests are valid for deterministic logic such as path building, parsing, candidate selection, and test-local reclaim behavior, but they do not prove ESP-IDF FATFS behavior on real SD media.

### Rationale

Linux filesystems and temporary directories do not reproduce FAT16 root-entry limits, ESP-IDF FATFS error behavior, SPI timing, or embedded mount/open/unlink behavior.

### Evidence or context references

* PR #368 and PR #369 both had host tests and ESP-IDF builds.
* Device SD-card runtime validation was not performed in the available context.

### Consequences

* Do not claim hardware validation unless it was performed.
* SD/FAT behavior remains environment-limited until tested on target hardware/media.

### Rejected alternatives

* Treat host tests as sufficient proof of FAT16 runtime behavior.
* Treat an ESP-IDF build as hardware validation.

### Follow-up tasks or validation needed

* Run hardware validation on real SD media with compact nested logs, ignored old root/long files, forced reclaim, and current-date protection checks.

---

## Decision 12 — Codex tasks must inspect current source and avoid broad work

Status: settled
Reference: Current workflow

### Decision

Every Codex planning, execution, review, validation, and next-action prompt must inspect the current source directly when source is available and must state the exact inspected context.

### Rationale

Receipts, old plans, comments, and prior summaries can become stale. The project is moving quickly and relies on precise branch/commit state.

### Evidence or context references

* Multiple recent tasks required source inspection before action.
* Branch aliases such as `work` can hide the actual product branch identity.
* PR head SHAs can differ from local receipt SHAs.

### Consequences

* Codex must report local path, branch, HEAD, remotes, status, anchors, and key files.
* ChatGPT reviews must verify PR head/current code where available.
* Tasks should remain narrow to conserve Codex usage.

### Rejected alternatives

* Rely only on previous receipts.
* Treat local branch name `work` as product branch name.
* Allow Codex to choose product policy.

### Follow-up tasks or validation needed

* Maintain project anchors so future tasks need less repeated context.
* Add validation ledger and roadmap anchors.

---

## Decision 13 — Anchor files are useful but should be created as explicit docs/admin tasks

Status: provisional
Reference: Current anchor drafting work

### Decision

Project intent, requirements/constraints, roadmap, decision log, documentation policy, and validation ledger anchors should be added or updated explicitly, not as incidental changes in unrelated firmware tasks.

### Rationale

Anchors reduce prompt size and preserve project policy, but creating them during implementation tasks risks scope creep.

### Evidence or context references

* Project intent and requirements/constraints anchors were drafted in chat but are not verified as committed.
* Decision log anchor is currently absent from `main`.

### Consequences

* Future Codex tasks should inspect anchors when present.
* Missing anchors should be reported, not silently invented as source authority.
* Anchor creation should be a focused docs task.

### Rejected alternatives

* Mix anchor creation with SD firmware changes.
* Treat uncommitted chat drafts as repository authority without inspection.

### Follow-up tasks or validation needed

* Create `docs/project_intent_anchor.md`.
* Create `docs/project_requirements_constraints_anchor.md`.
* Create `docs/decision_log_anchor.md`.
* Consider `docs/project_roadmap_anchor.md`, `docs/code_documentation_policy_anchor.md`, and `docs/validation_ledger_anchor.md`.

---

# Deprecated assumptions

These assumptions are deprecated and should not guide future work:

1. All PTLOG files live at SD root.
2. Root-only reclaim is sufficient after nested monthly PTLOG writes.
3. Byte free space is the only storage-pressure signal that matters.
4. `SDOUT` means a single precise SD condition.
5. Historical FRAM overrun is equivalent to active current FRAM pressure.
6. Host tests prove real FAT16 behavior.
7. Codex receipt claims are authoritative without source inspection.
8. Local branch name `work` identifies the product branch.
9. Existing comments and helper names define current product intent.
10. Same-day revision files may be deleted automatically without a product decision.

---

## Decision 11 — Remove firmware tail repair/resume scanning in a future scoped task

Status: accepted direction; not implemented by this documentation task
Reference: Task 2B-3B-DOC3

### Decision

Runtime firmware tail repair/resume scanning is abandoned and should be removed in a later dedicated code task. Firmware should not spend RAM or runtime repairing uncertain previous log tails. On boot, SD remount, recovery, restart, header/signature uncertainty, or prior-file uncertainty, firmware should create a new same-day revision file instead of appending to and repairing the old one.

Old, corrupt, or partial revision files shall be preserved for host-side recovery. Firmware must still avoid consuming or discarding FRAM records until append and verify to the current active revision succeeds. Runtime tail-repair code and large tail-scan buffers are historical baggage from the earlier CSV sequence-resume design.

Suggested future implementation name: `2B-3B-2 — Remove firmware tail repair/resume scan and always create new revision on uncertainty`.

### Consequences

* Do not remove tail-repair code in documentation-only tasks.
* Future removal must preserve FRAM records until the active revision append/verify succeeds.
* Host tools become the recovery location for partial/corrupt prior revisions.
* CSV tail-scan buffer sizing/removal review remains a future scoped task, not part of the path-workspace implementation.

---

## Decision 12 — Host tools must tolerate corrupt PTLOG tails and deduplicate records

Status: accepted requirement for future host-tool work
Reference: Task 2B-3B-DOC3

### Decision

Host tools must provide tolerant PTLOG ingestion for approved revision files. They must scan all approved compact nested PTLOG revision files, tolerate partial or corrupt trailing records/lines, extract all valid records that can be parsed safely, and deduplicate by `record_id`.

They must preserve auditability by reporting skipped/corrupt records, file names, offsets or line numbers when available, and reason codes. One corrupt revision or file must not prevent processing of other valid files. Host tools must support the compact nested PTLOG layout `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`. Firmware-created root/legacy files remain ignored by firmware; any host-tool support for historical files must remain separate and explicit.

Suggested future task: `5A-PTLOG-HOST-RECOVER — Host-side tolerant PTLOG ingestion and corrupt-tail handling`.

# Misleading implementation artifacts discovered

1. Old root-only reclaim helpers were misleading after nested monthly PTLOG layout was introduced.
2. `SDOUT` is an overloaded display/status term.
3. `fram_overrun_active` naming and behavior are likely misleading when driven by historical cumulative totals.
4. FRAM warning text can imply live data loss when observing historical overrun state.
5. Post-append mutation of `record.flags` for `LOG_RECORD_FLAG_FRAM_FULL` may not affect the stored FRAM record.
6. Generic placeholder comments such as “Execute FunctionName” should not be treated as documentation quality.
7. PR or receipt statements that tests “exercise” a production static function should be checked against whether the actual function or a test-local approximation ran.

# Patterns, rules, comments, tests, and docs that should not be treated as authority

Do not treat these as product authority without verification:

1. Root-only directory scan patterns.
2. Date-group deletion patterns.
3. Overloaded status labels such as `SDOUT`.
4. Historical cumulative counters named as active errors.
5. Existing helper/function names that predate nested PTLOG layout.
6. Existing tests that only cover root paths.
7. PR bodies or receipts that overstate coverage.
8. Comments that describe old behavior after model changes.
9. Host tests as proof of hardware behavior.
10. Missing anchors as evidence that no policy exists; use current chat and reviewed docs until anchors are committed.

# Last updated context

This decision log was drafted from:

* Current conversation through merged PR #369.
* Merged PR #368: `Move PTLOG writes to FAT16-safe log directories`.
* Merged PR #369: `Refactor SD reclaim to use SdPtlogFindOldestCandidate and add reclaim unit tests`.
* Current `main/sd_logger.c` reclaim implementation after PR #369.
* Current `main/sd_ptlog_paths.c` bounded PTLOG candidate scanner.
* `docs/sd_fram_failure_2026_06_23_analysis.md`.
* Codex Task 2A planning and execution receipts.
* ChatGPT senior review of Task 2A execution.
* Drafted project intent and requirements/constraints anchors from this chat.

Unverified at this update:

* Real hardware SD/FAT16 validation after PR #368 and PR #369.
* Long-duration field behavior after nested monthly PTLOG writes and reclaim changes.
* Full host-tool support for nested `/logs/YYYY-MM/` discovery.
* Display/ntfy status split implementation.
* FRAM active-vs-historical overrun fix implementation.
* Whether project intent, requirements/constraints, decision log, roadmap, code documentation policy, or validation ledger anchors have been committed after this draft.


## Decision 10 — Force YYYYMMDD.RRR nested PTLOG filenames and magic identity

Status: Accepted in Task 2B-3A.

New firmware-created nested PTLOG files use `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`, where `RRR` is a decimal same-day revision from `000` through `999`. Revision overflow above `999` fails closed. The `.RRR` extension is revision metadata, not file-type metadata. Old root and nested long-name `.ptlog` files are ignored by firmware scanning, stats, and reclaim; no automatic migration or renaming is performed. PTLOG identity is formalized as the first-line ASCII magic `#PT100_LOG_V1`.

PR #371 and PR #372 are treated as closed, unmerged, and superseded context; PR #372's YYMMDDRR.PTL policy and earlier legacy-compatibility assumptions are not accepted filename design. Follow-up work remains staged: 2B-3B read-only bounded pressure/status scan model, 2B-3C compact display progress, and 2B-3D pressure-targeted reclaim after validation. No hardware validation is claimed by this decision.
