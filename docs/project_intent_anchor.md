# Project Intent Anchor

Suggested path: `docs/project_intent_anchor.md`

## Project purpose

This project is an ESP32-based PT100 mesh temperature datalogger system. Its purpose is to collect temperature samples from PT100 sensor nodes, preserve those samples reliably during intermittent SD-card or runtime failures, and produce durable log data that can later be reviewed, plotted, validated, and reported.

The system must prioritize unattended data integrity over convenience. Logging, buffering, recovery, diagnostics, and retention behavior must be predictable and conservative because the device may run for long periods without operator supervision.

## Primary users, operators, and systems

Primary users and systems are:

* Operators who run the datalogger in the field or on a bench and need clear status, recovery, and logging behavior.
* Developers maintaining embedded firmware, SD/FRAM logging, diagnostics, and host-side tools.
* ESP32 firmware nodes that acquire PT100 data and participate in the mesh.
* The root/GUI/logging node that receives samples, writes PTLOG files, buffers records in FRAM, and exposes status/diagnostic commands.
* Host-side tooling that reads PTLOG data for plotting, analysis, validation, and reporting.
* External notification/status paths such as display attention codes and ntfy alerts.

## Core workflows

Core workflows include:

1. Acquire PT100 temperature samples from sensor nodes.
2. Move samples through the mesh to the root logging node.
3. Buffer records in FRAM when SD write progress is unavailable or delayed.
4. Write durable PTLOG files to SD with headers, record IDs, and recoverable append behavior.
5. Resume or repair log tails after power loss or interrupted writes.
6. Reclaim old PTLOG files only under approved retention conditions.
7. Surface SD, FRAM, and storage-stall status through diagnostics, display, and notification paths.
8. Allow destructive SD formatting only as an explicit operator action.
9. Preserve enough diagnostic detail to distinguish missing media, mount failure, I/O failure, create/open failure, byte-space pressure, directory-entry pressure, and historical FRAM data loss.
10. Use host-side tools to inspect, plot, trim, and report logged PT100 data.

## Desired product behavior

The product should behave as follows:

* SD logging should be durable, append-safe, and recoverable after reset or power loss.
* PTLOG file identity is the first-line ASCII magic `#PT100_LOG_V1`; new `.RRR` extensions are revision metadata rather than file-type metadata.
* New PTLOG files should use the FAT16-safe nested monthly 8.3 layout:

  * `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`

  `RRR` is a decimal same-day revision from `000` through `999`; the extension is revision metadata, not file-type metadata.
* Old long-name `.ptlog` files and root-level PTLOG-looking files are no longer supported by firmware scanning, stats, or reclaim. They may remain on a card, but firmware does not migrate, rename, parse, count, or reclaim them automatically.
* Automatic retention/reclaim must only delete parsed, regular PTLOG files from approved locations.
* Automatic retention/reclaim must never delete:

  * non-PTLOG files;
  * system directories;
  * PTLOG-looking directories;
  * unknown nested directories;
  * current-date PTLOG files unless a later explicit emergency policy approves it.
* Directory traversal must be bounded and defensive.
* Reclaim should scan only the approved layout:

  * `/sdcard`
  * `/sdcard/logs`
  * `/sdcard/logs/YYYY-MM`
* Reclaim should select candidates by parsed date/revision, not directory enumeration order.
* Reclaim should use fixed-size buffers and fail closed on truncation or ambiguous paths.
* Storage code must preserve task stack headroom. Large SD/PTLOG traversal path buffers should be moved to a narrow centrally owned static path workspace where practical rather than accommodated by increasing task stack sizes.
* Reusable SD/PTLOG path-workflow buffers should use the narrow static path-workspace model, preferably PSRAM-backed when safe and available, and guarded by explicit ownership, lifetime, and serialization rules.
* FRAM should preserve records while SD is unavailable and should distinguish current pressure from historical overrun/data-loss state.
* Diagnostics should preserve low-level error detail where practical, especially operation, path, errno, and ESP error state.
* Display and alert states should communicate the actual condition rather than overloaded or stale symptoms.
* Host tools should primarily support the current compact nested layout; CSV import may remain as a generic host-side feature.

## Current known behavior

As of the latest reviewed context:

* PR #368 moved new daily PTLOG writes into `/logs/YYYY-MM/`, added daily PTLOG diagnostics, and surfaced the last daily PTLOG diagnostic through `diag sd`.
* PR #369 updated `SdLoggerReclaimSpaceLocked()` to use `SdPtlogFindOldestCandidate()` for bounded nested monthly reclaim.
* Current reclaim remains byte-space-triggered only.
* Current reclaim does not yet add file-count, root-entry, or estimated FAT long-filename entry-pressure thresholds.
* Current reclaim does not yet run on daily PTLOG create/open failure and retry.
* Current SD format remains FAT16 with 16 KiB allocation units unless changed by a later explicit decision.
* Current reclaim uses broad current-date protection; no current-open PTLOG path field is tracked in `sd_logger_t`.
* Current FRAM overrun semantics are still known to be potentially misleading: historical cumulative overrun can appear as an active condition.
* Current `SDOUT` terminology is overloaded and should not be treated as precise out-of-space terminology.
* Hardware SD-card runtime validation for the recent reclaim/path changes remains unverified in the available context.
* Upcoming 2B-3B storage work must use the revised static SD/PTLOG path-workspace direction before adding or preserving large PTLOG traversal buffers on task stack. Host tests do not prove ESP32 task-stack safety; future validation must inspect stack, static, heap, and PSRAM buffer behavior. The generic scratch-owner approach from PR #376 is abandoned for now and preserved only as reference.

## Explicit non-goals and exclusions

Do not infer or implement these without explicit approval:

* Do not migrate to FAT32 merely because FAT16 has limits.
* Do not change `.allocation_unit_size = 16 * 1024` without an explicit storage-format decision.
* Do not change `.max_files = 5` without a specific reason and validation plan.
* Do not delete non-PTLOG files automatically.
* Do not delete current-date PTLOG files automatically.
* Do not delete same-day old revisions automatically unless a later product policy approves it.
* Do not delete or clean unknown host-created directories automatically.
* Do not migrate, rewrite, move, parse, count, or reclaim old long-name/root PTLOG files unless explicitly scoped.
* Do not treat free byte space as the only long-term retention signal.
* Do not treat a historical FRAM overrun as automatically equivalent to current live data-loss pressure.
* Do not collapse distinct SD states into a single overloaded status if touching display/status behavior.
* Do not change display, ntfy, runtime-health, FRAM, or SD format behavior as part of unrelated SD logging tasks.
* Do not broaden Codex tasks into general cleanup when a narrow reliability fix was requested.
* Do not preserve old root-only reclaim patterns merely because they existed before the nested PTLOG layout.

## Canonical terminology

Use these terms consistently:

* **PTLOG**: The project log file format used for PT100 sample records.
* **Daily PTLOG**: A PTLOG file for one UTC date.
* **Nested monthly PTLOG path**: `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`; this is the only supported firmware PTLOG layout.
* **Old root PTLOG path**: root-level `.ptlog` or compact-looking files are obsolete and ignored by firmware scanning, stats, and reclaim.
* **Revision PTLOG**: A same-date PTLOG whose compact nested filename encodes the revision as `.RRR` from `000` through `999`.
* **Log root**: `/sdcard/logs`.
* **Month directory**: `/sdcard/logs/YYYY-MM`.
* **Reclaim**: Automatic deletion of old eligible PTLOG files to restore required storage margin.
* **Retention trigger**: A condition that causes reclaim to run.
* **Byte-space pressure**: Low free-byte condition reported through filesystem space info.
* **Directory-entry pressure**: FAT metadata pressure caused by many directory entries, especially long filenames.
* **Current-date protection**: Reclaim exclusion for all PTLOG files whose parsed date equals the currently active date.
* **Current-open protection**: Reclaim exclusion for the exact currently open file path. This is a desired precision model but is not currently tracked in `sd_logger_t`.
* **Historical FRAM overrun**: A durable record that data loss happened earlier.
* **Active FRAM pressure**: Current live condition where FRAM is full, above a warning threshold, or actively overwriting.
* **Storage stall**: A high-level condition where SD progress and FRAM drain progress are not adequate.
* **SD create/open failure**: Failure to create or open the daily PTLOG, possibly caused by FAT directory-entry exhaustion even when free bytes remain.

## Misleading or deprecated terminology, patterns, and rules to avoid preserving

Avoid preserving these unless a task explicitly requires compatibility:

* **`SDOUT` as “out of space”**: This term is overloaded. It may mean card missing, card present but not mounted, mounted but unusable, or possibly out of byte space.
* **`fram_overrun_active` as purely historical state**: Active should not mean “cumulative total is greater than acknowledged total” unless the task explicitly says so. Historical/unacknowledged and current/live pressure should be separate.
* **Root-only PTLOG assumptions**: New code must not assume all PTLOG files live directly under `/sdcard`.
* **Date-group deletion as the only reclaim model**: Current policy is one oldest eligible candidate per pass unless a later task changes that.
* **Free bytes as the only final retention model**: Byte-space reclaim exists, but directory/file-count pressure is still needed.
* **Broad catch-all cleanup tasks**: Reliability fixes should be narrow, testable, and tied to a specific failure model.
* **Placeholder documentation comments** such as comments that merely repeat function names or parameter names without explaining behavior or invariants.
* **Claims of hardware validation** unless real hardware/media was used and the exact scenario is described.

## Data and state lifecycle assumptions

The intended data lifecycle is:

1. Sensor data is captured and converted into records.
2. Records are buffered through runtime state and FRAM as needed.
3. SD logging writes records into the current daily PTLOG file.
4. PTLOG files are organized by UTC date and, for new files, by monthly directory.
5. FRAM preserves pending records while SD is unavailable or delayed.
6. Runtime drains FRAM to SD when SD returns to a usable state.
7. Reclaim deletes only old eligible PTLOGs when approved retention conditions require it.
8. Diagnostics and alerts should distinguish current failures from historical evidence.

State assumptions:

* SD mount state, card presence, file open state, append progress, and daily PTLOG diagnostics are distinct.
* FRAM record count, FRAM capacity, FRAM overrun total, unacknowledged historical overrun, and active overwrite pressure are distinct.
* Formatting SD may clear active SD transient failure state, but must not erase FRAM contents unless explicitly designed and approved.
* Existing PTLOG files should remain readable and eligible for safe retention scanning unless explicitly excluded.
* PTLOG file deletion is destructive and must be restricted to parsed regular PTLOG candidates in approved paths.

## Safety, validation, logging, audit, and reliability expectations

General expectations:

* Embedded storage code must fail closed.
* Path construction must use bounded buffers and check truncation.
* Directory traversal must be bounded and must not follow arbitrary recursion.
* Automatic deletion must be limited to known-safe PTLOG candidates.
* Current-date files must be protected by default.
* Non-PTLOG files and system directories must be preserved.
* Low-level errors should preserve `errno`, ESP error code, operation, and path where practical.
* Recovery behavior should avoid repeated alerts for stale historical state.
* Host tests should cover real failure scenarios, not only import/compile paths.
* Firmware build validation is required before merge when embedded source changes.
* Hardware validation is required before claiming SD/FAT behavior is proven.
* Codex tasks must state exactly what source, commit, branch, PR, anchors, and receipts were inspected.

Audit and logging expectations:

* Log reclaim deletion path, parsed date, and revision for compact nested candidates.
* Log unlink failures with path and errno.
* Log daily PTLOG create/open diagnostic stage and path when available.
* Do not log misleading “current data loss” wording when only historical FRAM overrun state was observed.
* Keep repeated runtime logs concise; avoid alert spam from stale state.

## Architecture and model principles

Use these principles for future implementation:

* Prefer small, focused tasks with explicit exclusions.
* Treat source code, comments, tests, and receipts as claims to verify, not design authority.
* Use domain-specific helpers for storage/path behavior instead of duplicating parsing logic.
* Keep SD path parsing and bounded traversal centralized in PTLOG path helper code.
* Keep product policy decisions out of incidental implementation work.
* Separate mechanism from policy:

  * candidate scanning is mechanism;
  * retention thresholds are policy;
  * create/open retry rules are policy;
  * current-day deletion is policy.
* Prefer one-candidate-per-pass reclaim over large in-memory candidate arrays.
* Ignore old root/long-name PTLOG-looking files while using compact nested monthly paths.
* Keep host-testable seams small and purpose-specific.
* Avoid broad refactors unless the existing model is wrong and replanning has approved the change.
* Do not change display/notification semantics inside unrelated storage tasks.

## Known risks and failure scenarios

Known risks:

* FAT16 root directory or directory metadata exhaustion from long filenames.
* FAT long-filename directory-entry consumption not reflected in free-byte counts.
* Daily PTLOG `fopen` failure with `EACCES`, `ENOSPC`, or similar create denial while free bytes remain.
* ESP-IDF FATFS behavior differing from desktop Linux behavior.
* SD/SPI reliability issues that mimic storage failures.
* Host-created system directories such as `FOUND.000`, `.Trash-1000`, or `System Volume Information`.
* Root-only or unbounded traversal accidentally deleting the wrong files.
* Historical FRAM overrun repeatedly alerting as active after recovery.
* `LOG_RECORD_FLAG_FRAM_FULL` mutation after append not affecting the stored record.
* Operator confusion from overloaded display codes such as `SDOUT`.
* Hardware behavior remaining unverified when only host tests passed.

Real failure scenarios to keep in mind:

* A FAT16 card has plenty of free bytes but cannot create the next daily PTLOG because directory entries are exhausted.
* Old root PTLOG-looking files may remain on the card, but firmware ignores them for scan/stats/reclaim.
* New nested monthly PTLOGs accumulate and root-only reclaim finds nothing.
* A malformed or host-created directory contains PTLOG-looking names; automatic reclaim must not traverse or delete them.
* FRAM drains successfully after SD recovery, but stale overrun totals continue to trigger active alerts.
* SD format succeeds, but stale SD backoff or FRAM active-alert state remains latched.

## Open product decisions

These require explicit approval before implementation:

* Exact total PTLOG file-count threshold.
* Exact per-directory PTLOG file-count threshold.
* Exact root-level PTLOG count or estimated directory-entry threshold.
* Whether automatic reclaim may ever delete current-date PTLOGs.
* Whether same-day old revision PTLOGs may be deleted automatically.
* Whether empty month directories should be removed after reclaim.
* Whether to add and maintain a current-open PTLOG path field.
* Whether create/open failure should reclaim one file or multiple files before retry.
* Which errno values should trigger create/open reclaim retry.
* Compact nested PTLOG filenames are required as `YYYYMMDD.RRR`; legacy long filename compatibility is intentionally removed.
* Exact display/status codes replacing or supplementing `SDOUT`.
* Exact FRAM overrun acknowledgement and notification policy.
* Whether `LOG_RECORD_FLAG_FRAM_FULL` should be fixed before append or removed as a per-record flag.
* Whether project anchor files should be required before future execution tasks.
* Whether a separate validation ledger and decision log should be added.

## Roadmap and reference anchors

Current known roadmap document:

* `docs/sd_fram_failure_2026_06_23_analysis.md`

Recommended anchors to add if not present:

* `docs/project_intent_anchor.md` — this file.
* `docs/project_roadmap_anchor.md` — current ordered implementation roadmap and active task stack.
* `docs/project_requirements_constraints_anchor.md` — hard constraints, approved exclusions, hardware/media assumptions, and product policies.
* `docs/code_documentation_policy_anchor.md` — code comment/docstring policy for embedded C, host tools, and tests.
* `docs/decision_log_anchor.md` — product decisions, date, rationale, and affected code.
* `docs/validation_ledger_anchor.md` — build, host test, hardware test, SD media test, and known unverified items.

## Last updated context

This anchor was drafted from the following context:

* Current conversation through Task 2A.
* Merged PR #368: `Move PTLOG writes to FAT16-safe log directories`.
* Merged PR #369: `Refactor SD reclaim to use SdPtlogFindOldestCandidate and add reclaim unit tests`.
* Current `main/sd_logger.c` reclaim implementation after PR #369.
* Current `main/sd_ptlog_paths.c` bounded PTLOG path and candidate scanner implementation.
* `docs/sd_fram_failure_2026_06_23_analysis.md`.
* Codex Task 2A planning and execution receipts.
* ChatGPT senior review of Task 2A execution.

Unverified at this update:

* Real hardware SD/FAT16 validation of nested PTLOG write/reclaim behavior.
* Actual long-duration field behavior after PR #368 and PR #369.
* Whether host tools fully support nested `/logs/YYYY-MM/` PTLOG discovery.
* Whether current display/ntfy state names have been corrected.
* Whether FRAM active-vs-historical overrun semantics have been fixed.
* Whether formal roadmap, requirements, documentation policy, decision log, or validation ledger anchors exist in the repository after this draft.
