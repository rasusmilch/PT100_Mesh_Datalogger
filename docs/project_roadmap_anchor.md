# Project Roadmap Anchor

Suggested path: `docs/project_roadmap_anchor.md`

## Current project / workstream objective

Stabilize PT100 mesh datalogger SD/FRAM persistence so long-duration unattended logging is reliable, diagnosable, and recoverable.

The immediate workstream is the SD/FAT16 and FRAM-overrun recovery sequence triggered by the June 2026 storage incident. The working model is:

1. SD failure was most likely FAT directory-entry pressure from many long PTLOG filenames, especially at SD root.
2. FRAM later reported misleading active overrun state because historical cumulative overrun was treated like current live pressure.
3. SD storage mechanism fixes should be completed and validated before FRAM semantic fixes.
4. Each task should remain narrow to conserve Codex usage and prevent scope drift. Product intent remains unattended data preservation: avoid losing records, avoid firmware getting wedged by SD/FAT/path issues, avoid risky broad refactors, prefer bounded deterministic embedded behavior, and defer complex recovery/repair to host tools when that preserves data and reduces firmware risk.

## Current active feature / fix

Current active feature/fix: SD PTLOG storage and reclaim hardening.

Completed in this feature line:

* PR #368: Move new daily PTLOG writes into `/logs/YYYY-MM/`, add bounded path/candidate helper foundation, and add daily PTLOG diagnostics.
* PR #369: Wire bounded PTLOG candidate traversal into SD reclaim so reclaim can delete old eligible PTLOG files from compact nested monthly directories.

Current state after PR #369:

* New PTLOG writes use nested monthly directories.
* Legacy/root long-name PTLOG compatibility is removed; old files are ignored by firmware.
* Reclaim uses `SdPtlogFindOldestCandidate()`.
* Reclaim remains byte-space-triggered only.
* Current-date PTLOG files remain protected.
* No create/open reclaim retry exists yet.
* No file-count or directory-entry threshold policy exists yet.
* Hardware SD/FAT16 validation remains unverified in the available context.

## Phase 0 — Anchor and workflow stabilization

Purpose: reduce repeated context, prevent Codex scope drift, and make future task prompts smaller.

### 0A — Project intent anchor

Status: drafted in chat; repository commit unverified.

Create `docs/project_intent_anchor.md`.

Scope:

* Project purpose.
* Primary users/operators/systems.
* Core workflows.
* Desired behavior.
* Known behavior.
* Non-goals.
* Canonical terminology.
* Deprecated/misleading terminology.
* Data/state lifecycle assumptions.
* Safety/reliability expectations.
* Architecture principles.
* Risks and open decisions.

Task type:

* Focused docs execute task.

Branch/PR:

* May share one docs/admin draft PR with 0B, 0C, 0D, and 0E if no firmware source is touched.

Validation:

* Markdown review only.
* No firmware build required.

### 0B — Requirements and constraints anchor

Status: drafted in chat; repository commit unverified.

Create `docs/project_requirements_constraints_anchor.md`.

Scope:

* Product requirements.
* Workflow requirements.
* Data/state requirements.
* Architecture/model constraints.
* Security/permission constraints.
* Validation/testing requirements.
* Documentation requirements.
* Deployment/environment constraints.
* Code style and maintainability requirements.
* Prompting and receipt-format requirements.
* Scope-control rules.
* Known hazards.
* Settled vs provisional requirements.

Task type:

* Focused docs execute task.

Branch/PR:

* Can share the docs/admin anchor PR with 0A, 0C, 0D, and 0E.

Validation:

* Markdown review only.
* No firmware build required.

### 0C — Decision log anchor

Status: drafted in chat; repository commit unverified.

Create `docs/decision_log_anchor.md`.

Scope:

* Settled decisions from PR #368 and PR #369.
* Provisional decisions still needing product approval.
* Deprecated assumptions.
* Misleading implementation artifacts.
* Patterns that should not be treated as authority.

Task type:

* Focused docs execute task.

Branch/PR:

* Can share the docs/admin anchor PR with 0A, 0B, 0D, and 0E.

Validation:

* Markdown review only.
* No firmware build required.

### 0D — Roadmap anchor

Status: this document.

Create `docs/project_roadmap_anchor.md`.

Scope:

* Current workstream objective.
* Active feature/fix.
* Immediate tasks.
* Stabilization tasks.
* Pilot/validation tasks.
* Future tasks.
* Deferred/rejected paths.
* Sequencing and branch strategy.
* Planning vs execute requirements.
* High-risk checkpoints.
* Current next required action.

Task type:

* Focused docs execute task.

Branch/PR:

* Can share the docs/admin anchor PR with 0A, 0B, 0C, and 0E.

Validation:

* Markdown review only.
* No firmware build required.

### 0E — Code documentation policy anchor

Status: not drafted yet.

Create `docs/code_documentation_policy_anchor.md`.

Scope:

* Embedded C comment expectations.
* Host Python/tooling doc expectations.
* Doxygen/API comment rules.
* What comments to avoid.
* When docs must be updated.
* How to document safety-critical storage behavior.
* How to document test helper boundaries.

Task type:

* Read-only planning first if policy is not already agreed.
* Then focused docs execute task.

Branch/PR:

* Can share the docs/admin anchor PR if kept concise.

Validation:

* Markdown review only.
* No firmware build required.

## Phase 1 — Completed SD FAT16-safe layout foundation

### 1A — Bounded PTLOG path and candidate helper foundation

Status: completed in PR #368.

Outcome:

* Added PTLOG path helpers.
* Added nested monthly path construction.
* Added PTLOG filename parsing.
* Added bounded candidate traversal foundation.

Validation:

* Host PTLOG helper tests passed.
* Firmware built locally.
* Hardware SD runtime validation not performed.

### 1B — Wire daily writer to nested monthly paths

Status: completed in PR #368.

Outcome:

* New PTLOG files are written to `/logs/YYYY-MM/YYYYMMDD.RRR`.
* Same-day revisions are written under the same month directory.
* Legacy root compatibility preserved.
* Directory creation and daily PTLOG diagnostics added.

Validation:

* Host tests passed.
* Firmware built locally.
* Hardware SD runtime validation not performed.

### 1C — Daily PTLOG diagnostics

Status: completed in PR #368.

Outcome:

* Added diagnostics around path build, directory stat/mkdir/verify, revision scan, access, `fopen`, resume, header commit, and empty-file cleanup failures.
* Surfaced last daily PTLOG diagnostic through `diag sd`.

Validation:

* Firmware built locally.
* Hardware validation not performed.

## Phase 2 — SD reclaim and retention hardening

### 2A — Traversal-backed PTLOG reclaim

Status: completed in PR #369.

Outcome:

* `SdLoggerReclaimSpaceLocked()` now uses `SdPtlogFindOldestCandidate()`.
* Reclaim can select eligible compact PTLOGs from nested `/logs/YYYY-MM/`; root files are ignored by current policy.
* Reclaim deletes one candidate per pass.
* Reclaim recomputes free bytes after successful delete.
* Reclaim counts only successful `unlink()` calls.
* Current-date protection is preserved.
* No current-open path field was added.
* No file-count/directory-entry thresholds were added.
* No create/open retry was added.

Validation:

* Host path/reclaim-style tests passed.
* `git diff --check` passed.
* ESP-IDF firmware build was reported successful by user after local checkout.
* Hardware SD/FAT16 runtime validation not performed.

Branch/PR:

* Complete and merged.

Remaining checkpoint:

* Hardware validation still required before considering SD/FAT behavior fully proven.

### 2B — File-count and directory-entry-aware retention triggers

Status: next SD execute candidate after validation.

Goal:

* Add retention triggers beyond free byte space.

Likely scope:

* Count total PTLOG files across approved locations.
* Count PTLOG files per relevant directory.
* Add conservative estimated FAT long-filename directory-entry pressure metric if approved.
* Trigger reclaim when configured thresholds are exceeded.
* Preserve PTLOG-only, regular-file-only, bounded traversal, current-date protection, and one-candidate-per-pass deletion.
* Do not add create/open retry in this task unless explicitly approved.

Needs read-only planning first:

* Yes.

Why:

* Thresholds are product policy.
* Must decide exact compact nested limits; root legacy cleanup is not part of firmware reclaim.
* Must avoid Codex inventing retention policy.

Dependencies:

* 2A complete.
* Prefer hardware/build checkpoint after 2A before execution.
* Product decision required for threshold values.

Branch/PR:

* Separate draft PR from 2A.
* Can share one branch with only tightly related threshold/count code and tests.
* Do not combine with 2C.

High-risk checkpoint:

* Yes. Requires build and hardware/SD-media validation.

### 2C — Reclaim on daily create/open failure and retry

Status: deferred but important.

Goal:

* If daily PTLOG create/open fails with an approved errno/failure class, run reclaim even if free bytes are high, then retry open once.

Likely scope:

* Capture `errno` immediately on daily `fopen` failure.
* Classify operation as daily file open/create.
* If errno suggests space/create/directory-entry pressure, reclaim one approved candidate.
* Retry `fopen` once.
* Preserve diagnostics.
* If retry fails, report specific status/diagnostic.

Needs read-only planning first:

* Yes.

Why:

* Policy decisions are needed for errno list, retry count, candidate count, and status behavior.

Dependencies:

* 2A complete.
* Prefer 2B complete, unless a create/open failure is urgent enough to do 2C first.
* Requires precise diagnostic propagation review.

Branch/PR:

* Separate PR from 2B.
* Do not combine with display/ntfy status split.

High-risk checkpoint:

* Yes. Requires build plus hardware validation with induced create/open failure if feasible.

### 2D — SD diagnostic propagation and status specificity

Status: deferred after 2B/2C planning.

Goal:

* Preserve operation/path/errno/ESP error details from SD logger into runtime status and operator-visible diagnostics.

Likely scope:

* Structured SD failure detail.
* Last operation.
* Last path.
* Last errno and errno text.
* Whether reclaim was attempted.
* Whether create/open failure was suspected directory-entry pressure.

Needs read-only planning first:

* Yes.

Dependencies:

* 2C likely informs diagnostic fields.
* Must coordinate with display/ntfy split.

Branch/PR:

* Separate PR.
* May share with 2E only if planned together.

High-risk checkpoint:

* Moderate. Requires tests and operator-status review.

### 2E — SD display/status code split

Status: deferred.

Goal:

* Replace or supplement overloaded `SDOUT`.

Likely scope:

* Distinguish no card, mount failure, I/O error, out of byte space, create/directory failure, and degraded/backoff.
* Update display/status/ntfy wording only where scoped.
* Avoid changing storage mechanism in the same task.

Needs read-only planning first:

* Yes.

Dependencies:

* 2D recommended first.
* Must decide display code taxonomy.

Branch/PR:

* Separate PR from storage mechanism changes.

High-risk checkpoint:

* Medium. Operator-facing wording and alerts require review.

## Phase 3 — SD hardware and field validation

### 3A — Hardware validation after PR #368 and PR #369

Status: required; not completed in available context.

Goal:

* Validate nested write and traversal-backed reclaim on target ESP32 hardware and actual SD media.

Test matrix:

* Known-good industrial SD card.
* FAT16/current format.
* Old root/long-name PTLOG-looking files present and ignored.
* Nested `/logs/YYYY-MM/` PTLOGs.
* Low-free-space or forced reclaim condition.
* Current-date PTLOG present.
* PTLOG-looking directory present if safe to stage.
* Non-PTLOG file present.
* System directory present if naturally created.

Checks:

* New daily file creates under `/logs/YYYY-MM/`.
* Same-day revision behavior still works.
* Reclaim deletes old eligible nested PTLOG.
* Reclaim does not delete current-date PTLOG.
* Reclaim does not delete non-PTLOG files.
* Reclaim counters/status behave as expected.
* No unexpected mount/backoff behavior.

Needs read-only planning first:

* No, can be a validation checklist/task.

Branch/PR:

* No code branch required unless validation reveals defects.

High-risk checkpoint:

* Yes. This is the checkpoint before claiming SD/FAT behavior is proven.

### 3B — Long-duration pilot

Status: deferred until 2B/2C or at least 3A passes.

Goal:

* Run long enough to verify daily rollover, nested monthly file creation, FRAM drain, and absence of stale error states.

Scope:

* Multi-day logging.
* Date rollover.
* Optional month-boundary simulation if feasible.
* SD removal/reinsert recovery.
* FRAM buffering and drain.
* Status/ntfy observation.

Needs read-only planning first:

* No, but create a clear checklist.

Branch/PR:

* No code branch required unless defects are found.

High-risk checkpoint:

* Yes, field reliability checkpoint.

## Phase 4 — FRAM active-vs-historical overrun fix

### 4A — Plan FRAM overrun semantic split

Status: next major non-SD workstream after SD reclaim/create diagnostics are stable.

Goal:

* Separate historical data-loss accounting from current live FRAM pressure.
* Explicitly address repeated stale `fram_overrun - active` ntfy after SD recovery and FRAM drain.

Needs read-only planning first:

* Yes.

Why:

* This changes status/alert semantics and must not hide real data loss. The stale FRAM ntfy symptom is known and deferred; it is not part of static SD/PTLOG path-workspace work.

Dependencies:

* SD work should be stable enough that FRAM behavior can be evaluated without storage noise.
* Review current `fram_log.c`, `runtime_manager.c`, ntfy/status/display paths, and tests.

Branch/PR:

* Separate PR from SD tasks.

High-risk checkpoint:

* Yes. Alert semantics and audit behavior are high risk.

### 4B — Implement FRAM active-vs-historical status fields

Status: deferred until 4A plan approved.

Likely scope:

* Track lifetime overrun total.
* Track session baseline.
* Track last seen total.
* Track unacknowledged historical overrun separately from active live pressure.
* Alert on new/current overrun, not stale historical state.
* Preserve audit trail that data loss occurred.

Needs read-only planning first:

* Already covered by 4A.

Branch/PR:

* Separate PR.

Validation:

* Unit tests for boot with historical overrun.
* Unit tests for new overrun during session.
* Unit tests for SD recovery and FRAM drain clearing active pressure while preserving historical/unacknowledged state.

### 4C — Fix misleading FRAM warning logs

Status: deferred.

Goal:

* Replace live-overwrite wording when only historical overrun is observed.

Needs read-only planning first:

* Maybe not if 4A/4B already specify behavior.

Branch/PR:

* Can share with 4B if tightly scoped and planned.
* Otherwise separate.

### 4D — Fix or remove `LOG_RECORD_FLAG_FRAM_FULL`

Status: deferred.

Goal:

* Address post-append mutation of `record.flags` that may not affect stored FRAM record.

Needs read-only planning first:

* Yes.

Why:

* Must decide whether this flag is required as per-record audit data or should be removed.

Branch/PR:

* Separate PR unless 4A concludes it is part of the same status model.

## Phase 5 — Host tools and reporting alignment

### 5A — Verify host tools discover nested PTLOG files

Status: deferred but should occur before relying on field cards with nested logs.

Goal:

* Ensure host-side plotting/reporting/analysis tools handle `/logs/YYYY-MM/` and compact nested PTLOG files.

Needs read-only planning first:

* Yes, inspect host tools first.

Scope:

* Search for root-only assumptions.
* Add recursive/bounded discovery in host tools if needed.
* Preserve explicit user file selection behavior where present.
* Add tests with root and nested sample files.

### 5A-PTLOG-HOST-RECOVER — Host-side tolerant PTLOG ingestion and corrupt-tail handling

Status: deferred, required after firmware tail-repair direction changes.

Requirements:

* Scan all approved compact nested PTLOG revision files: `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`.
* Tolerate partial or corrupt trailing records/lines.
* Extract all valid records that can be parsed safely.
* Deduplicate by `record_id`.
* Report skipped/corrupt records with file names, offsets or line numbers when available, and reason codes.
* Do not let one corrupt revision/file prevent processing of other valid files.
* Keep any historical root/legacy file support separate and explicit because firmware ignores firmware-created root/legacy files.

Branch/PR:

* Separate host-tool PR.

High-risk checkpoint:

* Medium. Affects data retrieval/reporting, not embedded runtime.

### 5B — Validation ledger anchor

Status: deferred docs/admin task.

Goal:

* Track builds, host tests, hardware tests, SD media tests, known unverified items, and field pilot results.

Needs read-only planning first:

* No.

Branch/PR:

* Can share with docs/admin anchor PR if concise.

## Expansion / future tasks

These are future tasks, not part of the active SD/FAT16 fix unless explicitly promoted:

1. Task 2B-3A adopted new nested `YYYYMMDD.RRR` FAT 8.3 PTLOG filenames and `#PT100_LOG_V1` magic identity; next implement read-only pressure/status scanning.
2. Evaluate FAT32 migration only if field evidence justifies it.
3. Add richer SD diagnostics to ntfy messages.
4. Add operator acknowledgement workflow for historical FRAM data-loss state.
5. Add maintenance command to list PTLOG counts by root/month directory.
6. Add maintenance command to report estimated FAT directory-entry pressure.
7. Add optional empty-month-directory cleanup after retention, if approved.
8. Add current-open PTLOG path tracking, if same-day revision reclaim or more precise protection becomes necessary.
9. Add consolidated validation harness for SD/FRAM recovery scenarios.
10. Add field-pilot checklist and operator guide.

## Explicitly deferred tasks

Deferred until after SD Task 2A validation:

* File-count and directory-entry-aware retention triggers.
* Daily create/open reclaim retry.
* SD status/display/ntfy taxonomy replacement.
* FRAM active-vs-historical overrun semantics.
* `LOG_RECORD_FLAG_FRAM_FULL` fix/removal.
* Empty month directory cleanup.
* Short 8.3 filename design.
* FAT32 migration.
* Host tool recursive/nested discovery changes, unless host tool failure blocks data analysis.

## Explicitly rejected or deprecated paths

Rejected/deprecated unless a later decision reverses them:

1. Root-only PTLOG model.
2. Root-only reclaim.
3. Treating free byte space as the only final retention signal.
4. Treating `SDOUT` as precise “out of byte space.”
5. Treating historical cumulative FRAM overrun as current active pressure.
6. Unbounded SD directory traversal.
7. Automatic deletion of non-PTLOG files.
8. Automatic deletion of current-date PTLOG files.
9. Broad cleanup PRs mixed with reliability fixes.
10. Claiming hardware validation from host tests or firmware build alone.

## Task dependencies and sequencing

Current recommended sequence:

1. Complete the documentation correction that abandons the generic scratch-owner direction and records the revised static SD/PTLOG path-workspace roadmap.
2. Implement `2B-3B-1R` static PSRAM-backed SD/PTLOG path workspace only.
3. Implement `2B-3B-2` firmware tail-repair/resume removal and always-new-revision-on-uncertainty policy.
4. Implement `5A-PTLOG-HOST-RECOVER` host-side tolerant corrupt-log ingestion and `record_id` deduplication.
5. Plan and execute read-only bounded PTLOG pressure/status scan facts.
6. Plan Task 2B read-only: file-count/directory-entry-aware retention triggers.
7. Execute Task 2B after threshold policy approval.
8. Hardware validate Task 2B.
9. Plan Task 2C read-only: daily create/open reclaim retry.
10. Execute Task 2C after errno/retry policy approval.
11. Hardware validate Task 2C.
12. Plan and execute SD diagnostic/status split.
13. Plan and execute FRAM active-vs-historical overrun fix, including stale `fram_overrun - active` ntfy after recovery/drain.
14. Verify host tools discover nested PTLOG files.
15. Run long-duration pilot.

## Branch / PR grouping guidance

Tasks that can share one draft PR branch:

* Phase 0 anchor files can share one docs/admin PR if no firmware source is touched.
* A documentation-only validation ledger can share the same docs/admin PR if concise.

Tasks that should be separate PRs:

* Task 2B threshold-based retention triggers.
* Task 2C create/open reclaim retry.
* SD diagnostic/status propagation.
* SD display/ntfy status split.
* FRAM active-vs-historical overrun semantic fix.
* `LOG_RECORD_FLAG_FRAM_FULL` fix/removal.
* Host tool nested PTLOG discovery and tolerant corrupt-tail recovery/deduplication.

Do not combine:

* SD storage mechanism changes with FRAM semantic changes.
* Display/ntfy wording changes with reclaim mechanism changes.
* Product-policy threshold decisions with unrelated cleanup.
* Anchor docs with firmware behavior changes unless explicitly requested.

## Tasks requiring read-only planning first

Require read-only planning first:

1. Task 2B-3B-1R: static PSRAM-backed SD/PTLOG path workspace.
2. Task 2B: file-count/directory-entry-aware retention triggers.
3. Task 2C: create/open reclaim retry.
4. Task 2D: SD diagnostic propagation/status specificity.
5. Task 2E: SD display/status code split.
6. Task 4A/4B: FRAM active-vs-historical overrun model.
7. Task 4D: `LOG_RECORD_FLAG_FRAM_FULL` fix/removal.
8. Task 5A / 5A-PTLOG-HOST-RECOVER: host tools nested PTLOG discovery and tolerant corrupt-tail ingestion/deduplication.
9. Any FAT32 migration or short 8.3 filename change.
10. Any automatic deletion of current-date or same-day revision PTLOGs.

## Tasks that can be focused execute tasks

Can be focused execute tasks if already approved:

1. Create/update anchor docs from approved chat drafts.
2. Add validation ledger anchor.
3. Update PR/body wording or docs wording where behavior is already settled.
4. Add narrowly scoped tests for already-settled helper behavior.
5. Add hardware validation checklist docs.

## High-risk tasks requiring checkpoint validation

High-risk tasks:

1. File-count/directory-entry retention triggers.
2. Create/open reclaim retry.
3. Any automatic deletion policy change.
4. Any SD format or filesystem configuration change.
5. Any display/ntfy status taxonomy change.
6. FRAM active-vs-historical overrun semantic change.
7. Any host tool change that affects interpretation/export/reporting of logged data.
8. Any migration/rewrite/move of existing PTLOG files.

Checkpoint requirements:

* Source review.
* Host tests where practical.
* `git diff --check`.
* ESP-IDF build for firmware changes.
* Hardware validation for SD/FAT behavior.
* Operator-facing review for display/ntfy wording.
* Explicit list of unverified items.

## Completed tasks still needing integrated validation

Completed but not fully integrated-validated:

1. PR #368 nested monthly PTLOG writes.

   * Host tests passed.
   * Firmware build reported successful.
   * Hardware SD runtime validation not verified.

2. PR #369 traversal-backed PTLOG reclaim.

   * Host tests passed.
   * Firmware build reported successful.
   * Hardware SD/FAT16 reclaim validation not verified.

3. Drafted anchors in chat.

   * Project intent anchor drafted.
   * Requirements/constraints anchor drafted.
   * Decision log anchor drafted.
   * Roadmap anchor drafted here.
   * Repository commits unverified.

## Deferred/future scoped tasks that must remain visible

* `2B-3B-1R` static PSRAM-backed SD/PTLOG path workspace implementation.
* `2B-3B-2` firmware tail-repair/resume removal and always-new-revision-on-uncertainty policy.
* `5A-PTLOG-HOST-RECOVER` host-side corrupt log handling and `record_id` deduplication.
* PTLOG pressure/read-only scan facts.
* File-count/directory-entry-aware retention triggers.
* Daily create/open failure retry/reclaim policy.
* SD diagnostic/status specificity.
* Display/ntfy status wording split.
* FRAM active-vs-historical overrun semantic fix, including stale `fram_overrun` active alerts after recovery/drain.
* `LOG_RECORD_FLAG_FRAM_FULL` ordering/fix/removal if still applicable.
* Host tools nested PTLOG discovery/ingestion.
* Runtime file-buffer sizing review, including old 64 KiB fallback versus current Kconfig defaults.
* CSV tail-scan buffer sizing/removal review, including old 256 KiB fallback versus current configured default.
* Hardware validation after SD/PTLOG changes.
* Long-duration pilot validation.

## Current next required action

Recommended next actions, in order:

1. Documentation correction to abandon the generic scratch-owner direction and record the revised static SD/PTLOG path-workspace direction.
2. `2B-3B-1R — Replace SD/PTLOG stack path buffers with static PSRAM-backed path workspace`.
3. `2B-3B-2 — Remove firmware tail repair/resume scan and always create new revision on uncertainty`.
4. `5A-PTLOG-HOST-RECOVER — Host-side tolerant PTLOG ingestion and corrupt-tail handling`.
5. Resume the broader SD pressure/reclaim/status roadmap: read-only pressure scan facts, file-count/directory-entry retention triggers, create/open retry/reclaim policy, SD diagnostic specificity, display/ntfy wording split, FRAM semantic fixes, hardware validation, and long-duration pilot validation.

Do not merge PR #376 as-is. Its scratch-owner implementation is abandoned for now and preserved only on `archive/scratch-owner-pr376-0819853` for reference. Do not start read-only PTLOG pressure/status scan facts, retention thresholds, create/open retry, display/ntfy changes, FRAM semantic fixes, host-tool ingestion changes, or SD format changes as part of the path-workspace implementation. The repeated stale `fram_overrun - active` ntfy after SD recovery and FRAM drain remains a known deferred FRAM active-vs-historical semantic defect.

## Last updated context

This roadmap was drafted from:

* Current conversation through merged PR #369.
* Current GitHub repository `rasusmilch/PT100_Mesh_Datalogger`, default branch `main`.
* PR #368: `Move PTLOG writes to FAT16-safe log directories`, merged at `939ac9b717373a6ce5e9b38e9f4dfae112be2e43`.
* PR #369: `Refactor SD reclaim to use SdPtlogFindOldestCandidate and add reclaim unit tests`, merged at `1e9dffa51d6409c9c63dfbee5410525e9f406bbc`.
* Current `main/sd_logger.c` reclaim implementation after PR #369.
* Current `main/sd_ptlog_paths.c` bounded PTLOG scanner.
* `docs/sd_fram_failure_2026_06_23_analysis.md`.
* Codex Task 2A planning and execution receipts.
* ChatGPT senior review of Task 2A execution.
* Drafted project intent, requirements/constraints, and decision log anchors from this chat.

Unverified at this update:

* Real hardware SD/FAT16 validation after PR #368 and PR #369.
* Long-duration field behavior after nested monthly PTLOG writes and reclaim changes.
* Full host-tool support for nested `/logs/YYYY-MM/` PTLOG discovery.
* Display/ntfy status split implementation.
* FRAM active-vs-historical overrun fix implementation.
* Whether project intent, requirements/constraints, decision log, roadmap, code documentation policy, or validation ledger anchors have been committed after this draft.


## Task 2B-3 staged PTLOG filename, pressure scan, and reclaim work

### 2B-3A — YYYYMMDD.RRR filename and PTLOG magic foundation

Status: implemented in this branch. New nested firmware PTLOG files use `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR` with decimal revisions `000` through `999`; legacy/root long-name `.ptlog` compatibility is removed; old files are ignored by firmware. PTLOG identity is formalized around the first-line `#PT100_LOG_V1` magic. No migration, display progress, read-only pressure expansion, create/open retry, or reclaim policy change is included.

### 2B-3B-DOC1 / DOC2 — Superseded storage scratch-owner documentation

Status: superseded by Task 2B-3B-DOC3. Earlier anchors described a centrally owned PSRAM-backed storage scratch owner with a future slot-style design. That direction is abandoned for now.

### 2B-3B-DOC3 — Abandon scratch owner and record revised SD/PTLOG roadmap

Status: documentation-only correction. Record that PR #376 is not the intended architecture and must not be merged as-is; preserve it only on `archive/scratch-owner-pr376-0819853`; replace the generic scratch-owner path with a narrow static SD/PTLOG path-workspace direction; record the tail-repair removal decision; and add host-tool corrupt-log recovery/deduplication requirements.

### 2B-3B-1R — Replace SD/PTLOG stack path buffers with static PSRAM-backed path workspace

Status: next implementation after the documentation correction. Move SD/PTLOG path, filename, month directory, candidate path, and related path-workflow buffers that are currently local stack arrays to centrally owned static storage where practical. Prefer PSRAM-backed storage on target when safe and available. Keep the implementation narrow: path/candidate buffer placement only. Do not create a generic allocator, slot pool, borrow/release API, concurrency framework, CSV tail repair change, file-buffer sizing change, SD format change, reclaim policy change, display/ntfy wording change, FRAM semantic change, or host-tool ingestion change. Explicitly document ownership, lifetime, capacity, and runtime serialization/locking assumptions.

### 2B-3B-2 — Remove firmware tail repair/resume scan and always create new revision on uncertainty

Status: future dedicated implementation. Runtime firmware tail repair/resume scanning is abandoned. On boot, SD remount, recovery, restart, header/signature uncertainty, or prior-file uncertainty, firmware should create a new same-day revision file instead of repairing/appending to the uncertain old one. Old/corrupt/partial revision files are preserved for host-side recovery. Firmware must still avoid consuming or discarding FRAM records until append/verify to the active revision succeeds. This removes historical CSV sequence-resume baggage and associated large tail-scan buffers in a scoped code task, not in documentation.

### 2B-3B — Read-only bounded pressure/status scan model

Status: after 2B-3B-1R path-workspace implementation and tail-repair/host-recovery sequencing as approved. Add richer read-only scan facts without deletion or full-card recursion, but do not merely add scan facts on top of large task-stack PTLOG path buffers.

### 2B-3C — Compact MAX7219 scan progress

Status: later. Add throttled count-up display states only after scan/status architecture is validated.

### 2B-3D — Pressure-targeted reclaim

Status: later. Reclaim actions must directly relieve the pressure class; directory-count cleanup remains unapproved unless explicitly decided.
