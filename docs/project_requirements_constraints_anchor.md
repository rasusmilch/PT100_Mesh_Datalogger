# Project Requirements and Constraints Anchor

Suggested path: `docs/project_requirements_constraints_anchor.md`

## Product requirements

The PT100 mesh datalogger shall prioritize unattended data preservation, clear diagnostics, and safe recovery over convenience or broad automatic cleanup.

Core product requirements:

1. The firmware shall collect PT100 temperature samples and preserve them through SD-card interruptions where possible.
2. The root logging node shall write durable PTLOG files to SD storage.
3. The system shall use FRAM as a buffer when SD logging is unavailable, delayed, degraded, or recovering.
4. New PTLOG files shall use the nested monthly FAT 8.3 layout:

   * `/sdcard/logs/YYYY-MM/YYYYMMDD.RRR`

   `RRR` is a decimal same-day revision from `000` through `999`; the extension is revision metadata, not file-type metadata.
5. The firmware shall ignore old root-level and long-name `.ptlog` files for PTLOG scanning, stats, and reclaim.
6. SD reclaim shall delete only old eligible compact nested PTLOG files from approved locations.
7. SD reclaim shall preserve current-date PTLOG files unless a later explicit emergency policy approves otherwise.
8. SD diagnostics shall preserve enough detail to distinguish mount, card, byte-space, create/open, directory-entry, and I/O failure classes where practical.
9. FRAM diagnostics shall distinguish current live pressure from historical data-loss evidence.
10. Host tools shall continue to read and process compact PTLOG data required for plotting, reporting, and validation; CSV import may remain as a generic feature.
11. Host tools shall tolerate partial/corrupt PTLOG tails, process all valid compact nested revisions independently, deduplicate valid records by `record_id`, and report skipped/corrupt records with file/offset or line details when available.

## Workflow requirements

Developer workflow requirements:

1. Before any Codex planning, execution, review, validation, or next-action task, inspect the latest available source directly.
2. Prefer the latest user-provided local snapshot if available. If not available, inspect the current GitHub repository, PR, branch, or commit.
3. State exactly what source, branch, PR, commit, files, anchors, and receipts were inspected.
4. Treat Codex receipts, old plans, comments, tests, and code names as claims to verify, not authority.
5. Keep tasks narrow and staged.
6. Do not combine unrelated roadmap work into a single execute task.
7. For embedded firmware changes, require build validation with ESP-IDF before merge when available.
8. For storage/recovery behavior, distinguish host-test coverage from real hardware validation.
9. Do not claim hardware validation unless real target hardware/media was used and the scenario is described.
10. Preserve explicit exclusions in every task receipt.

Operator workflow requirements:

1. Operators shall have a clear way to inspect SD and FRAM status.
2. Destructive SD format shall remain an explicit operator action.
3. Runtime alerts shall distinguish active/current failures from stale historical conditions where practical. Repeated stale `fram_overrun - active` ntfy after SD recovery and FRAM drain is a known deferred defect, not expected product behavior.
4. Display/status codes shall not be treated as precise unless their semantics are explicit.
5. Recovery behavior should avoid requiring card format unless reclaim/retry policies cannot recover.

## Data and state requirements

PTLOG data requirements:

1. PTLOG files shall remain append-safe and recoverable after interrupted writes where practical.
2. New daily PTLOG file names shall include the UTC date as `YYYYMMDD` in `/logs/YYYY-MM/YYYYMMDD.RRR`.
3. New revision PTLOG files shall use decimal `.RRR` revision metadata from `000` through `999`; old long names with `-<revision>.ptlog` are ignored by firmware.
4. New daily PTLOG files shall be grouped under `/logs/YYYY-MM/`.
5. Root-level and old long-name PTLOG-looking files shall not be eligible for firmware scanning, stats, or reclaim.
6. Automatic retention shall use parsed PTLOG metadata, not directory enumeration order alone.
7. Automatic retention shall fail closed when names, paths, stat results, or directory traversal are ambiguous.
8. On boot, SD remount, recovery, restart, header/signature uncertainty, or prior-file uncertainty, firmware should create a new same-day revision file rather than repairing and appending to an uncertain old tail.
9. Old/corrupt/partial revision files shall be preserved for host-side recovery, and firmware must not consume/discard FRAM records until append/verify to the active revision succeeds.

FRAM state requirements:

1. FRAM total overrun count shall be treated as durable historical data-loss evidence.
2. FRAM active pressure shall represent current live pressure or active overwriting, not merely an old unacknowledged total.
3. Historical/unacknowledged FRAM overrun and current/live FRAM pressure shall be modeled separately.
4. FRAM contents shall not be cleared by SD format unless a later explicit policy approves that behavior.
5. After SD recovery, FRAM drain behavior shall preserve record ordering and record IDs.
6. Repeated `fram_overrun - active` ntfy after SD recovery and FRAM drain appears caused by historical cumulative overrun being treated as active/current pressure; later FRAM semantic work shall fix it, and unrelated SD storage tasks shall not modify ntfy/display/FRAM behavior unless explicitly scoped.

SD state requirements:

1. SD mounted/unmounted state, card-present state, create/open failure, I/O failure, byte-space pressure, directory-entry pressure, and degraded/backoff state are distinct.
2. Current daily PTLOG date shall be protected from reclaim.
3. Current open PTLOG path protection is desirable but not currently tracked; current-date protection is the current conservative guard.
4. Reclaim deletion count shall count only successful `unlink()` calls.
5. Reclaim shall recompute storage condition after each successful delete.

## Architecture and model constraints

General architecture constraints:

1. Keep storage/path behavior centralized in storage/path helper code where practical.
2. Avoid duplicating PTLOG name parsing in multiple modules.
3. Prefer bounded, fixed-buffer embedded logic, but do not treat fixed-size buffers as approval to place large storage path, file, candidate, scan-observation, or I/O buffers on a FreeRTOS task stack.
4. SD/PTLOG path, filename, month-directory, candidate-path, and related path-workflow buffers currently local to stack-heavy workflows should be moved to centrally owned static storage where practical. Prefer PSRAM-backed allocation/storage on the target when safe and available.
5. Largish reusable buffers should be centrally owned/static rather than repeatedly placed on task stacks. Do not increase FreeRTOS task stack sizes merely to compensate for storage/path stack pressure.
6. The abandoned generic storage scratch-owner direction from PR #376 is not the intended architecture and must not be merged as-is; it is preserved only on `archive/scratch-owner-pr376-0819853` for reference.
7. The next implementation must remain narrow: path/candidate buffer placement only, using a dedicated typed SD/PTLOG path workspace if useful. It must not create a generic allocator, slot pool, borrow/release API, or concurrency framework.
8. Future storage/path code must document ownership, lifetime, capacity, fail-closed behavior, diagnostics, and explicit runtime serialization or locking assumptions for shared/static workspace use.
9. Separate mechanism from policy:

   * candidate discovery is mechanism;
   * retention thresholds are policy;
   * create/open retry rules are policy;
   * current-date deletion is policy;
   * display/ntfy wording is product behavior.
10. Prefer one-candidate-per-pass reclaim over large in-memory candidate arrays.
11. Do not use unbounded recursive directory traversal on embedded firmware.
12. Do not follow arbitrary host-created directories.
13. Do not preserve obsolete root-only assumptions after the nested PTLOG layout.
14. Do not refactor broadly while fixing a specific reliability issue.
15. Do not change public/operator-visible behavior unless it is in scope.

PTLOG path constraints:

1. Approved automatic scan locations are:

   * `/sdcard`
   * `/sdcard/logs`
   * `/sdcard/logs/YYYY-MM`
2. Approved automatic delete candidates are regular files with parsed PTLOG names in approved locations.
3. Unknown subdirectories under `/logs` shall not be traversed unless they match `YYYY-MM`.
4. System directories such as `.Trash-1000`, `FOUND.000`, and `System Volume Information` shall be ignored by automatic retention.
5. Path joins shall check truncation and fail closed.
6. Scan/reclaim code shall remain bounded and non-recursive when storage path-workflow buffers are moved out of task stack.

## Security and permission constraints

1. Automatic firmware deletion shall be restricted to validated PTLOG candidates.
2. The firmware shall not automatically delete arbitrary user files, host-created system directories, or unknown directories.
3. Destructive operations such as SD format shall require explicit operator command/action.
4. Diagnostics may include path and errno information, but should avoid leaking unrelated sensitive data.
5. Network notification changes, if any, shall be scoped and reviewed separately.
6. Tasks shall not introduce new unauthenticated control paths or remote destructive operations without explicit approval.
7. Host tools shall not silently overwrite source data unless explicitly designed and documented.

## Validation and testing requirements

General validation requirements:

1. Every execute task shall list exact commands run and exact results.
2. Tests must be behavior-based where practical, not only compile/import smoke tests.
3. Host tests are acceptable for path parsing, candidate selection, and small deterministic helpers.
4. ESP-IDF build is required for embedded firmware changes before merge when `idf.py` is available.
5. If `idf.py` is unavailable, the exact error shall be reported and firmware build shall remain unverified.
6. Hardware validation is required before claiming real SD/FAT behavior is proven.
7. `git diff --check` shall be run for code changes.
8. Final receipts shall identify unverified and environment-limited items.

Storage-specific validation requirements:

1. PTLOG path construction tests shall cover base daily files and revision files.
2. PTLOG parsing tests shall reject malformed names and overflow revisions.
3. Candidate traversal tests shall cover root and nested candidates.
4. Candidate traversal tests shall verify current-date and current-path protection where applicable.
5. Reclaim tests shall verify:

   * nested candidate selection;
      * current-date protection;
   * same-date revision protection;
   * non-PTLOG preservation;
   * PTLOG-looking directory preservation;
   * system directory exclusion;
   * malformed month directory exclusion;
   * max delete limit behavior;
   * successful delete counting;
   * unlink-failure no-count behavior.
6. Hardware SD validation should include real FAT16 media, nested log directories, old ignored root files, low-space or forced reclaim conditions, and current-date file protection.

## Documentation requirements

1. Project anchors shall be concise and reusable.
2. Do not paste raw receipts into anchors.
3. Anchors shall summarize current intent, requirements, constraints, decisions, risks, and validation gaps.
4. Markdown anchor files should include a “Last updated context” section.
5. Code comments shall explain non-obvious behavior, invariants, safety constraints, or failure models.
6. Avoid placeholder comments that restate function names or parameter names without adding meaning.
7. Operator documentation shall be updated when commands, public workflows, display/status behavior, setup, or expected operator action changes.
8. PR bodies shall accurately distinguish direct production-function tests from test-local approximations.
9. If a code documentation policy anchor exists, future source changes shall read and follow it.
10. If no code documentation policy anchor exists, embedded tasks shall include focused documentation requirements.

## Deployment and environment constraints

1. Target firmware environment is ESP-IDF on ESP32-class hardware.
2. SD storage currently uses FAT-compatible ESP-IDF VFS behavior.
3. FAT16 with 16 KiB allocation units is an accepted current configuration.
4. `.allocation_unit_size = 16 * 1024` shall not be changed without explicit approval.
5. `.max_files = 5` shall not be changed without explicit approval and validation.
6. Real SD/FAT behavior may differ from Linux host filesystem behavior.
7. Host tests may use Linux temporary directories to simulate paths, but such tests do not prove FAT directory-entry behavior.
8. Some Codex/local environments may lack configured git remotes.
9. Some Codex/local environments may lack `idf.py`.
10. Codex may check out work into a local branch named `work`; this name shall not be treated as the product branch name.

## Code style and maintainability requirements

1. Prefer small, readable C helpers over large mixed-responsibility functions.
2. Use fixed-size buffers and explicit bounds checks in embedded path/storage code.
3. Preserve existing public interfaces unless a task explicitly changes them.
4. Remove dead static helpers when replacing obsolete behavior.
5. Avoid broad reformatting or unrelated cleanup.
6. Log useful failure detail without creating noisy repeated logs.
7. Keep task-specific comments focused on safety and model decisions.
8. Avoid duplicating parsing/validation rules in multiple places.
9. Prefer deterministic candidate ordering.
10. Keep host-test helpers small and purpose-specific.
11. Do not create a large fake ESP-IDF harness unless a task explicitly approves it.
12. Any execute task touching storage/path code shall report new stack buffers by file/function/size, new static buffers by owner/lifetime/size, new heap or PSRAM allocations with fallback behavior, and task-safety/locking assumptions for shared static path workspaces.
13. Future storage tasks shall not silently add or preserve large stack buffers; if a small stack path-fragment buffer is necessary, the task receipt shall justify it and distinguish it from reusable static SD/PTLOG path-workflow buffers.

## Prompting and Codex receipt formatting requirements

Codex planning tasks shall require:

1. Latest source inspection before planning.
2. No file changes.
3. Verified current behavior by file/function.
4. Comparison of current behavior to intended behavior.
5. Explicit behavior gap.
6. Architecture/model/pattern/rule checkpoint.
7. Minimal safe implementation path.
8. Split-vs-single-task recommendation.
9. Proposed tests.
10. Documentation requirements.
11. Risks, unverified items, product decisions, and exclusions.
12. Complete planning receipt inside one copy/paste-ready `~~~text` fence.

Codex execute tasks shall require:

1. Latest source inspection before editing.
2. Implementation of only approved behavior.
3. Preservation of prior validated behavior.
4. Avoidance of future roadmap work.
5. Behavior tests for intended workflow and real failure scenarios.
6. Documentation compliance.
7. Exact command results.
8. Final receipt inside one copy/paste-ready `~~~text` fence.
9. No nested fences in receipts.

Codex receipts shall include at minimum:

1. Inspected context.
2. Source state and branch/commit identity.
3. Anchor files and docs inspected.
4. Summary of changes or planning decision.
5. Files changed, if any.
6. Verified behavior.
7. Tests and commands.
8. Acceptance criteria.
9. Documentation compliance.
10. Explicitly preserved exclusions.
11. Risks and unverified items.
12. Environment-limited items.
13. Next recommended action.

## Scope-control rules

1. Do not let Codex decide product policy.
2. Mark undecided behavior as a product decision.
3. Do not broaden a task because related issues are visible.
4. Do not start the next roadmap task until the current PR has at least build validation.
5. Do not claim validation that was not performed.
6. If a task touches embedded storage, preserve safety exclusions unless explicitly changed.
7. If a task reveals a wrong model, stop and replan instead of patching around it.
8. Do not combine SD path/reclaim changes with FRAM semantic changes.
9. Do not combine display/status/ntfy wording changes with storage mechanism changes unless explicitly scoped.
10. Do not create anchors during unrelated implementation tasks unless explicitly requested.

## Known project-specific hazards

1. FAT16 root-directory entry exhaustion from many long PTLOG filenames.
2. Long FAT filenames consuming multiple directory entries.
3. Free byte space appearing sufficient while directory-entry allocation fails.
4. Linux reading a card successfully while ESP-IDF FATFS fails create/open.
5. Root-only reclaim failing after logs move to nested monthly directories.
6. Automatic deletion accidentally targeting non-PTLOG files if parsing/traversal is too broad.
7. Current-date PTLOG deletion causing active logging data loss.
8. Historical FRAM overrun being displayed or alerted as active/live pressure.
9. Misleading `SDOUT` interpretation.
10. Post-append mutation of a record flag not affecting the already-stored FRAM record.
11. Environment receipts that report local branch `work` without remote metadata.
12. PR or receipt claims overstating test coverage.

## Settled requirements

These are currently settled unless the user explicitly changes them:

1. New daily PTLOG files use nested monthly directories with `YYYYMMDD.RRR` FAT 8.3 basenames.
2. Legacy/root long-name PTLOG compatibility is intentionally removed; old files are ignored, not migrated.
3. Automatic PTLOG traversal is bounded to approved locations.
4. Automatic reclaim deletes only parsed regular PTLOG candidates.
5. Current-date PTLOG files are protected.
6. Reclaim deletes one oldest eligible candidate per pass.
7. Reclaim currently remains byte-space-triggered only until threshold policy is added.
8. FAT16 / 16 KiB allocation unit configuration remains accepted.
9. Hardware validation is required before claiming SD/FAT runtime behavior is proven.
10. FRAM historical overrun and live pressure must be separated in future FRAM work.
11. Display/ntfy status changes are separate tasks.
12. Project tasks should minimize Codex usage through focused planning and execution.

## Provisional requirements and product decisions still needed

These require user decision or explicit task approval:

1. Exact max total PTLOG file count.
2. Exact max PTLOG files per directory.
3. Exact compact nested PTLOG count threshold.
4. Exact estimated FAT directory-entry pressure threshold.
5. Whether reclaim should run before daily open based on file-count/directory-entry pressure.
6. Which daily `fopen`/create errors trigger reclaim and retry.
7. Whether create/open reclaim retry deletes one file or multiple files.
8. Whether current-date PTLOGs may ever be deleted in an emergency.
9. Whether same-day old revision PTLOGs may be deleted automatically.
10. Whether empty month directories should be removed after reclaim.
11. Whether to track current open PTLOG path in `sd_logger_t`.
12. Compact PTLOG names are mandatory for firmware PTLOG files as `YYYYMMDD.RRR`; legacy `.ptlog` compatibility is removed.
13. Replacement display/status code taxonomy for `SDOUT`.
14. FRAM acknowledgement policy for historical data loss.
15. Whether `LOG_RECORD_FLAG_FRAM_FULL` should be fixed before append or removed.
16. Whether formal decision log and validation ledger anchors should be required for future tasks.

## References and related anchors

Known current reference:

* `docs/sd_fram_failure_2026_06_23_analysis.md`

Recommended related anchors:

* `docs/project_intent_anchor.md`
* `docs/project_roadmap_anchor.md`
* `docs/code_documentation_policy_anchor.md`
* `docs/decision_log_anchor.md`
* `docs/validation_ledger_anchor.md`

At this update, the requirements/constraints anchor itself was not present in current `main`.

## Last updated context

This anchor was drafted from:

* Current conversation through merged PR #369.
* Merged PR #368: `Move PTLOG writes to FAT16-safe log directories`.
* Merged PR #369: `Refactor SD reclaim to use SdPtlogFindOldestCandidate and add reclaim unit tests`.
* Current `main/sd_logger.c` reclaim implementation after PR #369.
* Current `main/sd_ptlog_paths.c` bounded PTLOG path and candidate scanner implementation.
* `docs/sd_fram_failure_2026_06_23_analysis.md`.
* Codex Task 2A planning and execution receipts.
* ChatGPT senior review of Task 2A execution.
* User direction to conserve Codex usage through narrow, staged tasks.

Unverified at this update:

* Real hardware SD/FAT16 validation after PR #368 and PR #369.
* Long-duration field behavior after nested monthly PTLOG writes and reclaim changes.
* Host tool support for nested `/logs/YYYY-MM/` discovery across all workflows.
* Whether display/ntfy state names have been corrected.
* Whether FRAM active-vs-historical overrun semantics have been fixed.
* Whether formal roadmap, code documentation policy, decision log, or validation ledger anchors exist in the repository after this draft.
