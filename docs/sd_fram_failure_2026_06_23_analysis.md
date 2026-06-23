# PT100 SD creation failure and stale FRAM overrun analysis

Date of incident: 2026-06-22 to 2026-06-23

Related artifacts reviewed:

- Serial console logs from the recovery session.
- `2026-06-22Z.ptlog` from the SD card.
- `File_list.txt` showing Linux dmesg, free-space output, and SD-card root-directory listing.
- Uploaded `main/` firmware source snapshot.
- Connected GitHub repository source for `main/sd_logger.c`, `main/runtime_manager.c`, and `main/fram_log.c`.

This document records the working theory, code evidence, likely root causes, and proposed firmware fixes for the SD-card failure and the later misleading FRAM-overrun alert behavior.

## Executive summary

The incident appears to involve two related but separate problems.

First, the SD card most likely did not fail at the flash/media level. The best current explanation is FAT directory-entry exhaustion, most likely in the card root directory, when the firmware tried to create the next daily long-filename log file, `2026-06-23Z.ptlog`. Linux could read the card, saw no kernel I/O errors, showed plenty of free byte space, and listed the existing files. Formatting the card fixed the issue. That combination strongly points away from a bad card and toward filesystem metadata exhaustion or directory-entry allocation failure.

Second, after formatting and successfully draining FRAM, later `fram overrun - active` ntfy alerts were likely caused by stale cumulative FRAM overrun state being treated as an active condition. The current code appears to derive `fram_overrun_active` from whether cumulative overrun total exceeds an acknowledged baseline, not from whether FRAM is currently full or currently overwriting. That can keep a historical data-loss event active after the system has recovered.

The main SD-side fix is to stop keeping many long daily filenames in the FAT root directory and to add directory-entry/file-count-aware retention. The main FRAM-side fix is to separate current FRAM pressure from historical overrun counters and alert only on new/current overrun risk.

## Observed field behavior

During a long-running logging session with the serial monitor opened in `--no-reset` mode, ntfy repeatedly reported:

- `system error storage stall - active` twice.
- `system error storage stall - cleared`.
- `system error fram overrun - active`.

The LED display showed `SDOUT`.

After connecting to the serial console, the logs showed SD access failures. The SD card appeared unmounted or unusable from the datalogger. A soft reboot via the console `reboot` command and a hard power cycle by pulling USB power both resulted in the SD card mounting again but then failing with an access error.

Removing and reinserting the SD card did not recover operation in the datalogger.

Removing the card and reading it on Linux showed:

- No obvious kernel I/O errors in `dmesg`.
- The card appeared normally as a USB mass-storage device.
- The filesystem mounted.
- `df -h` showed plenty of free space.
- Existing file sizes looked normal.
- The root contained many `.ptlog` files and some filesystem-maintenance directories.

Reinserting the SD card into the datalogger still produced the same access issue.

Running the datalogger console command `sd format` successfully formatted the card. After that, `run start` successfully drained FRAM and logging appeared to work again.

After this recovery, ntfy still reported `system error fram overrun - active`, even though:

- The SD card was mounted.
- FRAM was not full.
- The serial console did not show current errors.
- `run stop` drained FRAM correctly.
- `run start` logged or displayed a misleading full/overrun condition.
- A manual status showed only a few records, for example approximately 5 records buffered.

## Important code paths and current behavior

### Daily file open/create path

The daily PTLOG file path is built by `SdLoggerEnsureDailyFileWithHeader()`. It checks mount state, builds the current UTC-date filename, closes the old file if needed, resets `last_record_id_on_sd`, then opens the new daily file with:

```c
logger->file = fopen(path, "a+b");
```

If `fopen` fails, the function logs the path, `strerror(errno)`, and `errno`, then returns `ESP_FAIL`.

Important current behavior:

- Failure is reduced to generic `ESP_FAIL` at the `sd_logger` API boundary.
- The useful low-level `errno` value is logged locally but not cleanly propagated to runtime status.
- There is no special handling for create/open denial due to directory-entry exhaustion.
- There is no create-failure-specific reclaim attempt.

### SD sync and append failure path

The runtime drain path calls `EnsureSdMountedLocked()` and `EnsureSdSyncedForEpoch()`. If sync fails, it unmounts the SD logger and calls `MarkSdFailure()`.

For append failures, it similarly unmounts and calls `MarkSdFailure()` with diagnostic operation and errno from append stats.

Important current behavior:

- Sync/open failure can unmount and back off.
- Repeated backoff/remount does not fix a persistent directory-entry exhaustion condition.
- Error escalation is focused on I/O-style errors such as `EIO`; a persistent `EACCES`/permission-denied create failure is not treated as a special recoverable-by-delete condition.

### Space reclaim path

`SdLoggerReclaimSpaceLocked()` uses `esp_vfs_fat_info()` to get total/free bytes, then deletes oldest `.ptlog` dates while `free_bytes < required_free_bytes`, up to a maximum delete count per attempt.

Important current behavior:

- Reclaim is byte-space-driven only.
- It does not run when byte free space is high.
- It does not account for root-directory entry exhaustion.
- It does not account for total PTLOG file count.
- It does not account for long-filename directory-entry consumption.
- It searches only the current mount point directory, not nested log directories.

### FRAM append and overrun counter behavior

`FramLogAppend()` increments `overrun_records_total` when the circular buffer is full and an append overwrites the oldest record.

That cumulative counter is expected and useful for durable data-loss accounting, but it is not the same as current live pressure.

Current runtime code calls `UpdateFramOverrunActive()` using the cumulative overrun total.

`UpdateFramOverrunActive()` computes:

```c
const bool overrun_active = (overrun_total > state->fram_overrun_ack_total);
```

It then uses active/clear streak thresholds to update `cached_status.fram_overrun_active`.

Important current behavior:

- `fram_overrun_active` can remain active when the current FRAM buffer is mostly empty.
- Active state is based on historical unacknowledged cumulative overrun, not necessarily current overwriting or current data-loss risk.
- This can produce repeated ntfy alerts even after SD recovery and FRAM drain.

### Misleading FRAM warning

`LogFramOverrunWarning()` logs:

```text
FRAM buffer full: overwriting oldest records (data loss possible until SD flush recovers).
```

when:

```c
state->last_overrun_records_total == 0 && overrun_total > 0
```

Important current behavior:

- This message can fire on first observation of historical overrun state after reboot/restart.
- It does not require the current buffer to be full.
- It does not require the overrun total to have advanced during the current append.
- The wording strongly implies a live/full condition even when FRAM currently contains only a handful of records.

### Post-append FRAM_FULL flag mutation

In the sample-storage path, the code appends the record to FRAM, then checks FRAM count/capacity and calls `UpdateFramFillState()`, then conditionally mutates `record.flags` with `LOG_RECORD_FLAG_FRAM_FULL`.

Important current behavior:

- The record has already been copied into FRAM by `FramLogAppend()`.
- Mutating `record.flags` after append does not affect the stored record.
- This is either dead code, incorrectly ordered, or misleading status logic.

### Display attention code

The display maps the SD-out attention bit to `SDOUT` when either:

- the SD card is not present, or
- the SD card is not mounted.

Important current behavior:

- `SDOUT` is not strictly out-of-space.
- `SDOUT` is not strictly card missing.
- In this incident it likely meant card present but SD unmounted or unusable after failure.
- The display code should distinguish card missing, mount failure, I/O failure, out of byte space, and create/directory failure.

## Why FAT root-directory entry exhaustion fits the evidence

The strongest theory is FAT root-directory entry exhaustion or closely related directory metadata exhaustion.

Evidence:

1. The datalogger could mount the card after reboot/power cycle but failed when trying to access/create the next daily file.
2. The low-level error observed in logs was `fopen failed for /sdcard/2026-06-23Z.ptlog: Permission denied (13)`.
3. Linux showed the card was readable and had free byte space.
4. Linux did not show obvious media errors in `dmesg`.
5. Formatting the card fixed the problem immediately.
6. The root directory had many long daily filenames.
7. The reclaim logic only considers free bytes and would not trigger when directory entries are exhausted.

The uploaded file listing showed about 171 visible root entries, mostly long daily names like:

```text
2026-06-22Z.ptlog
2026-06-21Z.ptlog
2026-06-20Z.ptlog
```

A long FAT filename consumes multiple directory entries. A name like `2026-06-22Z.ptlog` is not 8.3 format. In a typical FAT long-filename implementation it can consume multiple directory entries: one short-name entry plus one or more long-name entries.

A rough estimate:

```text
171 visible entries * about 3 FAT directory entries per long filename ~= 513 directory entries
```

Classic FAT16 root directories often have a fixed 512-entry root directory. Even if the exact card format varies, this estimate is close enough to explain the failure mechanism.

The old files remained readable because reading existing entries does not require allocating a new directory entry. Creating `2026-06-23Z.ptlog` required allocating new directory entries and failed.

That also explains why removing/reinserting the card and rebooting did not help. The filesystem metadata state persisted. Formatting cleared the directory and allowed file creation again.

## Why this is probably not a bad SD card

A bad or failing SD card is possible, but it is not the leading theory from the current evidence.

Reasons:

- Linux could read the card.
- Linux reported plenty of free space.
- File sizes looked normal.
- No kernel I/O errors were observed when reading with the USB reader.
- Formatting fixed the issue immediately.
- The failure was tightly tied to daily-file creation/opening.

A card/media problem would more likely produce read/write I/O errors, mount failure, CRC/timeouts, or recurring errors even after format. The observed behavior fits filesystem metadata exhaustion better.

## Why ntfy reported storage stall

The storage-stall logic uses both FRAM pressure and SD issue state.

The storage-stall condition looks for progress via either:

- SD last record ID increasing, or
- FRAM count decreasing.

It considers FRAM pressure active if FRAM overrun is active or if FRAM count is above the flush watermark.

It considers SD issue active if SD is degraded, SD is unmounted while card present, SD is in backoff, SD fail count increased, SD I/O error is active, or out-of-space state is active.

In this incident:

```text
SD daily file creation failed
-> runtime unmounted SD and marked SD degraded/backoff
-> FRAM continued accumulating records
-> FRAM reached capacity and overran
-> storage stall condition became active
-> intermittent state changes caused active/cleared/active alerts
```

So the ntfy storage-stall alert was a valid high-level symptom, but it did not identify the underlying file-create/directory-entry issue.

## Why FRAM overrun stayed active after recovery

`fram_overrun_active` is currently based on cumulative overrun total compared against an acknowledged count.

This means a historical overrun can remain active even after:

- SD is mounted.
- FRAM is drained.
- Current FRAM count is low.
- No current write errors are happening.

That makes the alert semantically wrong for live monitoring. It is useful to know that data loss happened, but that should be a historical or unacknowledged event, not an active system error once the system has recovered.

The correct model should separate:

- FRAM currently full.
- FRAM currently above warning watermark.
- FRAM currently overwriting records.
- FRAM overrun happened during this boot/run.
- FRAM overrun total lifetime.
- FRAM overrun has been acknowledged by operator.
- FRAM overrun event has been notified.

## Root-cause theories ranked

### Theory 1: FAT root-directory entry exhaustion from many long filenames

Confidence: high.

This explains:

- `fopen` permission denied on new daily file.
- Existing files still readable.
- Free bytes still available.
- Linux dmesg clean.
- Format fixes the issue.
- Many long filenames in root.
- Reclaim not triggering because free bytes were fine.

### Theory 2: FAT metadata corruption that Linux tolerated but ESP-IDF FATFS refused

Confidence: moderate.

This is possible because the card had `FOUND.000`, and embedded FAT stacks can be less tolerant than desktop Linux. However, the file count and long-name entry estimate make directory-entry exhaustion the cleaner theory.

### Theory 3: SD/SPI electrical or timing failure

Confidence: low to moderate for the primary incident.

SPI reliability issues can cause intermittent SD failures, but they do not explain why Linux saw a clean filesystem with free space and why format fixed the issue. This should remain a secondary check, especially if failures recur after directory/file-count fixes.

### Theory 4: Bad SD card

Confidence: low for this specific incident.

The behavior is more consistent with filesystem metadata exhaustion than flash failure. Continue using a known-good industrial SD card, but firmware should not depend on card replacement to solve this.

### Theory 5: Serial logger/no-reset caused the problem

Confidence: very low.

The serial logger was relevant for observing the logs without resetting. The SD failure occurred in the datalogger firmware SD file-open/sync path.

## Proposed fix group 1: Move logs out of root and/or use shorter names

### Preferred fix: monthly subdirectories

Store logs under monthly directories:

```text
/sdcard/logs/2026-06/2026-06-23Z.ptlog
```

Advantages:

- Avoids accumulating all daily logs in the root directory.
- Keeps filenames human readable.
- Keeps each directory small.
- Makes retention traversal logical by year/month/date.
- Reduces risk of fixed root-directory exhaustion.

Required implementation details:

1. Add a configurable log root such as `/sdcard/logs`.
2. Build month directory from the record date, for example `YYYY-MM`.
3. Ensure the log root exists before creating month directory.
4. Ensure the month directory exists before opening the daily file.
5. Use bounded path buffers and check truncation.
6. Keep compatibility for existing root-level `.ptlog` files during retention scans.
7. Ensure `sd format` recreates expected directories, or create them lazily.
8. Update any `ls`, status, or console commands that assume root files only.
9. Update host tools if they assume all logs are in the root.

### Alternative or additional fix: short 8.3 filenames

Use names like:

```text
260623.PTL
```

or:

```text
20260623.PTL
```

Advantages:

- Short names reduce FAT long-filename entry consumption.
- Smaller metadata footprint.
- Easier for embedded FAT.

Disadvantages:

- Less human readable.
- Existing host tools may need to support both old and new naming.
- Revision naming must be designed carefully within 8.3 constraints.

### Recommendation

Use monthly subdirectories first. Consider short filenames if the card is confirmed to be FAT16/root-entry constrained or if directory-entry pressure remains high.

## Proposed fix group 2: Directory traversal for oldest-file discovery and retention

The retention system must traverse directories, not only the mount root.

Current logic finds oldest daily files only in `logger->mount_point`. That is insufficient once logs move into `logs/YYYY-MM/` directories and insufficient for cleaning old root-level logs during migration.

### Required traversal behavior

Add a recursive or bounded-depth traversal that discovers PTLOG files under:

```text
/sdcard
/sdcard/logs
/sdcard/logs/YYYY-MM
```

It should support both legacy and new paths:

```text
/sdcard/2026-06-23Z.ptlog
/sdcard/2026-06-23Z-1.ptlog
/sdcard/logs/2026-06/2026-06-23Z.ptlog
/sdcard/logs/2026-06/2026-06-23Z-1.ptlog
```

If 8.3 names are added, it should also support:

```text
/sdcard/logs/2026-06/260623.PTL
/sdcard/logs/2026-06/260623-1.PTL
```

### Traversal safety rules

Embedded directory traversal must be bounded and defensive.

Rules:

1. Do not recurse indefinitely.
2. Maximum depth should be small, for example 2 or 3.
3. Do not follow special entries `.` or `..`.
4. Do not follow unknown nested directories beyond the allowed layout.
5. Ignore `FOUND.000`, `.Trash-1000`, `System Volume Information`, and unknown system directories unless a maintenance command explicitly handles them.
6. Use fixed-size path buffers and check every join/truncation.
7. Close every `DIR*` on all paths.
8. Continue scanning if one directory cannot be opened, but record a diagnostic warning.
9. Never delete non-PTLOG files during automatic retention.
10. Never delete the currently open file.
11. Never delete today's file unless an explicit emergency policy is later approved.
12. Sort candidates by parsed date/revision, not by directory enumeration order.
13. If date parsing fails, do not delete automatically.

### Candidate structure

Introduce a candidate type such as:

```c
typedef struct {
  char path[128];
  char name[64];
  char date_string[16];
  uint32_t revision;
  uint64_t size_bytes;
  bool is_current_open_file;
  bool is_legacy_root_file;
} sd_ptlog_candidate_t;
```

If memory is tight, do not store a large array of all candidates. Instead, scan and keep only the current oldest candidate, then delete one candidate per pass and rescan.

### Oldest-file deletion algorithm

Use a repeated scan-and-delete algorithm:

```text
while retention condition is violated and deletes < max_deletes:
    oldest = find_oldest_deletable_ptlog(root)
    if none:
        break
    unlink(oldest.path)
    deleted++
    optionally remove empty month directory
    recompute retention condition
```

This avoids storing many candidates in RAM.

### Date parsing and revision ordering

For files from the same date, delete revisions in a deterministic order.

Suggested policy:

1. Oldest date first.
2. Within a date, lowest revision first, because root file or `-1` likely came first.
3. Never delete current open file.
4. If current date has old revision files from header-signature changes, do not delete them during automatic space reclamation unless absolutely necessary and separately approved.

### Empty directory cleanup

After deleting the last PTLOG in a month directory, optionally remove the empty month directory.

Safety:

- Only remove directories matching `YYYY-MM` under `/logs`.
- Only remove if empty after deleting PTLOGs.
- Ignore failure.

## Proposed fix group 3: File-count and directory-entry-aware retention

Retention must not only look at free bytes.

Add thresholds such as:

```c
max_ptlog_files_total
max_ptlog_files_per_directory
min_free_bytes
max_root_ptlog_files
max_estimated_root_dir_entries
```

### Required retention triggers

Run reclaim before opening a new daily file if any of these are true:

1. Free bytes below required threshold.
2. Total PTLOG file count above configured limit.
3. Root-level PTLOG count above migration/legacy limit.
4. Current target directory PTLOG count above configured limit.
5. Estimated directory-entry usage is approaching a configured threshold.
6. A previous file create failed with `EACCES`, `ENOSPC`, or equivalent FATFS create denial.

### Estimated directory-entry accounting

FAT long filenames consume multiple entries. Add a conservative estimator:

```text
entries = 1 + ceil(strlen(long_name_utf16) / 13)
```

For simple ASCII names, `strlen` is acceptable as an approximation.

For 8.3 names, entries = 1.

Use this only for risk estimation and warning. Do not rely on it as exact filesystem state.

### Root directory special handling

If the card can be FAT16, root directory may have a fixed entry limit. Treat root as special:

- Avoid writing new PTLOG files to root.
- During migration, prefer deleting or moving legacy root PTLOG files first.
- Warn if root PTLOG count is high.
- Warn if root contains many long filenames.

### Reclaim on create failure

If `fopen` fails when creating/opening the daily file:

1. Capture `errno` immediately.
2. Classify the operation as `daily_file_open` or `daily_file_create`.
3. If errno suggests space/permission/create denial, run reclaim even if free bytes are high.
4. Delete at least one oldest PTLOG candidate.
5. Retry `fopen` once.
6. If still failing, unmount/backoff and set a specific status: `sd_create_error_active` or `sd_dir_full_active`.

## Proposed fix group 4: Preserve and expose precise SD error diagnostics

Current low-level logs contain the useful errno, but runtime status often loses it.

Add a structured SD error diagnostic object:

```c
typedef struct {
  const char* operation;
  char path[128];
  esp_err_t esp_error;
  int errno_value;
  char errno_text[32];
  bool did_unmount;
  bool create_or_open_failure;
  bool suspected_directory_entry_exhaustion;
} sd_failure_detail_t;
```

Propagate this from `sd_logger.c` to `runtime_manager.c` for sync/open failures.

Status should show:

```text
sd_last_op: fopen
sd_last_path: /sdcard/2026-06-23Z.ptlog
sd_last_errno: 13
sd_last_errno_text: Permission denied
sd_suspected: directory entry exhaustion or create denied
```

The ntfy alert should include a concise diagnosis:

```text
SD create failed: Permission denied opening /sdcard/2026-06-23Z.ptlog. Free bytes OK; possible FAT directory entry exhaustion. Oldest PTLOG reclaim attempted: yes/no.
```

## Proposed fix group 5: Better SD display/status codes

Replace or supplement `SDOUT`.

Current `SDOUT` conflates:

- Card missing.
- Card present but not mounted.
- Card mounted but unusable.
- Possibly out of byte space.

Proposed display codes:

```text
SDMISS: no card detected
SDMNT : card present but not mounted
SDIO  : mounted but I/O error active
SDFUL : out of byte space
SDDIR : create/directory-entry failure suspected
SDDEG : degraded/backoff state
```

If display width is constrained, use shorter codes:

```text
NOCARD
SDMNT
SDIO
SDFUL
SDDIR
```

Runtime health should carry distinct booleans:

```c
sd_card_present
sd_mounted
sd_io_error_active
sd_out_of_space_active
sd_create_error_active
sd_dir_entry_pressure_active
sd_degraded
```

## Proposed fix group 6: Correct FRAM overrun semantics

Separate active state from historical state.

### Proposed FRAM status fields

```c
uint64_t fram_overrun_total;
uint64_t fram_overrun_session_baseline;
uint64_t fram_overrun_notified_total;
bool fram_overrun_active;
bool fram_overrun_unacknowledged;
bool fram_currently_full;
bool fram_above_watermark;
bool fram_data_loss_this_session;
```

### Runtime start behavior

On boot/runtime init:

```text
fram_overrun_session_baseline = FramLogGetOverrunRecordsTotal()
fram_overrun_active = false
fram_overrun_unacknowledged = overrun_total > fram_overrun_ack_total
```

The unacknowledged historical condition can be shown in status, but it should not behave like an active current error unless policy explicitly says so.

### Active alert behavior

Set `fram_overrun_active` only when:

```text
current_overrun_total > last_seen_overrun_total
```

or when append is currently overwriting due to full buffer.

Clear `fram_overrun_active` when:

- SD is healthy.
- FRAM count is below a safe threshold.
- No new overruns have occurred for N cycles.

Historical data-loss alert:

- Send once when a new overrun event occurs.
- Mark as unacknowledged until operator acknowledges or until policy clears it.
- Do not repeat every five minutes as active if no new overrun occurs.

### Fix `UpdateFramOverrunActive()`

Current logic:

```c
const bool overrun_active = (overrun_total > state->fram_overrun_ack_total);
```

Proposed logic should use new-overrun delta and current fill pressure, not only ack baseline.

Pseudo:

```c
const bool overrun_advanced = overrun_total > state->last_overrun_records_total;
const bool current_pressure = state->cached_status.fram_count >= state->cached_status.fram_capacity;
const bool overrun_active = overrun_advanced || current_pressure;
```

Then separately:

```c
const bool overrun_unacknowledged = overrun_total > state->fram_overrun_ack_total;
```

These should drive different ntfy messages.

## Proposed fix group 7: Fix misleading FRAM warning logs

Current first-observation behavior can log live data loss when it merely sees historical cumulative overrun.

Replace with two separate messages.

On startup/first observation if total is already nonzero:

```text
FRAM overrun history present: total=N ack=M current_buffer=X/Y
```

On actual new overrun advancement:

```text
FRAM buffer full: overwriting oldest records; overrun_total=N buffered=X/Y sd_mounted=yes/no sd_degraded=yes/no
```

Do not print the second message unless the overrun total advanced during this append/run.

## Proposed fix group 8: Fix FRAM_FULL flag ordering or remove it

Current code mutates `record.flags` after `FramLogAppend()` already copied the record.

Choose one policy.

### Option A: Flag before append

If `LOG_RECORD_FLAG_FRAM_FULL` should mark records captured while FRAM was already full or at capacity, compute it before append:

```c
const size_t fram_count_before = FramLogGetCountRecords(&state->fram_log);
const size_t fram_capacity = FramLogGetCapacityRecords(&state->fram_log);
if (fram_count_before >= fram_capacity) {
  record.flags |= LOG_RECORD_FLAG_FRAM_FULL;
}
FramLogAppend(&state->fram_log, &record);
```

### Option B: Remove per-record flag mutation

If FRAM full is a runtime/system status, not a property of the sample record, do not put it into `record.flags` at all. Keep it in runtime status and alerts.

Recommendation: Option B is cleaner unless there is a specific audit requirement to tag records that were captured during FRAM pressure.

## Proposed fix group 9: Recovery behavior after SD format

After `sd format`, the firmware should reset SD and FRAM-related transient runtime state cleanly.

Expected behavior after successful format:

- SD mounted true.
- SD degraded false.
- SD I/O error false.
- SD create error false.
- SD out-of-space false.
- SD backoff cleared.
- SD fail count reset or clearly marked historical.
- FRAM drain can proceed.
- Historical FRAM overrun remains recorded but not active.
- ntfy active system-error alerts stop unless new errors occur.

If `sd format` is intended to preserve FRAM data and then drain it, it must not reset FRAM contents. But it should reset stale SD error state and active alert latches.

## Proposed implementation plan

### Task 1: Add forensic docs and tests for reproduced incident

- Add this document.
- Add fixtures or synthetic tests for many root long filenames and create failure.
- Add tests for FRAM overrun active logic using cumulative historical total.

### Task 2: Add SD PTLOG traversal helper

Implement bounded traversal of legacy root and new `/logs/YYYY-MM` directories.

Acceptance:

- Finds old root files.
- Finds nested month files.
- Ignores unknown/system directories.
- Never deletes current open file.
- Selects oldest candidate by parsed date/revision.

### Task 3: Add file-count/directory-entry-aware retention

Acceptance:

- Reclaim can trigger even when byte free space is high.
- Reclaim deletes oldest PTLOG before opening new daily file if file count/directory pressure is high.
- Root legacy files can be cleaned up.

### Task 4: Move daily logs into `/logs/YYYY-MM/`

Acceptance:

- New daily files go into month directories.
- Old root files remain readable/discoverable for retention and host tools.
- Path buffers are checked.
- Directory creation failures are diagnosed.

### Task 5: Classify and recover from create/open failure

Acceptance:

- `fopen` errno is preserved.
- Create/open failure runs targeted PTLOG reclaim and retries once.
- Status reports create/directory failure distinctly.
- ntfy mentions suspected directory-entry exhaustion when appropriate.

### Task 6: Split SD display states

Acceptance:

- Card missing is distinct from mounted failure.
- I/O error is distinct from out-of-space.
- Directory/create failure is distinct from card missing.
- `SDOUT` is removed or no longer used for unrelated states.

### Task 7: Refactor FRAM overrun active vs historical state

Acceptance:

- Historical cumulative overrun does not repeatedly alert as active after recovery.
- New overruns alert once and remain acknowledged/unacknowledged separately.
- Current full/pressure state is displayed independently.

### Task 8: Fix misleading FRAM logs and record flags

Acceptance:

- `FRAM buffer full` logs only for current new overwrite events.
- Historical overrun logs are worded as historical.
- `LOG_RECORD_FLAG_FRAM_FULL` is either set before append or removed from post-append mutation.

### Task 9: Recovery and soak validation

Acceptance:

- A card with many legacy root files does not get stuck at daily rollover.
- A test card near root-directory-entry pressure triggers reclaim before failure.
- FRAM drains after SD recovery.
- No repeated stale `fram overrun active` ntfy messages after drain.
- 48-hour soak with daily rollover passes.

## Manual validation checklist

### SD directory-entry reproduction

1. Format a test SD card with the same filesystem type used by the datalogger.
2. Populate root with many long PTLOG-style filenames until near the suspected directory-entry limit.
3. Confirm Linux still shows free byte space.
4. Insert into datalogger.
5. Force daily file creation or simulate next-day rollover.
6. Confirm old firmware fails with create/open denial.
7. Confirm fixed firmware deletes/moves/reclaims and succeeds.

### SD nested-directory validation

1. Format SD.
2. Start logger.
3. Confirm `/logs/YYYY-MM/YYYY-MM-DDZ.ptlog` is created.
4. Let it roll to the next UTC day.
5. Confirm next daily file is created in the same or next month directory.
6. Confirm host plotter can load nested logs.

### Retention traversal validation

1. Put legacy root PTLOG files on the card.
2. Put nested month PTLOG files on the card.
3. Trigger retention by file count.
4. Confirm oldest legacy/nested file is selected correctly.
5. Confirm current open file is not deleted.
6. Confirm empty month directories are removed only when safe.

### FRAM alert validation

1. Force SD failure long enough to fill FRAM and cause overrun.
2. Restore SD.
3. Drain FRAM.
4. Confirm one historical data-loss message remains if desired.
5. Confirm active FRAM overrun alert clears.
6. Restart runtime.
7. Confirm historical overrun does not re-fire as active unless a new overrun occurs.

## Open questions

1. Should automatic retention delete old root-level legacy files first before nested files?

Recommendation: yes, because root-entry pressure is the known failure mechanism.

2. Should `sd format` reset FRAM overrun acknowledgement state?

Recommendation: no. Formatting SD should not erase evidence of data loss. It should clear active SD error state and active FRAM alert state after recovery, but historical overrun should remain visible until acknowledged.

3. Should the firmware move existing root files into `/logs/YYYY-MM/` automatically?

Recommendation: no for MVP. Moving many files on embedded FAT creates risk. Prefer writing new files into nested directories and deleting old root files by retention policy.

4. Should the datalogger use 8.3 filenames?

Recommendation: optional. Monthly directories likely solve the immediate issue while preserving readability. Use 8.3 only if FAT entry pressure remains a problem.

5. Should `FOUND.000`, `.Trash-1000`, and `System Volume Information` be deleted automatically?

Recommendation: no for automatic runtime retention. Add a manual maintenance command if needed, with warnings. Automatic retention should only delete PTLOG files.

## Final working theory

The SD problem was most likely caused by FAT directory-entry exhaustion from many long PTLOG filenames in the SD card root directory. The firmware reclaim path only considered free byte space, so it did not delete old files before new daily file creation failed. The failure then caused SD unmount/backoff, FRAM accumulation, FRAM overrun, and storage-stall alerts.

The later FRAM alert problem was most likely stale cumulative FRAM overrun state being treated as active state. The system was no longer full and the SD had recovered, but the historical overrun counter remained above the acknowledgement baseline, causing repeated active alerts and misleading log/display behavior.

The fixes should be implemented in firmware, not handled operationally by periodic manual formatting. The central changes are: nested log directories, recursive/bounded oldest-file traversal, file-count/directory-entry-aware retention, create-failure recovery, clearer SD status codes, and corrected FRAM overrun active/historical semantics.
