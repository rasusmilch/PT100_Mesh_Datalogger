# Code Documentation Policy

Suggested path: `docs/code_documentation_policy.md`

## Purpose and scope

This policy defines how source code documentation shall be written and reviewed in this repository.

It applies to:

* Embedded C firmware.
* Public and internal C headers.
* Python host tools.
* Tests and test helpers.
* Build/tooling scripts when they contain project behavior.
* Future C++ code if added.
* Codex-generated code and review tasks.

The goal is not to add large comments everywhere. The goal is to preserve product intent, safety constraints, hardware assumptions, validation boundaries, and non-obvious behavior so future maintainers and Codex sessions do not accidentally preserve stale patterns or introduce unsafe behavior.

## Audience

Code documentation should help:

* Future maintainers.
* Junior developers.
* Codex sessions.
* Human reviewers.
* Operators investigating failures.
* Hardware/debug support.
* Test authors.
* Anyone validating SD, FRAM, mesh, display, notification, calibration, or host-tool behavior.

Documentation should assume the reader can read code, but may not know the project history, hardware constraints, SD/FAT failure modes, FRAM semantics, or prior product decisions.

## Repository languages detected

The repository currently contains:

* Embedded C firmware under `main/`.
* C headers under `main/`.
* Python host tools under `host_tools/`.
* C host tests under `host_tools/tests_c/`.

C++ source was checked but not found as an active implementation language in the inspected context. C headers use `extern "C"` guards, so public headers should remain compatible with future C++ consumers.

Markdown documentation exists under `docs/`.

## General documentation standard

Document behavior when the reader needs information that is not obvious from the code.

Good documentation explains:

* Why this behavior exists.
* What invariant must remain true.
* Which product decision it encodes.
* Which safety or reliability issue it prevents.
* Which hardware or filesystem assumption it depends on.
* Which states are intentionally distinct.
* Which failures are expected and how they are handled.
* Which behavior is provisional or intentionally deferred.

Avoid comments that merely translate syntax into English.

Documentation must not claim validation that was not performed.

## Python standard

Python modules, classes, and functions shall follow PEP 257-compatible docstrings.

Use concise one-line docstrings for simple private helpers where the behavior is obvious but the helper name benefits from clarification.

Use a full docstring block when a function or class has:

* Nontrivial parsing or validation.
* File I/O.
* Report/export behavior.
* User-facing behavior.
* Error handling.
* Side effects.
* Data conversion.
* Time zone logic.
* Calibration or units.
* Compatibility with firmware schemas.
* Regression coverage for a previous bug.

Prefer Google-style sections when they add clarity:

* `Args:`
* `Returns:`
* `Raises:`
* `Side Effects:`
* `Notes:`

Python docstring example:

```python
def parse_ptlog_timestamp(value: str, default_tz: ZoneInfo) -> datetime.datetime:
    """Parse a PTLOG timestamp and attach the expected source timezone.

    Args:
        value: Timestamp text from a PTLOG or CSV export.
        default_tz: Timezone to apply when the source timestamp is naive.

    Returns:
        A timezone-aware datetime suitable for sorting and report ranges.

    Raises:
        ValueError: If the timestamp cannot be parsed.

    Notes:
        PTLOG source timestamps may be naive in older exports. Do not silently
        treat them as local workstation time; that changes report ordering and
        trim behavior.
    """
```

Short private-helper example:

```python
def _is_blank_row(row: dict[str, str]) -> bool:
    """Return true when every parsed CSV field is empty."""
```

Do not add large docstrings to trivial wrappers where the name and type hints already explain the behavior.

## C standard

Use Doxygen-compatible comments for:

* Public header functions.
* Public structs, enums, and typedefs.
* Callback types.
* Macros that behave like APIs or encode product limits.
* Nontrivial static functions when they encode safety, policy, hardware, timing, or failure behavior.
* Test helpers that model production behavior.

Preferred tags:

* `@brief`
* `@param`
* `@return`
* `@retval`
* `@note`
* `@warning`
* `@pre`
* `@post`

Document the following when relevant:

* Ownership and lifetime.
* Buffer sizes.
* Units.
* Error codes and return meanings.
* Blocking behavior.
* ISR safety.
* Thread/task safety.
* Locking assumptions.
* Allocation behavior.
* Stack usage when a helper uses more than small scalar or path-fragment locals.
* Buffer ownership, lifetime, and required capacity for static SD/PTLOG path-workspace buffers.
* Whether an SD/PTLOG path-workspace buffer is stack, static, heap, PSRAM, or caller-owned.
* Task/lock assumptions for shared static SD/PTLOG path-workspace buffers, including whether the function may run while holding the SD I/O lock.
* Truncation and fail-closed behavior for path construction.
* Allocation failure behavior and fallback allocation caps.
* Whether a function may log.
* Whether a function may mutate persistent state.
* Whether a function may delete files or otherwise perform destructive operations.


Storage-specific documentation requirements:

* Do not add comments that normalize large task-stack buffers in storage code.
* Public structs, public helpers, and nontrivial static helpers that use or receive SD/PTLOG path-workspace buffers must document buffer ownership, lifetime, required capacity, placement (stack/static/heap/PSRAM/caller-owned), task/lock assumptions, SD I/O lock expectations, truncation/fail-closed behavior, and allocation failure behavior.
* If a function intentionally uses a stack buffer larger than a small scalar/path-fragment buffer, the source comment must justify why that stack use is safe and the task receipt must report the file, function, and size.
* Documentation must not claim target stack high-water, heap, PSRAM, FAT, or hardware validation unless that validation was actually performed and described.

C header example:

```c
/**
 * @brief Find the oldest PTLOG file that automatic retention may delete.
 *
 * @param mount_point Mounted SD root path, such as "/sdcard".
 * @param current_path Optional exact path of the currently open PTLOG file.
 * @param current_date Optional current UTC date string in "YYYY-MM-DDZ" form.
 * @param candidate_out Receives the selected candidate when true is returned.
 *
 * @return true when an eligible regular PTLOG file was found.
 * @return false when no eligible file was found or required arguments are invalid.
 *
 * @note Traversal is intentionally bounded to root, /logs, and /logs/YYYY-MM.
 * @warning The caller may delete the returned path. This function must fail
 * closed for malformed names, path truncation, special files, and system dirs.
 */
bool SdPtlogFindOldestCandidate(const char* mount_point,
                                const char* current_path,
                                const char* current_date,
                                sd_ptlog_candidate_t* candidate_out);
```

C struct example:

```c
/**
 * @brief Parsed PTLOG candidate returned by bounded SD retention scanning.
 *
 * All string fields are null-terminated. Paths are absolute within the mounted
 * filesystem. A candidate is safe to consider for retention only after the
 * scanner has verified it is a regular file with a parsed PTLOG basename.
 */
typedef struct
{
  char path[SD_PTLOG_MAX_PATH_LEN]; /**< Full path to the candidate file. */
  char name[SD_PTLOG_MAX_NAME_LEN]; /**< Basename as found in the directory. */
  char date[SD_PTLOG_DATE_LEN + 1u]; /**< Parsed UTC date: YYYY-MM-DDZ. */
  uint32_t revision;                /**< Parsed same-day revision, or 0. */
  bool legacy_root;                 /**< True when candidate came from root. */
  bool current_open;                /**< True only if exact current path matched. */
} sd_ptlog_candidate_t;
```

C static helper example:

```c
/**
 * @brief Join a directory and child name without allowing truncation.
 *
 * @return true when the full path fit in the destination buffer.
 * @return false when any argument is invalid or the joined path would truncate.
 *
 * @note Storage code must fail closed on truncated paths because later callers
 * may open, create, or delete the returned path.
 */
static bool JoinPath(const char* dir_path,
                     const char* child_name,
                     char* out_path,
                     size_t out_path_size);
```

## C++ standard

No active C++ implementation files were found in the inspected context. If C++ is added later, use Doxygen-compatible comments for:

* Public classes.
* Public methods and functions.
* Templates.
* Interfaces.
* Enums.
* RAII ownership types.
* Hardware abstraction wrappers.
* Error/status types.

Document:

* Class invariants.
* Ownership and lifetime.
* Move/copy behavior.
* Exception or error-return behavior.
* Thread safety.
* Side effects.
* Blocking behavior.
* Hardware/resource ownership.

C++ example:

```cpp
/**
 * @brief Owns an SD log file and closes it on destruction.
 *
 * The object is movable but not copyable because it owns a file handle.
 *
 * @note Methods are not thread-safe unless the caller holds the runtime lock.
 * @warning Destruction may flush buffered data and can block on filesystem I/O.
 */
class SdLogFile {
 public:
  SdLogFile(SdLogFile&& other) noexcept;
  SdLogFile& operator=(SdLogFile&& other) noexcept;

  SdLogFile(const SdLogFile&) = delete;
  SdLogFile& operator=(const SdLogFile&) = delete;
};
```

## Embedded C/C++ standard

Embedded documentation must capture hardware and runtime assumptions that are not obvious from generic C code.

Document these when relevant:

* Hardware revision assumptions.
* Pin maps and peripheral maps.
* SPI/I2C/UART bus ownership.
* Clock and timer assumptions.
* Interrupt behavior and ISR/main interactions.
* DMA ownership and alignment requirements.
* RTOS task names, priorities, queues, event groups, and synchronization.
* Stack, heap, static allocation, and PSRAM assumptions.
* Calibration constants, units, scaling, and fixed-point math.
* Power/reset/watchdog behavior.
* Datasheet, errata, and register-sequence assumptions.
* Required delays and timing constraints.
* Safety-critical retry limits.
* Persistent storage layout.
* Filesystem format assumptions.
* Failure states and recovery boundaries.

Embedded example:

```c
/**
 * @brief Attempt bounded SD reclaim before another append attempt.
 *
 * @pre Caller holds the SD logger lock.
 * @pre logger->mount_point is a mounted FAT filesystem path.
 *
 * @param logger Active SD logger instance.
 * @param required_free_bytes Minimum byte-space target before append.
 * @param deleted_files Optional output count of successfully unlinked PTLOGs.
 *
 * @retval ESP_OK Reclaim completed or no eligible PTLOG candidate existed.
 * @retval ESP_ERR_INVALID_ARG logger was NULL.
 * @retval ESP_FAIL Free-space query failed.
 *
 * @note Reclaim is currently byte-space-triggered only. Directory-entry and
 * file-count pressure are handled by later retention policy work.
 * @warning Automatic reclaim must only delete parsed regular PTLOG candidates
 * from approved locations and must protect the current date.
 */
static esp_err_t SdLoggerReclaimSpaceLocked(sd_logger_t* logger,
                                            uint64_t required_free_bytes,
                                            uint32_t* deleted_files);
```

## Inline comment expectations

Inline comments should explain “why,” not trivial “what.”

Use inline comments for:

* Non-obvious domain assumptions.
* Safety constraints.
* Security/permission boundaries.
* Hardware timing assumptions.
* Peripheral register sequences.
* Interrupt, task, or lock interactions.
* Ownership and lifetime rules.
* Strict rejection cases.
* Regression protections for previous quality escapes.
* Why a broader-looking case is intentionally not handled.
* Why a policy is deferred.

Good inline comment example:

```c
/* Current-date protection is intentionally broad because sd_logger_t does not
 * yet track the exact open PTLOG path.  Do not narrow this until current-path
 * lifetime is modeled and tested. */
```

Bad inline comment example:

```c
/* Increment i. */
i++;
```

## What not to comment

Do not add:

* Comments that narrate syntax.
* Speculative comments.
* Stale comments.
* Comments that contradict tests or source behavior.
* Comments that claim hardware validation without hardware results.
* Large copied receipts in source comments.
* TODOs without owner, condition, or follow-up context.
* Comments that preserve deprecated assumptions.
* Comments that repeat function or parameter names without explaining behavior.

Do not remove useful product, hardware, safety, or failure-model comments unless replacing them with more accurate documentation.

## Codex task requirements

Every Codex task that adds or modifies source code must read this policy before editing.

Codex prompts must restate documentation requirements in acceptance criteria. Do not only say “follow the policy.”

Minimum Codex source-task acceptance language:

```text
Code documentation acceptance criteria:
- Read `docs/code_documentation_policy.md` before editing source.
- Add or update Doxygen-compatible comments for new or changed public C headers,
  structs, enums, callbacks, macros that behave like APIs, and nontrivial
  functions whose behavior is not obvious.
- Add or update PEP 257-compatible Python docstrings for new or changed
  modules, classes, public functions, and nontrivial private helpers.
- Add inline comments only for non-obvious product, safety, hardware, timing,
  ownership, concurrency, validation, or failure-model assumptions.
- Do not add comments that merely narrate syntax.
- Do not remove useful safety/product/hardware comments without replacing them
  with more accurate comments.
- Report documentation changes and any intentional documentation omissions in
  the final receipt.
```

Every Codex final receipt for a source task must include:

```text
Code documentation compliance:
- Policy file inspected: yes/no and path.
- Public APIs changed: yes/no.
- New/touched functions/classes/modules documented: yes/no.
- Inline safety/product comments added or updated: yes/no.
- Documentation intentionally omitted for trivial code: yes/no and why.
- Stale/misleading comments removed or corrected: yes/no.
```

## Documentation-only source-pass safety

Documentation-only passes must prove behavior did not change.

For Python documentation-only changes:

* Parse changed Python files with `ast.parse`.
* Compare AST after removing docstrings when practical.
* Run available tests.
* Run formatting/static checks if already part of the project workflow.
* Do not change imports, executable statements, defaults, type behavior, or command-line behavior during doc-only work.

Python AST equivalence guidance:

```text
For documentation-only Python changes, compare the AST with module, class, and
function docstrings stripped. If this guard is unavailable, explain the gap and
run the strongest available tests.
```

For C/C++ documentation-only changes:

* Prefer compile or build checks.
* Use preprocessing or token comparison where practical.
* Run available host tests.
* Run `git diff --check`.
* Run ESP-IDF build when firmware files changed and `idf.py` is available.
* If target hardware behavior is relevant, state that hardware was not validated unless it was.

C/C++ guard guidance:

```text
For documentation-only C/C++ changes, run the relevant compiler/build target
when available. If a token/preprocess equivalence check is available, use it.
If not, report that equivalence was guarded by compile/build/tests only.
```

Embedded verification limits:

* A successful host test does not prove target hardware behavior.
* A successful ESP-IDF build does not prove SD/FAT, timing, ISR, FRAM, or hardware behavior.
* Any hardware claim must list the target board, media/peripheral used, scenario, and observed result.

## Documentation backlog

Use this backlog for future focused tasks.

### Phase D1 — Add or verify behavior-preservation guard tooling

Goal:

* Provide repeatable checks for documentation-only changes.

Examples:

* Python docstring-stripped AST comparison helper.
* C/C++ preprocess/token comparison guidance.
* Documentation-only PR checklist.

### Phase D2 — Document core firmware entry points

Focus:

* `app_main.c`
* runtime manager entry points
* console command registration
* storage startup/shutdown
* task lifecycle

### Phase D3 — Document public APIs and headers

Focus:

* SD logger APIs
* PTLOG path helpers
* FRAM log APIs
* runtime health/status APIs
* alert manager APIs
* display/status APIs
* MAX31865 reader/calibration APIs

### Phase D4 — Document hardware abstraction and peripheral code

Focus:

* SD card detect and SD/FAT assumptions
* FRAM SPI/I2C behavior
* MAX31865 behavior
* I2C bus ownership
* Wi-Fi/mesh stack assumptions
* display hardware
* GPIO units/buttons
* watchdog/reset/power assumptions

### Phase D5 — Document complex tests and fixtures

Focus:

* Host PTLOG parsing/path tests.
* Reclaim behavior tests.
* FRAM drain/overrun tests.
* Host plotter/report tests.
* Any fake filesystem, fake clock, fake SD, or fake FRAM helpers.

### Phase D6 — Remove or correct misleading comments

Focus:

* Placeholder comments.
* Comments that preserve root-only PTLOG assumptions.
* Comments that imply `SDOUT` means only out-of-space.
* Comments that imply historical FRAM overrun is always active/current.
* Comments that overstate test or hardware validation.

## Policy for anchor files and related docs

If this policy is committed at `docs/code_documentation_policy.md`, future anchors should refer to that path.

Do not create duplicate competing policy files such as both:

* `docs/code_documentation_policy.md`
* `docs/code_documentation_policy_anchor.md`

If a legacy anchor path already exists, update the canonical policy file and add a short pointer from the older path if needed.

## Last updated context

This policy was drafted from:

* Current repository: `rasusmilch/PT100_Mesh_Datalogger`, branch `main`.
* Current ESP-IDF embedded C source list in `main/CMakeLists.txt`.
* Current Python host tool example `host_tools/pt100_plotter.py`.
* Current C public header style in `main/sd_ptlog_paths.h`.
* Recent SD/FAT16 PTLOG storage work through PR #368 and PR #369.
* Current chat drafts for project intent, requirements/constraints, decision log, and roadmap anchors.
* User preference for focused Codex tasks, explicit exclusions, and precise receipts.

Unverified at this update:

* Full repository language inventory beyond inspected/search-visible C, headers, Markdown, CMake, and Python.
* Whether C++ source exists in an uninspected path.
* Whether generated code exists and should be exempt from this policy.
* Whether a future docs/admin PR has already committed other anchor files.
* Exact target hardware revision, full pin map, and complete peripheral map.
* Project-wide static-analysis tooling beyond commands seen in prior receipts.
* Any hardware validation results beyond the user-reported successful ESP-IDF build after PR #369.
