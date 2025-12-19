# Diagnostics commands

The `diag` console command now supports several subsystem checks. Typical usage:

```
diag help
diag all quick
```

Available routes:

- `diag help`
- `diag all quick|full [--verbose N]`
- `diag sd quick|full [--format-if-needed] [--mount] [--verbose N]`
- `diag fram quick|full [--bytes N] [--verbose N]`
- `diag rtd quick|full [--samples N] [--delay_ms M] [--verbose N]`
- `diag rtc quick|full [--set-known] [--verbose N]`
- `diag wifi quick|full [--scan] [--connect] [--verbose N]`
- `diag mesh quick|full [--start] [--stop] [--verbose N]`
- `diag check` (alias for `diag all quick`)

Return codes follow the console convention: `0` for success, `1` for failures during
checks, and `2` for usage errors.
