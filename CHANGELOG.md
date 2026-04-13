# Changelog

## 2026-04-13
- Calibration: replaced calibration-window drift computation with a full-window least-squares regression slope of raw temperature vs sample timestamp (`C/min`), while retaining begin/end segment means and delta (`end_mean - begin_mean`) as separate metrics.
- Console/help: clarified `cal live`, `cal capture`, and top-level `help cal` wording so drift is explicitly regression-based, delta remains begin/end movement, and capture drift gating applies to `|regression drift|`.

## 2026-04-09
- Console: `cal capture` status output now includes instantaneous `last` raw temperature and `last_ohm` raw resistance values (with `n/a` when no samples are available), matching live-visibility expectations during capture.
