import datetime
import pytest
pd = pytest.importorskip('pandas')

from host_tools import pt100_plotter as plotter


def test_subtitle_is_concise_and_uses_display_timezone():
    start_utc = pd.Timestamp('2026-05-05T00:00:00Z')
    end_utc = pd.Timestamp('2026-05-06T12:33:00Z')
    cdt = datetime.timezone(datetime.timedelta(hours=-5), name='Central Daylight Time')
    start_label, end_label, tz_label = plotter._format_time_range_for_display(start_utc, end_utc, cdt)
    subtitle = plotter._build_report_subtitle('98:A3:16:12:57:B0', start_label, end_label, tz_label)

    assert 'selected' not in subtitle
    assert 'plotted' not in subtitle
    assert 'Log/source time' not in subtitle
    assert '98:A3:16:12:57:B0' in subtitle
    assert '2026-05-04 19:00' in subtitle
    assert '2026-05-06 07:33' in subtitle
    assert 'Central Daylight Time' in subtitle


def test_measurement_window_utc_labels_when_display_is_utc():
    start_utc = pd.Timestamp('2026-05-05T00:00:00Z')
    end_utc = pd.Timestamp('2026-05-06T12:33:17Z')
    start_label, end_label, tz_label = plotter._format_time_range_for_display(start_utc, end_utc, datetime.timezone.utc)
    span = plotter._format_span_label(start_utc, end_utc)
    measurement = f"{start_label} → {end_label} (span {span})"

    assert start_label == '2026-05-05 00:00'
    assert end_label == '2026-05-06 12:33'
    assert tz_label == 'UTC'
    assert measurement.startswith('2026-05-05 00:00 → 2026-05-06 12:33')


def test_measurement_window_fixed_offset_labels_when_display_is_localized():
    start_utc = pd.Timestamp('2026-05-05T00:00:00Z')
    end_utc = pd.Timestamp('2026-05-06T12:33:17Z')
    tz = datetime.timezone(datetime.timedelta(hours=-5), name='UTC-05')
    start_label, end_label, tz_label = plotter._format_time_range_for_display(start_utc, end_utc, tz)

    assert start_label == '2026-05-04 19:00'
    assert end_label == '2026-05-06 07:33'
    assert tz_label == 'UTC-05'
