import datetime
import pytest
pd = pytest.importorskip('pandas')
pytest.importorskip('numpy')
from host_tools import pt100_plotter as p


def test_canonical_time_selection_independent_of_display_tz():
    utc = pd.Series(pd.date_range('2026-01-01T00:00:00Z', periods=5, freq='min'))
    df = pd.DataFrame({'__time': utc, 'cal_temp_c':[1,2,3,4,5], 'raw_temp_c':[1,2,3,4,5], 'flags':[0,0,0,0,0], 'fault_status':[0,0,0,0,0]})
    trimmed_utc, *_ = p._validate_and_trim_by_minute(df, '__time', '2026-01-01 00:01', '2026-01-01 00:03', time_series_utc=utc, input_timezone_mode='UTC')
    local_tz = datetime.timezone(datetime.timedelta(hours=-5))
    local_start = '2025-12-31 19:01'
    local_end = '2025-12-31 19:03'
    trimmed_local, *_ = p._validate_and_trim_by_minute(df, '__time', local_start, local_end, time_series_utc=utc, input_timezone_mode='Same as display', display_tz=local_tz)
    assert trimmed_utc.index.tolist() == trimmed_local.index.tolist() == [1,2,3]


def test_trim_invalid_minute_raises_with_nearest():
    utc = pd.Series(pd.date_range('2026-01-01T00:00:00Z', periods=3, freq='min'))
    df = pd.DataFrame({'__time': utc})
    with pytest.raises(ValueError, match='Nearest valid minute'):
        p._validate_and_trim_by_minute(df, '__time', '2026-01-01 00:00', '2026-01-01 00:07', time_series_utc=utc, input_timezone_mode='UTC')


def test_temp_convert_and_downsample_spike_preservation():
    c = pd.Series([0.0, 100.0])
    f = p._convert_temperature_series(c, 'F')
    assert f.tolist() == pytest.approx([32.0, 212.0])
    series = [pd.Series([1, 50, 2, -10, 3, 40, 4])]
    idx = p._downsample_positions_minmax(series, max_plot_points=4)
    assert 1 in idx and 3 in idx


def test_fault_status_zero_classified_as_sensor_read_failure():
    df = pd.DataFrame({"flags": [0x0013], "fault_status": [0x00]})
    summary = p._format_fault_status_summary(df)
    assert "Sensor read failure" in summary
    assert "no MAX31865 fault byte" in summary


def test_fault_status_nonzero_decodes_max31865_bits():
    df = pd.DataFrame({"flags": [0x0013], "fault_status": [0x80]})
    summary = p._format_fault_status_summary(df)
    assert "RTD high threshold" in summary


def test_fault_status_grouped_counts_mixed_zero_and_nonzero():
    df = pd.DataFrame({"flags": [0x0013, 0x0013, 0x0013], "fault_status": [0x00, 0x80, 0x80]})
    summary = p._format_fault_status_summary(df)
    assert "Sensor read failure / no MAX31865 fault byte recorded: 1 row" in summary
    assert "0x80: 2 rows" in summary


def test_fault_status_detail_excludes_rows_without_sensor_fault_flag():
    df = pd.DataFrame({"flags": [0x0003, 0x0013], "fault_status": [0x80, 0x80]})
    summary = p._format_fault_status_summary(df)
    assert "0x80: 1 row" in summary
    assert "2 rows" not in summary


def test_flags_summary_small_nonzero_percent_uses_two_decimals():
    flags = [0x0013] * 20 + [0x0003] * (43601 - 20)
    df = pd.DataFrame({"flags": flags})
    summary = p._format_flags_summary(df, display_tz=None, time_source="device")
    assert "Sensor fault: 20/43601 (0.05%)" in summary


def test_compact_ptlog_filename_sort_key_and_detection():
    assert p._is_compact_ptlog_name('/tmp/20260626.012')
    assert not p._is_compact_ptlog_name('/tmp/20260626.12')
    assert p._parse_log_filename_sort_key('/tmp/20260626.012')[:3] == ('2026-06-26', 12, 0)
    assert p._parse_log_filename_sort_key('/tmp/2026-06-26Z-2.ptlog')[:3] == ('2026-06-26', 2, 0)


def test_compact_ptlog_folder_dedupe_sorting_preserves_csv_behavior():
    paths = [
        '/tmp/20260626.012',
        '/tmp/2026-06-25Z.ptlog',
        '/tmp/2026-06-25Z.csv',
        '/tmp/other.csv',
    ]
    sorted_paths = p._dedupe_folder_file_paths(paths)
    assert '/tmp/2026-06-25Z.csv' not in sorted_paths
    assert sorted_paths[:2] == ['/tmp/2026-06-25Z.ptlog', '/tmp/20260626.012']
    assert '/tmp/other.csv' in sorted_paths
