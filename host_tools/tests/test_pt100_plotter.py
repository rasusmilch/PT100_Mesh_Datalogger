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
    summary = p._format_flags_summary(df, display_tz=None, time_source="device", sensor_fault_threshold_percent=0.01)
    assert "Sensor fault: 20/43601 (0.05%)" in summary


def test_sensor_fault_below_threshold_omitted_from_pdf_summary():
    flags = [0x0013] * 20 + [0x0003] * (43601 - 20)
    df = pd.DataFrame({"flags": flags, "fault_status": [0x00] * 43601})
    summary = p._format_flags_summary(df, display_tz=None, time_source="device", sensor_fault_threshold_percent=0.10)
    assert "Sensor fault" not in summary
    detail = p._format_fault_status_summary(df, include_zero_fault_status=False)
    assert detail == "n/a"


def test_sensor_fault_above_threshold_included_in_pdf_summary():
    flags = [0x0013] * 20 + [0x0003] * (43601 - 20)
    df = pd.DataFrame({"flags": flags})
    summary = p._format_flags_summary(df, display_tz=None, time_source="device", sensor_fault_threshold_percent=0.01)
    assert "Sensor fault: 20/43601 (0.05%)" in summary


def test_sensor_fault_nonzero_included_regardless_of_threshold():
    df = pd.DataFrame({"flags": [0x0013], "fault_status": [0x80]})
    summary = p._format_fault_status_summary(df, include_zero_fault_status=False)
    assert "RTD high threshold" in summary


def test_build_status_flag_rows_includes_informational_and_counts():
    flags = [0x0013] * 2 + [0x004B] * 3
    df = pd.DataFrame({"flags": flags})
    rows = p._build_status_flag_rows(df)
    names = {r.short_name for r in rows}
    assert "MESH_CONNECTED" in names
    assert "RTD_EMA" in names
    sensor_row = next(r for r in rows if r.short_name == "SENSOR_FAULT")
    assert sensor_row.count == 2
    assert sensor_row.percent == pytest.approx(40.0)


def test_parse_nonnegative_float_accepts_and_defaults():
    assert p._parse_nonnegative_float("", "field", 0.10) == pytest.approx(0.10)
    assert p._parse_nonnegative_float("0", "field", 0.10) == pytest.approx(0.0)
    assert p._parse_nonnegative_float("0.10", "field", 0.10) == pytest.approx(0.10)
    assert p._parse_nonnegative_float("1.5", "field", 0.10) == pytest.approx(1.5)


def test_parse_nonnegative_float_rejects_negative_and_nonnumeric():
    with pytest.raises(ValueError):
        p._parse_nonnegative_float("-0.1", "field", 0.10)
    with pytest.raises(ValueError):
        p._parse_nonnegative_float("abc", "field", 0.10)


def test_plotter_app_initializes_pdf_sensor_fault_threshold_text():
    tk = pytest.importorskip("tkinter")
    root = None
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        assert hasattr(app, "pdf_sensor_fault_threshold_text")
        assert app.pdf_sensor_fault_threshold_text.get() == "0.10"
    finally:
        root.destroy()
