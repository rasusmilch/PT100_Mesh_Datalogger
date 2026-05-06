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



def test_filter_status_flag_rows_nonzero_default():
    rows = [
        p.FlagSummaryRow("SENSOR_FAULT", "Sensor fault", 2, 40.0, "bad", True, True),
        p.FlagSummaryRow("FRAM_FULL", "FRAM full", 0, 0.0, "full", True, True),
    ]
    filtered = p._filter_status_flag_rows_for_display(rows)
    assert [r.short_name for r in filtered] == ["SENSOR_FAULT"]


def test_format_percent_for_status_table_small_nonzero():
    assert p._format_percent_for_status_table(0.05) == "0.05%"


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
        from tkinter import ttk
        assert isinstance(app.status_flags_tree, ttk.Treeview)
        app._refresh_status_flags_table(pd.DataFrame({"flags": [0x0013]}))
        assert len(app.status_flags_tree.get_children()) >= 1
    finally:
        root.destroy()


def test_create_default_report_config_values():
    cfg = p.create_default_report_config()
    assert cfg.input_timezone_mode == "Log/source time"
    assert cfg.y_axis_series == "cal_temp_c"
    assert cfg.pdf_sensor_fault_threshold_percent == pytest.approx(0.10)


def test_report_config_round_trip(tmp_path):
    path = tmp_path / "report_config.json"
    cfg = p.create_default_report_config()
    p.save_report_config(cfg, str(path))
    loaded = p.load_report_config(str(path))
    assert loaded == cfg


def test_report_config_missing_required_fields_rejected(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text('{"input_timezone_mode": "UTC"}', encoding='utf-8')
    with pytest.raises(ValueError, match='Invalid report config schema'):
        p.load_report_config(str(path))


def test_report_config_invalid_timezone_mode_rejected(tmp_path):
    path = tmp_path / "bad_mode.json"
    cfg = p.create_default_report_config()
    raw = cfg.__dict__.copy()
    raw["input_timezone_mode"] = "Mars"
    import json
    path.write_text(json.dumps(raw), encoding='utf-8')
    with pytest.raises(ValueError, match='input_timezone_mode'):
        p.load_report_config(str(path))


def test_report_config_invalid_numeric_values_rejected():
    cfg = p.create_default_report_config()
    cfg.max_plot_points = 0
    with pytest.raises(ValueError, match='max_plot_points'):
        p.validate_report_config(cfg)


def test_startup_gating_helper_state():
    tk = pytest.importorskip("tkinter")
    root = None
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        app._set_actions_enabled(False)
        assert str(app.plot_btn['state']) == tk.DISABLED
        assert str(app.save_trim_btn['state']) == tk.DISABLED
        assert str(app.pdf_btn['state']) == tk.DISABLED
        assert str(app.select_range_btn['state']) == tk.DISABLED
    finally:
        root.destroy()


def test_options_apply_updates_in_memory_config():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        cfg = p.create_default_report_config()
        app._apply_report_config_to_ui(cfg)
        app.rolling_mean_divisor_text.set("22")
        updated = app._build_report_config_from_ui()
        assert updated.rolling_mean_divisor == 22
    finally:
        root.destroy()


def test_save_config_writes_json(tmp_path):
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        cfg = p.create_default_report_config()
        path = tmp_path / "c.json"
        p.save_report_config(cfg, str(path))
        app.config_path = str(path)
        app._apply_report_config_to_ui(cfg)
        app.max_plot_points.set(1234)
        app.save_config()
        loaded = p.load_report_config(str(path))
        assert loaded.max_plot_points == 1234
    finally:
        root.destroy()

def _mk_loaded_log(df):
    return p.LoadedLog(
        dataframe=df,
        time_column="__time",
        time_source="epoch_utc",
        tzinfo=datetime.timezone.utc,
        source_files=[],
        dropped_no_time_rows=0,
        file_headers={},
        audit_summary=p.AuditSummary([], [], "", "", "", None, "", "", None),
    )


def test_build_plot_options_from_config_maps_fields():
    cfg = p.create_default_report_config()
    cfg.display_temperature_f = True
    cfg.rolling_mean_divisor = 9
    cfg.downsample_enabled = False
    cfg.max_plot_points = 123
    cfg.highlight_above_enabled = True
    cfg.highlight_above_value = 10.5
    cfg.show_std_band = True
    df = pd.DataFrame({"cal_temp_c": [1.0, 2.0]})
    opts = p.build_plot_options_from_config(cfg, _mk_loaded_log(df), df, p.DisplayTimeConfig(None, "n/a"))
    assert opts.temp_unit == "F"
    assert opts.rolling_mean_divisor == 9
    assert opts.enable_downsample is False
    assert opts.max_plot_points == 123
    assert opts.highlights.highlight_above is True
    assert opts.highlights.upper_limit == pytest.approx(10.5)


def test_build_plot_options_missing_series_raises_clear_error():
    cfg = p.create_default_report_config()
    cfg.y_axis_series = "raw_temp_c"
    df = pd.DataFrame({"cal_temp_c": [1.0]})
    with pytest.raises(ValueError, match="Configured y_axis_series not found"):
        p.build_plot_options_from_config(cfg, _mk_loaded_log(df), df, p.DisplayTimeConfig(None, "n/a"))


def test_build_plot_options_invalid_display_timezone_blocks():
    cfg = p.create_default_report_config()
    cfg.display_timezone = "Bad/Timezone"
    with pytest.raises(ValueError, match="display_timezone"):
        p.validate_report_config(cfg)


def test_validate_report_config_invalid_input_timezone_mode_blocks():
    cfg = p.create_default_report_config()
    cfg.input_timezone_mode = "BadMode"
    with pytest.raises(ValueError, match="input_timezone_mode"):
        p.validate_report_config(cfg)


def test_main_window_action_buttons_exist():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        assert app.plot_btn.cget("text") == "Plot"
        assert app.save_trim_btn.cget("text") == "Save Trimmed CSV"
        assert app.pdf_btn.cget("text") == "Export PDF Report"
    finally:
        root.destroy()


def test_config_labels_update_after_apply_report_config():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        cfg = p.create_default_report_config()
        cfg.display_timezone = "UTC"
        cfg.y_axis_series = "cal_temp_c"
        app.config_path = "/tmp/demo_config.json"
        app._apply_report_config_to_ui(cfg)
        assert "demo_config.json" in app.config_summary_label.cget("text")
        details = app.config_details_label.cget("text")
        assert "Input time zone:" in details
        assert "Display time zone: UTC" in details
        assert "Y-axis series: cal_temp_c" in details
    finally:
        root.destroy()


def test_status_rows_based_on_trimmed_range():
    df = pd.DataFrame({
        "__time": pd.to_datetime([
            "2026-01-01T00:00:00Z",
            "2026-01-01T00:01:00Z",
            "2026-01-01T00:02:00Z",
        ]),
        "flags": [0x0003, 0x0013, 0x0003],
        "cal_temp_c": [1.0, 1.1, 1.2],
    })
    cfg = p.create_default_report_config()
    trimmed, *_ = p._validate_and_trim_by_minute(
        df=df,
        time_column="__time",
        start_text="2026-01-01 00:01",
        end_text="2026-01-01 00:01",
        time_series_utc=df["__time"],
        input_timezone_mode="UTC",
        display_tz=datetime.timezone.utc,
        source_tz=None,
    )
    rows = p._build_status_flag_rows(trimmed)
    sensor = next(r for r in rows if r.short_name == "SENSOR_FAULT")
    assert sensor.count == 1


def test_status_meaning_updates_from_selected_row():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        app._refresh_status_flags_table(pd.DataFrame({"flags": [0x0013]}))
        item = app.status_flags_tree.get_children()[0]
        app.status_flags_tree.selection_set(item)
        app._on_status_flag_selected()
        assert "Meaning:" in app.status_meaning_label.cget("text")
    finally:
        root.destroy()


def test_plot_export_paths_do_not_require_legacy_direct_vars():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        assert not hasattr(app, "csv_path_var")
        assert not hasattr(app, "pdf_path_var")
    finally:
        root.destroy()

def test_sensor_fault_threshold_summary_below_and_included():
    rows = [p.FlagSummaryRow("SENSOR_FAULT", "Sensor fault", 1, 0.05, "", True, True)]
    app = type("A", (), {})()
    msg = p.PlotterApp._status_summary_for_rows(app, rows, threshold_percent=0.10)
    assert "Below threshold 0.10%" in msg
    rows2 = [p.FlagSummaryRow("SENSOR_FAULT", "Sensor fault", 1, 0.10, "", True, True)]
    msg2 = p.PlotterApp._status_summary_for_rows(app, rows2, threshold_percent=0.10)
    assert "Included in PDF at/above threshold 0.10%" in msg2


def test_status_summary_no_nonzero_flags_message():
    rows = [p.FlagSummaryRow("SENSOR_FAULT", "Sensor fault", 0, 0.0, "", True, True)]
    app = type("A", (), {})()
    msg = p.PlotterApp._status_summary_for_rows(app, rows, threshold_percent=0.10)
    assert msg == "Status: No nonzero status flags in selected range."


def test_invalid_range_blocks_plot_and_export_pdf_buttons():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        cfg = p.create_default_report_config()
        cfg.display_timezone = "UTC"
        cfg.y_axis_series = "cal_temp_c"
        app.report_config_loaded = True
        app._apply_report_config_to_ui(cfg)
        app.loaded = SimpleNamespace(
            dataframe=pd.DataFrame({"__time": pd.to_datetime(["2026-01-01T00:00:00Z"]), "flags": [0x0003], "cal_temp_c": [1.0]}),
            tzinfo=datetime.timezone.utc,
        )
        app._get_trimmed_df = lambda _cfg: (_ for _ in ()).throw(ValueError("End time is earlier than start time."))
        app._refresh_status_from_current_range()
        assert "Range is invalid" in app.status_summary_label.cget("text")
        assert str(app.plot_btn["state"]) == tk.DISABLED
        assert str(app.pdf_btn["state"]) == tk.DISABLED
    finally:
        root.destroy()
