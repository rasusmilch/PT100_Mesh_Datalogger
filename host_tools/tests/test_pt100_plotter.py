import datetime
from types import SimpleNamespace

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


def test_escape_reportlab_text_escapes_xml_chars_and_handles_none():
    assert p._escape_reportlab_text(None) == ""
    assert p._escape_reportlab_text("a&b<c>d") == "a&amp;b&lt;c&gt;d"


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


def test_apply_flag_filters_for_stats_excludes_sensor_fault_rows_from_stats():
    sensor_fault_mask = next(mask for mask, name in p._LOG_RECORD_FLAG_DEFS if name == "SENSOR_FAULT")
    df = pd.DataFrame({"flags": [0, sensor_fault_mask, 0]})
    y = pd.Series([10.0, 999.0, 30.0])

    filtered, notes = p._apply_flag_filters_for_stats(df, y, "raw_temp_c")

    assert filtered.tolist() == [10.0, 30.0]
    assert any("excluded SENSOR_FAULT rows: 1" in n for n in notes)


def test_apply_flag_filters_for_stats_excludes_cal_invalid_for_cal_temp_stats():
    sensor_fault_mask = next(mask for mask, name in p._LOG_RECORD_FLAG_DEFS if name == "SENSOR_FAULT")
    cal_valid_mask = next(mask for mask, name in p._LOG_RECORD_FLAG_DEFS if name == "CAL_VALID")
    df = pd.DataFrame({"flags": [cal_valid_mask, 0, cal_valid_mask | sensor_fault_mask]})
    y = pd.Series([1.0, 50.0, 100.0])

    filtered, notes = p._apply_flag_filters_for_stats(df, y, "cal_temp_c")

    assert filtered.tolist() == [1.0]
    assert any("excluded SENSOR_FAULT rows: 1" in n for n in notes)
    assert any("excluded CAL_VALID==0 rows: 1" in n for n in notes)

def test_human_config_label_mappings():
    assert p._human_config_label("display_temperature_f") == "Display temperature in degrees F"
    assert p._human_config_label("rolling_mean_divisor") == "Rolling mean divisor"
    assert p._human_config_label("show_std_band") == "Show standard deviation band"
    assert p._human_config_label("highlight_outside_std") == "Highlight outside mean plus or minus 1 sigma"


def test_status_table_values_only_flag_count_percent():
    row = p.FlagSummaryRow("SENSOR_FAULT", "Sensor fault", 11, 0.04, "Sensor read failed or sensor fault was detected.", True, True)
    values = p._status_table_values(row)
    assert values == ("SENSOR_FAULT", 11, "0.04%")
    assert "Sensor read failed" not in values


def test_status_flag_meaning_for_key_selected_row():
    assert p._status_flag_meaning_for_key("SENSOR_FAULT") == "Sensor read failed or sensor fault was detected."


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
    assert '"config_version": 1' in path.read_text(encoding="utf-8")


def test_report_config_missing_required_fields_rejected(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text('{"config_version": 1, "input_timezone_mode": "UTC"}', encoding='utf-8')
    with pytest.raises(ValueError, match='Missing required config field'):
        p.load_report_config(str(path))


def test_report_config_missing_config_version_rejected(tmp_path):
    path = tmp_path / "missing_version.json"
    cfg = p.create_default_report_config()
    raw = cfg.__dict__.copy()
    import json
    path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="config_version"):
        p.load_report_config(str(path))


def test_report_config_future_config_version_rejected(tmp_path):
    path = tmp_path / "future_version.json"
    cfg = p.create_default_report_config()
    raw = cfg.__dict__.copy()
    raw["config_version"] = 999
    import json
    path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="Unsupported config_version"):
        p.load_report_config(str(path))


def test_report_config_invalid_json_produces_controlled_error(tmp_path):
    path = tmp_path / "invalid.json"
    path.write_text('{"config_version": 1,', encoding="utf-8")
    with pytest.raises(ValueError, match="Invalid JSON"):
        p.load_report_config(str(path))


def test_report_config_invalid_timezone_mode_rejected(tmp_path):
    path = tmp_path / "bad_mode.json"
    cfg = p.create_default_report_config()
    raw = cfg.__dict__.copy()
    raw["config_version"] = p.REPORT_CONFIG_VERSION
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


def _mk_loaded_for_bounds(source_tz):
    utc = pd.Series(pd.date_range('2026-01-01T00:00:00Z', periods=3, freq='min'))
    return SimpleNamespace(
        time_column="__time",
        dataframe=pd.DataFrame({"__time": utc}),
        time_source="epoch_utc",
        tzinfo=source_tz,
    )


def test_autofill_format_uses_utc_mode():
    cfg = p.create_default_report_config()
    cfg.input_timezone_mode = "UTC"
    cfg.display_timezone = "America/Chicago"
    loaded = _mk_loaded_for_bounds(datetime.timezone(datetime.timedelta(hours=-5)))
    display_tz = p._resolve_display_tz(cfg.display_timezone)
    start_utc, end_utc = p._get_loaded_time_bounds_utc(loaded)
    start_text, end_text = p._format_range_utc_for_input_fields(start_utc, end_utc, cfg, loaded, display_tz)
    assert start_text == "2026-01-01 00:00"
    assert end_text == "2026-01-01 00:02"


def test_autofill_format_uses_log_source_mode():
    cfg = p.create_default_report_config()
    cfg.input_timezone_mode = "Log/source time"
    cfg.display_timezone = "UTC"
    source_tz = datetime.timezone(datetime.timedelta(hours=-5))
    loaded = _mk_loaded_for_bounds(source_tz)
    display_tz = p._resolve_display_tz(cfg.display_timezone)
    start_utc, end_utc = p._get_loaded_time_bounds_utc(loaded)
    start_text, end_text = p._format_range_utc_for_input_fields(start_utc, end_utc, cfg, loaded, display_tz)
    assert start_text == "2025-12-31 19:00"
    assert end_text == "2025-12-31 19:02"


def test_validate_trim_record_id_shape_and_error():
    df = pd.DataFrame({"__x": [1, 2, 3], "record_id": [1, 2, 3]})
    out = p._validate_and_trim_by_minute(df, "__x", "", "")
    assert len(out) == 5
    assert out[4] == "full range"
    with pytest.raises(ValueError, match="Time trimming is unavailable"):
        p._validate_and_trim_by_minute(df, "__x", "2026-01-01 00:00", "")


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


def test_menu_contains_config_actions_and_config_buttons_removed():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        menuname = root["menu"]
        assert menuname
        labels = [root.nametowidget(menuname).entrycget(i, "label") for i in range(root.nametowidget(menuname).index("end") + 1)]
        assert "File" in labels
        assert "Options" in labels
        text_widgets = [w.cget("text") for w in app.root.winfo_children()[0].winfo_children() if hasattr(w, "cget")]
        assert "Load Config" not in text_widgets
        assert "New Config" not in text_widgets
        assert "Save Config" not in text_widgets
        assert "Edit Options" not in text_widgets
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
        cfg.rolling_mean_divisor = 22
        app._apply_report_config_to_ui(cfg)
        updated = app._require_report_config()
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
        cfg.max_plot_points = 1234
        app._apply_report_config_to_ui(cfg)
        app.save_config()
        loaded = p.load_report_config(str(path))
        assert loaded.max_plot_points == 1234
    finally:
        root.destroy()


def test_save_report_config_rejects_invalid_config_and_does_not_write(tmp_path):
    cfg = p.create_default_report_config()
    cfg.display_temperature_f = "false"
    path = tmp_path / "bad_config.json"
    with pytest.raises(ValueError):
        p.save_report_config(cfg, str(path))
    assert not path.exists()


def test_save_config_as_updates_path_and_clears_dirty(tmp_path, monkeypatch):
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
        app._config_dirty = True
        new_path = tmp_path / "new_config.json"
        monkeypatch.setattr(p.filedialog, "asksaveasfilename", lambda **kwargs: str(new_path))
        app.save_config_as()
        assert app.config_path == str(new_path)
        assert app._config_dirty is False
        reloaded = p.load_report_config(str(new_path))
        assert reloaded == cfg
        cfg.max_plot_points = 222
        app._apply_report_config_to_ui(cfg)
        app.save_config()
        reloaded2 = p.load_report_config(str(new_path))
        assert reloaded2.max_plot_points == 222
    finally:
        root.destroy()


def test_options_dialog_collection_includes_stats_and_highlighting():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        vars_map = {
            "input_timezone_mode": tk.StringVar(value="UTC"),
            "display_timezone": tk.StringVar(value="Local"),
            "y_axis_series": tk.StringVar(value="cal_temp_c"),
            "display_temperature_f": tk.BooleanVar(value=False),
            "overlay_raw_temp_c": tk.BooleanVar(value=False),
            "smooth_enabled": tk.BooleanVar(value=True),
            "rolling_mean_divisor": tk.StringVar(value="40"),
            "downsample_enabled": tk.BooleanVar(value=True),
            "max_plot_points": tk.StringVar(value="20000"),
            "pdf_plot_dpi": tk.StringVar(value="600"),
            "vector_pdf": tk.BooleanVar(value=False),
            "show_minimum": tk.BooleanVar(value=True),
            "show_maximum": tk.BooleanVar(value=True),
            "show_average": tk.BooleanVar(value=True),
            "show_std_band": tk.BooleanVar(value=True),
            "highlight_outside_std": tk.BooleanVar(value=True),
            "highlight_mask_from_rolling_mean": tk.BooleanVar(value=True),
            "highlight_above_enabled": tk.BooleanVar(value=True),
            "highlight_above_value": tk.StringVar(value="12.3"),
            "highlight_below_enabled": tk.BooleanVar(value=True),
            "highlight_below_value": tk.StringVar(value="-1.2"),
            "pdf_sensor_fault_threshold_percent": tk.StringVar(value="0.10"),
        }
        cfg = app._collect_report_config_from_options_dialog(vars_map)
        assert cfg.show_minimum is True and cfg.show_maximum is True
        assert cfg.show_average is True and cfg.show_std_band is True
        assert cfg.highlight_outside_std is True
        assert cfg.highlight_mask_from_rolling_mean is True
        assert cfg.highlight_above_enabled is True
        assert cfg.highlight_above_value == pytest.approx(12.3)
        assert cfg.highlight_below_enabled is True
        assert cfg.highlight_below_value == pytest.approx(-1.2)
    finally:
        root.destroy()


def test_options_dialog_collection_disabled_highlights_store_none():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        vars_map = {
            "input_timezone_mode": tk.StringVar(value="UTC"),
            "display_timezone": tk.StringVar(value="UTC"),
            "y_axis_series": tk.StringVar(value="cal_temp_c"),
            "display_temperature_f": tk.BooleanVar(value=False),
            "overlay_raw_temp_c": tk.BooleanVar(value=False),
            "smooth_enabled": tk.BooleanVar(value=True),
            "rolling_mean_divisor": tk.StringVar(value="1"),
            "downsample_enabled": tk.BooleanVar(value=True),
            "max_plot_points": tk.StringVar(value="10"),
            "pdf_plot_dpi": tk.StringVar(value="300"),
            "vector_pdf": tk.BooleanVar(value=False),
            "show_minimum": tk.BooleanVar(value=False),
            "show_maximum": tk.BooleanVar(value=False),
            "show_average": tk.BooleanVar(value=False),
            "show_std_band": tk.BooleanVar(value=False),
            "highlight_outside_std": tk.BooleanVar(value=False),
            "highlight_mask_from_rolling_mean": tk.BooleanVar(value=False),
            "highlight_above_enabled": tk.BooleanVar(value=False),
            "highlight_above_value": tk.StringVar(value="99"),
            "highlight_below_enabled": tk.BooleanVar(value=False),
            "highlight_below_value": tk.StringVar(value="-99"),
            "pdf_sensor_fault_threshold_percent": tk.StringVar(value="0.1"),
        }
        cfg = app._collect_report_config_from_options_dialog(vars_map)
        assert cfg.highlight_above_value is None
        assert cfg.highlight_below_value is None
    finally:
        root.destroy()


def test_options_dialog_collection_highlight_enabled_requires_numeric():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        vars_map = {
            "input_timezone_mode": tk.StringVar(value="UTC"),
            "display_timezone": tk.StringVar(value="UTC"),
            "y_axis_series": tk.StringVar(value="cal_temp_c"),
            "display_temperature_f": tk.BooleanVar(value=False),
            "overlay_raw_temp_c": tk.BooleanVar(value=False),
            "smooth_enabled": tk.BooleanVar(value=True),
            "rolling_mean_divisor": tk.StringVar(value="1"),
            "downsample_enabled": tk.BooleanVar(value=True),
            "max_plot_points": tk.StringVar(value="10"),
            "pdf_plot_dpi": tk.StringVar(value="300"),
            "vector_pdf": tk.BooleanVar(value=False),
            "show_minimum": tk.BooleanVar(value=False),
            "show_maximum": tk.BooleanVar(value=False),
            "show_average": tk.BooleanVar(value=False),
            "show_std_band": tk.BooleanVar(value=False),
            "highlight_outside_std": tk.BooleanVar(value=False),
            "highlight_mask_from_rolling_mean": tk.BooleanVar(value=False),
            "highlight_above_enabled": tk.BooleanVar(value=True),
            "highlight_above_value": tk.StringVar(value=""),
            "highlight_below_enabled": tk.BooleanVar(value=True),
            "highlight_below_value": tk.StringVar(value=""),
            "pdf_sensor_fault_threshold_percent": tk.StringVar(value="0.1"),
        }
        with pytest.raises(ValueError):
            app._collect_report_config_from_options_dialog(vars_map)
    finally:
        root.destroy()


def test_apply_invalid_options_does_not_mutate_existing_report_config():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        original = p.create_default_report_config()
        app._apply_report_config_to_ui(original)
        vars_map = {
            "input_timezone_mode": tk.StringVar(value=original.input_timezone_mode),
            "display_timezone": tk.StringVar(value=original.display_timezone),
            "y_axis_series": tk.StringVar(value=original.y_axis_series),
            "display_temperature_f": tk.BooleanVar(value=original.display_temperature_f),
            "overlay_raw_temp_c": tk.BooleanVar(value=original.overlay_raw_temp_c),
            "smooth_enabled": tk.BooleanVar(value=original.smooth_enabled),
            "rolling_mean_divisor": tk.StringVar(value="40.5"),
            "downsample_enabled": tk.BooleanVar(value=original.downsample_enabled),
            "max_plot_points": tk.StringVar(value=str(original.max_plot_points)),
            "pdf_plot_dpi": tk.StringVar(value=str(original.pdf_plot_dpi)),
            "vector_pdf": tk.BooleanVar(value=original.vector_pdf),
            "show_minimum": tk.BooleanVar(value=original.show_minimum),
            "show_maximum": tk.BooleanVar(value=original.show_maximum),
            "show_average": tk.BooleanVar(value=original.show_average),
            "show_std_band": tk.BooleanVar(value=original.show_std_band),
            "highlight_outside_std": tk.BooleanVar(value=original.highlight_outside_std),
            "highlight_mask_from_rolling_mean": tk.BooleanVar(value=original.highlight_mask_from_rolling_mean),
            "highlight_above_enabled": tk.BooleanVar(value=original.highlight_above_enabled),
            "highlight_above_value": tk.StringVar(value=""),
            "highlight_below_enabled": tk.BooleanVar(value=original.highlight_below_enabled),
            "highlight_below_value": tk.StringVar(value=""),
            "pdf_sensor_fault_threshold_percent": tk.StringVar(value=str(original.pdf_sensor_fault_threshold_percent)),
        }
        with pytest.raises(ValueError):
            p.validate_report_config(app._collect_report_config_from_options_dialog(vars_map))
        assert app.report_config == original
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
    with pytest.raises(ValueError, match="raw_temp_c"):
        p.build_plot_options_from_config(cfg, _mk_loaded_log(df), df, p.DisplayTimeConfig(None, "n/a"))


def test_build_plot_options_invalid_display_timezone_blocks():
    cfg = p.create_default_report_config()
    cfg.display_timezone = "Bad/Timezone"
    with pytest.raises(ValueError, match="Bad/Timezone"):
        p.validate_report_config(cfg)


def test_operator_validation_messages_do_not_expose_tracebacks():
    cfg = p.create_default_report_config()
    cfg.display_timezone = "Bad/Timezone"
    with pytest.raises(ValueError) as exc:
        p.validate_report_config(cfg)
    msg = str(exc.value)
    assert "Traceback" not in msg
    assert "File \"" not in msg


def test_validate_report_config_invalid_input_timezone_mode_blocks():
    cfg = p.create_default_report_config()
    cfg.input_timezone_mode = "BadMode"
    with pytest.raises(ValueError, match="input_timezone_mode"):
        p.validate_report_config(cfg)


@pytest.mark.parametrize("bad_value", ["false", "true", "yes", "no", "0", "1"])
def test_validate_report_config_rejects_string_booleans(bad_value):
    cfg = p.create_default_report_config()
    cfg.display_temperature_f = bad_value
    with pytest.raises(ValueError, match="boolean"):
        p.validate_report_config(cfg)


def test_validate_report_config_rejects_float_integer_field():
    cfg = p.create_default_report_config()
    cfg.rolling_mean_divisor = 40.5
    with pytest.raises(ValueError, match="integer"):
        p.validate_report_config(cfg)


def test_validate_report_config_rejects_numeric_string_integer_field():
    cfg = p.create_default_report_config()
    cfg.rolling_mean_divisor = "40"
    with pytest.raises(ValueError, match="integer"):
        p.validate_report_config(cfg)


def test_validate_report_config_rejects_negative_integer_field():
    cfg = p.create_default_report_config()
    cfg.max_plot_points = -1
    with pytest.raises(ValueError, match=">= 1"):
        p.validate_report_config(cfg)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_validate_report_config_rejects_non_finite_threshold(bad_value):
    cfg = p.create_default_report_config()
    cfg.pdf_sensor_fault_threshold_percent = bad_value
    with pytest.raises(ValueError, match="finite"):
        p.validate_report_config(cfg)


def test_validate_report_config_highlight_upper_enabled_with_null_fails():
    cfg = p.create_default_report_config()
    cfg.highlight_above_enabled = True
    cfg.highlight_above_value = None
    with pytest.raises(ValueError, match="highlight_above_value"):
        p.validate_report_config(cfg)


def test_validate_report_config_highlight_upper_disabled_with_value_fails():
    cfg = p.create_default_report_config()
    cfg.highlight_above_enabled = False
    cfg.highlight_above_value = 10.0
    with pytest.raises(ValueError, match="highlight_above_value"):
        p.validate_report_config(cfg)


def test_validate_report_config_highlight_lower_enabled_with_null_fails():
    cfg = p.create_default_report_config()
    cfg.highlight_below_enabled = True
    cfg.highlight_below_value = None
    with pytest.raises(ValueError, match="highlight_below_value"):
        p.validate_report_config(cfg)


def test_validate_report_config_highlight_lower_disabled_with_value_fails():
    cfg = p.create_default_report_config()
    cfg.highlight_below_enabled = False
    cfg.highlight_below_value = -10.0
    with pytest.raises(ValueError, match="highlight_below_value"):
        p.validate_report_config(cfg)


def test_validate_report_config_unresolved_timezone_fails():
    cfg = p.create_default_report_config()
    cfg.display_timezone = "Mars/Olympus"
    with pytest.raises(ValueError, match="Invalid timezone"):
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


def test_actions_without_config_are_blocked():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        messages = []
        original = p.messagebox.showerror
        p.messagebox.showerror = lambda title, msg: messages.append((title, msg))
        try:
            app.plot()
            app.export_pdf()
            app.save_trimmed_csv()
        finally:
            p.messagebox.showerror = original
        assert len(messages) == 3
        assert all("Load a report configuration" in msg for _title, msg in messages)
    finally:
        root.destroy()


def test_get_trimmed_df_uses_config_input_timezone_mode():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        cfg = p.create_default_report_config()
        cfg.input_timezone_mode = "UTC"
        cfg.display_timezone = "UTC"
        app._apply_report_config_to_ui(cfg)
        df = pd.DataFrame({"__time": pd.to_datetime(["2026-01-01T00:00:00Z", "2026-01-01T00:01:00Z"]), "flags":[0,0], "cal_temp_c":[1.0,2.0]})
        app.loaded = _mk_loaded_log(df)
        app.start_time_text.set("2026-01-01 00:00")
        app.end_time_text.set("2026-01-01 00:00")
        trimmed, *_ = app._get_trimmed_df(cfg)
        assert len(trimmed) == 1
    finally:
        root.destroy()


def test_plot_and_pdf_share_same_settings_object():
    cfg = p.create_default_report_config()
    cfg.y_axis_series = "cal_temp_c"
    df = pd.DataFrame({"cal_temp_c":[1.0], "raw_temp_c":[1.0], "flags":[0], "fault_status":[0]})
    loaded = _mk_loaded_log(df)
    display = p.DisplayTimeConfig(datetime.timezone.utc, "UTC")
    opts1 = p.build_plot_options_from_config(cfg, loaded, df, display)
    opts2 = p.build_plot_options_from_config(cfg, loaded, df, display)
    assert opts1 == opts2


def test_compact_main_window_uses_no_legacy_tk_variables():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk unavailable in this environment")
    root.withdraw()
    try:
        app = p.PlotterApp(root)
        for legacy in ("scenario_choice", "csv_path_var", "pdf_path_var"):
            assert not hasattr(app, legacy)
    finally:
        root.destroy()

def test_get_available_y_axis_series_choices_defaults_when_unloaded():
    app = p.PlotterApp.__new__(p.PlotterApp)
    app.loaded = None
    cfg = p.create_default_report_config()
    choices = p.PlotterApp._get_available_y_axis_series_choices(app, cfg)
    assert choices[:3] == ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms"]


def test_get_available_y_axis_series_choices_uses_numeric_loaded_columns():
    app = p.PlotterApp.__new__(p.PlotterApp)
    app.loaded = SimpleNamespace(dataframe=pd.DataFrame({
        "__time": pd.date_range("2026-01-01", periods=3, tz="UTC"),
        "cal_temp_c": [1.0, 2.0, 3.0],
        "raw_temp_c": [2.0, 3.0, 4.0],
        "raw_rtd_ohms": [100.0, 101.0, 102.0],
        "custom_metric": [9.0, 8.0, 7.0],
        "flags": [0, 0, 0],
    }))
    cfg = p.create_default_report_config()
    cfg.y_axis_series = "raw_temp_c"
    choices = p.PlotterApp._get_available_y_axis_series_choices(app, cfg)
    assert choices[:3] == ["cal_temp_c", "raw_temp_c", "raw_rtd_ohms"]
    assert "custom_metric" in choices
    assert "flags" not in choices


def test_invalid_configured_y_axis_series_blocks_action_with_clear_error():
    app = p.PlotterApp.__new__(p.PlotterApp)
    cfg = p.create_default_report_config()
    cfg.y_axis_series = "does_not_exist"
    app.report_config_loaded = True
    app.report_config = cfg
    app.loaded = SimpleNamespace(
        dataframe=pd.DataFrame({"cal_temp_c": [1.0], "flags": [0], "fault_status": [0]}),
        tzinfo=datetime.timezone.utc,
    )
    with pytest.raises(ValueError, match="Configured y-axis series does_not_exist is not present"):
        p.PlotterApp._validate_loaded_config_for_action(app)
