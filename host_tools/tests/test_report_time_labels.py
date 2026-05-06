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


def test_pdf_summary_rows_put_node_id_first_and_keep_measurement_window():
    summary_rows = [
        ["Metric", "Result"],
        ["Node ID", "98:A3:16:12:57:B0"],
        ["Reporting time zone", "Central Daylight Time (UTC-05:00)"],
        ["Measurement window", "2026-05-04 19:00 → 2026-05-05 18:59 (span 23h 59m)"],
    ]
    assert summary_rows[1][0] == "Node ID"
    assert summary_rows[1][1] == "98:A3:16:12:57:B0"
    assert any(row[0] == "Measurement window" for row in summary_rows)


def test_export_pdf_omits_subtitle_when_blank(monkeypatch, tmp_path):
    captured = {"paragraphs": []}

    class DummyDoc:
        def __init__(self, *args, **kwargs):
            pass

        def build(self, elements):
            captured["elements"] = elements

    class DummyTable:
        def __init__(self, *args, **kwargs):
            pass

        def setStyle(self, *args, **kwargs):
            return None

    class DummyImage:
        def __init__(self, *args, **kwargs):
            pass

    def fake_paragraph(text, _style):
        captured["paragraphs"].append(text)
        return ("P", text)

    monkeypatch.setattr(plotter, "SimpleDocTemplate", DummyDoc)
    monkeypatch.setattr(plotter, "Table", DummyTable)
    monkeypatch.setattr(plotter, "Image", DummyImage)
    monkeypatch.setattr(plotter, "Paragraph", fake_paragraph)

    plotter._export_pdf_report(
        save_path=str(tmp_path / "report.pdf"),
        fig_png_path=str(tmp_path / "plot.png"),
        source_files=["a.csv"],
        summary_rows=[["Metric", "Result"], ["Node ID", "98:A3:16:12:57:B0"], ["Measurement window", "x"]],
        calibration_rows=[["Calibration item", "Recorded value"], ["Calibration points used", "3"]],
        title="PT100 Temperature Log Report",
        subtitle=None,
        warning_text=None,
    )

    assert captured["paragraphs"][0] == "PT100 Temperature Log Report"
    assert not any("Nodes:" in text and "Time range:" in text for text in captured["paragraphs"])
