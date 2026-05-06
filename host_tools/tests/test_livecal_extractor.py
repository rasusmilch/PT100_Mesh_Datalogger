import pytest
from host_tools.livecal_extractor import parse_livecal_line


def test_parse_livecal_line_happy_path():
    line = "cal livecal: ts=2026-04-17T20:32:38Z n=120 raw_ohm=118.838 std_ohm=0.006 mean_ohm=118.840 cal_temp=48.428C std_temp=0.016C mean_temp=48.433C drift=0.003C/min"
    out = parse_livecal_line(line)
    assert out == "2026-04-17T20:32:38Z,48.433,118.840,0.016,0.006,0.003"


def test_parse_livecal_line_rejects_malformed():
    with pytest.raises(ValueError):
        parse_livecal_line("cal livecal: missing required numeric fields")


def test_parse_livecal_multiple_points_and_optional_tail():
    line1 = "ts=2026-01-01T00:00:00Z std_ohm=0.001 mean_ohm=100.100 std_temp=0.002C mean_temp=25.000C drift=-0.001C/min"
    line2 = "ts=2026-01-01T00:01:00Z std_ohm=0.004 mean_ohm=100.200 std_temp=0.005C mean_temp=25.100C drift=0.000C/min fault=none"
    assert parse_livecal_line(line1).startswith("2026-01-01T00:00:00Z,25.000")
    assert parse_livecal_line(line2).endswith(",0.000")
