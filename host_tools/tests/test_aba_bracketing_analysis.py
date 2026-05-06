from pathlib import Path
import pytest
from host_tools import aba_bracketing_analysis as aba


def _write_csv(path: Path, text: str) -> Path:
    path.write_text(text, encoding='utf-8')
    return path


def test_parse_and_analyze_aba(tmp_path):
    csv_path = _write_csv(tmp_path / 'in.csv', "run,bore,timestamp,mean_temp_C,mean_ohm\n1,A,2026-01-01T00:00:00Z,10.0,100.0\n1,B,2026-01-01T00:10:00Z,11.0,101.0\n1,A,2026-01-01T00:20:00Z,12.0,102.0\n")
    records = aba.load_records_from_csv(csv_path)
    grouped = aba.group_records_by_run(records)
    result = aba.analyze_single_run('1', grouped['1'])
    assert result.expected_middle_temp_c == pytest.approx(11.0)
    assert result.temp_difference_c == pytest.approx(0.0)


def test_analyze_bab_ordering_supported(tmp_path):
    csv_path = _write_csv(tmp_path / 'in.csv', "run,bore,timestamp,mean_temp_C,mean_ohm\n2,B,2026-01-01T00:00:00Z,20.0,200.0\n2,A,2026-01-01T00:10:00Z,21.0,201.0\n2,B,2026-01-01T00:20:00Z,22.0,202.0\n")
    records = aba.load_records_from_csv(csv_path)
    result = aba.analyze_single_run('2', records)
    assert result.expected_middle_ohm == pytest.approx(201.0)


def test_reject_insufficient_bracket_data(tmp_path):
    csv_path = _write_csv(tmp_path / 'bad.csv', "run,bore,timestamp,mean_temp_C,mean_ohm\n1,A,2026-01-01T00:00:00Z,10.0,100.0\n1,B,2026-01-01T00:10:00Z,11.0,101.0\n")
    records = aba.load_records_from_csv(csv_path)
    with pytest.raises(ValueError):
        aba.analyze_single_run('1', records)
