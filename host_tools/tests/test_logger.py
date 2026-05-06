import pytest
serial = pytest.importorskip("serial")
from pathlib import Path
import importlib.util
import host_tools.logger as logger


def test_parse_reset_sequence_valid_and_empty():
    assert logger._parse_reset_sequence("") == []
    assert logger._parse_reset_sequence("R0|D1|W0.5") == [("R", "0"), ("D", "1"), ("W", "0.5")]


def test_parse_reset_sequence_invalid_opcode_and_arg():
    import pytest
    with pytest.raises(ValueError):
        logger._parse_reset_sequence("X1")
    with pytest.raises(ValueError):
        logger._parse_reset_sequence("R2")


def test_format_host_prefix_toggle():
    assert logger._format_host_prefix(False) == ""
    pref = logger._format_host_prefix(True)
    assert pref.startswith("[") and pref.endswith("] ")


def test_module_import_no_file_side_effects(tmp_path):
    before = set(tmp_path.iterdir())
    spec = importlib.util.spec_from_file_location("logger_module", Path("host_tools/logger.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    after = set(tmp_path.iterdir())
    assert before == after
