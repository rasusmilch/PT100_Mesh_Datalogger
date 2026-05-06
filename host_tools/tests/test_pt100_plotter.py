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
