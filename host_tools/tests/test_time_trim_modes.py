import pytest
pd = pytest.importorskip("pandas")
import datetime

from host_tools import pt100_plotter as plotter


def _sample_df():
    utc_index = pd.date_range('2026-05-05 00:00', periods=60*30, freq='min', tz='UTC')
    return pd.DataFrame({'__time': utc_index, 'value': range(len(utc_index)), 'record_id': range(len(utc_index))})


def test_utc_input_same_rows_across_display_tz():
    df = _sample_df()
    utc_series = plotter._canonicalize_time_to_utc(df['__time'], 'epoch_utc', datetime.timezone.utc)
    trimmed_utc, *_ = plotter._validate_and_trim_by_minute(
        df, '__time', '2026-05-05 12:35', '2026-05-06 04:57',
        time_series_utc=utc_series, input_timezone_mode='UTC', display_tz=datetime.timezone.utc, source_tz=datetime.timezone.utc
    )
    local_tz = plotter._get_local_tz()
    trimmed_local_display, *_ = plotter._validate_and_trim_by_minute(
        df, '__time', '2026-05-05 12:35', '2026-05-06 04:57',
        time_series_utc=utc_series, input_timezone_mode='UTC', display_tz=local_tz, source_tz=datetime.timezone.utc
    )
    assert list(trimmed_utc.index) == list(trimmed_local_display.index)


def test_same_as_display_can_change_rows():
    df = _sample_df()
    utc_series = plotter._canonicalize_time_to_utc(df['__time'], 'epoch_utc', datetime.timezone.utc)
    utc_minus_5 = datetime.timezone(datetime.timedelta(hours=-5), name="UTC-05")
    trimmed_utc, *_ = plotter._validate_and_trim_by_minute(
        df, '__time', '2026-05-05 00:35', '2026-05-05 00:57',
        time_series_utc=utc_series, input_timezone_mode='Same as display', display_tz=datetime.timezone.utc, source_tz=datetime.timezone.utc
    )
    trimmed_local, *_ = plotter._validate_and_trim_by_minute(
        df, '__time', '2026-05-05 00:35', '2026-05-05 00:57',
        time_series_utc=utc_series, input_timezone_mode='Same as display', display_tz=utc_minus_5, source_tz=datetime.timezone.utc
    )
    assert len(trimmed_utc) != len(trimmed_local) or trimmed_utc.index[0] != trimmed_local.index[0]


def test_local_input_equivalent_utc_instants():
    df = _sample_df()
    utc_series = plotter._canonicalize_time_to_utc(df['__time'], 'epoch_utc', datetime.timezone.utc)
    local_tz = plotter._get_local_tz()
    start_local = pd.Timestamp('2026-05-05 12:35').tz_localize(local_tz).tz_convert('UTC').strftime('%Y-%m-%d %H:%M')
    end_local = pd.Timestamp('2026-05-06 04:57').tz_localize(local_tz).tz_convert('UTC').strftime('%Y-%m-%d %H:%M')
    trimmed_local_mode, *_ = plotter._validate_and_trim_by_minute(
        df, '__time', '2026-05-05 12:35', '2026-05-06 04:57',
        time_series_utc=utc_series, input_timezone_mode='Local', display_tz=local_tz, source_tz=datetime.timezone.utc
    )
    trimmed_utc_mode, *_ = plotter._validate_and_trim_by_minute(
        df, '__time', start_local, end_local,
        time_series_utc=utc_series, input_timezone_mode='UTC', display_tz=datetime.timezone.utc, source_tz=datetime.timezone.utc
    )
    assert list(trimmed_local_mode.index) == list(trimmed_utc_mode.index)
