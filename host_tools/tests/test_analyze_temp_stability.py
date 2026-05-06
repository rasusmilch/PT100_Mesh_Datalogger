import pytest
pd = pytest.importorskip('pandas')
np = pytest.importorskip('numpy')
from host_tools import analyze_temp_stability as ats


def test_prepare_and_drift_calculation():
    df = pd.DataFrame({
        'epoch_utc':[0,60,120,180,240,300],
        'cal_temp_c':[10,10.1,10.2,10.3,10.4,10.5],
        'fault_status':['0x00']*6,
    })
    prepared = ats.prepare_dataframe(df, 'cal_temp_c', include_faulted=False)
    out = ats.compute_trailing_window_drift(prepared, window_sec=180, min_samples=3)
    assert out['drift_c_per_min'].dropna().iloc[-1] == pytest.approx(0.1, rel=1e-6)


def test_find_stable_segments_and_nan_handling():
    df = pd.DataFrame({
        'elapsed_sec':[0,60,120,180],
        'timestamp': pd.to_datetime(['2026-01-01T00:00:00Z','2026-01-01T00:01:00Z','2026-01-01T00:02:00Z','2026-01-01T00:03:00Z']),
        'temperature_c':[20.0,20.0,np.nan,20.0],
        'drift_c_per_min':[0.01,0.01,0.5,0.01],
    })
    stable = ats.find_stable_segments(df, drift_threshold_c_per_min=0.05, hold_sec=0, target_temp_c=None, setpoint_band_c=None)
    assert len(stable) >= 1
