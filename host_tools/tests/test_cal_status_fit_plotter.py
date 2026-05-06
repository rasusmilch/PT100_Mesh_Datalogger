import pytest
pytest.importorskip('numpy')
pytest.importorskip('matplotlib')
from host_tools import cal_status_fit_plotter as c

SAMPLE = """
Calibration Model Fit Report:
Calibration equation: y = a + b x
Fit domain: Temperature (C)
Model type: poly
Polynomial degree: 1
Calibration points used: 2
Model parameters: 2
Degrees of freedom: 0
Method: test
Effective last UTC: 2026-01-01T00:00:00Z
Due date: 2026-12-31
a: 1.0
b: 1.0
  1: reference_temp_C=10 ideal_ref_res_Ohm=100 captured_raw_temp_avg_C=10 captured_raw_res_avg_Ohm=100 residual_C=0 residual_res_Ohm=0
  2: reference_temp_C=20 ideal_ref_res_Ohm=200 captured_raw_temp_avg_C=20 captured_raw_res_avg_Ohm=200 residual_C=0 residual_res_Ohm=0
Fit Table:
1 10 100 10 100 10 100 0 0 0
2 20 200 20 200 20 200 0 0 0
"""

def test_parse_report_and_figures():
    report = c._parse_report(SAMPLE, 'sample.txt')
    assert report.coefficients['a'] == pytest.approx(1.0)
    assert len(report.fit_rows) == 2
    fig1 = c._build_curve_figure(report)
    fig2 = c._build_residuals_figure(report, use_symlog=False)
    fig1.clf(); fig2.clf()
