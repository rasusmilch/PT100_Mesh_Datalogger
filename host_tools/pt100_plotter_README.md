# PT100 Plotter Config Workflow

## Operator workflow
1. **Start the tool**: run `python host_tools/pt100_plotter.py`.
2. **Load an existing config or create a new one** from the startup config prompt.
3. **Select logs** with *Select Log Files* or *Select Folder*.
4. **Select range** by entering start/end minute timestamps or using *Select range…*.
5. **Review status flags** in the Data Quality / Status Flags section.
6. **Plot** with the *Plot* button.
7. **Export PDF** with *Export PDF Report*.
8. **Save trimmed CSV** with *Save Trimmed CSV*.
9. **Edit options** with *Edit Options*.
10. **Save config** with *Save Config* after option edits.

## Notes on config-controlled behavior
- The **input timezone**, **display timezone**, and **y-axis series** are controlled by the JSON config and are not main-window operator knobs.
- If the configured y-axis series or timezone does not match selected logs, plotting/export is blocked so the issue can be investigated instead of silently producing unintended output.

## PDF sensor read failure threshold
- `pdf_sensor_fault_threshold_percent` controls when SENSOR_FAULT summary rows appear in the PDF status summary.
- At or above threshold: SENSOR_FAULT is included in the report summary.
- Below threshold: SENSOR_FAULT is treated as below-reporting threshold unless there are explicit non-zero MAX31865 fault bytes.
