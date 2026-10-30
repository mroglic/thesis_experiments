# Notes: commands to copy

Run from the `thesis_experiments` folder (cmd).

## Which Python and package versions

`spectacularAI` needs depthai 2.x. If one of these prints depthai `3.…`, that environment will not work with Spectacular AI.

```
.venv\Scripts\python -c "import sys, depthai, importlib.metadata as m; print(sys.version, depthai.__version__, m.version('spectacularAI'))"
python -c "import sys, depthai, importlib.metadata as m; print(sys.version, depthai.__version__, m.version('spectacularAI'))"
```

The first line is the venv, the second the system Python. `oak\*.bat` use the venv if it exists.

## Looking at a recording (`oak\sai_preview.bat` or `ir_vio.py --record`)

A recording is a folder with `data.jsonl` (IMU, frame times, features), `calibration.json` and `vio_config.yaml`; video only with `--gray`.
`oak\diagnose.bat` makes an HTML report of the newest `recordings\sai\*` (or the folder given) and opens it: IMU and frame rates, gaps, timestamps. Missing frames show up there.

```
oak\diagnose.bat
oak\diagnose.bat recordings\sai\2026-10-30_15-20-00
.venv\Scripts\python -m spectacularAI.cli.sai_cli diagnose recordings\sai\<folder> recordings\report.html
```

Quick look without the report: `dir recordings\sai\<folder>` (is `data.jsonl` empty?) and `type recordings\sai\<folder>\data.jsonl | more`.
