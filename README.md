# thesis_experiments

Thesis experiments: VIO (Spectacular AI) with an IR image, so tracking works in the dark next to a projector.

| Folder | Camera | What it does |
|---|---|---|
| `oak/` | OAK-D Pro W | Live VIO on the mono IR cameras, with the IR flood light and a 3D view |
| `orbbec/` | Orbbec Femto Mega | Live VIO on the colour camera; recording ToF IR + depth + IMU and playing it through VIO (Replay) |

Everything was tried on Windows 11 with Python 3.12.

## Quick start

Use a virtual environment: `spectacularAI[full]` needs depthai 2.x, and the machine may already have depthai 3 installed for something else. Any Python 3.8–3.13 works.

```powershell
git clone https://github.com/mroglic/thesis_experiments.git
cd thesis_experiments
python -m venv .venv
.venv\Scripts\python -m pip install "spectacularAI[full]" opencv-python
.venv\Scripts\python oak\ir_vio.py      # OAK on a USB 3 port
```

Next time: `cd thesis_experiments`, then `.venv\Scripts\python oak\ir_vio.py`.
FFmpeg 7.1.1 is needed only for `--record`. If it exits silently or with a DLL error, check Smart App Control (step 4 below).

## Windows setup (once)

1. **Python 3.12**: `winget install -e --id Python.Python.3.12`
2. **FFmpeg 7.1.1**: `winget install -e --id Gyan.FFmpeg --version 7.1.1`.
   Do not use 8 or 9: Spectacular AI reads recordings with the `-vsync` option, which those versions removed, so Replay reports `Failed to read frame #0`.
3. **Packages**: `python -m pip install "spectacularAI[full]" opencv-python numpy pyorbbecsdk2`
4. **Smart App Control** (Windows Security → App & browser control) must be **Off**. Otherwise Windows silently blocks unsigned DLLs (OrbbecSDK, Spectacular AI). The error code is `0xC0E90002`, and the block shows up in Event Viewer → CodeIntegrity.
5. Plug the camera into a **USB 3** port.

## OAK-D Pro W: live IR VIO (lab)

```
cd oak
python ir_vio.py                 # flood 0.6, dots off, 3D view + IR image window
python ir_vio.py --flood 0       # same without IR light, for comparison
python ir_vio.py --record out\dark --no_slam
```

- In the IR window: `+` / `-` change the flood brightness (up to 0.8, `--max_flood`), `d` toggles the dot projector, `r` resets the cube in front of the camera, `q` quits.
  It opens at the top left of the main monitor; `--ir_pos x,y` places it elsewhere (e.g. `1940,40` for a monitor to the right).
- In the 3D window: the view stays put (`--follow` makes it follow the camera); the mouse turns the view, the wheel zooms, `c` changes the view, `q` quits.
- AR test: a 20 cm wireframe cube is placed 1 m in front of the camera when tracking starts, and drawn both in the 3D view and in the IR image. If VIO works, it stays in place while the camera moves. `--no_cube`, `--cube_size`, `--cube_distance`.
- Above flood 0.8 the light switched itself off (the image kept running), most likely the LED driver's protection on USB power; hence the limit.
- The IR window also shows the sensor's auto exposure (exposure time, ISO). In the dark it rises: long exposure means motion blur, high ISO means noise.
- `--low_latency`: a pose on every IMU sample instead of every frame, as in ARML. Matters for the projector; here the cube then runs slightly ahead of the (older) image.
- Poses are written to `poses.jsonl` (time, position, status, flood, dots, exposure, ISO).

Here VIO tracks on the two mono cameras, which see IR. That is why the flood light is enough to work in the dark, with no changes to the SDK.
ARML does the same (stereo mono cameras, OAK feature tracker), but without SLAM, with `lowLatency`, and it never turns the IR light on.

### Light test: light, dark, dark_flood

The same route under three lights, to show what the flood light changes:

| Test | Room | Flood |
|---|---|---|
| `light` | lights on | off |
| `dark` | dark (lights off, curtains drawn, monitors off or turned away) | off |
| `dark_flood` | dark | 0.6 |

Each run is VIO only (`--no_slam`): with SLAM, coming back to the start closes the loop and hides the drift.

1. Mark a start spot with tape and pick a route of 30–40 s: 5 s still on the mark, slowly around the table or room (4–6 m), one quicker left-right turn, back to the mark, 5 s still. Walk it the same way every time (direction, speed, what the camera looks at).
2. Run `oak\light.bat`, `oak\dark.bat` or `oak\dark_flood.bat` (or `python oak\ir_vio.py --test dark_flood`). Start on the mark, wait for `TRACKING` in the IR window, walk the route, come back to the mark, wait, press `q`.
3. Do each test 2–3 times. Every run goes to a new file, `runs\<test>_<n>.jsonl`. `oak\del_runs.bat` deletes all runs (it asks first).
4. After each run the .bat prints the comparison of all runs so far (`python oak\compare_runs.py`): duration, tracking %, lost periods, path length, end–start distance (drift), median exposure and ISO, and the mean per test.

The path length should be about the same in all runs (same route); end–start is the drift.

## Orbbec Femto Mega

The Spectacular AI Orbbec plugin is not on pip. Download `spectacularAI_orbbecPlugin_cpp_non-commercial_1.53.2.tar.gz` from
[SpectacularAI/sdk releases (v1.53.2)](https://github.com/SpectacularAI/sdk/releases/tag/v1.53.2) and unpack it.
Put the files from `orbbec/` next to the package's `Windows\` folder:

```
sai\
├─ Windows\bin\sai-record-orbbec.exe, vio_jsonl.exe, *.dll   (from the package, closed source)
├─ start_vio.bat  test_vio.bat  record_ir.bat  replay_ir.bat
├─ ir_record.py  ir_replay.py
└─ recordings\
```

**Frame timestamps (once, with the camera connected).** In PowerShell as administrator, run
[`obsensor_metadata_win10.ps1`](https://github.com/orbbec/OrbbecSDK/blob/v1.10.18/misc/scripts/obsensor_metadata_win10.ps1) `-op install_all`, then unplug and replug the camera.
Without this, VIO discards frames (`Duplicate or unordered frame timestamps`).

### Live VIO (colour)

- `test_vio.bat`: poses as JSON in the console.
- `start_vio.bat`: 3D view + recording to `recordings\<time>\`. Add `--no_record` for the view only.

### IR only (record → Replay)

The plugin always tracks on the colour camera and offers no way to give it another image. So `ir_record.py` records the ToF IR image, depth and IMU in the same format as `sai-record-orbbec`, and `ir_replay.py` plays the recording through VIO as if live.

1. `record_ir.bat`: an IR window opens. Walk around with the camera, come back to where you started, and press `q`.
   Options: `--resolution 640x576` (narrower view, no black circle), `--ir_max 800` (brighter image), `--no_depth`.
2. `replay_ir.bat`: plays the newest `recordings\ir_*` with the 3D view. At the end it prints the share of time spent tracking, the lost-tracking periods, the path length and the end–start distance (drift).
   To play a colour recording for comparison: `replay_ir.bat recordings\<time>`.

`ir_record.py` takes `imuToCamera` from the calibration of a `sai-record-orbbec` recording, because `camera1` there is the same ToF camera. So record a few seconds with `start_vio.bat` first.

Depth is recorded but not used for tracking (`useStereo: false` in `vio_config.yaml`). IR and depth come from the same sensor and share one calibration, and Spectacular AI then treats them as a stereo pair with zero baseline: tracking diverged within 3–6 s and was lost for most of the recording. With depth off, the same recordings track the whole time. To experiment with internal parameters: `replay_ir.bat <folder> --set key=value`.

First results (2026-10-29, home, lights on; separate sessions, not the same path, so only indicative):

| Recording | Tracking | Path | End–start |
|---|---|---|---|
| colour, 92 s | 100 % | 7.4 m | 0.12 m |
| ToF IR 512×512, 39 s | 100 % | 6.5 m | 0.75 m |
| ToF IR 512×512, 27 s | 100 % | 5.9 m | 0.29 m |

Undistorting the wide ToF image to a 90° or 70° pinhole camera did not help (slightly worse drift), so the original image is used.

### Troubleshooting

- **Replay freezes**: tracking is lost. `ir_replay.py` keeps drawing the last good pose and prints the lost periods in the console.
- **`IMU buffer max size exceeded` over and over, no poses**: the camera is not sending frames. Either another program is using it, or it was left in a bad state after being force-closed. Unplug and replug it.
- **`sai-record-orbbec` crashes after 2–3 s**: `ffmpeg` is not on PATH.

## What is open and what is closed

These scripts and `sai-record-orbbec` / `vio_jsonl` are open source. The VIO/SLAM engine (`spectacularAI_orbbecPlugin.dll`, `spectacularAI_native*.pyd`, `libspectacularAI_depthaiPlugin.so`) is closed and free for non-commercial use only. Feeding your own frames live (the Input API) needs a commercial license, so Replay is the only free way in.
