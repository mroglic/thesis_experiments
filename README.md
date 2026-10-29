# thesis_experiments

Thesis experiments: VIO (Spectacular AI) with an IR image, so tracking works in the dark next to a projector.

| Folder | Camera | What it does |
|---|---|---|
| `oak/` | OAK-D Pro W | Live VIO on the mono IR cameras, with the IR flood light and a 3D view |
| `orbbec/` | Orbbec Femto Mega | Live VIO on the colour camera; recording ToF IR + depth + IMU and playing it through VIO (Replay) |

Everything was tried on Windows 11 with Python 3.12.

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

- In the IR window: `+` / `-` change the flood brightness, `d` toggles the dot projector, `q` quits.
- In the 3D window: the mouse turns the view, the wheel zooms, `c` changes the view, `q` quits.
- Poses are written to `poses.jsonl` (time, position, status, flood).

Here VIO tracks on the two mono cameras, which see IR. That is why the flood light is enough to work in the dark, with no changes to the SDK.

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
