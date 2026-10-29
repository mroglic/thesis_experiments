# thesis_experiments

Eksperimenti za tezu: VIO (Spectacular AI) sa IR slikom, da praćenje radi u mraku uz projektor.

| Folder | Kamera | Šta radi |
|---|---|---|
| `oak/` | OAK-D Pro W | VIO uživo na mono IR kamerama, sa IR flood svetlom i 3D prikazom |
| `orbbec/` | Orbbec Femto Mega | VIO uživo na color kameri; snimanje ToF IR slike + dubine + IMU i puštanje kroz VIO (Replay) |

Sve je probano na Windows 11, Python 3.12.

## Priprema Windows računara (jednom)

1. **Python 3.12**: `winget install -e --id Python.Python.3.12`
2. **FFmpeg 7.1.1**: `winget install -e --id Gyan.FFmpeg --version 7.1.1`.
   Ne instaliraj 8 ili 9: Spectacular AI čita snimke sa opcijom `-vsync`, koje tamo više nema, pa Replay javlja `Failed to read frame #0`.
3. **Paketi**: `python -m pip install "spectacularAI[full]" opencv-python numpy pyorbbecsdk2`
4. **Smart App Control** (Windows Security → App & browser control) mora biti **Off**. Inače Windows bez poruke blokira nepotpisane DLL-ove (OrbbecSDK, Spectacular AI). Greška je `0xC0E90002`, a vidi se u Event Viewer → CodeIntegrity.
5. Kameru uključi u **USB 3** port.

## OAK-D Pro W: IR VIO uživo (lab)

```
cd oak
python ir_vio.py                 # flood 0.6, tačke ugašene, 3D prikaz + prozor sa IR slikom
python ir_vio.py --flood 0       # isto bez IR svetla, za poređenje
python ir_vio.py --record out\dark --no_slam
```

- U IR prozoru: `+` / `-` menja jačinu flood svetla, `d` pali i gasi projektor tačaka, `q` gasi program.
- U 3D prozoru: miš okreće pogled, točkić zumira, `c` menja pogled, `q` gasi.
- Poze se upisuju u `poses.jsonl` (vreme, pozicija, status, flood).

VIO ovde prati tačke na dve mono kamere koje vide IR. Zato je flood svetlo dovoljno za rad u mraku i ništa ne treba menjati u SDK-u.

## Orbbec Femto Mega

Spectacular AI Orbbec dodatak se ne instalira preko pip-a. Skini `spectacularAI_orbbecPlugin_cpp_non-commercial_1.53.2.tar.gz` sa
[SpectacularAI/sdk releases (v1.53.2)](https://github.com/SpectacularAI/sdk/releases/tag/v1.53.2) i raspakuj ga.
Fajlove iz `orbbec/` stavi pored foldera `Windows\` iz paketa:

```
sai\
├─ Windows\bin\sai-record-orbbec.exe, vio_jsonl.exe, *.dll   (iz paketa, zatvoren kod)
├─ start_vio.bat  test_vio.bat  record_ir.bat  replay_ir.bat
├─ ir_record.py  ir_replay.py
└─ recordings\
```

**Vremenske oznake (jednom, sa povezanom kamerom).** Pokreni u PowerShell-u kao administrator
[`obsensor_metadata_win10.ps1`](https://github.com/orbbec/OrbbecSDK/blob/v1.10.18/misc/scripts/obsensor_metadata_win10.ps1) `-op install_all`, pa izvuci i vrati kabl.
Bez ovoga VIO odbacuje frejmove (`Duplicate or unordered frame timestamps`).

### VIO uživo (color)

- `test_vio.bat`: poze kao JSON u konzoli.
- `start_vio.bat`: 3D prikaz + snimak u `recordings\<vreme>\`. Ako hoćeš samo prikaz, dodaj `--no_record`.

### Samo IR (snimak → Replay)

Dodatak uvek prati tačke na color kameri i ne nudi način da mu se pošalje druga slika. Zato `ir_record.py` snima ToF IR sliku, dubinu i IMU u istom formatu kao `sai-record-orbbec`, a `ir_replay.py` pušta snimak kroz VIO kao da je uživo.

1. `record_ir.bat`: pojavi se IR prozor. Prošetaj sa kamerom i vrati se na početno mesto, pa pritisni `q`.
   Opcije: `--resolution 640x576` (uži ugao, bez crnog kruga), `--ir_max 800` (svetlija slika), `--no_depth`.
2. `replay_ir.bat`: pušta najnoviji `recordings\ir_*` sa 3D prikazom. Na kraju ispisuje procenat vremena u praćenju, periode gubitka praćenja, dužinu putanje i rastojanje kraj–početak (drift).
   Snimak u boji se pušta za poređenje ovako: `replay_ir.bat recordings\<vreme>`.

`ir_record.py` koristi `imuToCamera` iz kalibracije nekog `sai-record-orbbec` snimka, jer je `camera1` tamo ista ToF kamera. Zato prvo jednom snimi par sekundi sa `start_vio.bat`.

Prvi rezultat (29. 10. 2026, ista soba, ~40 s hoda, sa svetlom): color radi 100 % vremena, a ToF IR 512×512 31 % vremena (izgubi praćenje posle 5,7 s i vrati ga posle 27 s).

### Problemi

- **Replay se zamrzne**: praćenje je izgubljeno. `ir_replay.py` tada nastavlja da crta poslednju dobru pozu, a periode gubitka ispisuje u konzoli.
- **`IMU buffer max size exceeded` stalno, bez poza**: kamera ne šalje frejmove. Ili kameru koristi drugi program, ili je ostala u lošem stanju posle nasilnog gašenja. Izvuci i vrati kabl.
- **`sai-record-orbbec` pukne posle 2–3 s**: `ffmpeg` nije u PATH-u.

## Šta je otvoreno, a šta zatvoreno

Ove skripte i `sai-record-orbbec` / `vio_jsonl` imaju otvoren kod. VIO/SLAM engine (`spectacularAI_orbbecPlugin.dll`, `spectacularAI_native*.pyd`, `libspectacularAI_depthaiPlugin.so`) je zatvoren i besplatan samo za nekomercijalnu upotrebu. Slanje sopstvenih frejmova uživo (Input API) traži komercijalnu licencu, pa je jedini besplatan ulaz Replay.
