# UQ Open Day 2025 — Webots simulation

Archived Webots simulation scenarios and controllers used for the University of Queensland Open Day demonstration. The principal world is [`worlds/UqOpenDay.wbt`](worlds/UqOpenDay.wbt). This repository is a **preservation baseline** pending a fresh-machine reproducibility test.

## Known working environment (reference UQ computer)

- Windows; Webots **R2019b revision 1** (`C:\Program Files\Webots`)
- Python **3.7.9** (`C:\Program Files\Python37\python.exe`)
- NumPy **1.21.6**, `playsound` **1.3.0**, `playsound2` **1.3.0**
- Python dependencies are pinned in [`requirements.txt`](requirements.txt). This records observed versions, **not** an independently tested fresh installation.
- Webots supplies `controller` and `vehicle`; do **not** install unrelated PyPI packages with those names. `controllers/ContrlPara.py` is local project code.

## Setup on a Windows computer

1. Install **Webots R2019b revision 1** from the official Cyberbotics release archive. Verify the installed version in `C:\Program Files\Webots\resources\version.txt`. The precise installer URL and compatibility on a new machine have not yet been verified.
2. Install **Python 3.7.9**. Python 3.7 is end-of-life; use it in an isolated, trusted environment for this legacy demo.
3. Clone this repository (`git clone <REPOSITORY-URL>`) and enter its folder.
4. Run `py -3.7 -m pip install -r requirements.txt` in PowerShell. If the `py` launcher is unavailable, use the full Python 3.7 executable path.
5. Open Webots and select **File → Open World**, then open `worlds/UqOpenDay.wbt`. Start the simulation and inspect Webots console messages.
6. Verify that the participant vehicle, surrounding vehicles, traffic lights and audio behave as expected. **This clean-machine verification is still outstanding.**

## Active controllers referenced by the world

`auto_backup`, `auto_surrounding_merge`, `auto_stop_and_go`, `auto-ringroad-driver`, `auto_broken`, and `traffic_light_red`. The traffic-light controller is written in C; its source and bundled Windows executable are retained.

## Controller dependencies and configuration

- `controllers/ContrlPara.py` is a local module.
- The Python controllers import Webots `controller` and `vehicle` modules; Webots may configure these paths when launching controllers. A direct `python -c "import controller"` from PowerShell failed on the reference computer, which does not establish whether launching from Webots works.
- Several controllers read `Controller_input_param.csv`. **Do not delete or globally ignore CSV files** without confirming whether they are input data.
- Some controllers use `playsound`/`playsound2` and bundled audio files.
- Controller-specific Python interpreter configuration in Webots, driving-wheel integration, and the exact startup/interaction sequence require confirmation on the reference machine.

## Preservation and cleanup

This initial archive preserves the original folder layout, source, worlds, assets, controller CSVs, and bundled executables to reduce the risk of breaking relative paths. Only Python bytecode/cache, `.cache` files, and `.bak` backups were removed from this copy. The original ZIP is untouched. Large recorded CSVs, historical worlds and other potential surplus files remain **pending a dependency audit**. Do not assume they are safe to remove.

## Known limitations / next verification

1. Confirm the demo runs from this repository rather than an external directory.
2. Confirm Webots' configured Python interpreter and API path.
3. Check controller CSV reads/writes, and distinguish runtime inputs from generated telemetry.
4. Confirm peripheral requirements (e.g. racing wheel), audio and any compiled-controller requirements.
5. Verify installation and execution on a second Windows computer.

## Ownership and archival note

Initial publication should be private pending review of participant information, personal data, proprietary assets and third-party licences. The source archive contains a `Participants_info` directory; review and exclude personal information before publishing or sharing access.
