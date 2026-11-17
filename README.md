# Lamp Robot

A 5-servo robotic desk lamp (FEETECH ST3215 serial bus servos) that keeps itself
busy with an **idle show** — expressive poses, gestures, a wide search and naps —
plus a **MuJoCo simulation** of the lamp that can preview anything before it runs,
mirror the real lamp live, or drive the real lamp from sliders.

---

## Quick start

```bash
# 1. set up (once)
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# 2. plug in the servo board (USB) and switch on the servo power supply, then:
python scan_ids.py                 # all five ids 0-4 should respond

# 3. run the show
python idle_show.py                # Ctrl+C to stop - it eases back to sleep first
```

> **macOS:** anything that opens the MuJoCo viewer must be started with
> **`mjpython`** instead of `python` (it comes with the `mujoco` package).
> That's every command with `--sim` / `--mirror`, plus `live_mirror.py`,
> `sim_to_real.py` and `pose_gallery.py` without `--list`/`--real`.
> Plain real-lamp runs (`python idle_show.py`, `scan_ids.py`, `pose_jog.py`) use normal `python`.

---

## What's in here

| File | What it does |
|---|---|
| `idle_show.py` | **The main program.** The lamp keeps itself busy: 12 poses, gestures, the wide search, occasional naps — with base movement throughout. |
| `pose_gallery.py` | **Sim → real pose design.** Poses described as body shape + head angle, solved into raw servo values against the sim, previewed in the sim, then played on the lamp. |
| `sim_to_real.py` | **Sim → real, live.** Drag joint sliders in the MuJoCo viewer; the real lamp follows. |
| `live_mirror.py` | **Real → sim, live.** The sim shows whatever pose the real lamp is in. |
| `pose_jog.py` | Keyboard control of every joint on the real lamp, for posing it by hand. |
| `scan_ids.py` | Health check — lists which servos answer on the bus. |
| `poses.py` | Motion building blocks: smooth `move()`, breathing `idle()`, `searching()`, the watching/sleepy poses. |
| `driver.py` | Talks to the real servos; `make_driver()` picks real / `--sim` / `--mirror`. |
| `sim_viewer_driver.py` | The sim-viewer version of the driver, and the real+sim `MirroredDriver`. |
| `joint_angles.py` | Raw servo position ↔ sim joint angle conversion (the calibration). |
| `joint_config.py` | Servo ids, calibrated joint ranges, gear ratio. |
| `local_config.py` | Finds the servo board's serial port automatically; points at the sim model. |
| `scservo_sdk/` | FEETECH servo SDK (vendored). |
| `sim/` | MuJoCo model of the lamp (`scene.xml` → `robot.xml` + meshes). |

---

## Running things

Every motion script takes the same flags:

| Flag | Runs on |
|---|---|
| *(none)* | the real lamp |
| `--sim` | the simulator only — safe for trying things out |
| `--mirror` | the real lamp, with the same motion shown live in the sim |

### Idle show
```bash
python idle_show.py             # real lamp
mjpython idle_show.py --sim     # simulator only        (Windows: python)
mjpython idle_show.py --mirror  # real lamp + simulator (Windows: python)
```
Ctrl+C eases the lamp back into its sleepy pose before exiting.

For long runs on a Mac, stop it from sleeping (or the lamp freezes mid-pose):
```bash
caffeinate -dims python idle_show.py
```

### Pose gallery (sim → real)
```bash
python pose_gallery.py --list           # every pose's raw servo values
mjpython pose_gallery.py                # play all poses in the sim
mjpython pose_gallery.py 5 11           # just poses 5 and 11
python pose_gallery.py --real 5 11      # play them slowly on the real lamp
mjpython pose_gallery.py --mirror 5 11  # real lamp + sim together
```
To add a pose: add a row to `GALLERY` in `pose_gallery.py` (lower arm, upper arm,
head up/down degrees, head left/right degrees, base degrees), preview it in the
sim, try it with `--real`, then copy its printed raw values into `SHOW_POSES` in
`idle_show.py`.

### Drive the lamp from the sim (sim → real)
```bash
mjpython sim_to_real.py
```
Open the viewer's right-hand **Control** panel and drag the `a c d f e` sliders.
The lamp follows at a capped speed; raw values print once a second so you can
save a pose you like.

### Watch the real lamp in the sim (real → sim)
```bash
mjpython live_mirror.py
```
Shows the real lamp's current pose. Only one program can use the servo port at a
time, so to watch the lamp *while it moves*, run the moving script with `--mirror`.

### Pose the lamp by hand
```bash
python pose_jog.py              # needs a real terminal
```
`1`–`5` pick a joint, arrows move it, `+`/`-` change step size, `p` prints the
pose, `ESC` quits (the lamp holds its pose).

---

## The lamp

| Joint | Servo id | Part | Raw range | Raw value going up = |
|---|---|---|---|---|
| `a` | 0 | base (geared 1:4) | kept within 300–3800 | — |
| `c` | 1 | lower arm | 583–2196 | leans forward |
| `d` | 2 | upper arm | 913–3048 | swings back |
| `f` | 3 | head tilt | 1688–3628 | head tilts down |
| `e` | 4 | head turn | 27–1842 | head turns right |

- Raw positions are servo encoder counts: 4096 per full servo turn.
- Servos are daisy-chained on one serial bus at 1,000,000 baud, all powered from the
  servo supply (not USB).
- The sim's joint `b` is the base output; it follows `a` through the 1:4 gear.

---

## Safety — read before changing motions

The lamp has tipped over several times while this was developed. **Every fall came
from a fast move, not from a particular pose.** The current code avoids them by:

- **Smooth motion only** — `poses.move()` eases in and out with no jolt. Don't send
  big position jumps straight to the servos.
- **Slow rises** — getting up out of the sleepy pose takes 6 s, with the lower arm
  moving last. Fast backward swings of the lower arm tipped it over backward.
- **Base stays within one motor turn** — big base spins (via the servo's continuous
  "wheel" mode) tipped it over, and a script killed mid-spin leaves the motor
  spinning. Nothing here uses wheel mode.
- **Big pose changes go via the watching pose**, with more time for bigger moves.

Also: **weigh down or clamp the base** for demos. Keep a hand nearby the first time
any new motion runs on the real lamp.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `WARNING: servo board not found by USB id` | USB cable unplugged, or the board shows up differently. Run `ls /dev/cu.*` (Mac) and `export LAMP_SERVO_PORT=/dev/cu.<the board>`. |
| `could not open port ... Access is denied` / `Resource busy` | Another program is using the servo port — only one at a time. Close it. |
| `No servos responded` / `There is no status packet` | Servo power supply off, or a loose daisy-chain connector. Check, then `scan_ids.py`. |
| Mac doesn't list the board at all | The servo board uses a WCH **CH343** USB chip; recent macOS usually supports it out of the box, otherwise install WCH's CH34x/CH343 driver for macOS. |
| `launch_passive requires that the Python script be run under mjpython` | macOS — use `mjpython` instead of `python` for viewer scripts. |
| Segmentation fault right after the viewer window closes | Harmless — MuJoCo occasionally crashes on shutdown after everything has finished. |
| Lamp holds a weird pose after a crash | Normal — servos hold their last position. Run `idle_show.py` and Ctrl+C it, or re-pose with `pose_jog.py`. |
| Sim doesn't match the real lamp | Recalibrate with `live_mirror.py`: neutral pose → read raw values → update `ZERO_RAW` in `joint_angles.py`; flip `SIGN` for a joint that moves backwards. |
