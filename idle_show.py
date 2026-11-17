#!/usr/bin/env python
#
# Idle show: the lamp keeps itself busy with poses, gestures and searching.
#
# Picks at random (never the same thing twice in a row) from:
#   - 12 poses: glide in facing a random direction, hold with gentle
#     breathing, drift the base partway through, move on
#   - the wide search (poses.searching)
#   - small gestures: hello nod, head tilt, happy sway
#   - an occasional nap: doze into sleepy for a while, then the slow wake-up
#
# Big jumps between distant poses go via the watching pose, and move time
# grows with distance. Rising out of sleepy always uses the slow lower-arm-
# last rise - fast rises tipped the real lamp over.
#
#   python idle_show.py             real lamp
#   python idle_show.py --sim       sim viewer only
#   python idle_show.py --mirror    real lamp, mirrored live in the sim
# Ctrl+C eases it back to sleepy and exits.
#

import random
import sys

from poses import WATCHING, SLEEPY, idle, move, pause, searching

BASE = WATCHING["a"]
STRAIGHT = WATCHING["e"]

# From pose_gallery.py (head tilts solved against the sim model there), stored
# as plain numbers so this runs without MuJoCo installed.
SHOW_POSES = {
    "alert":        {"c": 1286, "d": 1840, "f": 2773, "e": STRAIGHT, "a": BASE},
    "tall / proud": {"c": 1682, "d": 2771, "f": 2905, "e": 931, "a": BASE},
    "shy":          {"c": 1150, "d": 1250, "f": 2716, "e": 600, "a": 1813},
    "sad":          {"c": 1450, "d": 1150, "f": 2430, "e": STRAIGHT, "a": BASE},
    "peek left":    {"c": 1000, "d": 1500, "f": 2661, "e": 258, "a": 1768},
    "peek right":   {"c": 1000, "d": 1500, "f": 2661, "e": 1624, "a": 2314},
    "thinking":     {"c": 1100, "d": 1600, "f": 2490, "e": 1339, "a": BASE},
    "bow":          {"c": 1600, "d": 1300, "f": 2487, "e": STRAIGHT, "a": BASE},
    "reading lamp": {"c": 1500, "d": 1600, "f": 3114, "e": STRAIGHT, "a": BASE},
    "spotlight":    {"c": 1100, "d": 1700, "f": 2955, "e": STRAIGHT, "a": BASE},
    "stretch":      {"c": 1500, "d": 2500, "f": 2934, "e": STRAIGHT, "a": BASE},
    "tucked in":    {"c": 1050, "d": 950, "f": 2573, "e": STRAIGHT, "a": BASE},
}

VIA_WATCHING_OVER = 1000   # joint travel (counts) beyond which a transition goes via WATCHING


class IdleShow:
    def __init__(self, driver):
        self.d = driver
        self.current = dict(WATCHING)
        self.asleep = False

    # --- moving between poses -------------------------------------------

    @staticmethod
    def _travel(a, b):
        return max(abs(a[j] - b[j]) for j in ("c", "d", "f"))

    def _glide(self, target):
        t = min(5.0, 2.5 + self._travel(self.current, target) / 800.0)
        move(self.d, target, t, stagger={"f": 0.1, "a": 0.15})
        self.current = dict(target)

    def go_to(self, target):
        if self.asleep:
            self.wake()
        if self._travel(self.current, target) > VIA_WATCHING_OVER and self.current != WATCHING:
            self._glide(WATCHING)
        self._glide(target)

    # --- behaviors -------------------------------------------------------

    def hold_pose(self, name):
        """Take the pose facing a random direction (poses with their own base
        turn keep it, plus a little), hold, drift the base partway through as
        if shifting attention, hold some more."""
        print("pose: %s" % name, flush=True)
        pose = dict(SHOW_POSES[name])
        pose["a"] += random.choice([-1, 1]) * random.randint(150, 650)   # up to ~15 deg either way
        self.go_to(pose)
        idle(self.d, self.current, duration=random.uniform(1.5, 3.0), amplitude=10)
        drift = dict(self.current, a=self.current["a"] + random.choice([-1, 1]) * random.randint(150, 300))
        move(self.d, {"a": drift["a"]}, random.uniform(2.0, 3.0))
        self.current = drift
        idle(self.d, self.current, duration=random.uniform(1.5, 3.0), amplitude=10)

    def search(self):
        print("searching", flush=True)
        self.go_to(WATCHING)
        searching(self.d)
        self.current = dict(WATCHING)

    def nod_hello(self):
        print("nod hello", flush=True)
        self.go_to(WATCHING)
        # turn toward someone off to one side, nod at them, turn back
        move(self.d, {"a": BASE + random.choice([-1, 1]) * random.randint(300, 600)}, 1.5)
        f = WATCHING["f"]
        for df, dur in [(120, 0.35), (0, 0.35), (120, 0.35), (0, 0.45)]:
            move(self.d, {"f": f + df}, dur)
        pause(random.uniform(0.5, 1.0))
        move(self.d, {"a": BASE}, 1.5)

    def head_tilt(self):
        print("head tilt", flush=True)
        self.go_to(WATCHING)
        side = random.choice([-1, 1])
        # head and base turn the same way (base - turns right, head + turns right)
        move(self.d, {"e": STRAIGHT + side * 250, "f": WATCHING["f"] - 80, "a": BASE - side * 250}, 1.0,
             stagger={"a": 0.2})
        pause(random.uniform(0.8, 1.4))
        move(self.d, {"e": STRAIGHT, "f": WATCHING["f"], "a": BASE}, 1.0)

    def happy_sway(self):
        print("happy sway", flush=True)
        self.go_to(WATCHING)
        for side in (1, -1, 1, -1):
            move(self.d, {"a": BASE + side * 150, "e": STRAIGHT - side * 120}, 0.6)
        move(self.d, {"a": BASE, "e": STRAIGHT}, 0.6)

    def nap(self):
        print("nap", flush=True)
        self.doze()
        idle(self.d, SLEEPY, duration=random.uniform(8.0, 15.0), amplitude=12)
        self.wake()

    def doze(self):
        if self.current != WATCHING:
            self._glide(WATCHING)
        move(self.d, SLEEPY, 3.0, stagger={"c": 0.3, "d": 0.2, "a": 0.1})
        self.current = dict(SLEEPY)
        self.asleep = True

    def wake(self):
        # slow, lower arm (c) last - fast rises tipped the real lamp over backward
        move(self.d, WATCHING, 6.0, stagger={"c": 0.4, "f": 0.15, "e": 0.25})
        self.current = dict(WATCHING)
        self.asleep = False

    # --- the show --------------------------------------------------------

    def run(self, cycles=None):
        """Loop forever (or for `cycles` picks). Poses are picked most often;
        gestures, the search and naps are sprinkled in."""
        choices = ([("pose", n) for n in SHOW_POSES]
                   + [("search", None)] * 2
                   + [("nod_hello", None), ("head_tilt", None), ("happy_sway", None)]
                   + [("nap", None)])
        last = None
        n = 0
        while cycles is None or n < cycles:
            pick = random.choice([c for c in choices if c != last])
            last = pick
            kind, name = pick
            if kind == "pose":
                self.hold_pose(name)
            else:
                getattr(self, kind)()
            n += 1


def main():
    from driver import make_driver
    driver = make_driver(sys.argv)

    show = IdleShow(driver)
    try:
        show.wake()   # slow start - the lamp may be sitting in sleepy
        show.run()
    except KeyboardInterrupt:
        print("stopping - dozing off", flush=True)
        show.doze()
    finally:
        driver.close()


if __name__ == "__main__":
    main()
