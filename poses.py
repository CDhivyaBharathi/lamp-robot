#
# Motion building blocks shared by everything that moves the lamp.
#
#   move(driver, target, duration, stagger)   smooth, jolt-free move to a pose
#   idle(driver, pose, duration, amplitude)   gentle "breathing" around a pose
#   pause(seconds)                            a sleep that can be interrupted
#   searching(driver)                         wide look-around at three heights
#
# A pose is a dict of joint name -> raw servo position, e.g.
#   {"a": 2041, "c": 927, "d": 1411, "f": 2703, "e": 941}
# Any subset of joints works; the rest stay where they are.
#
# Lesson learned on the real lamp: fast moves tip it over (fast backward
# lower-arm swings, sudden jumps, spinning the base far). Keep moves slow and
# smooth, and keep the base within one motor turn.
#

import math
import random
import time

WATCHING = {"a": 2041, "c": 927, "d": 1411, "f": 2703, "e": 941}   # the neutral, awake pose
SLEEPY = {"a": 2041, "c": 1753, "d": 913, "f": 2903, "e": 894}


# --- interrupts: lets a caller cut any motion short (e.g. a future sensor) ---

class Interrupted(Exception):
    """Raised mid-motion when the interrupt check fires."""


_should_stop = None


def set_interrupt(fn):
    """Every motion step calls fn(); if it returns True the motion stops by
    raising Interrupted. Pass None to make motions uninterruptible again."""
    global _should_stop
    _should_stop = fn


def _check():
    if _should_stop is not None and _should_stop():
        raise Interrupted()


def pause(seconds):
    """time.sleep that can be interrupted."""
    end = time.time() + seconds
    while True:
        _check()
        left = end - time.time()
        if left <= 0:
            return
        time.sleep(min(0.05, left))


# --- moving ------------------------------------------------------------------

def ease_in_out(t):
    """Smootherstep: zero velocity AND zero acceleration at both ends, so a
    motion starts and stops without any jolt - jolts are what tipped the lamp."""
    return t * t * t * (t * (6 * t - 15) + 10)


MOVE_DT = 0.025   # seconds between updates during a move


def move(driver, target, duration=1.0, stagger=None):
    """Eased move to `target`. stagger maps joint -> fraction of the duration
    to wait before that joint starts, so the chain can move like a body
    (shoulder leads, head follows) instead of all at once.

    Each update is sent with just enough servo speed to arrive as the next
    one is sent, so joints flow continuously (sending each small step at full
    speed made them sprint-and-stop 40 times a second). Timing follows the
    wall clock, so bus delays don't stretch the move."""
    stagger = stagger or {}
    start = {j: driver.get_pos(j) for j in target}
    last = dict(start)
    t0 = time.time()
    while True:
        _check()
        t = min(1.0, (time.time() - t0) / duration) if duration > 0 else 1.0
        for j, goal in target.items():
            if start[j] is None:
                continue
            lag = stagger.get(j, 0.0)
            tj = min(1.0, max(0.0, (t - lag) / (1.0 - lag)))
            pos = int(round(start[j] + (goal - start[j]) * ease_in_out(tj)))
            speed = max(40, int(abs(pos - last[j]) / MOVE_DT * 1.5))   # never 0 - that means "max speed"
            driver.set_pos(j, pos, speed=speed, acc=0)
            last[j] = pos
        if t >= 1.0:
            return
        time.sleep(MOVE_DT)


def idle(driver, base_pose, duration=10.0, amplitude=40, update_dt=0.05):
    """Subtle continuous drift around a pose so the lamp never looks frozen.
    Each joint drifts on its own slow sine wave (random period and phase), so
    it reads as natural micro-movement rather than mechanical breathing.
    duration=None runs until interrupted."""
    joints = list(base_pose.keys())
    periods = {j: random.uniform(3.0, 6.0) for j in joints}
    phases = {j: random.uniform(0, 2 * math.pi) for j in joints}

    start = time.time()
    while duration is None or time.time() - start < duration:
        _check()
        t = time.time() - start
        for j in joints:
            offset = amplitude * math.sin(2 * math.pi * t / periods[j] + phases[j])
            driver.set_pos(j, int(base_pose[j] + offset))
        time.sleep(update_dt)


# --- routines ----------------------------------------------------------------

def searching(driver):
    """Hunting all over: sweep side to side at three heights - across the
    table, at eye level, then up high - zig-zagging between them. Head and
    base turn the same way at each end (base raw down + head raw up = both
    look right), covering ~105 degrees each side while the base stays inside
    one motor turn. Head-tilt values keep the intended angles for each body
    shape (solved against the sim model with pose_gallery.py)."""
    BASE = WATCHING["a"]
    STRAIGHT = WATCHING["e"]
    DOWN = {"c": 1300, "d": 1500, "f": 2987}   # looking at the table
    MID = {"c": 1100, "d": 1600, "f": 2661}    # eye level
    UP = {"c": 1100, "d": 1700, "f": 2534}     # looking up
    RIGHT = {"e": 1750, "a": BASE - 1500}
    LEFT = {"e": 150, "a": BASE + 1500}

    move(driver, {**DOWN, "e": STRAIGHT, "a": BASE}, 2.0, stagger={"f": 0.2})
    move(driver, RIGHT, 2.5, stagger={"a": 0.15})
    pause(random.uniform(0.4, 0.8))

    move(driver, LEFT, 5.5, stagger={"a": 0.1})          # across the table
    pause(random.uniform(0.5, 0.9))

    move(driver, MID, 1.8, stagger={"f": 0.2})
    move(driver, RIGHT, 5.5, stagger={"a": 0.1})         # across at eye level
    pause(random.uniform(0.5, 0.9))

    move(driver, UP, 1.8, stagger={"f": 0.2})
    move(driver, LEFT, 5.5, stagger={"a": 0.1})          # across up high
    pause(random.uniform(0.8, 1.2))

    move(driver, WATCHING, 3.0, stagger={"a": 0.15})
