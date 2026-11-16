"""HC-SR04 ultrasonic sensor on the Jetson Nano: driver + bench test (Python 3.6).

    python3 safety.py hcsr04                       # 150 readings, TRIG pin 31, ECHO pin 33
    python3 safety.py hcsr04 --trig 31 --echo 33   # other pins
    python3 safety.py hcsr04 --slow                # old Jetson.GPIO polling, for comparison

The obstacle-stop logic itself goes into the class notebook (v20); this file only
reads and validates the sensor. In the notebook: `from safety import HCSR04Source`.
"""
from __future__ import print_function

import argparse
import os
import time

SENSOR_HZ = 15.0     # one ping per ~66 ms; faster and late echoes read as near objects
MEDIAN_N = 3         # median of the last N readings removes single spikes


class HCSR04Source(object):
    """read() -> metres; float('inf') if nothing within max_m; None if no echo (fault).

    Distance = echo pulse width x 343 m/s / 2, so 1 cm = 58 us of echo. The pulse is
    timed by polling the ECHO pin, so the poll period is the resolution. Jetson.GPIO's
    input() takes ~700 us per call (9 cm spread); reading the pin's sysfs value file
    with os.pread takes ~44 us (0.6 cm spread after the median). Jetson.GPIO is only
    used to set the pins up and to send the trigger pulse."""
    SPEED = 343.0                                             # m/s at ~20 C

    def __init__(self, trig=31, echo=33, max_m=2.0):
        import Jetson.GPIO as GPIO
        self.G = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig, GPIO.OUT, initial=GPIO.LOW)
        before = self._exported()
        GPIO.setup(echo, GPIO.IN)
        new = self._exported() - before
        self.trig, self.echo = trig, echo
        self.max_s = 2.0 * max_m / self.SPEED
        self.fd, self.path = None, None
        if len(new) == 1:                                     # the sysfs gpio Jetson.GPIO just exported
            path = "/sys/class/gpio/%s/value" % new.pop()
            try:
                self.fd, self.path = os.open(path, os.O_RDONLY), path
            except OSError:
                self.fd = None
        self.last = None                                      # (wait for rise s, pulse s, polls) of the last ping
        time.sleep(0.05)

    @staticmethod
    def _exported():
        try:
            return set(n for n in os.listdir("/sys/class/gpio") if n.startswith("gpio") and "chip" not in n)
        except OSError:
            return set()

    def _echo(self):
        if self.fd is not None:
            return os.pread(self.fd, 1, 0) == b"1"
        return bool(self.G.input(self.echo))

    def read(self):
        G, t, echo = self.G, time.perf_counter, self._echo
        if echo():                                            # still high from the last ping
            return None
        G.output(self.trig, G.HIGH)
        time.sleep(0.00001)                                   # >= 10 us trigger pulse
        G.output(self.trig, G.LOW)
        t0 = t()
        while not echo():                                     # sensor sends its burst, then raises ECHO
            if t() - t0 > 0.02:
                return None                                   # no echo at all: wiring / power fault
        start = t()
        polls = 0
        while echo():
            polls += 1
            if t() - start > self.max_s:
                return float("inf")                           # nothing within max_m
        width = t() - start
        self.last = (start - t0, width, polls)
        if width > self.max_s * 0.95:
            return float("inf")
        return width * self.SPEED / 2.0

    def close(self):
        try:
            if self.fd is not None:
                os.close(self.fd)
            self.G.cleanup([self.trig, self.echo])
        except Exception:  # noqa: BLE001
            pass


def spread(v):
    """(median, p10-p90 spread) in cm."""
    v = sorted(v)
    return 100 * v[len(v) // 2], 100 * (v[int(0.9 * (len(v) - 1))] - v[int(0.1 * (len(v) - 1))])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sensor", nargs="?", default="hcsr04", choices=["hcsr04"])
    ap.add_argument("--trig", type=int, default=31)
    ap.add_argument("--echo", type=int, default=33)
    ap.add_argument("--n", type=int, default=150, help="number of readings")
    ap.add_argument("--slow", action="store_true", help="poll ECHO with Jetson.GPIO.input (old way, to compare)")
    a = ap.parse_args()
    src = HCSR04Source(a.trig, a.echo)
    if a.slow:
        src.fd = None
    print("ECHO read via %s" % ("sysfs fd " + src.path if src.fd is not None else "Jetson.GPIO.input (slow)"))
    vals, med, us, win, bad = [], [], [], [], 0
    try:
        for i in range(a.n):
            t0 = time.perf_counter()
            d = src.read()
            ms = 1000 * (time.perf_counter() - t0)
            if d is None:
                bad += 1
                print("%3d  no reading (fault)            %.1f ms" % (i, ms))
            elif d == float("inf"):
                print("%3d  nothing in range              %.1f ms" % (i, ms))
            else:
                vals.append(d)
                win = (win + [d])[-MEDIAN_N:]
                med.append(sorted(win)[len(win) // 2])
                extra = ""
                if src.last and src.last[2]:
                    us.append(1e6 * src.last[1] / src.last[2])
                    extra = " rise %.2f ms, %d polls, %.0f us/poll" % (1000 * src.last[0], src.last[2], us[-1])
                print("%3d  %6.1f cm  (median-%d %5.1f)  %.1f ms%s" % (i, 100 * d, MEDIAN_N, 100 * med[-1], ms, extra))
            time.sleep(max(0.0, 1.0 / SENSOR_HZ - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
    if vals:
        print("readings %d, faults %d, raw median %.1f cm spread (p10-p90) %.1f cm" % ((len(vals), bad) + spread(vals)))
        print("filtered (median-%d): median %.1f cm spread %.1f cm" % ((MEDIAN_N,) + spread(med)))
        if us:
            print("poll period: median %.0f us (1 cm = 58 us of echo)" % sorted(us)[len(us) // 2])


if __name__ == "__main__":
    main()
