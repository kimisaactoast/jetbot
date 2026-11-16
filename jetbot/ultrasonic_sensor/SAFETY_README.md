# Ultrasonic sensor (HC-SR04) on the Jetson Nano: wiring and validation

`safety.py` reads the sensor and runs the bench test. The obstacle-stop logic goes into the v20 notebook.

## Wiring (Waveshare JETSON-IO-BASE-A, 40-pin header, BOARD numbers)

| Sensor | Nano pin |
|---|---|
| VCC | 3.3 V (pin 17 or 1) |
| GND | GND (pin 30 or 34) |
| TRIG | 31 |
| ECHO | 33, wired directly |

The sensor is a 5 V HC-SR04 but runs on 3.3 V (tested 2026-11-16). On 3.3 V, ECHO can't rise above 3.3 V, so it connects straight to pin 33.
**If you ever power it from 5 V, ECHO must go through a divider** (ECHO → 220 Ω → pin 33 → 220 Ω + 220 Ω → GND, giving 3.33 V), otherwise 5 V reaches a 3.3 V pin.

## Setup (once)

```bash
sudo groupadd -f -r gpio && sudo usermod -aG gpio $USER    # then log out and back in
```

## Voltage checks (multimeter on DC volts, black probe on GND pin 30)

| Check | How | Expect |
|---|---|---|
| 3.3 V supply | red probe on the pin feeding the sensor's VCC | 3.2 to 3.4 V |
| TRIG pin works | run the line below, red probe on pin 31 | about 3.3 V for 10 s, then 0 V |
| ECHO is safe | red probe on the ECHO wire while the bench test runs | never above 3.3 V |

```bash
python3 -c "import Jetson.GPIO as G,time; G.setmode(G.BOARD); G.setup(31,G.OUT); G.output(31,1); time.sleep(10); G.cleanup()"
```

Result (2026-11-16): pin 31 read 3.2 V. The sensor on 3.3 V was checked with the meter and is safe for pin 33.

## Bench test

```bash
python3 safety.py hcsr04          # 150 readings, about 10 s
python3 safety.py hcsr04 --slow   # same, with the old slow pin read, for comparison
```

Hold a flat object at a measured distance. Each line shows the raw distance, the median of the last 3 readings, and the timing. The last lines summarise the run.

**Pass:** the first line says `ECHO read via sysfs fd ...`, faults are 0 to 2, the filtered spread is 1 cm or less, and the median is within 1 cm of the ruler.

## Results (2026-11-16, phone 14.7 cm tall standing upright)

| Pin read | µs per poll | raw spread (p10 to p90) | after median-3 | median |
|---|---|---|---|---|
| Jetson.GPIO.input (`--slow`) | 723 | 9.0 cm | 5.0 cm | 14.0 cm |
| sysfs file (default) | 44 | 2.1 cm | **0.6 cm** | **14.4 cm** |

The sensor is fine. The first run's 9 cm spread came from the slow pin read: 1 cm of distance is only 58 µs of echo, so a 723 µs poll misses whole centimetres. Reading the pin's sysfs file directly fixes this. The few spikes left over come from Linux pausing the thread mid-measurement, and the median of 3 removes them.

## Troubleshooting

| Symptom | Cause |
|---|---|
| every line `no reading (fault)` | no echo reaching pin 33: check VCC, GND and the ECHO wire |
| first line says `Jetson.GPIO.input (slow)` | the pin was left exported by a crashed run: reboot, or `echo 38 \| sudo tee /sys/class/gpio/unexport` |
| readings jump in steps of about 2 cm | slow pin read (see above) |
| always `nothing in range` | sensor pointing at the ceiling, or the target is too small or soft |

## Using it in the notebook

```python
from safety import HCSR04Source
sensor = HCSR04Source(trig=31, echo=33)
d = sensor.read()        # metres, float('inf') = nothing in range, None = no echo
```
Ping at most 15 times a second, and take the median of the last 3 readings.
