#!/usr/bin/env python3
"""Geometry planning calculator for the IR receivers (inputs are ASSUMPTIONS, not measurements).

Robot data from the brief: receiver height ~101.6 mm (4 in), ball diameter 42 mm, proposed tilt
20-25 deg outward/downward, reported ~40 mm side gap. The receiver's vertical/horizontal half-angle
is NOT measured here: the TSOP datasheet could not be fetched; a search snippet mentioned a ~90 deg
detection angle (=> 45 deg half-angle) -- treat as unverified. Change --half and see the effect.
"""
import argparse, math

ap = argparse.ArgumentParser()
ap.add_argument("--height", type=float, default=101.6, help="receiver height above floor, mm")
ap.add_argument("--ball", type=float, default=42.0, help="ball diameter, mm")
ap.add_argument("--half", type=float, nargs="+", default=[30.0, 45.0, 60.0], help="assumed half-angle(s), deg")
ap.add_argument("--tilts", type=float, nargs="+", default=[0, 10, 20, 22, 25, 30, 45], help="downward tilt, deg")
ap.add_argument("--gap", type=float, default=40.0, help="reported side gap between rings, mm")
a = ap.parse_args()

r = a.ball / 2.0
dz = a.height - r            # vertical drop from receiver to ball centre
print(f"receiver height {a.height:.1f} mm, ball centre at {r:.1f} mm -> receiver is {dz:.1f} mm above the ball centre\n")

print("1) VERTICAL: depression angle from receiver to the ball centre = atan(dz/d)")
for d in (50, 100, 200, 400, 600):
    print(f"   d={d:4d} mm  depression {math.degrees(math.atan2(dz, d)):5.1f} deg")
print()

print("2) Nearest horizontal distance d at which the ball CENTRE is inside the vertical cone")
print("   (visible while |depression - tilt| <= half-angle; far end is limited by signal, not by geometry)")
print("   tilt    " + "  ".join(f"half={h:>4.0f}" for h in a.half) + "     boresight meets floor   boresight meets ball centre")
for t in a.tilts:
    row = []
    for h in a.half:
        lim = t + h                      # max depression still inside the cone
        if lim >= 90: row.append("   0 mm  ")
        else: row.append(f"{dz / math.tan(math.radians(lim)):6.0f} mm ")
    floor = "      (never)" if t == 0 else f"{a.height / math.tan(math.radians(t)):9.0f} mm"
    ball = "      (never)" if t == 0 else f"{dz / math.tan(math.radians(t)):9.0f} mm"
    print(f"   {t:4.0f}   " + "  ".join(row) + f"   {floor:>20}   {ball:>24}")
print()

print("3) SIDE GAP model (azimuth, two outward-facing receivers separated by s along the chassis side)")
print("   Cones overlap only beyond x = (s/2)/tan(half_h) from the chassis; a ball nearer than that, between")
print("   them, can be invisible. s here = the reported gap; a real value needs a measurement.")
for h in a.half:
    x = (a.gap / 2.0) / math.tan(math.radians(h))
    print(f"   half-angle {h:4.0f} deg: blind pocket reaches ~{x:5.1f} mm out from the chassis (s={a.gap:.0f} mm)")
print()

print("4) AZIMUTH coverage of N evenly spaced receivers (the firmware's placeholder layout)")
for h in a.half:
    n = 12
    gap = 360.0 / n - 2 * h
    print(f"   {n} receivers, half-angle {h:4.0f}: uncovered azimuth between neighbours = {max(0.0, gap):5.1f} deg;"
          f" every bearing seen by >= {max(1, int(2*h/(360.0/n)))} receivers")
print("\nNothing here proves coverage. Use ':learn <deg>' sweeps with the real ball (docs/CALIBRATION_AND_TEST_PLAN.md).")
