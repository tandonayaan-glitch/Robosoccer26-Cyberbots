#!/usr/bin/env python3
"""One-line summary of the host benchmark (SIMULATION) for a parameter set.

  tools/bench_summary.py [--bin PATH] [--imm] [name=value ...]

Runs the benchmark suites acq, loss, interf, soak, miss, drop and dir with the given firmware parameters
(any ':set' name, e.g. engine=0 losthold=500 acqspan1=140) and prints one line:
  acquisition 1/2/3 receivers (median/p95 ms) | strong-track loss | weak-track loss (30 %/50 % misses) |
  burst duration at which half of the phases are accepted (1 receiver / 2 neighbours / 3 neighbours) |
  mixed soak: share of eligible bursts accepted, phantom tracking s/min, false losses/min |
  false losses/min under 10/30/50/70 % per-visit misses (1 receiver | 2 receivers) |
  longest dropout survived by a steady / a flickering track | direction updates | translation <= 10 % after a loss
With --imm the deterministic burst-acceptance profile (suite imm) is added: expected acceptance of a burst whose
duration is uniform in 20..250 ms. All figures are simulated, see docs/LATENCY_OPTIMIZATION.md.
"""
import os, re, subprocess, sys

argv = sys.argv[1:]
BIN = "/tmp/rs26_bench_all"
imm = False
args = []
i = 0
while i < len(argv):
    if argv[i] == "--bin":
        BIN = argv[i + 1]; i += 2
    elif argv[i] == "--imm":
        imm = True; i += 1
    else:
        args.append(argv[i]); i += 1
BIN = os.environ.get("RS_BIN", BIN)


def run(suite, extra=()):
    return subprocess.run([BIN, "--bench", "suite=" + suite] + list(extra) + args, capture_output=True, text=True).stdout


out = "".join(run(s) + "\n" for s in ("acq", "loss", "interf", "soak", "miss", "drop", "dir"))


def block(title, n=1500):
    k = out.find(title)
    return out[k:k + n] if k >= 0 else ""


def med_p95(blk, label):
    m = re.search(re.escape(label) + r"\s+n=\s*\d+\s+median=\s*([\d.]+)\s+p95=\s*([\d.]+)", blk)
    return "%4.0f/%4.0f" % (float(m.group(1)), float(m.group(2))) if m else "  n/a   "


res = []
for k in ("1 receiver", "2 receivers", "3 receivers"):
    res.append("acq%s %s" % (k[0], med_p95(block("ACQUISITION, ball appears, " + k), "ball confirmed (valid)")))
res.append("loss " + med_p95(block("LOSS after a STRONG track, ball disappears, 1 receiver"), "ball declared lost"))
for w in ("30", "50"):
    m = re.search(r"WEAK track %s %% misses, 1 rx: declared lost\s+n=\s*\d+\s+median=\s*([\d.]+)\s+p95=\s*([\d.]+)" % w, out)
    res.append("weak%s %s" % (w, ("%4.0f/%4.0f" % (float(m.group(1)), float(m.group(2)))) if m else "n/a"))

thr = []
for shape in ("1 receiver ", "2 neighbours", "3 neighbours"):
    m = re.search(r"\n  " + shape + r"\s+\|(.*)\n", out)
    cells = re.findall(r"(\d+) ms:\s*(\d+)/48", m.group(1)) if m else []
    thr.append(str(next((int(d) for d, c in cells if int(c) >= 24), "-")))
res.append("burst50 " + "/".join(thr))
if imm:
    e = re.findall(r"E\[accept \| U\(20,250\)\] = ([\d.]+)", run("imm"))
    res.append("E[accept] " + "/".join(e))

acc = re.findall(r"eligible (\d+), accepted (\d+)", out)
ph = re.findall(r"phantom tracking ([\d.]+) s/min", out)
fl = re.findall(r"false losses (\d+) ", out)
el = sum(int(a) for a, b in acc); ac = sum(int(b) for a, b in acc)
res.append("soak: bursts accepted %.0f%% phantom %.1f s/min FL %.2f/min" % (100.0 * ac / max(el, 1), sum(map(float, ph)) / max(len(ph), 1), sum(map(int, fl)) / 30.0))
mi = re.findall(r"miss\s*(\d+)%:\s*([\d.]+)/min", out)
res.append("FL/min 1rx " + "/".join(b for a, b in mi[:4]) + " 2rx " + "/".join(b for a, b in mi[4:8]))

def survived(label):
    m = re.search(label + r"\s+\|(.*)\n", out)
    if not m:
        return 0
    cells = re.findall(r"\|\s*(\d+):\s*(\d+)/", m.group(0))
    return max([int(g) for g, c in cells if c == "0"] + [0])

res.append("survives %d/%d ms" % (survived("strong, 1 receiver"), survived("weak 50 % misses, 1 rx")))


def pair(label):
    m = re.search(re.escape(label) + r"\s+n=\s*\d+\s+median=\s*([\d.]+)\s+p95=\s*([\d.]+)", out)
    return "%3.0f/%3.0f" % (float(m.group(1)), float(m.group(2))) if m else "n/a"

res.append("dir adj " + pair("adjacent sector 90 -> 120 estimate") + " opp " + pair("opposite sector 90 -> 270 estimate") + " seam " + pair("rear seam 150 -> 210 estimate"))
m = re.search(r"LOSS after a STRONG track, ball disappears, 1 receiver.*?translation PWM <= 10 % of steady\s+n=\s*\d+\s+median=\s*([\d.]+)\s+p95=\s*([\d.]+)", out, re.S)
res.append("stop<=10%% %s" % (("%3.0f/%3.0f" % (float(m.group(1)), float(m.group(2)))) if m else "n/a"))
print(" ".join(args) or "(defaults)", "::", " | ".join(res))
