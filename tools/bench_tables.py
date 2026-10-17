#!/usr/bin/env python3
"""Turn raw benchmark outputs (docs/bench/v3_*_seed*.txt) into the markdown tables of docs/LATENCY_OPTIMIZATION.md.

  tools/bench_tables.py GROUP1_FILE... -- GROUP2_FILE... [-- GROUP3_FILE...] [--names A,B,C]

Each group lists one or more seed files of one configuration; a cell shows the first file's value and, in brackets, the range
of the same figure over all files of the group. Everything is simulated.
"""
import re, sys

args = sys.argv[1:]
names = None
if "--names" in args:
    k = args.index("--names"); names = args[k + 1].split(","); args = args[:k]
groups, cur = [], []
for a in args:
    if a == "--":
        groups.append(cur); cur = []
    else:
        cur.append(a)
groups.append(cur)
if names is None:
    names = ["group %d" % (i + 1) for i in range(len(groups))]


def load(paths):
    return [open(p).read() for p in paths]


def dist(txt, section, label, field):
    """median/p95/p99/max of one 'name n= ... median= ... p95= ... p99= ... max=' line inside `section`."""
    i = txt.find(section)
    if i < 0:
        return None
    blk = txt[i:i + 4000]
    m = re.search(re.escape(label) + r"\s+n=\s*(\d+)\s+median=\s*([\d.]+)\s+p95=\s*([\d.]+)\s+p99=\s*([\d.\-]+)\s+max=\s*([\d.]+)", blk)
    if not m:
        return None
    n, med, p95, p99, mx = m.groups()
    return {"n": int(n), "median": float(med), "p95": float(p95), "p99": None if p99 == "-" else float(p99), "max": float(mx)}[field]


def cell(texts, section, label, field):
    v = [dist(t, section, label, field) for t in texts]
    v = [x for x in v if x is not None]
    if not v:
        return "n/a"
    s = "%.0f" % v[0]
    if len(v) > 1 and (max(v) - min(v) >= 1.0):
        s += " [%.0f-%.0f]" % (min(v), max(v))
    return s


def row(name, section, label):
    out = "| %s |" % name
    for i in range(len(groups)):
        t = TEXTS[i]
        out += " %s / %s / %s / %s |" % (cell(t, section, label, "median"), cell(t, section, label, "p95"),
                                          cell(t, section, label, "p99"), cell(t, section, label, "max"))
    return out


TEXTS = [load(g) for g in groups]
def header(first):
    print("| %s |%s" % (first, "".join(" %s: median / p95 / p99 / max |" % n for n in names)))
    print("|---|%s" % ("---|" * len(groups)))
header("Stage (ms from the event)")
for nrx, title in ((1, "1 receiver"), (2, "2 receivers"), (3, "3 receivers")):
    sec = "ACQUISITION, ball appears, " + title
    for label, name in (("first window closed on a ball receiver", "first window closed on a ball receiver"),
                        ("first candidate evidence (DETECT window)", "first candidate evidence"),
                        ("ball confirmed (valid)", "**ball confirmed**"),
                        ("navigation command (translation)", "navigation command"),
                        ("wheels reach start PWM (minpwm)", "wheels reach start PWM")):
        print(row("%s, %s" % (name, title), sec, label))
for nrx, title in ((1, "1 receiver"), (2, "2 receivers"), (3, "3 receivers")):
    sec = "LOSS after a STRONG track, ball disappears, " + title
    for label, name in (("ball declared lost", "**ball declared lost**"),
                        ("translation request cleared", "translation request cleared"),
                        ("translation PWM first < 90 % of steady", "translation PWM first < 90 %"),
                        ("translation PWM <= 10 % of steady", "translation PWM <= 10 %")):
        print(row("%s, strong track, %s" % (name, title), sec, label))
print()
header("Weak-track loss (ms after removal)")
for pm in (30, 50, 70):
    for rx in (1, 2):
        print(row("declared lost, %d %% misses, %d rx" % (pm, rx), "LOSS after a STRONG", "WEAK track %d %% misses, %d rx: declared lost" % (pm, rx)))
print()
header("Direction update (ms after the jump)")
for name in ("adjacent sector 90 -> 120", "adjacent sector 90 -> 60", "opposite sector 90 -> 270", "front seam 330 -> 0", "front seam 0 -> 330",
             "front seam 330 -> 30", "rear seam 150 -> 180", "rear seam 180 -> 210", "rear seam 150 -> 210"):
    print(row(name, "DIRECTION UPDATE", name + " estimate"))
