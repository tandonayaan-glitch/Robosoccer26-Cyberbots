#!/usr/bin/env python3
"""Default vs fast profile over the benchmark seeds (SIMULATION): tools/bench_compare.py A_FILES... -- B_FILES..."""
import re, sys
a = sys.argv[1:sys.argv.index("--")]; b = sys.argv[sys.argv.index("--") + 1:]
T = lambda fs: [open(f).read() for f in fs]
def sect(t, s, e):
    i = t.find(s); j = t.find(e, i + 1) if e else len(t); return t[i:j] if i >= 0 else ""
def p95(t, sec, label):
    m = re.search(re.escape(label) + r"\s+n=\s*\d+\s+median=\s*([\d.]+)\s+p95=\s*([\d.]+)", sect(t, sec, None)[:3500]); return float(m.group(2)), float(m.group(1))
def eacc(t, shape):
    m = re.search(r"\n  " + shape + r"\s+\|(.*)\n", sect(t, "FALSE ACQUISITION by ball-like", "INTERFERENCE WHILE"))
    c = [(int(d), int(n) / 48.0) for d, n in re.findall(r"(\d+) ms:\s*(\d+)/48", m.group(1)) if 20 <= int(d) <= 250]
    return sum((c[i + 1][0] - c[i][0]) * (c[i][1] + c[i + 1][1]) / 2 for i in range(len(c) - 1)) / (c[-1][0] - c[0][0])
def soak(t):
    acc = re.findall(r"eligible (\d+), accepted (\d+)", t); ph = re.findall(r"phantom tracking ([\d.]+)", t); fl = re.findall(r"false losses (\d+) ", t)
    el = sum(int(x) for x, y in acc); ac = sum(int(y) for x, y in acc)
    return 100.0 * ac / el, sum(map(float, ph)) / len(ph), sum(map(int, fl)) / 3.0, sum(int(x) for x in re.findall(r"false acquisitions (\d+)", t)) / 3.0 / 10.0
def drop(t, kind):
    m = re.search(r"\n  " + re.escape(kind) + r"\s+\|(.*)\n", sect(t, "DROPOUT of a tracked", "FALSE LOSS"))
    cells = re.findall(r"(\d+):\s*(\d+)/\s*\d+/(\S+)", m.group(1)); ok = max([int(g) for g, l, r in cells if l == "0"] + [0])
    re_ = [int(r) for g, l, r in cells if r != "-" and int(g) >= 200]
    return ok, (sum(re_) / len(re_) if re_ else None)
def brief(t, rx, ms):
    m = re.search(r"\n  %d receivers?\s*:\s*\|(.*)\n" % rx, sect(t, "BRIEF APPEARANCE", "APPROACH SPEED"))
    d = dict((int(k), (int(n), v)) for k, n, v in re.findall(r"\|?\s*(\d+) ms:\s*(\d+)/48 lost@(\S+)", m.group(1))); return d[ms]
def miss(t):
    return max(float(x) for x in re.findall(r"([\d.]+)/min", sect(t, "FALSE LOSS under independent", "FALSE ACQ")))
rows = []
for name, fs in (("default", a), ("fast", b)):
    ts = T(fs); r = {}
    r["acq p95 1/2/3 rx (ms, per seed)"] = " ; ".join("%.0f/%.0f/%.0f" % tuple(p95(t, "ACQUISITION, ball appears, " + k, "ball confirmed (valid)")[0] for k in ("1 receiver", "2 receivers", "3 receivers")) for t in ts)
    r["E[accept] 1rx/2nb/3nb (tables, per seed)"] = " ; ".join("%.2f/%.2f/%.2f" % (eacc(t, "1 receiver "), eacc(t, "2 neighbours"), eacc(t, "3 neighbours")) for t in ts)
    r["ring bursts accepted (all durations)"] = str(sum(int(x) for t in ts for x in re.findall(r"ring (?:coherent|random)\s+\|(.*)\n", t) for x in re.findall(r":\s*(\d+)/48", x)))
    s = [soak(t) for t in ts]
    r["soak bursts accepted %% (per seed)"] = " ; ".join("%.0f" % x[0] for x in s)
    r["soak phantom s/min (per seed)"] = " ; ".join("%.1f" % x[1] for x in s)
    r["soak phantom acquisitions /min (per seed)"] = " ; ".join("%.1f" % x[3] for x in s)
    r["soak false losses /min (per seed)"] = " ; ".join("%.2f" % (x[2] / 10.0) for x in s)
    r["false losses/min at 10-70%% misses (max)"] = " ; ".join("%.2f" % miss(t) for t in ts)
    r["steady dropout survived (ms)"] = " ; ".join(str(drop(t, "strong, 1 receiver")[0]) for t in ts)
    r["flickering (50%%) dropout survived (ms)"] = " ; ".join(str(drop(t, "weak 50 % misses, 1 rx")[0]) for t in ts)
    rr = [drop(t, "strong, 1 receiver")[1] for t in ts]
    r["re-acquisition after a lost steady gap (ms)"] = " ; ".join("%.0f" % x for x in rr)
    r["brief 2rx 80 ms: acquired of 48 (loss median/p95)"] = " ; ".join("%d (%s)" % brief(t, 2, 80) for t in ts)
    r["brief 2rx 100 ms: acquired of 48 (loss median/p95)"] = " ; ".join("%d (%s)" % brief(t, 2, 100) for t in ts)
    rows.append((name, r))
print("| metric | default (balanced) | fast |\n|---|---|---|")
for k in rows[0][1]:
    print("| %s | %s | %s |" % (k.replace("%%", "%"), rows[0][1][k], rows[1][1][k]))
