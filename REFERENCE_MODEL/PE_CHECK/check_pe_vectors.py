#!/usr/bin/env python3
"""Per-vector reference check of ps_pe. Input: dut_log.txt from tb_ps_pe_vec.v.
Everything is recomputed with plain Python integers from the logged stimulus.
Checks per run:
  1. stimulus sanity   : 16 IN, 6 IDLE, 22 OBS lines; corner vectors at k=3,4,5 are the intended ones
  2. every vector k    : forwarded a_out/b_out (low w bits), va/vb = 1, multiplier product p == x*y
  3. every idle cycle  : va/vb = 0, p == 0, forwarding of the garbage operands (low w bits)
  4. every cycle       : active slice of acc_q == running sum of the products that have arrived
  5. end of run        : final acc_q == total sum of the 16 products
  6. gating evidence   : INT4 -> mid/high net changes = 0, INT8 -> high = 0, and the active slices DO toggle
Usage: python check_pe_vectors.py dut_log.txt [--lat N]   (N = accumulator latency in loop iterations, default 2)
"""
import sys

def sx(v, w):
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v

def parse(path):
    runs, cur = [], None
    for line in open(path):
        t = line.split()
        if not t: continue
        if t[0] == "RUN":
            cur = dict(id=int(t[1]), mode=int(t[2]), w=int(t[3]), IN={}, IDLE={}, OBS={}, ACC=None, EV=None); runs.append(cur)
        elif t[0] == "IN":   cur["IN"][int(t[1])]   = (int(t[2], 16), int(t[3], 16))
        elif t[0] == "IDLE": cur["IDLE"][int(t[1])] = (int(t[2], 16), int(t[3], 16))
        elif t[0] == "OBS":  cur["OBS"][int(t[1])]  = dict(ao=int(t[2], 16), bo=int(t[3], 16), va=t[4], vb=t[5], p=int(t[6], 16), acc=int(t[7], 16))
        elif t[0] == "ACC":  cur["ACC"] = int(t[1], 16)
        elif t[0] == "EV":   cur["EV"] = (int(t[1]), int(t[2]))
    return runs

def acc_bad(r, lat):
    w, aw = r["w"], {4: 12, 8: 20, 16: 36}[r["w"]]
    prods = [sx(r["IN"][k][0], w) * sx(r["IN"][k][1], w) for k in range(16)]
    bad = []
    for n in range(22):
        exp = sx(sum(prods[:max(0, min(16, n - lat + 1))]), aw)
        if sx(r["OBS"][n]["acc"], aw) != exp: bad.append((n, exp, sx(r["OBS"][n]["acc"], aw)))
    return bad

def check(path, lat=2):
    runs = parse(path); total_fail = 0; ncheck = 0
    for r in runs:
        w = r["w"]; aw = {4: 12, 8: 20, 16: 36}[w]; mk = (1 << w) - 1; m2 = (1 << 2 * w) - 1
        errs = []
        if not (len(r["IN"]) == 16 and len(r["IDLE"]) == 6 and len(r["OBS"]) == 22 and r["ACC"] is not None and r["EV"] is not None):
            errs.append("incomplete log for this run"); 
        else:
            xs = [sx(r["IN"][k][0], w) for k in range(16)]; ys = [sx(r["IN"][k][1], w) for k in range(16)]
            lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
            for k, (ex, ey) in {3: (lo, lo), 4: (lo, hi), 5: (hi, hi)}.items():     # 1. stimulus sanity
                ncheck += 1
                if (xs[k], ys[k]) != (ex, ey): errs.append(f"corner vector k={k}: got ({xs[k]},{ys[k]}) expected ({ex},{ey})")
            for k in range(16):                                                      # 2. per vector
                o = r["OBS"][k]; a, b = r["IN"][k]; prod = xs[k] * ys[k]
                ncheck += 4
                if (o["ao"] & mk) != (a & mk) or (o["bo"] & mk) != (b & mk): errs.append(f"k={k}: forwarded operand mismatch")
                if o["va"] != "1" or o["vb"] != "1": errs.append(f"k={k}: va/vb out not 1")
                if (o["p"] & m2) != (prod & m2): errs.append(f"k={k}: product p={sx(o['p'],2*w)} expected {prod} (x={xs[k]}, y={ys[k]})")
                if sx(o["p"], 32) != prod and (o["p"] & m2) == (prod & m2): pass       # upper bits are not used downstream
            for i in range(6):                                                       # 3. idle cycles
                n = 16 + i; o = r["OBS"][n]; a, b = r["IDLE"][n]; ncheck += 3
                if o["va"] != "0" or o["vb"] != "0": errs.append(f"idle {i}: va/vb out not 0")
                if o["p"] != 0: errs.append(f"idle {i}: product not 0 (p={o['p']:#x}) -> multiplier not isolated")
                if (o["ao"] & mk) != (a & mk) or (o["bo"] & mk) != (b & mk): errs.append(f"idle {i}: forwarded operand mismatch")
            bad = acc_bad(r, lat); ncheck += 22                                      # 4. accumulator every cycle
            if bad:
                alt = [L for L in range(0, 6) if not acc_bad(r, L)]
                for n, e, g in bad[:3]: errs.append(f"iteration {n}: acc={g} expected {e} (latency {lat})")
                if alt: errs.append(f"  -> accumulator IS consistent with latency {alt[0]} (re-run with --lat {alt[0]} if that is your pipeline)")
            ncheck += 1                                                              # 5. final value
            tot = sum(x * y for x, y in zip(xs, ys))
            if sx(r["ACC"], aw) != sx(tot, aw): errs.append(f"final acc {sx(r['ACC'],aw)} expected {sx(tot,aw)}")
            mid, high = r["EV"]; ncheck += 3                                         # 6. gating evidence
            if w == 4 and (mid or high): errs.append(f"INT4 gating: mid={mid} high={high} (must be 0)")
            if w == 8 and high:          errs.append(f"INT8 gating: high={high} (must be 0)")
            if w >= 8 and mid == 0:      errs.append("mid slice never toggled in INT8/INT16 (probe not live?)")
            if w == 16 and high == 0:    errs.append("high slice never toggled in INT16 (probe not live?)")
        total_fail += bool(errs)
        print(f"run {r['id']} INT{w:<2} 16 vectors + 6 idle cycles : {'FAIL' if errs else 'PASS'}"
              + (f"   (final sum {sx(tot, aw)}, EV mid={r['EV'][0]} high={r['EV'][1]})" if not errs else ""))
        for e in errs[:8]: print("     ", e)
    print("-" * 70)
    print(f"{len(runs) - total_fail}/{len(runs)} runs PASS   ({ncheck} individual checks)")
    return 1 if total_fail or not runs else 0

if __name__ == "__main__":
    a = sys.argv[1:]; lat = int(a[a.index("--lat") + 1]) if "--lat" in a else 2
    path = [x for i, x in enumerate(a) if not x.startswith("--") and (i == 0 or a[i-1] != "--lat")]
    sys.exit(check(path[0] if path else "dut_log.txt", lat))
