#!/usr/bin/env python3
"""Independent reference checker for ps_pe. Reads dut_log.txt written by tb_ps_pe_log.v.
Recomputes every 16-term dot product from the logged operands (pure Python integers)
and compares with the accumulator value the RTL produced."""
import sys

def sx(v, w):                      # interpret low w bits of v as two's complement
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v

def check(path):
    runs, cur = [], None
    for line in open(path):
        t = line.split()
        if not t: continue
        if t[0] == "RUN":   cur = dict(id=int(t[1]), mode=int(t[2]), w=int(t[3]), ins=[], acc=None); runs.append(cur)
        elif t[0] == "IN":  cur["ins"].append((int(t[2], 16), int(t[3], 16)))
        elif t[0] == "ACC": cur["acc"] = int(t[1], 16)
    fails = 0
    for r in runs:
        w = r["w"]; aw = {4: 12, 8: 20, 16: 36}[w]
        assert {0: 4, 1: 8, 2: 16}[r["mode"]] == w, "mode/width mismatch in log"
        exp = sum(sx(a, w) * sx(b, w) for a, b in r["ins"])   # bits above w ignored
        got = sx(r["acc"], aw)                                # active slice of acc_q
        exp_wrapped = sx(exp, aw)                             # what an aw-bit accumulator holds
        ok = (len(r["ins"]) == 16 and got == exp_wrapped)
        fails += (not ok)
        print(f"run {r['id']} INT{w:<2} terms={len(r['ins'])} expected={exp:>14} "
              f"acc_q(active {aw}b)={got:>14}  {'PASS' if ok else 'FAIL'}"
              + ("" if exp == exp_wrapped else "  (note: true sum overflowed accumulator width)"))
    print("-" * 60)
    print(f"{len(runs) - fails}/{len(runs)} runs PASS" if runs else "no runs found")
    return 1 if fails or not runs else 0

if __name__ == "__main__":
    sys.exit(check(sys.argv[1] if len(sys.argv) > 1 else "dut_log.txt"))
