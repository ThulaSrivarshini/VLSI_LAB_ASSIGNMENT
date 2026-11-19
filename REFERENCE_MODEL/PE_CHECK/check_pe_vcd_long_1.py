#!/usr/bin/env python3
"""Check ps_pe against a Python reference using the VCD written by tb_ps_pe.v (+MODE=0/1/2 -> ps_pe_int4/int8/int16.vcd).
Everything is rebuilt from the waveform: stimulus (a_in, b_in, va_in, vb_in, mode, start) and results.
Sampling: at every falling clock edge each signal's value just BEFORE that time step is used.
Per cycle (all ~3000 vectors + drain):
  forwarding : a_out/b_out equal a_in/b_in on the mode's bits, va_out/vb_out equal va_in/vb_in
  product    : u_pe.p == x*y (low 2w bits) when va&vb, and == 0 when either valid is low   (if u_pe.p is in the VCD)
  accumulator: active slice of acc_q == running sum of the valid products that have reached it
  end        : final acc_q == total of all valid products
  stimulus   : the testbench's directed plan is found in the waveform (25 corner pairs, 3 overflow bursts)
  coverage   : (report) 5x5 corner-pair matrix {min,-1,0,+1,max}^2, wrap-arounds of the exact running sum in both
               directions, closest approach of the running sum to the accumulator limits
  gating     : INT4 -> mid/high slice nets never change, INT8 -> high never changes (if u_pe internals are in the VCD)
Usage: python check_pe_vcd_long.py ps_pe_int4.vcd [--lat N] [--strict]
  --strict : an incomplete corner-pair matrix or a missing overflow in either direction makes the RESULT FAIL
             (default: coverage is only reported)
"""
import sys, bisect

class VCD:
    def __init__(self, path):
        self.names = {}          # full path -> id
        self.t = {}; self.v = {} # id -> times, values
        scope, cur = [], 0
        toks = open(path).read().split()
        i = 0
        while i < len(toks):
            tk = toks[i]
            if tk == "$scope": scope.append(toks[i + 2]); i += 4; continue
            if tk == "$upscope": scope.pop(); i += 2; continue
            if tk == "$var":
                end = toks.index("$end", i); width = int(toks[i + 2]); idc = toks[i + 3]; nm = toks[i + 4]
                self.names[".".join(scope + [nm])] = (idc, width)
                self.t.setdefault(idc, []); self.v.setdefault(idc, []); i = end + 1; continue
            if tk.startswith("$"):                       # $date, $version, $timescale, $dumpvars ... skip keyword only
                i += 1; continue
            if tk[0] == "#": cur = int(tk[1:]); i += 1; continue
            if tk[0] in "bB": val, idc = tk[1:], toks[i + 1]; i += 2
            elif tk[0] in "rR": i += 2; continue
            else: val, idc = tk[0], tk[1:]; i += 1
            if idc in self.t: self.t[idc].append(cur); self.v[idc].append(val)
    def find(self, name, within=None):
        c = [(p.count("."), p) for p in self.names if p.split(".")[-1] == name and (within is None or p.endswith(within + "." + name))]
        return self.names[min(c)[1]] if c else None
    def before(self, sig, t):            # value just before time step t
        idc, w = sig; k = bisect.bisect_left(self.t[idc], t) - 1
        if k < 0: return None
        s = self.v[idc][k]
        return None if any(ch in "xzXZ" for ch in s) else int(s, 2)
    def changes(self, sig, t0, t1):      # list of values at change times in (t0, t1]
        idc, w = sig; out = []
        for tt, s in zip(self.t[idc], self.v[idc]):
            if t0 < tt <= t1: out.append(None if any(ch in "xzXZ" for ch in s) else int(s, 2))
        return out

def count_changes(vcd, sig, t0, t1, lo=0, width=None):
    prev = vcd.before(sig, t0 + 1)
    sl = lambda x: None if x is None else (x >> lo) & ((1 << width) - 1) if width else x
    n, last = 0, sl(prev)
    for val in vcd.changes(sig, t0, t1):
        if sl(val) != last: n += 1; last = sl(val)
    return n


def sx(v, w):
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v

def main(path, lat=2, strict=False):
    vcd = VCD(path)
    need = ["clk", "rst", "start", "mode", "a_in", "b_in", "va_in", "vb_in", "a_out", "b_out", "va_out", "vb_out", "acc_q"]
    S = {n: vcd.find(n) for n in need}
    miss = [n for n, s in S.items() if s is None]
    if miss: sys.exit(f"VCD is missing signals: {miss}")
    P = vcd.find("p", "u_pe")
    G = {k: vcd.find(k, "u_pe") for k in ("a_mid", "b_mid", "a_hi", "b_hi", "p1", "p2", "x1", "x2")}
    idc = S["clk"][0]; tt, vv = vcd.t[idc], vcd.v[idc]
    edges = [tt[k] for k in range(1, len(tt)) if vv[k] == "0" and vv[k - 1] == "1"]
    smp = []
    for t in edges:
        d = {n: vcd.before(S[n], t) for n in need if n != "clk"}; d["t"] = t; d["p"] = vcd.before(P, t) if P else None; smp.append(d)
    i_s = next((i for i, s in enumerate(smp) if s["start"] == 1 and s["rst"] == 0 and s["mode"] in (0, 1, 2)), None)
    if i_s is None: sys.exit("no start pulse with a legal mode found in the waveform")
    mode = smp[i_s]["mode"]; w = {0: 4, 1: 8, 2: 16}[mode]; aw = {4: 12, 8: 20, 16: 36}[w]
    mk = (1 << w) - 1; m2 = (1 << 2 * w) - 1; amask = (1 << aw) - 1
    i0 = next((i for i in range(i_s + 1, len(smp)) if smp[i]["va_in"] == 1), None)
    if i0 is None: sys.exit("no valid vectors found after start")
    cyc = smp[i0:]; C = len(cyc)
    print(f"{path}: mode {mode} (INT{w}), {C} cycles after the first vector")

    errs = {"forwarding": [], "product": [], "accumulator": [], "stimulus": [], "gating": [], "x/z": []}
    prods = []                                           # valid products per cycle (0 when not valid)
    nvalid = nhalf = 0
    for c, s in enumerate(cyc):
        if None in (s["a_in"], s["b_in"], s["va_in"], s["vb_in"]): errs["x/z"].append(f"cycle {c}: unknown stimulus"); prods.append(0); continue
        val = s["va_in"] == 1 and s["vb_in"] == 1
        nvalid += val; nhalf += (s["va_in"] != s["vb_in"])
        prods.append(sx(s["a_in"], w) * sx(s["b_in"], w) if val else 0)
    lo_w, hi_w = -(1 << (w - 1)), (1 << (w - 1)) - 1
    TAB = [lo_w, -1, 0, 1, hi_w]                                 # same order as the testbench's dvec(): min, -1, 0, +1, max
    plan = [(TAB[j // 5], TAB[j % 5]) for j in range(25)] + [(lo_w, lo_w)] * 40 + [(lo_w, hi_w)] * 80 + [(hi_w, hi_w)] * 40
    def lowbits_ok(c, x, y):
        s_ = cyc[c]
        return s_["a_in"] is not None and s_["b_in"] is not None and sx(s_["a_in"], w) == x and sx(s_["b_in"], w) == y and s_["va_in"] == 1 and s_["vb_in"] == 1
    directed = C >= len(plan) and all(lowbits_ok(c, *plan[c]) for c in range(len(plan)))
    legacy = [(0x8000, 0x8000), (0xFFFF, 0xFFFF), (0x7FFF, 0x8000), (0x0008, 0x0008)]
    legacy_ok = C >= 4 and all((cyc[c]["a_in"], cyc[c]["b_in"]) == legacy[c] and cyc[c]["va_in"] == 1 and cyc[c]["vb_in"] == 1 for c in range(4))
    if directed: print(f"  stimulus plan: directed (25 corner pairs + 40 min*min + 80 min*max + 40 max*max) found")
    elif legacy_ok: print("  stimulus plan: legacy (4 corner vectors, +NODIR run) - no forced overflow in this VCD")
    else:
        bad_c = next((c for c in range(min(C, len(plan))) if not lowbits_ok(c, *plan[c])), 0)
        errs["stimulus"].append(f"first vectors match neither the directed plan (first mismatch at vector {bad_c}) nor the legacy corner vectors")
    run = 0; exp_at = []                                 # expected accumulator per cycle (latency lat)
    for c in range(C):
        if c - lat >= 0: run += prods[c - lat]
        exp_at.append(run)
    def acc_bad(L):
        r = 0; bad = []
        for c in range(C):
            if c - L >= 0: r += prods[c - L]
            g = cyc[c]["acc_q"]
            if g is None or (g & amask) != (r & amask): bad.append(c)
        return bad
    for c, s in enumerate(cyc):
        if None in (s["a_out"], s["b_out"], s["va_out"], s["vb_out"]): errs["x/z"].append(f"cycle {c}: unknown forwarded outputs"); continue
        if ((s["a_out"] ^ s["a_in"]) & mk) or ((s["b_out"] ^ s["b_in"]) & mk) or s["va_out"] != s["va_in"] or s["vb_out"] != s["vb_in"]:
            errs["forwarding"].append(f"cycle {c}: a_out={s['a_out']:04x} a_in={s['a_in']:04x} b_out={s['b_out']:04x} b_in={s['b_in']:04x} va/vb out={s['va_out']}{s['vb_out']} in={s['va_in']}{s['vb_in']}")
        if s["p"] is not None:
            val = s["va_in"] == 1 and s["vb_in"] == 1
            if val and (s["p"] & m2) != (prods[c] & m2): errs["product"].append(f"cycle {c}: p={sx(s['p'], 2*w)} expected {prods[c]}")
            if not val and s["p"] != 0: errs["product"].append(f"cycle {c}: p={s['p']:#x} but a valid input was low (must be 0)")
    bad = acc_bad(lat)
    for c in bad[:10]:
        g = cyc[c]["acc_q"]; errs["accumulator"].append(f"cycle {c}: acc_q={None if g is None else sx(g, aw)} expected {sx(exp_at[c], aw)}")
    if bad:
        alt = [L for L in range(0, 6) if not acc_bad(L)]
        if alt: errs["accumulator"].append(f"-> accumulator IS consistent with latency {alt[0]}; re-run with --lat {alt[0]}")
    total = sum(prods); final = cyc[-1]["acc_q"]
    if final is None or (final & amask) != (total & amask): errs["gating"].append(f"final acc_q {None if final is None else sx(final, aw)} != total {sx(total, aw)}") if False else errs["accumulator"].append(f"final acc_q != total {sx(total, aw)}")
    ev = None
    if all(G.values()):
        t0, t1 = smp[i0 - 1]["t"], cyc[-1]["t"]
        mid = sum(count_changes(vcd, G[g], t0, t1) for g in ("a_mid", "b_mid", "p1", "x1")) + count_changes(vcd, S["acc_q"], t0, t1, 12, 8)
        hi = sum(count_changes(vcd, G[g], t0, t1) for g in ("a_hi", "b_hi", "p2", "x2")) + count_changes(vcd, S["acc_q"], t0, t1, 20, 16)
        ev = (mid, hi)
        if w == 4 and (mid or hi): errs["gating"].append(f"INT4: mid={mid} high={hi} net changes (must be 0)")
        if w == 8 and hi:          errs["gating"].append(f"INT8: high={hi} net changes (must be 0)")
        if w >= 8 and mid == 0:    errs["gating"].append("mid slice never toggled (probe not live?)")
        if w == 16 and hi == 0:    errs["gating"].append("high slice never toggled (probe not live?)")


    # ---- coverage report: corner pairs and accumulator wrap-around ----
    cname = {lo_w: "min", -1: "-1", 0: "0", 1: "+1", hi_w: "max"}
    labels = ["min", "-1", "0", "+1", "max"]
    pair = {(i, j): 0 for i in labels for j in labels}
    other = 0
    for c, s_ in enumerate(cyc):
        if None in (s_["a_in"], s_["b_in"]) or not (s_["va_in"] == 1 and s_["vb_in"] == 1): continue
        x, y = sx(s_["a_in"], w), sx(s_["b_in"], w)
        if x in cname and y in cname: pair[(cname[x], cname[y])] += 1
        else: other += 1
    missing = [f"{i}*{j}" for i in labels for j in labels if pair[(i, j)] == 0]
    half = 1 << (aw - 1); run = 0; kprev = 0; wp = wn = 0; smax = smin = 0
    for p_ in prods:
        run += p_
        if p_ == 0: continue
        k = (run + half) >> aw
        if k > kprev: wp += 1
        if k < kprev: wn += 1
        kprev = k; smax = max(smax, run); smin = min(smin, run)
    cov_ok = not missing and wp > 0 and wn > 0

    print(f"  vectors with both valid: {nvalid}, one valid low: {nhalf}, cycles with va=vb=0 or drain: {C - nvalid - nhalf}")
    print(f"  python reference: sum of valid products = {sx(total, aw)}   acc_q & mask (compare with the testbench PASS line) = {total & amask:09x}")
    print(f"  VCD final acc_q & mask                   = {(final or 0) & amask:09x}")
    names = {"stimulus": "stimulus sanity", "forwarding": "forwarding", "product": "product p" + ("" if P else " (not in VCD, skipped)"),
             "accumulator": "accumulator", "gating": "gating" + ("" if ev else " (probes not in VCD, skipped)"), "x/z": "X/Z values"}
    for k, label in names.items():
        print(f"  {label:<34}: {'PASS' if not errs[k] else 'FAIL (' + str(len(errs[k])) + ' issues)'}")
        for e in errs[k][:5]: print("       ", e)
    if ev: print(f"  net changes in the mid slice = {ev[0]}, high slice = {ev[1]}")

    print(f"  coverage, valid vectors per corner pair (rows x, cols y; {other} valid vectors are non-corner):")
    print("            " + "".join(f"{j:>7}" for j in labels))
    for i in labels: print(f"    x={i:<5} " + "".join(f"{pair[(i, j)]:>7}" for j in labels))
    print(f"  corner pairs hit: {25 - len(missing)}/25" + (f"   missing: {', '.join(missing)}" if missing else ""))
    full_p, full_n = half - 1, half
    if wp or wn:
        print(f"  accumulator wrap-arounds of the exact running sum: {wp} upward (past +{full_p}), {wn} downward (past -{full_n})")
        print(f"  peak exact running sum: max {smax} ({100.0 * smax / full_p:.1f} % of +limit), min {smin} ({100.0 * -smin / full_n:.1f} % of -limit)")
    else:
        print(f"  accumulator never wrapped: peak sum {smax} = {100.0 * smax / full_p:.1f} % of +limit {full_p}, trough {smin} = {100.0 * -smin / full_n:.1f} % of -limit -{full_n}")
        print(f"  closest approach to +limit: {full_p - smax}, to -limit: {smin + full_n}")
    print(f"  coverage: {'COMPLETE' if cov_ok else 'INCOMPLETE'} (all 25 corner pairs and wrap-around in both directions)")
    ok = not any(errs.values())
    if strict and not cov_ok: print("  --strict: coverage incomplete -> FAIL"); ok = False
    print("RESULT:", "PASS" if ok else "FAIL"); return 0 if ok else 1

if __name__ == "__main__":
    a = sys.argv[1:]; lat = int(a[a.index("--lat") + 1]) if "--lat" in a else 2
    p = [x for i, x in enumerate(a) if not x.startswith("--") and (i == 0 or a[i - 1] != "--lat")]
    sys.exit(main(p[0], lat, "--strict" in a))
