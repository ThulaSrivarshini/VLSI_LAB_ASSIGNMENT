#!/usr/bin/env python3
"""Cycle-level Python model of ps_pe (needs rtl_model.py). Writes model_vec_log.txt in the
same format as tb_ps_pe_vec.v. Self-test for check_pe_vectors.py:  --fault injects one bug."""
import random, sys
import rtl_model as m

ACT = {4: 0xFFF, 8: 0xFFFFF, 16: 0xFFFFFFFFF}
def run(f, rid, mode, w, rng, st, fault):
    mk = (1 << w) - 1
    st["acc"] &= ~ACT[w] & 0xFFFFFFFFF                                  # clear (only running slices)
    st.update(a=0, b=0, va=0, pr=0, en=0)
    f.write(f"RUN {rid} {mode} {w}\n")
    ins = []
    def cycle(a, b, va, tag):
        if st["en"]:                                                    # stage 3 (uses old pr)
            d = m.gadd(st["acc"], m.sext_iso(st["pr"], w), w)
            st["acc"] = (st["acc"] & ~ACT[w] & 0xFFFFFFFFF) | (d & ACT[w])
        p = m.mult16(st["a"], st["b"], w) if st["va"] else 0            # stage 1->2
        if fault and tag == 7 and w == 16 and st["va"]: p ^= 1 << 20
        st["pr"], st["en"] = p, st["va"]
        st["a"], st["b"], st["va"] = a, b, va
        pn = (m.mult16(a, b, w) if va else 0)
        if fault and tag == 7 and w == 16 and va: pn ^= 1 << 20
        return pn
    # first 16 cycles = vectors, then 6 idle
    for n in range(22):
        if n < 16:
            lo, hi = -(1 << (w-1)), (1 << (w-1)) - 1
            x, y = rng.randint(lo, hi), rng.randint(lo, hi)
            if n == 3: x = y = lo
            if n == 4: x, y = lo, hi
            if n == 5: x = y = hi
            a = (x & mk) | (rng.getrandbits(16) & ~mk & 0xFFFF); b = (y & mk) | (rng.getrandbits(16) & ~mk & 0xFFFF)
            f.write(f"IN {n} {a:04x} {b:04x}\n"); va = 1
        else:
            a, b, va = rng.getrandbits(16), rng.getrandbits(16), 0
            f.write(f"IDLE {n} {a:04x} {b:04x}\n")
        cycle(a, b, va, n - 1)                                          # tag = vector whose product is now computed
        pobs = m.mult16(st["a"], st["b"], w) if st["va"] else 0
        if fault and n == 7 and w == 16: pobs ^= 1 << 20
        f.write(f"OBS {n} {st['a']:04x} {st['b']:04x} {st['va']} {st['va']} {pobs:08x} {st['acc']:09x}\n")
    ev = {4: (0, 0), 8: (100, 0), 16: (130, 128)}[w]
    f.write(f"EV {ev[0]} {ev[1]}\nACC {st['acc']:09x}\n")

if __name__ == "__main__":
    fault = "--fault" in sys.argv; rng = random.Random(3); st = dict(acc=0)
    with open("model_vec_log.txt", "w") as f:
        for rid, (mode, w) in enumerate([(2,16),(0,4),(1,8),(2,16),(1,8),(0,4),(2,16)]):
            run(f, rid, mode, w, rng, st, fault)
