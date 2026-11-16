#!/usr/bin/env python3
"""Bit-accurate Python mirror of ps_mult16_v3 + ps_sext_iso + gated BK adder/accumulator.
Writes model_log.txt in the same format as the Verilog testbench log."""
import random, sys
M32 = 0xFFFFFFFF
K = {4: 0xFFFFFF90, 8: 0xFFFF8100, 16: 0x80010000}

def mult16(a, b, w):
    en = {4:[1,0,0,0], 8:[1,1,0,0], 16:[1,1,1,1]}[w]
    sgn = {4:[1,0,0,0], 8:[0,1,0,0], 16:[0,0,0,1]}[w]   # nib_sgn = {m16,0,m8,m4}
    acc = K[w]
    for i in range(4):
        for j in range(4):
            e = en[max(i, j)]
            ag = ((a >> 4*i) & 0xF) * e; bg = ((b >> 4*j) & 0xF) * e
            sa = sgn[i] & e; sb = sgn[j] & e
            for y in range(4):
                for x in range(4):
                    pp = ((ag >> x) & 1) & ((bg >> y) & 1)
                    if x == 3 and y < 3: pp ^= sa
                    elif y == 3 and x < 3: pp ^= sb
                    elif x == 3 and y == 3: pp ^= (sa ^ sb)
                    acc = (acc + (pp << (4*(i+j) + x + y))) & M32
    return acc

def sext_iso(pr, w):
    en_mid, en_hi = w >= 8, w == 16
    b = lambda n: (pr >> n) & 1
    se0 = (pr >> 8) & 0xF if en_mid else 0xF * b(7)
    se1 = (pr >> 16) & 0xF if en_hi else 0xF * b(15)
    x0 = (se0 << 8) | (pr & 0xFF)
    x1 = (((se1 << 4) | ((pr >> 12) & 0xF)) & 0xFF) if en_mid else 0
    sg = b(31) & en_hi
    x2 = ((0xF * sg) << 12 | (((pr >> 20) & 0xFFF) if en_hi else 0))
    return (x2 << 20) | (x1 << 12) | x0

def gadd(a, b, w):
    em, eh = w >= 8, w == 16
    s0 = (a & 0xFFF) + (b & 0xFFF); c0 = s0 >> 12
    s1 = ((a >> 12) & 0xFF) * em + ((b >> 12) & 0xFF) * em + (c0 & em); c1 = s1 >> 8
    s2 = ((a >> 20) & 0xFFFF) * eh + ((b >> 20) & 0xFFFF) * eh + (c1 & eh)
    return ((s2 & 0xFFFF) << 20) | ((s1 & 0xFF) << 12) | (s0 & 0xFFF)

def run(w, rng, acc_prev, fault=False):
    mask = (1 << w) - 1
    acc = 0                                    # clear cycle
    ins = []
    for k in range(16):
        x = rng.randint(-(1 << (w-1)), (1 << (w-1)) - 1); y = rng.randint(-(1 << (w-1)), (1 << (w-1)) - 1)
        if k == 3: x = y = -(1 << (w-1))
        if k == 4: x, y = -(1 << (w-1)), (1 << (w-1)) - 1
        if k == 5: x = y = (1 << (w-1)) - 1
        a = (x & mask) | (rng.getrandbits(16) & ~mask & 0xFFFF)   # garbage above width
        b = (y & mask) | (rng.getrandbits(16) & ~mask & 0xFFFF)
        ins.append((a, b))
        p = mult16(a, b, w)
        if fault and w == 16 and k == 7: p ^= 1 << 20          # injected bug (self-test only)
        acc = gadd(acc, sext_iso(p, w), w)
    return ins, acc

if __name__ == "__main__":
    fault = "--fault" in sys.argv
    rng = random.Random(1); acc = 0
    with open("model_log.txt", "w") as f:
        for rid, (m, w) in enumerate([(2,16),(0,4),(1,8),(2,16),(1,8),(0,4),(2,16)]):
            ins, acc = run(w, rng, acc, fault)
            f.write(f"RUN {rid} {m} {w}\n")
            for k, (a, b) in enumerate(ins): f.write(f"IN {k} {a:04x} {b:04x}\n")
            f.write(f"ACC {acc:09x}\n")
