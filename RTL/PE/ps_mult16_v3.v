//=============================================================================
// ps_mult16_v3.v   (area/power revision of ps_mult16_v2.v; ports unchanged)
//
// Precision-scalable signed multiplier  INT4 / INT8 / INT16
//   mode : 00 INT4 | 01 INT8 | 10 INT16 | 11 illegal      en : global enable
//   p    : signed product sign-extended to 32 bit         illegal : mode == 11
//
// What changed w.r.t. v2
//   v2 : 16 bw4x4_mul blocks, each with its own 9-bit carry-propagate adder
//        and a run-time correction subtractor, then a 17-operand 32-bit add.
//   v3 : NO per-block adders.  All 16 blocks only produce their 16 partial-
//        product bits (Baugh-Wooley, sign-flipped by the nibble sign flags).
//        All 256 bits go into ONE carry-save tree and ONE final adder.
//        The Baugh-Wooley "-2^k" corrections of every block are folded into a
//        single mode-dependent constant K (only 3 possible values).
//
// v3b (timing): only the final summation changed.  The 64 weighted rows + K
//   are now added by an explicit FA/HA compressor tree and a 32-bit
//   Kogge-Stone adder instead of a behavioural '+' chain.  Ports, gating,
//   partial products and K are untouched, results are bit-identical.
//
// Gating (operand isolation)
//   Block (i,j) is on when nib_en[max(i,j)] = 1.  Both of its operand
//   nibbles are ANDed with that one signal, so a disabled block sees all
//   zeros and its partial products do not toggle.  Identical AND gates are
//   shared by the tool because blk_en only takes 3 distinct values.
//=============================================================================
`timescale 1ns/1ps

module ps_mult16 (
    input  wire [15:0]        a,
    input  wire [15:0]        b,
    input  wire [1:0]         mode,
    input  wire               en,
    output wire signed [31:0] p,
    output wire               illegal,
    output wire [15:0]        blk_en
);

    // Sum of all Baugh-Wooley corrections, per mode (mod 2^32).
    // INT4  : block (0,0) signed x signed                      = -112
    // INT8  : blocks (0,1),(1,0) = -120*2^4 each, (1,1) = -112*2^8
    // INT16 : blocks (3,j),(i,3) = -120*2^(4(i+j)), (3,3) = -112*2^24
    localparam [31:0] K4  = 32'hFFFFFF90;
    localparam [31:0] K8  = 32'hFFFF8100;
    localparam [31:0] K16 = 32'h80010000;

    // ---- Mode decode (one-hot, qualified with en) --------------------------
    wire m_int4  = en & ~mode[1] & ~mode[0];
    wire m_int8  = en & ~mode[1] &  mode[0];
    wire m_int16 = en &  mode[1] & ~mode[0];
    assign illegal = mode[1] & mode[0];

    // ---- Per-nibble enable and sign flag -----------------------------------
    wire [3:0] nib_en  = { m_int16, m_int16, (m_int8 | m_int16), (m_int4 | m_int8 | m_int16) };
    wire [3:0] nib_sgn = { m_int16, 1'b0,    m_int8,             m_int4 };

    // ---- Mode-dependent constant (all zero when the multiplier is off) ----
    wire [31:0] k_corr = ({32{m_int4}}  & K4)
                       | ({32{m_int8}}  & K8)
                       | ({32{m_int16}} & K16);

    // ---- Partial-product array --------------------------------------------
    wire [63:0]   a_g_flat;      // 16 x 4 bit  gated A nibble per block
    wire [63:0]   b_g_flat;      // 16 x 4 bit  gated B nibble per block
    wire [2047:0] term_flat;     // 64 rows x 32 bit, already weighted

    // Simulation-only 9-bit block products, kept so a testbench probing
    // dut.pblk_flat still compiles.  Hidden from the synthesis tool.
    // synopsys translate_off
    wire [143:0]  pblk_flat;
    // synopsys translate_on

    genvar i, j, x, y;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_i
            for (j = 0; j < 4; j = j + 1) begin : g_j
                assign blk_en[4*i+j] = nib_en[(i > j) ? i : j];

                wire [3:0] a_g  = a[4*i +: 4] & {4{blk_en[4*i+j]}};
                wire [3:0] b_g  = b[4*j +: 4] & {4{blk_en[4*i+j]}};
                wire       sa_g = nib_sgn[i]  & blk_en[4*i+j];
                wire       sb_g = nib_sgn[j]  & blk_en[4*i+j];

                assign a_g_flat[4*(4*i+j) +: 4] = a_g;
                assign b_g_flat[4*(4*i+j) +: 4] = b_g;

                // synopsys translate_off
                wire signed [4:0] av = {sa_g & a_g[3], a_g};
                wire signed [4:0] bv = {sb_g & b_g[3], b_g};
                wire signed [9:0] pv = av * bv;
                assign pblk_flat[9*(4*i+j) +: 9] = pv[8:0];
                // synopsys translate_on

                // pp[4*y+x] = a_g[x] * b_g[y]   (Baugh-Wooley sign flips)
                wire [15:0] pp;
                for (y = 0; y < 4; y = y + 1) begin : g_y
                    for (x = 0; x < 4; x = x + 1) begin : g_x
                        if (x == 3 && y < 3) begin : neg_a_msb
                            assign pp[4*y+x] = (a_g[x] & b_g[y]) ^ sa_g;
                        end else if (y == 3 && x < 3) begin : neg_b_msb
                            assign pp[4*y+x] = (a_g[x] & b_g[y]) ^ sb_g;
                        end else if (x == 3 && y == 3) begin : msb_msb
                            assign pp[4*y+x] = (a_g[x] & b_g[y]) ^ (sa_g ^ sb_g);
                        end else begin : plain
                            assign pp[4*y+x] = a_g[x] & b_g[y];
                        end
                    end
                    // row y of this block, weight 2^(4(i+j)+y)
                    assign term_flat[32*(4*(4*i+j)+y) +: 32]
                           = {28'b0, pp[4*y +: 4]} << (4*(i+j)+y);
                end
            end
        end
    endgenerate

    // ---- Summation of 64 weighted rows + constant K ----------------------
    // v3b: replaces the behavioural 'acc = acc + term' loop (carry-save tree
    // + whatever final adder the tool picked) by an explicit structure:
    //   1) timing-driven FA/HA compressor tree (three-greedy: the 3 EARLIEST
    //      bits of a column go into the next full adder, late bits are touched
    //      last, carries always move to the next column);  219 FA + 5 HA
    //   2) 32-bit Kogge-Stone parallel-prefix final adder (log2(32) = 5 levels).
    // Function is identical: p = k_corr + sum(term rows)  (mod 2^32).

    // ---- column 0 : 1 bits in ----
    // ---- column 1 : 2 bits in ----
    // ---- column 2 : 3 bits in ----
    wire ha0_s = term_flat[2] ^ term_flat[34];
    wire ha0_c = term_flat[2] & term_flat[34];
    // ---- column 3 : 5 bits in ----
    wire fa0_x = term_flat[35] ^ term_flat[67];
    wire fa0_s = fa0_x ^ term_flat[3];
    wire fa0_c = (term_flat[35] & term_flat[67]) | (term_flat[3] & fa0_x);
    wire ha1_s = term_flat[99] ^ ha0_c;
    wire ha1_c = term_flat[99] & ha0_c;
    // ---- column 4 : 8 bits in ----
    wire fa1_x = term_flat[68] ^ term_flat[132];
    wire fa1_s = fa1_x ^ term_flat[516];
    wire fa1_c = (term_flat[68] & term_flat[132]) | (term_flat[516] & fa1_x);
    wire fa2_x = k_corr[4] ^ term_flat[36];
    wire fa2_s = fa2_x ^ term_flat[100];
    wire fa2_c = (k_corr[4] & term_flat[36]) | (term_flat[100] & fa2_x);
    wire fa3_x = fa0_c ^ ha1_c;
    wire fa3_s = fa3_x ^ fa1_s;
    wire fa3_c = (fa0_c & ha1_c) | (fa1_s & fa3_x);
    // ---- column 5 : 9 bits in ----
    wire fa4_x = term_flat[133] ^ term_flat[165];
    wire fa4_s = fa4_x ^ term_flat[517];
    wire fa4_c = (term_flat[133] & term_flat[165]) | (term_flat[517] & fa4_x);
    wire fa5_x = term_flat[549] ^ term_flat[69];
    wire fa5_s = fa5_x ^ term_flat[101];
    wire fa5_c = (term_flat[549] & term_flat[69]) | (term_flat[101] & fa5_x);
    wire fa6_x = fa1_c ^ fa4_s;
    wire fa6_s = fa6_x ^ fa2_c;
    wire fa6_c = (fa1_c & fa4_s) | (fa2_c & fa6_x);
    wire ha2_s = fa5_s ^ fa3_c;
    wire ha2_c = fa5_s & fa3_c;
    // ---- column 6 : 11 bits in ----
    wire fa7_x = term_flat[134] ^ term_flat[166];
    wire fa7_s = fa7_x ^ term_flat[198];
    wire fa7_c = (term_flat[134] & term_flat[166]) | (term_flat[198] & fa7_x);
    wire fa8_x = term_flat[518] ^ term_flat[550];
    wire fa8_s = fa8_x ^ term_flat[582];
    wire fa8_c = (term_flat[518] & term_flat[550]) | (term_flat[582] & fa8_x);
    wire fa9_x = term_flat[102] ^ fa4_c;
    wire fa9_s = fa9_x ^ fa7_s;
    wire fa9_c = (term_flat[102] & fa4_c) | (fa7_s & fa9_x);
    wire fa10_x = fa8_s ^ fa5_c;
    wire fa10_s = fa10_x ^ fa6_c;
    wire fa10_c = (fa8_s & fa5_c) | (fa6_c & fa10_x);
    wire ha3_s = fa9_s ^ ha2_c;
    wire ha3_c = fa9_s & ha2_c;
    // ---- column 7 : 14 bits in ----
    wire fa11_x = term_flat[167] ^ term_flat[199];
    wire fa11_s = fa11_x ^ term_flat[551];
    wire fa11_c = (term_flat[167] & term_flat[199]) | (term_flat[551] & fa11_x);
    wire fa12_x = term_flat[583] ^ k_corr[7];
    wire fa12_s = fa12_x ^ term_flat[135];
    wire fa12_c = (term_flat[583] & k_corr[7]) | (term_flat[135] & fa12_x);
    wire fa13_x = term_flat[231] ^ term_flat[519];
    wire fa13_s = fa13_x ^ term_flat[615];
    wire fa13_c = (term_flat[231] & term_flat[519]) | (term_flat[615] & fa13_x);
    wire fa14_x = fa7_c ^ fa8_c;
    wire fa14_s = fa14_x ^ fa11_s;
    wire fa14_c = (fa7_c & fa8_c) | (fa11_s & fa14_x);
    wire fa15_x = fa12_s ^ fa13_s;
    wire fa15_s = fa15_x ^ fa9_c;
    wire fa15_c = (fa12_s & fa13_s) | (fa9_c & fa15_x);
    wire fa16_x = fa14_s ^ fa10_c;
    wire fa16_s = fa16_x ^ fa15_s;
    wire fa16_c = (fa14_s & fa10_c) | (fa15_s & fa16_x);
    // ---- column 8 : 16 bits in ----
    wire fa17_x = term_flat[200] ^ term_flat[264];
    wire fa17_s = fa17_x ^ term_flat[584];
    wire fa17_c = (term_flat[200] & term_flat[264]) | (term_flat[584] & fa17_x);
    wire fa18_x = term_flat[648] ^ term_flat[1032];
    wire fa18_s = fa18_x ^ k_corr[8];
    wire fa18_c = (term_flat[648] & term_flat[1032]) | (k_corr[8] & fa18_x);
    wire fa19_x = term_flat[168] ^ term_flat[232];
    wire fa19_s = fa19_x ^ term_flat[552];
    wire fa19_c = (term_flat[168] & term_flat[232]) | (term_flat[552] & fa19_x);
    wire fa20_x = term_flat[616] ^ fa11_c;
    wire fa20_s = fa20_x ^ fa12_c;
    wire fa20_c = (term_flat[616] & fa11_c) | (fa12_c & fa20_x);
    wire fa21_x = fa17_s ^ fa18_s;
    wire fa21_s = fa21_x ^ fa13_c;
    wire fa21_c = (fa17_s & fa18_s) | (fa13_c & fa21_x);
    wire fa22_x = fa19_s ^ fa14_c;
    wire fa22_s = fa22_x ^ fa20_s;
    wire fa22_c = (fa19_s & fa14_c) | (fa20_s & fa22_x);
    wire fa23_x = fa21_s ^ fa15_c;
    wire fa23_s = fa23_x ^ fa22_s;
    wire fa23_c = (fa21_s & fa15_c) | (fa22_s & fa23_x);
    // ---- column 9 : 18 bits in ----
    wire fa24_x = term_flat[265] ^ term_flat[297];
    wire fa24_s = fa24_x ^ term_flat[649];
    wire fa24_c = (term_flat[265] & term_flat[297]) | (term_flat[649] & fa24_x);
    wire fa25_x = term_flat[681] ^ term_flat[1033];
    wire fa25_s = fa25_x ^ term_flat[1065];
    wire fa25_c = (term_flat[681] & term_flat[1033]) | (term_flat[1065] & fa25_x);
    wire fa26_x = k_corr[9] ^ term_flat[201];
    wire fa26_s = fa26_x ^ term_flat[233];
    wire fa26_c = (k_corr[9] & term_flat[201]) | (term_flat[233] & fa26_x);
    wire fa27_x = term_flat[585] ^ term_flat[617];
    wire fa27_s = fa27_x ^ fa17_c;
    wire fa27_c = (term_flat[585] & term_flat[617]) | (fa17_c & fa27_x);
    wire fa28_x = fa18_c ^ fa24_s;
    wire fa28_s = fa28_x ^ fa25_s;
    wire fa28_c = (fa18_c & fa24_s) | (fa25_s & fa28_x);
    wire fa29_x = fa19_c ^ fa26_s;
    wire fa29_s = fa29_x ^ fa27_s;
    wire fa29_c = (fa19_c & fa26_s) | (fa27_s & fa29_x);
    wire fa30_x = fa20_c ^ fa21_c;
    wire fa30_s = fa30_x ^ fa28_s;
    wire fa30_c = (fa20_c & fa21_c) | (fa28_s & fa30_x);
    wire fa31_x = fa29_s ^ fa22_c;
    wire fa31_s = fa31_x ^ fa30_s;
    wire fa31_c = (fa29_s & fa22_c) | (fa30_s & fa31_x);
    // ---- column 10 : 20 bits in ----
    wire fa32_x = term_flat[266] ^ term_flat[298];
    wire fa32_s = fa32_x ^ term_flat[330];
    wire fa32_c = (term_flat[266] & term_flat[298]) | (term_flat[330] & fa32_x);
    wire fa33_x = term_flat[650] ^ term_flat[682];
    wire fa33_s = fa33_x ^ term_flat[714];
    wire fa33_c = (term_flat[650] & term_flat[682]) | (term_flat[714] & fa33_x);
    wire fa34_x = term_flat[1034] ^ term_flat[1066];
    wire fa34_s = fa34_x ^ term_flat[1098];
    wire fa34_c = (term_flat[1034] & term_flat[1066]) | (term_flat[1098] & fa34_x);
    wire fa35_x = k_corr[10] ^ term_flat[234];
    wire fa35_s = fa35_x ^ term_flat[618];
    wire fa35_c = (k_corr[10] & term_flat[234]) | (term_flat[618] & fa35_x);
    wire fa36_x = fa24_c ^ fa25_c;
    wire fa36_s = fa36_x ^ fa32_s;
    wire fa36_c = (fa24_c & fa25_c) | (fa32_s & fa36_x);
    wire fa37_x = fa33_s ^ fa34_s;
    wire fa37_s = fa37_x ^ fa26_c;
    wire fa37_c = (fa33_s & fa34_s) | (fa26_c & fa37_x);
    wire fa38_x = fa27_c ^ fa35_s;
    wire fa38_s = fa38_x ^ fa28_c;
    wire fa38_c = (fa27_c & fa35_s) | (fa28_c & fa38_x);
    wire fa39_x = fa36_s ^ fa37_s;
    wire fa39_s = fa39_x ^ fa29_c;
    wire fa39_c = (fa36_s & fa37_s) | (fa29_c & fa39_x);
    wire fa40_x = fa38_s ^ fa30_c;
    wire fa40_s = fa40_x ^ fa39_s;
    wire fa40_c = (fa38_s & fa30_c) | (fa39_s & fa40_x);
    // ---- column 11 : 22 bits in ----
    wire fa41_x = term_flat[299] ^ term_flat[331];
    wire fa41_s = fa41_x ^ term_flat[683];
    wire fa41_c = (term_flat[299] & term_flat[331]) | (term_flat[683] & fa41_x);
    wire fa42_x = term_flat[715] ^ term_flat[1067];
    wire fa42_s = fa42_x ^ term_flat[1099];
    wire fa42_c = (term_flat[715] & term_flat[1067]) | (term_flat[1099] & fa42_x);
    wire fa43_x = k_corr[11] ^ term_flat[267];
    wire fa43_s = fa43_x ^ term_flat[363];
    wire fa43_c = (k_corr[11] & term_flat[267]) | (term_flat[363] & fa43_x);
    wire fa44_x = term_flat[651] ^ term_flat[747];
    wire fa44_s = fa44_x ^ term_flat[1035];
    wire fa44_c = (term_flat[651] & term_flat[747]) | (term_flat[1035] & fa44_x);
    wire fa45_x = term_flat[1131] ^ fa32_c;
    wire fa45_s = fa45_x ^ fa33_c;
    wire fa45_c = (term_flat[1131] & fa32_c) | (fa33_c & fa45_x);
    wire fa46_x = fa34_c ^ fa41_s;
    wire fa46_s = fa46_x ^ fa42_s;
    wire fa46_c = (fa34_c & fa41_s) | (fa42_s & fa46_x);
    wire fa47_x = fa35_c ^ fa43_s;
    wire fa47_s = fa47_x ^ fa44_s;
    wire fa47_c = (fa35_c & fa43_s) | (fa44_s & fa47_x);
    wire fa48_x = fa36_c ^ fa37_c;
    wire fa48_s = fa48_x ^ fa45_s;
    wire fa48_c = (fa36_c & fa37_c) | (fa45_s & fa48_x);
    wire fa49_x = fa46_s ^ fa38_c;
    wire fa49_s = fa49_x ^ fa47_s;
    wire fa49_c = (fa46_s & fa38_c) | (fa47_s & fa49_x);
    wire fa50_x = fa39_c ^ fa48_s;
    wire fa50_s = fa50_x ^ fa49_s;
    wire fa50_c = (fa39_c & fa48_s) | (fa49_s & fa50_x);
    // ---- column 12 : 24 bits in ----
    wire fa51_x = term_flat[332] ^ term_flat[396];
    wire fa51_s = fa51_x ^ term_flat[716];
    wire fa51_c = (term_flat[332] & term_flat[396]) | (term_flat[716] & fa51_x);
    wire fa52_x = term_flat[780] ^ term_flat[1100];
    wire fa52_s = fa52_x ^ term_flat[1164];
    wire fa52_c = (term_flat[780] & term_flat[1100]) | (term_flat[1164] & fa52_x);
    wire fa53_x = term_flat[1548] ^ k_corr[12];
    wire fa53_s = fa53_x ^ term_flat[300];
    wire fa53_c = (term_flat[1548] & k_corr[12]) | (term_flat[300] & fa53_x);
    wire fa54_x = term_flat[364] ^ term_flat[684];
    wire fa54_s = fa54_x ^ term_flat[748];
    wire fa54_c = (term_flat[364] & term_flat[684]) | (term_flat[748] & fa54_x);
    wire fa55_x = term_flat[1068] ^ term_flat[1132];
    wire fa55_s = fa55_x ^ fa41_c;
    wire fa55_c = (term_flat[1068] & term_flat[1132]) | (fa41_c & fa55_x);
    wire fa56_x = fa42_c ^ fa51_s;
    wire fa56_s = fa56_x ^ fa52_s;
    wire fa56_c = (fa42_c & fa51_s) | (fa52_s & fa56_x);
    wire fa57_x = fa53_s ^ fa43_c;
    wire fa57_s = fa57_x ^ fa44_c;
    wire fa57_c = (fa53_s & fa43_c) | (fa44_c & fa57_x);
    wire fa58_x = fa54_s ^ fa55_s;
    wire fa58_s = fa58_x ^ fa45_c;
    wire fa58_c = (fa54_s & fa55_s) | (fa45_c & fa58_x);
    wire fa59_x = fa46_c ^ fa56_s;
    wire fa59_s = fa59_x ^ fa47_c;
    wire fa59_c = (fa46_c & fa56_s) | (fa47_c & fa59_x);
    wire fa60_x = fa57_s ^ fa58_s;
    wire fa60_s = fa60_x ^ fa48_c;
    wire fa60_c = (fa57_s & fa58_s) | (fa48_c & fa60_x);
    wire fa61_x = fa59_s ^ fa49_c;
    wire fa61_s = fa61_x ^ fa60_s;
    wire fa61_c = (fa59_s & fa49_c) | (fa60_s & fa61_x);
    // ---- column 13 : 26 bits in ----
    wire fa62_x = term_flat[397] ^ term_flat[429];
    wire fa62_s = fa62_x ^ term_flat[781];
    wire fa62_c = (term_flat[397] & term_flat[429]) | (term_flat[781] & fa62_x);
    wire fa63_x = term_flat[813] ^ term_flat[1165];
    wire fa63_s = fa63_x ^ term_flat[1197];
    wire fa63_c = (term_flat[813] & term_flat[1165]) | (term_flat[1197] & fa63_x);
    wire fa64_x = term_flat[1549] ^ term_flat[1581];
    wire fa64_s = fa64_x ^ k_corr[13];
    wire fa64_c = (term_flat[1549] & term_flat[1581]) | (k_corr[13] & fa64_x);
    wire fa65_x = term_flat[333] ^ term_flat[365];
    wire fa65_s = fa65_x ^ term_flat[717];
    wire fa65_c = (term_flat[333] & term_flat[365]) | (term_flat[717] & fa65_x);
    wire fa66_x = term_flat[749] ^ term_flat[1101];
    wire fa66_s = fa66_x ^ term_flat[1133];
    wire fa66_c = (term_flat[749] & term_flat[1101]) | (term_flat[1133] & fa66_x);
    wire fa67_x = fa51_c ^ fa52_c;
    wire fa67_s = fa67_x ^ fa53_c;
    wire fa67_c = (fa51_c & fa52_c) | (fa53_c & fa67_x);
    wire fa68_x = fa62_s ^ fa63_s;
    wire fa68_s = fa68_x ^ fa64_s;
    wire fa68_c = (fa62_s & fa63_s) | (fa64_s & fa68_x);
    wire fa69_x = fa54_c ^ fa55_c;
    wire fa69_s = fa69_x ^ fa65_s;
    wire fa69_c = (fa54_c & fa55_c) | (fa65_s & fa69_x);
    wire fa70_x = fa66_s ^ fa56_c;
    wire fa70_s = fa70_x ^ fa67_s;
    wire fa70_c = (fa66_s & fa56_c) | (fa67_s & fa70_x);
    wire fa71_x = fa68_s ^ fa57_c;
    wire fa71_s = fa71_x ^ fa58_c;
    wire fa71_c = (fa68_s & fa57_c) | (fa58_c & fa71_x);
    wire fa72_x = fa69_s ^ fa59_c;
    wire fa72_s = fa72_x ^ fa70_s;
    wire fa72_c = (fa69_s & fa59_c) | (fa70_s & fa72_x);
    wire fa73_x = fa60_c ^ fa71_s;
    wire fa73_s = fa73_x ^ fa72_s;
    wire fa73_c = (fa60_c & fa71_s) | (fa72_s & fa73_x);
    // ---- column 14 : 28 bits in ----
    wire fa74_x = term_flat[398] ^ term_flat[430];
    wire fa74_s = fa74_x ^ term_flat[462];
    wire fa74_c = (term_flat[398] & term_flat[430]) | (term_flat[462] & fa74_x);
    wire fa75_x = term_flat[782] ^ term_flat[814];
    wire fa75_s = fa75_x ^ term_flat[846];
    wire fa75_c = (term_flat[782] & term_flat[814]) | (term_flat[846] & fa75_x);
    wire fa76_x = term_flat[1166] ^ term_flat[1198];
    wire fa76_s = fa76_x ^ term_flat[1230];
    wire fa76_c = (term_flat[1166] & term_flat[1198]) | (term_flat[1230] & fa76_x);
    wire fa77_x = term_flat[1550] ^ term_flat[1582];
    wire fa77_s = fa77_x ^ term_flat[1614];
    wire fa77_c = (term_flat[1550] & term_flat[1582]) | (term_flat[1614] & fa77_x);
    wire fa78_x = k_corr[14] ^ term_flat[366];
    wire fa78_s = fa78_x ^ term_flat[750];
    wire fa78_c = (k_corr[14] & term_flat[366]) | (term_flat[750] & fa78_x);
    wire fa79_x = term_flat[1134] ^ fa62_c;
    wire fa79_s = fa79_x ^ fa63_c;
    wire fa79_c = (term_flat[1134] & fa62_c) | (fa63_c & fa79_x);
    wire fa80_x = fa64_c ^ fa74_s;
    wire fa80_s = fa80_x ^ fa75_s;
    wire fa80_c = (fa64_c & fa74_s) | (fa75_s & fa80_x);
    wire fa81_x = fa76_s ^ fa77_s;
    wire fa81_s = fa81_x ^ fa65_c;
    wire fa81_c = (fa76_s & fa77_s) | (fa65_c & fa81_x);
    wire fa82_x = fa66_c ^ fa78_s;
    wire fa82_s = fa82_x ^ fa67_c;
    wire fa82_c = (fa66_c & fa78_s) | (fa67_c & fa82_x);
    wire fa83_x = fa68_c ^ fa79_s;
    wire fa83_s = fa83_x ^ fa80_s;
    wire fa83_c = (fa68_c & fa79_s) | (fa80_s & fa83_x);
    wire fa84_x = fa81_s ^ fa69_c;
    wire fa84_s = fa84_x ^ fa82_s;
    wire fa84_c = (fa81_s & fa69_c) | (fa82_s & fa84_x);
    wire fa85_x = fa70_c ^ fa83_s;
    wire fa85_s = fa85_x ^ fa71_c;
    wire fa85_c = (fa70_c & fa83_s) | (fa71_c & fa85_x);
    wire fa86_x = fa84_s ^ fa72_c;
    wire fa86_s = fa86_x ^ fa85_s;
    wire fa86_c = (fa84_s & fa72_c) | (fa85_s & fa86_x);
    // ---- column 15 : 30 bits in ----
    wire fa87_x = term_flat[431] ^ term_flat[463];
    wire fa87_s = fa87_x ^ term_flat[815];
    wire fa87_c = (term_flat[431] & term_flat[463]) | (term_flat[815] & fa87_x);
    wire fa88_x = term_flat[847] ^ term_flat[1199];
    wire fa88_s = fa88_x ^ term_flat[1231];
    wire fa88_c = (term_flat[847] & term_flat[1199]) | (term_flat[1231] & fa88_x);
    wire fa89_x = term_flat[1583] ^ term_flat[1615];
    wire fa89_s = fa89_x ^ k_corr[15];
    wire fa89_c = (term_flat[1583] & term_flat[1615]) | (k_corr[15] & fa89_x);
    wire fa90_x = term_flat[399] ^ term_flat[495];
    wire fa90_s = fa90_x ^ term_flat[783];
    wire fa90_c = (term_flat[399] & term_flat[495]) | (term_flat[783] & fa90_x);
    wire fa91_x = term_flat[879] ^ term_flat[1167];
    wire fa91_s = fa91_x ^ term_flat[1263];
    wire fa91_c = (term_flat[879] & term_flat[1167]) | (term_flat[1263] & fa91_x);
    wire fa92_x = term_flat[1551] ^ term_flat[1647];
    wire fa92_s = fa92_x ^ fa74_c;
    wire fa92_c = (term_flat[1551] & term_flat[1647]) | (fa74_c & fa92_x);
    wire fa93_x = fa75_c ^ fa76_c;
    wire fa93_s = fa93_x ^ fa77_c;
    wire fa93_c = (fa75_c & fa76_c) | (fa77_c & fa93_x);
    wire fa94_x = fa87_s ^ fa88_s;
    wire fa94_s = fa94_x ^ fa89_s;
    wire fa94_c = (fa87_s & fa88_s) | (fa89_s & fa94_x);
    wire fa95_x = fa78_c ^ fa90_s;
    wire fa95_s = fa95_x ^ fa91_s;
    wire fa95_c = (fa78_c & fa90_s) | (fa91_s & fa95_x);
    wire fa96_x = fa92_s ^ fa79_c;
    wire fa96_s = fa96_x ^ fa80_c;
    wire fa96_c = (fa92_s & fa79_c) | (fa80_c & fa96_x);
    wire fa97_x = fa81_c ^ fa93_s;
    wire fa97_s = fa97_x ^ fa94_s;
    wire fa97_c = (fa81_c & fa93_s) | (fa94_s & fa97_x);
    wire fa98_x = fa82_c ^ fa95_s;
    wire fa98_s = fa98_x ^ fa83_c;
    wire fa98_c = (fa82_c & fa95_s) | (fa83_c & fa98_x);
    wire fa99_x = fa96_s ^ fa97_s;
    wire fa99_s = fa99_x ^ fa84_c;
    wire fa99_c = (fa96_s & fa97_s) | (fa84_c & fa99_x);
    wire fa100_x = fa98_s ^ fa85_c;
    wire fa100_s = fa100_x ^ fa99_s;
    wire fa100_c = (fa98_s & fa85_c) | (fa99_s & fa100_x);
    // ---- column 16 : 30 bits in ----
    wire fa101_x = term_flat[464] ^ term_flat[848];
    wire fa101_s = fa101_x ^ term_flat[912];
    wire fa101_c = (term_flat[464] & term_flat[848]) | (term_flat[912] & fa101_x);
    wire fa102_x = term_flat[1232] ^ term_flat[1296];
    wire fa102_s = fa102_x ^ term_flat[1616];
    wire fa102_c = (term_flat[1232] & term_flat[1296]) | (term_flat[1616] & fa102_x);
    wire fa103_x = term_flat[1680] ^ k_corr[16];
    wire fa103_s = fa103_x ^ term_flat[432];
    wire fa103_c = (term_flat[1680] & k_corr[16]) | (term_flat[432] & fa103_x);
    wire fa104_x = term_flat[496] ^ term_flat[816];
    wire fa104_s = fa104_x ^ term_flat[880];
    wire fa104_c = (term_flat[496] & term_flat[816]) | (term_flat[880] & fa104_x);
    wire fa105_x = term_flat[1200] ^ term_flat[1264];
    wire fa105_s = fa105_x ^ term_flat[1584];
    wire fa105_c = (term_flat[1200] & term_flat[1264]) | (term_flat[1584] & fa105_x);
    wire fa106_x = term_flat[1648] ^ fa87_c;
    wire fa106_s = fa106_x ^ fa88_c;
    wire fa106_c = (term_flat[1648] & fa87_c) | (fa88_c & fa106_x);
    wire fa107_x = fa89_c ^ fa101_s;
    wire fa107_s = fa107_x ^ fa102_s;
    wire fa107_c = (fa89_c & fa101_s) | (fa102_s & fa107_x);
    wire fa108_x = fa103_s ^ fa90_c;
    wire fa108_s = fa108_x ^ fa91_c;
    wire fa108_c = (fa103_s & fa90_c) | (fa91_c & fa108_x);
    wire fa109_x = fa92_c ^ fa104_s;
    wire fa109_s = fa109_x ^ fa105_s;
    wire fa109_c = (fa92_c & fa104_s) | (fa105_s & fa109_x);
    wire fa110_x = fa93_c ^ fa94_c;
    wire fa110_s = fa110_x ^ fa106_s;
    wire fa110_c = (fa93_c & fa94_c) | (fa106_s & fa110_x);
    wire fa111_x = fa107_s ^ fa95_c;
    wire fa111_s = fa111_x ^ fa108_s;
    wire fa111_c = (fa107_s & fa95_c) | (fa108_s & fa111_x);
    wire fa112_x = fa109_s ^ fa96_c;
    wire fa112_s = fa112_x ^ fa97_c;
    wire fa112_c = (fa109_s & fa96_c) | (fa97_c & fa112_x);
    wire fa113_x = fa110_s ^ fa98_c;
    wire fa113_s = fa113_x ^ fa111_s;
    wire fa113_c = (fa110_s & fa98_c) | (fa111_s & fa113_x);
    wire fa114_x = fa99_c ^ fa112_s;
    wire fa114_s = fa114_x ^ fa113_s;
    wire fa114_c = (fa99_c & fa112_s) | (fa113_s & fa114_x);
    // ---- column 17 : 29 bits in ----
    wire fa115_x = term_flat[913] ^ term_flat[945];
    wire fa115_s = fa115_x ^ term_flat[1297];
    wire fa115_c = (term_flat[913] & term_flat[945]) | (term_flat[1297] & fa115_x);
    wire fa116_x = term_flat[1329] ^ term_flat[1681];
    wire fa116_s = fa116_x ^ term_flat[1713];
    wire fa116_c = (term_flat[1329] & term_flat[1681]) | (term_flat[1713] & fa116_x);
    wire fa117_x = k_corr[17] ^ term_flat[465];
    wire fa117_s = fa117_x ^ term_flat[497];
    wire fa117_c = (k_corr[17] & term_flat[465]) | (term_flat[497] & fa117_x);
    wire fa118_x = term_flat[849] ^ term_flat[881];
    wire fa118_s = fa118_x ^ term_flat[1233];
    wire fa118_c = (term_flat[849] & term_flat[881]) | (term_flat[1233] & fa118_x);
    wire fa119_x = term_flat[1265] ^ term_flat[1617];
    wire fa119_s = fa119_x ^ term_flat[1649];
    wire fa119_c = (term_flat[1265] & term_flat[1617]) | (term_flat[1649] & fa119_x);
    wire fa120_x = fa101_c ^ fa102_c;
    wire fa120_s = fa120_x ^ fa103_c;
    wire fa120_c = (fa101_c & fa102_c) | (fa103_c & fa120_x);
    wire fa121_x = fa115_s ^ fa116_s;
    wire fa121_s = fa121_x ^ fa104_c;
    wire fa121_c = (fa115_s & fa116_s) | (fa104_c & fa121_x);
    wire fa122_x = fa105_c ^ fa117_s;
    wire fa122_s = fa122_x ^ fa118_s;
    wire fa122_c = (fa105_c & fa117_s) | (fa118_s & fa122_x);
    wire fa123_x = fa119_s ^ fa106_c;
    wire fa123_s = fa123_x ^ fa107_c;
    wire fa123_c = (fa119_s & fa106_c) | (fa107_c & fa123_x);
    wire fa124_x = fa120_s ^ fa121_s;
    wire fa124_s = fa124_x ^ fa108_c;
    wire fa124_c = (fa120_s & fa121_s) | (fa108_c & fa124_x);
    wire fa125_x = fa109_c ^ fa122_s;
    wire fa125_s = fa125_x ^ fa110_c;
    wire fa125_c = (fa109_c & fa122_s) | (fa110_c & fa125_x);
    wire fa126_x = fa123_s ^ fa124_s;
    wire fa126_s = fa126_x ^ fa111_c;
    wire fa126_c = (fa123_s & fa124_s) | (fa111_c & fa126_x);
    wire fa127_x = fa125_s ^ fa112_c;
    wire fa127_s = fa127_x ^ fa126_s;
    wire fa127_c = (fa125_s & fa112_c) | (fa126_s & fa127_x);
    wire ha4_s = fa113_c ^ fa114_c;
    wire ha4_c = fa113_c & fa114_c;
    // ---- column 18 : 28 bits in ----
    wire fa128_x = term_flat[914] ^ term_flat[946];
    wire fa128_s = fa128_x ^ term_flat[978];
    wire fa128_c = (term_flat[914] & term_flat[946]) | (term_flat[978] & fa128_x);
    wire fa129_x = term_flat[1298] ^ term_flat[1330];
    wire fa129_s = fa129_x ^ term_flat[1362];
    wire fa129_c = (term_flat[1298] & term_flat[1330]) | (term_flat[1362] & fa129_x);
    wire fa130_x = term_flat[1682] ^ term_flat[1714];
    wire fa130_s = fa130_x ^ term_flat[1746];
    wire fa130_c = (term_flat[1682] & term_flat[1714]) | (term_flat[1746] & fa130_x);
    wire fa131_x = k_corr[18] ^ term_flat[498];
    wire fa131_s = fa131_x ^ term_flat[882];
    wire fa131_c = (k_corr[18] & term_flat[498]) | (term_flat[882] & fa131_x);
    wire fa132_x = term_flat[1266] ^ term_flat[1650];
    wire fa132_s = fa132_x ^ fa115_c;
    wire fa132_c = (term_flat[1266] & term_flat[1650]) | (fa115_c & fa132_x);
    wire fa133_x = fa116_c ^ fa128_s;
    wire fa133_s = fa133_x ^ fa129_s;
    wire fa133_c = (fa116_c & fa128_s) | (fa129_s & fa133_x);
    wire fa134_x = fa130_s ^ fa117_c;
    wire fa134_s = fa134_x ^ fa118_c;
    wire fa134_c = (fa130_s & fa117_c) | (fa118_c & fa134_x);
    wire fa135_x = fa119_c ^ fa131_s;
    wire fa135_s = fa135_x ^ fa132_s;
    wire fa135_c = (fa119_c & fa131_s) | (fa132_s & fa135_x);
    wire fa136_x = fa120_c ^ fa121_c;
    wire fa136_s = fa136_x ^ fa133_s;
    wire fa136_c = (fa120_c & fa121_c) | (fa133_s & fa136_x);
    wire fa137_x = fa122_c ^ fa134_s;
    wire fa137_s = fa137_x ^ fa135_s;
    wire fa137_c = (fa122_c & fa134_s) | (fa135_s & fa137_x);
    wire fa138_x = fa123_c ^ fa124_c;
    wire fa138_s = fa138_x ^ fa136_s;
    wire fa138_c = (fa123_c & fa124_c) | (fa136_s & fa138_x);
    wire fa139_x = fa125_c ^ fa137_s;
    wire fa139_s = fa139_x ^ fa126_c;
    wire fa139_c = (fa125_c & fa137_s) | (fa126_c & fa139_x);
    wire fa140_x = fa138_s ^ fa139_s;
    wire fa140_s = fa140_x ^ fa127_c;
    wire fa140_c = (fa138_s & fa139_s) | (fa127_c & fa140_x);
    // ---- column 19 : 26 bits in ----
    wire fa141_x = term_flat[947] ^ term_flat[979];
    wire fa141_s = fa141_x ^ term_flat[1331];
    wire fa141_c = (term_flat[947] & term_flat[979]) | (term_flat[1331] & fa141_x);
    wire fa142_x = term_flat[1363] ^ term_flat[1715];
    wire fa142_s = fa142_x ^ term_flat[1747];
    wire fa142_c = (term_flat[1363] & term_flat[1715]) | (term_flat[1747] & fa142_x);
    wire fa143_x = k_corr[19] ^ term_flat[915];
    wire fa143_s = fa143_x ^ term_flat[1011];
    wire fa143_c = (k_corr[19] & term_flat[915]) | (term_flat[1011] & fa143_x);
    wire fa144_x = term_flat[1299] ^ term_flat[1395];
    wire fa144_s = fa144_x ^ term_flat[1683];
    wire fa144_c = (term_flat[1299] & term_flat[1395]) | (term_flat[1683] & fa144_x);
    wire fa145_x = term_flat[1779] ^ fa128_c;
    wire fa145_s = fa145_x ^ fa129_c;
    wire fa145_c = (term_flat[1779] & fa128_c) | (fa129_c & fa145_x);
    wire fa146_x = fa130_c ^ fa141_s;
    wire fa146_s = fa146_x ^ fa142_s;
    wire fa146_c = (fa130_c & fa141_s) | (fa142_s & fa146_x);
    wire fa147_x = fa131_c ^ fa132_c;
    wire fa147_s = fa147_x ^ fa143_s;
    wire fa147_c = (fa131_c & fa132_c) | (fa143_s & fa147_x);
    wire fa148_x = fa144_s ^ fa133_c;
    wire fa148_s = fa148_x ^ fa145_s;
    wire fa148_c = (fa144_s & fa133_c) | (fa145_s & fa148_x);
    wire fa149_x = fa146_s ^ fa134_c;
    wire fa149_s = fa149_x ^ fa135_c;
    wire fa149_c = (fa146_s & fa134_c) | (fa135_c & fa149_x);
    wire fa150_x = fa147_s ^ fa136_c;
    wire fa150_s = fa150_x ^ fa148_s;
    wire fa150_c = (fa147_s & fa136_c) | (fa148_s & fa150_x);
    wire fa151_x = fa137_c ^ fa149_s;
    wire fa151_s = fa151_x ^ fa138_c;
    wire fa151_c = (fa137_c & fa149_s) | (fa138_c & fa151_x);
    wire fa152_x = fa150_s ^ fa139_c;
    wire fa152_s = fa152_x ^ fa151_s;
    wire fa152_c = (fa150_s & fa139_c) | (fa151_s & fa152_x);
    // ---- column 20 : 24 bits in ----
    wire fa153_x = term_flat[980] ^ term_flat[1364];
    wire fa153_s = fa153_x ^ term_flat[1428];
    wire fa153_c = (term_flat[980] & term_flat[1364]) | (term_flat[1428] & fa153_x);
    wire fa154_x = term_flat[1748] ^ term_flat[1812];
    wire fa154_s = fa154_x ^ k_corr[20];
    wire fa154_c = (term_flat[1748] & term_flat[1812]) | (k_corr[20] & fa154_x);
    wire fa155_x = term_flat[948] ^ term_flat[1012];
    wire fa155_s = fa155_x ^ term_flat[1332];
    wire fa155_c = (term_flat[948] & term_flat[1012]) | (term_flat[1332] & fa155_x);
    wire fa156_x = term_flat[1396] ^ term_flat[1716];
    wire fa156_s = fa156_x ^ term_flat[1780];
    wire fa156_c = (term_flat[1396] & term_flat[1716]) | (term_flat[1780] & fa156_x);
    wire fa157_x = fa141_c ^ fa142_c;
    wire fa157_s = fa157_x ^ fa153_s;
    wire fa157_c = (fa141_c & fa142_c) | (fa153_s & fa157_x);
    wire fa158_x = fa154_s ^ fa143_c;
    wire fa158_s = fa158_x ^ fa144_c;
    wire fa158_c = (fa154_s & fa143_c) | (fa144_c & fa158_x);
    wire fa159_x = fa155_s ^ fa156_s;
    wire fa159_s = fa159_x ^ fa145_c;
    wire fa159_c = (fa155_s & fa156_s) | (fa145_c & fa159_x);
    wire fa160_x = fa146_c ^ fa157_s;
    wire fa160_s = fa160_x ^ fa147_c;
    wire fa160_c = (fa146_c & fa157_s) | (fa147_c & fa160_x);
    wire fa161_x = fa158_s ^ fa159_s;
    wire fa161_s = fa161_x ^ fa148_c;
    wire fa161_c = (fa158_s & fa159_s) | (fa148_c & fa161_x);
    wire fa162_x = fa160_s ^ fa149_c;
    wire fa162_s = fa162_x ^ fa161_s;
    wire fa162_c = (fa160_s & fa149_c) | (fa161_s & fa162_x);
    wire fa163_x = fa150_c ^ fa151_c;
    wire fa163_s = fa163_x ^ fa162_s;
    wire fa163_c = (fa150_c & fa151_c) | (fa162_s & fa163_x);
    // ---- column 21 : 22 bits in ----
    wire fa164_x = term_flat[1429] ^ term_flat[1461];
    wire fa164_s = fa164_x ^ term_flat[1813];
    wire fa164_c = (term_flat[1429] & term_flat[1461]) | (term_flat[1813] & fa164_x);
    wire fa165_x = term_flat[1845] ^ k_corr[21];
    wire fa165_s = fa165_x ^ term_flat[981];
    wire fa165_c = (term_flat[1845] & k_corr[21]) | (term_flat[981] & fa165_x);
    wire fa166_x = term_flat[1013] ^ term_flat[1365];
    wire fa166_s = fa166_x ^ term_flat[1397];
    wire fa166_c = (term_flat[1013] & term_flat[1365]) | (term_flat[1397] & fa166_x);
    wire fa167_x = term_flat[1749] ^ term_flat[1781];
    wire fa167_s = fa167_x ^ fa153_c;
    wire fa167_c = (term_flat[1749] & term_flat[1781]) | (fa153_c & fa167_x);
    wire fa168_x = fa154_c ^ fa164_s;
    wire fa168_s = fa168_x ^ fa165_s;
    wire fa168_c = (fa154_c & fa164_s) | (fa165_s & fa168_x);
    wire fa169_x = fa155_c ^ fa156_c;
    wire fa169_s = fa169_x ^ fa166_s;
    wire fa169_c = (fa155_c & fa156_c) | (fa166_s & fa169_x);
    wire fa170_x = fa167_s ^ fa157_c;
    wire fa170_s = fa170_x ^ fa168_s;
    wire fa170_c = (fa167_s & fa157_c) | (fa168_s & fa170_x);
    wire fa171_x = fa158_c ^ fa159_c;
    wire fa171_s = fa171_x ^ fa169_s;
    wire fa171_c = (fa158_c & fa159_c) | (fa169_s & fa171_x);
    wire fa172_x = fa160_c ^ fa170_s;
    wire fa172_s = fa172_x ^ fa161_c;
    wire fa172_c = (fa160_c & fa170_s) | (fa161_c & fa172_x);
    wire fa173_x = fa171_s ^ fa172_s;
    wire fa173_s = fa173_x ^ fa162_c;
    wire fa173_c = (fa171_s & fa172_s) | (fa162_c & fa173_x);
    // ---- column 22 : 20 bits in ----
    wire fa174_x = term_flat[1430] ^ term_flat[1462];
    wire fa174_s = fa174_x ^ term_flat[1494];
    wire fa174_c = (term_flat[1430] & term_flat[1462]) | (term_flat[1494] & fa174_x);
    wire fa175_x = term_flat[1814] ^ term_flat[1846];
    wire fa175_s = fa175_x ^ term_flat[1878];
    wire fa175_c = (term_flat[1814] & term_flat[1846]) | (term_flat[1878] & fa175_x);
    wire fa176_x = k_corr[22] ^ term_flat[1014];
    wire fa176_s = fa176_x ^ term_flat[1398];
    wire fa176_c = (k_corr[22] & term_flat[1014]) | (term_flat[1398] & fa176_x);
    wire fa177_x = term_flat[1782] ^ fa164_c;
    wire fa177_s = fa177_x ^ fa165_c;
    wire fa177_c = (term_flat[1782] & fa164_c) | (fa165_c & fa177_x);
    wire fa178_x = fa174_s ^ fa175_s;
    wire fa178_s = fa178_x ^ fa166_c;
    wire fa178_c = (fa174_s & fa175_s) | (fa166_c & fa178_x);
    wire fa179_x = fa167_c ^ fa176_s;
    wire fa179_s = fa179_x ^ fa168_c;
    wire fa179_c = (fa167_c & fa176_s) | (fa168_c & fa179_x);
    wire fa180_x = fa177_s ^ fa178_s;
    wire fa180_s = fa180_x ^ fa169_c;
    wire fa180_c = (fa177_s & fa178_s) | (fa169_c & fa180_x);
    wire fa181_x = fa179_s ^ fa170_c;
    wire fa181_s = fa181_x ^ fa180_s;
    wire fa181_c = (fa179_s & fa170_c) | (fa180_s & fa181_x);
    wire fa182_x = fa171_c ^ fa172_c;
    wire fa182_s = fa182_x ^ fa181_s;
    wire fa182_c = (fa171_c & fa172_c) | (fa181_s & fa182_x);
    // ---- column 23 : 18 bits in ----
    wire fa183_x = term_flat[1463] ^ term_flat[1495];
    wire fa183_s = fa183_x ^ term_flat[1847];
    wire fa183_c = (term_flat[1463] & term_flat[1495]) | (term_flat[1847] & fa183_x);
    wire fa184_x = term_flat[1879] ^ k_corr[23];
    wire fa184_s = fa184_x ^ term_flat[1431];
    wire fa184_c = (term_flat[1879] & k_corr[23]) | (term_flat[1431] & fa184_x);
    wire fa185_x = term_flat[1527] ^ term_flat[1815];
    wire fa185_s = fa185_x ^ term_flat[1911];
    wire fa185_c = (term_flat[1527] & term_flat[1815]) | (term_flat[1911] & fa185_x);
    wire fa186_x = fa174_c ^ fa175_c;
    wire fa186_s = fa186_x ^ fa183_s;
    wire fa186_c = (fa174_c & fa175_c) | (fa183_s & fa186_x);
    wire fa187_x = fa184_s ^ fa176_c;
    wire fa187_s = fa187_x ^ fa185_s;
    wire fa187_c = (fa184_s & fa176_c) | (fa185_s & fa187_x);
    wire fa188_x = fa177_c ^ fa178_c;
    wire fa188_s = fa188_x ^ fa186_s;
    wire fa188_c = (fa177_c & fa178_c) | (fa186_s & fa188_x);
    wire fa189_x = fa179_c ^ fa187_s;
    wire fa189_s = fa189_x ^ fa180_c;
    wire fa189_c = (fa179_c & fa187_s) | (fa180_c & fa189_x);
    wire fa190_x = fa188_s ^ fa189_s;
    wire fa190_s = fa190_x ^ fa181_c;
    wire fa190_c = (fa188_s & fa189_s) | (fa181_c & fa190_x);
    // ---- column 24 : 16 bits in ----
    wire fa191_x = term_flat[1496] ^ term_flat[1880];
    wire fa191_s = fa191_x ^ term_flat[1944];
    wire fa191_c = (term_flat[1496] & term_flat[1880]) | (term_flat[1944] & fa191_x);
    wire fa192_x = k_corr[24] ^ term_flat[1464];
    wire fa192_s = fa192_x ^ term_flat[1528];
    wire fa192_c = (k_corr[24] & term_flat[1464]) | (term_flat[1528] & fa192_x);
    wire fa193_x = term_flat[1848] ^ term_flat[1912];
    wire fa193_s = fa193_x ^ fa183_c;
    wire fa193_c = (term_flat[1848] & term_flat[1912]) | (fa183_c & fa193_x);
    wire fa194_x = fa184_c ^ fa191_s;
    wire fa194_s = fa194_x ^ fa185_c;
    wire fa194_c = (fa184_c & fa191_s) | (fa185_c & fa194_x);
    wire fa195_x = fa192_s ^ fa193_s;
    wire fa195_s = fa195_x ^ fa186_c;
    wire fa195_c = (fa192_s & fa193_s) | (fa186_c & fa195_x);
    wire fa196_x = fa194_s ^ fa187_c;
    wire fa196_s = fa196_x ^ fa195_s;
    wire fa196_c = (fa194_s & fa187_c) | (fa195_s & fa196_x);
    wire fa197_x = fa188_c ^ fa189_c;
    wire fa197_s = fa197_x ^ fa196_s;
    wire fa197_c = (fa188_c & fa189_c) | (fa196_s & fa197_x);
    // ---- column 25 : 14 bits in ----
    wire fa198_x = term_flat[1945] ^ term_flat[1977];
    wire fa198_s = fa198_x ^ k_corr[25];
    wire fa198_c = (term_flat[1945] & term_flat[1977]) | (k_corr[25] & fa198_x);
    wire fa199_x = term_flat[1497] ^ term_flat[1529];
    wire fa199_s = fa199_x ^ term_flat[1881];
    wire fa199_c = (term_flat[1497] & term_flat[1529]) | (term_flat[1881] & fa199_x);
    wire fa200_x = term_flat[1913] ^ fa191_c;
    wire fa200_s = fa200_x ^ fa198_s;
    wire fa200_c = (term_flat[1913] & fa191_c) | (fa198_s & fa200_x);
    wire fa201_x = fa192_c ^ fa193_c;
    wire fa201_s = fa201_x ^ fa199_s;
    wire fa201_c = (fa192_c & fa193_c) | (fa199_s & fa201_x);
    wire fa202_x = fa194_c ^ fa200_s;
    wire fa202_s = fa202_x ^ fa195_c;
    wire fa202_c = (fa194_c & fa200_s) | (fa195_c & fa202_x);
    wire fa203_x = fa201_s ^ fa202_s;
    wire fa203_s = fa203_x ^ fa196_c;
    wire fa203_c = (fa201_s & fa202_s) | (fa196_c & fa203_x);
    // ---- column 26 : 12 bits in ----
    wire fa204_x = term_flat[1946] ^ term_flat[1978];
    wire fa204_s = fa204_x ^ term_flat[2010];
    wire fa204_c = (term_flat[1946] & term_flat[1978]) | (term_flat[2010] & fa204_x);
    wire fa205_x = k_corr[26] ^ term_flat[1530];
    wire fa205_s = fa205_x ^ term_flat[1914];
    wire fa205_c = (k_corr[26] & term_flat[1530]) | (term_flat[1914] & fa205_x);
    wire fa206_x = fa198_c ^ fa204_s;
    wire fa206_s = fa206_x ^ fa199_c;
    wire fa206_c = (fa198_c & fa204_s) | (fa199_c & fa206_x);
    wire fa207_x = fa205_s ^ fa200_c;
    wire fa207_s = fa207_x ^ fa206_s;
    wire fa207_c = (fa205_s & fa200_c) | (fa206_s & fa207_x);
    wire fa208_x = fa201_c ^ fa202_c;
    wire fa208_s = fa208_x ^ fa207_s;
    wire fa208_c = (fa201_c & fa202_c) | (fa207_s & fa208_x);
    // ---- column 27 : 10 bits in ----
    wire fa209_x = term_flat[1979] ^ term_flat[2011];
    wire fa209_s = fa209_x ^ k_corr[27];
    wire fa209_c = (term_flat[1979] & term_flat[2011]) | (k_corr[27] & fa209_x);
    wire fa210_x = term_flat[1947] ^ term_flat[2043];
    wire fa210_s = fa210_x ^ fa204_c;
    wire fa210_c = (term_flat[1947] & term_flat[2043]) | (fa204_c & fa210_x);
    wire fa211_x = fa209_s ^ fa205_c;
    wire fa211_s = fa211_x ^ fa210_s;
    wire fa211_c = (fa209_s & fa205_c) | (fa210_s & fa211_x);
    wire fa212_x = fa206_c ^ fa211_s;
    wire fa212_s = fa212_x ^ fa207_c;
    wire fa212_c = (fa206_c & fa211_s) | (fa207_c & fa212_x);
    // ---- column 28 : 8 bits in ----
    wire fa213_x = term_flat[2012] ^ k_corr[28];
    wire fa213_s = fa213_x ^ term_flat[1980];
    wire fa213_c = (term_flat[2012] & k_corr[28]) | (term_flat[1980] & fa213_x);
    wire fa214_x = term_flat[2044] ^ fa209_c;
    wire fa214_s = fa214_x ^ fa213_s;
    wire fa214_c = (term_flat[2044] & fa209_c) | (fa213_s & fa214_x);
    wire fa215_x = fa210_c ^ fa214_s;
    wire fa215_s = fa215_x ^ fa211_c;
    wire fa215_c = (fa210_c & fa214_s) | (fa211_c & fa215_x);
    // ---- column 29 : 6 bits in ----
    wire fa216_x = k_corr[29] ^ term_flat[2013];
    wire fa216_s = fa216_x ^ term_flat[2045];
    wire fa216_c = (k_corr[29] & term_flat[2013]) | (term_flat[2045] & fa216_x);
    wire fa217_x = fa213_c ^ fa216_s;
    wire fa217_s = fa217_x ^ fa214_c;
    wire fa217_c = (fa213_c & fa216_s) | (fa214_c & fa217_x);
    // ---- column 30 : 4 bits in ----
    wire fa218_x = k_corr[30] ^ term_flat[2046];
    wire fa218_s = fa218_x ^ fa216_c;
    wire fa218_c = (k_corr[30] & term_flat[2046]) | (fa216_c & fa218_x);
    // ---- column 31 : 2 bits in ----

    // ---- two rows out of the tree (a missing second bit is a constant 0) ----
    wire [31:0] row_a;
    wire [31:0] row_b;
    assign row_a[0] = term_flat[0];
    assign row_b[0] = 1'b0;
    assign row_a[1] = term_flat[1];
    assign row_b[1] = term_flat[33];
    assign row_a[2] = term_flat[66];
    assign row_b[2] = ha0_s;
    assign row_a[3] = fa0_s;
    assign row_b[3] = ha1_s;
    assign row_a[4] = fa2_s;
    assign row_b[4] = fa3_s;
    assign row_a[5] = fa6_s;
    assign row_b[5] = ha2_s;
    assign row_a[6] = fa10_s;
    assign row_b[6] = ha3_s;
    assign row_a[7] = ha3_c;
    assign row_b[7] = fa16_s;
    assign row_a[8] = fa16_c;
    assign row_b[8] = fa23_s;
    assign row_a[9] = fa23_c;
    assign row_b[9] = fa31_s;
    assign row_a[10] = fa31_c;
    assign row_b[10] = fa40_s;
    assign row_a[11] = fa40_c;
    assign row_b[11] = fa50_s;
    assign row_a[12] = fa50_c;
    assign row_b[12] = fa61_s;
    assign row_a[13] = fa61_c;
    assign row_b[13] = fa73_s;
    assign row_a[14] = fa73_c;
    assign row_b[14] = fa86_s;
    assign row_a[15] = fa86_c;
    assign row_b[15] = fa100_s;
    assign row_a[16] = fa100_c;
    assign row_b[16] = fa114_s;
    assign row_a[17] = fa127_s;
    assign row_b[17] = ha4_s;
    assign row_a[18] = ha4_c;
    assign row_b[18] = fa140_s;
    assign row_a[19] = fa140_c;
    assign row_b[19] = fa152_s;
    assign row_a[20] = fa152_c;
    assign row_b[20] = fa163_s;
    assign row_a[21] = fa173_s;
    assign row_b[21] = fa163_c;
    assign row_a[22] = fa173_c;
    assign row_b[22] = fa182_s;
    assign row_a[23] = fa190_s;
    assign row_b[23] = fa182_c;
    assign row_a[24] = fa190_c;
    assign row_b[24] = fa197_s;
    assign row_a[25] = fa203_s;
    assign row_b[25] = fa197_c;
    assign row_a[26] = fa203_c;
    assign row_b[26] = fa208_s;
    assign row_a[27] = fa212_s;
    assign row_b[27] = fa208_c;
    assign row_a[28] = fa215_s;
    assign row_b[28] = fa212_c;
    assign row_a[29] = fa217_s;
    assign row_b[29] = fa215_c;
    assign row_a[30] = fa218_s;
    assign row_b[30] = fa217_c;
    assign row_a[31] = k_corr[31];
    assign row_b[31] = fa218_c;

    // ---- final adder: Kogge-Stone, carry-in = 0 ----------------------------
    reg [31:0] ks_p0, ks_g, ks_p, ks_gn, ks_pn;
    integer    ks_l, ks_d, ks_i;
    always @* begin
        ks_p0 = row_a ^ row_b;                    // half sum (kept for the final XOR)
        ks_g  = row_a & row_b;                    // generate
        ks_p  = ks_p0;                            // propagate
        for (ks_l = 0; ks_l < 5; ks_l = ks_l + 1) begin
            ks_d  = 1 << ks_l;
            ks_gn = ks_g;
            ks_pn = ks_p;
            for (ks_i = ks_d; ks_i < 32; ks_i = ks_i + 1) begin
                ks_gn[ks_i] = ks_g[ks_i] | (ks_p[ks_i] & ks_g[ks_i - ks_d]);
                ks_pn[ks_i] = ks_p[ks_i] & ks_p[ks_i - ks_d];
            end
            ks_g = ks_gn;
            ks_p = ks_pn;
        end
    end

    // carry into bit i = prefix generate of bits [i-1:0]
    assign p = ks_p0 ^ {ks_g[30:0], 1'b0};

endmodule
