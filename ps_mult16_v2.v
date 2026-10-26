//=============================================================================
// ps_mult16_v2.v   (low-power revision of ps_mult16.v; module name unchanged)
// Precision-scalable signed multiplier: sixteen 4x4 Baugh-Wooley blocks,
// INT4 / INT8 / INT16, operand isolation, one flat carry-save-friendly sum.
//
//   a, b   : 16-bit operands          mode : 00 INT4 | 01 INT8 | 10 INT16 | 11 illegal
//   en     : global enable            p    : signed product, 32 bit
//   illegal: high in mode 11 (p = 0)  blk_en[4*i+j] : block (i,j) powered
//
// Changes w.r.t. v1 (ports, mode behaviour, blk_en and the names a_g_flat,
// b_g_flat, pblk_flat are unchanged, so tb_ps_mult16 runs as before)
//   1. Two-level operand isolation: each operand nibble is gated ONCE at the
//      input (a_n / b_n), then each block only ANDs with the enable of the
//      OTHER nibble.  Unused upper operand bits now hit one AND gate, not four.
//   2. No 32-bit sign extension of the block products.  A 9-bit two's
//      complement value is  -256*s + low8  =  {~s, low8} - 256 , so each block
//      contributes an UNSIGNED 9-bit term and one constant CORR cancels all
//      sixteen "-256 * 2^(4(i+j))" offsets:
//          CORR = -256 * (sum_i 2^(4i))^2 = -256 * 32'h01234321 = 32'hDCBCDF00
//      A disabled block has pb = 0, so its term is the constant 256*2^(4(i+j))
//      which CORR cancels exactly (p = 0 when en = 0 or mode = 11).
//   3. The 16 terms plus CORR are written as ONE multi-operand addition so the
//      synthesis tool can build a carry-save tree and a single final adder,
//      instead of 15 chained 32-bit adders.
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

    // constant that cancels the 16 offsets of the unsigned block terms
    localparam [31:0] CORR = 32'hDCBCDF00;

    // ---- Mode decode (one-hot, qualified with en) --------------------------
    wire m_int4  = en & ~mode[1] & ~mode[0];
    wire m_int8  = en & ~mode[1] &  mode[0];
    wire m_int16 = en &  mode[1] & ~mode[0];
    assign illegal = mode[1] & mode[0];

    // ---- Per-nibble enable and sign flag -----------------------------------
    wire [3:0] nib_en  = { m_int16, m_int16, (m_int8 | m_int16), (m_int4 | m_int8 | m_int16) };
    wire [3:0] nib_sgn = { m_int16, 1'b0,    m_int8,             m_int4 };

    // ---- Level-1 isolation: gate every operand nibble once ----------------
    wire [15:0] a_n;
    wire [15:0] b_n;
    genvar i, j;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_nib
            assign a_n[4*i +: 4] = a[4*i +: 4] & {4{nib_en[i]}};
            assign b_n[4*i +: 4] = b[4*i +: 4] & {4{nib_en[i]}};
        end
    endgenerate

    // ---- Per-block enable ---------------------------------------------------
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_en_i
            for (j = 0; j < 4; j = j + 1) begin : g_en_j
                assign blk_en[4*i+j] = nib_en[i] & nib_en[j];
            end
        end
    endgenerate

    // ---- Blocks (level-2 isolation) and weighted unsigned terms -----------
    wire [63:0]  a_g_flat;     // 16 x 4 bit  gated A nibble per block
    wire [63:0]  b_g_flat;     // 16 x 4 bit  gated B nibble per block
    wire [143:0] pblk_flat;    // 16 x 9 bit  block products (two's complement)
    wire [511:0] u_flat;       // 16 x 32 bit unsigned terms, already weighted

    generate
        for (i = 0; i < 4; i = i + 1) begin : g_i
            for (j = 0; j < 4; j = j + 1) begin : g_j
                // a_n[i] is already zero if nibble i is off; b_n[j] likewise.
                // The block is off if EITHER nibble is off, so AND with the
                // other nibble's enable (same value as AND with blk_en).
                wire [3:0] a_g  = a_n[4*i +: 4] & {4{nib_en[j]}};
                wire [3:0] b_g  = b_n[4*j +: 4] & {4{nib_en[i]}};
                wire       sa_g = nib_sgn[i] & blk_en[4*i+j];
                wire       sb_g = nib_sgn[j] & blk_en[4*i+j];
                wire [8:0] pb;

                bw4x4_mul u_blk (
                    .a (a_g),
                    .b (b_g),
                    .sa(sa_g),
                    .sb(sb_g),
                    .p (pb)
                );

                assign a_g_flat[4*(4*i+j) +: 4]  = a_g;
                assign b_g_flat[4*(4*i+j) +: 4]  = b_g;
                assign pblk_flat[9*(4*i+j) +: 9] = pb;

                // -256*s + low8 = {~s, low8} - 256 : unsigned 9-bit term, no
                // sign extension; the -256 offsets are folded into CORR.
                assign u_flat[32*(4*i+j) +: 32]  = {23'b0, ~pb[8], pb[7:0]} << (4*(i+j));
            end
        end
    endgenerate

    // ---- One multi-operand addition (carry-save tree + one final adder) ---
    assign p = CORR
             + u_flat[32*0 +: 32] 
             + u_flat[32*1 +: 32] 
             + u_flat[32*2 +: 32] 
             + u_flat[32*3 +: 32] 
             + u_flat[32*4 +: 32] 
             + u_flat[32*5 +: 32] 
             + u_flat[32*6 +: 32] 
             + u_flat[32*7 +: 32] 
             + u_flat[32*8 +: 32] 
             + u_flat[32*9 +: 32] 
             + u_flat[32*10 +: 32] 
             + u_flat[32*11 +: 32] 
             + u_flat[32*12 +: 32] 
             + u_flat[32*13 +: 32] 
             + u_flat[32*14 +: 32] 
             + u_flat[32*15 +: 32];

endmodule
