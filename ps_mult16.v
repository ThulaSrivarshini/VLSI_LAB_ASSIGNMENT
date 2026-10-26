//=============================================================================
// ps_mult16.v
// Precision-scalable signed multiplier built from sixteen 4x4 Baugh-Wooley
// blocks, with operand isolation (data gating) per block.
//
//   a, b   : 16-bit operand ports (always 16 bits wide)
//   mode   : 2'b00 INT4 | 2'b01 INT8 | 2'b10 INT16 | 2'b11 reserved/illegal
//   en     : global enable; en=0 switches every block off
//   p      : signed product, sign-extended to 32 bits
//              INT4  : signed(a[3:0])  * signed(b[3:0])    (range +-2^6)
//              INT8  : signed(a[7:0])  * signed(b[7:0])    (range +-2^14)
//              INT16 : signed(a[15:0]) * signed(b[15:0])   (range +-2^30)
//   illegal: high in the reserved mode (p = 0, all blocks off)
//   blk_en : which 4x4 blocks are powered up, bit index = 4*i + j,
//            block (i,j) multiplies A nibble i with B nibble j, weight 2^(4(i+j))
//
// Active blocks per mode
//   INT4  : (0,0)                    -> 1 block
//   INT8  : (0..1, 0..1)             -> 4 blocks
//   INT16 : (0..3, 0..3)             -> 16 blocks
//
// Signedness of each nibble (only the TOP nibble of an operand is signed)
//   INT4: nibble 0   INT8: nibble 1   INT16: nibble 3
//
// Operand isolation
//   Every block gets its own AND-gated copy of the A nibble, B nibble and the
//   two sign flags.  A disabled block therefore sees constant zeros on all of
//   its inputs, so nothing inside it (partial products, adders) can toggle
//   no matter what is on the upper operand bits.  This is NOT output masking:
//   the arithmetic itself is held still.
//
// Purely combinational (registers belong to the surrounding PE).
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

    // ---- Mode decode (one-hot, qualified with en) --------------------------
    wire m_int4  = en & ~mode[1] & ~mode[0];
    wire m_int8  = en & ~mode[1] &  mode[0];
    wire m_int16 = en &  mode[1] & ~mode[0];
    assign illegal = mode[1] & mode[0];          // reserved 2'b11 never computes

    // ---- Per-nibble enable and sign flag -----------------------------------
    wire [3:0] nib_en  = { m_int16, m_int16, (m_int8 | m_int16), (m_int4 | m_int8 | m_int16) };
    wire [3:0] nib_sgn = { m_int16, 1'b0,    m_int8,             m_int4 };

    // ---- Per-block enable: block (i,j) on iff A nibble i and B nibble j on -
    genvar i, j;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_en_i
            for (j = 0; j < 4; j = j + 1) begin : g_en_j
                assign blk_en[4*i+j] = nib_en[i] & nib_en[j];
            end
        end
    endgenerate

    // ---- Gated operands, block outputs and shifted terms (flat buses) -----
    wire [63:0]  a_g_flat;     // 16 blocks x 4 bit  (gated A nibble)
    wire [63:0]  b_g_flat;     // 16 blocks x 4 bit  (gated B nibble)
    wire [143:0] pblk_flat;    // 16 blocks x 9 bit  (block products)
    wire [511:0] t_flat;       // 16 blocks x 32 bit (sign-extended, shifted)

    generate
        for (i = 0; i < 4; i = i + 1) begin : g_i
            for (j = 0; j < 4; j = j + 1) begin : g_j
                localparam K = 4*i + j;

                // operand isolation: AND every input with the block enable
                wire [3:0] a_g  = a[4*i +: 4] & {4{blk_en[K]}};
                wire [3:0] b_g  = b[4*j +: 4] & {4{blk_en[K]}};
                wire       sa_g = nib_sgn[i]  &  blk_en[K];
                wire       sb_g = nib_sgn[j]  &  blk_en[K];
                wire [8:0] pb;

                bw4x4_mul u_blk (
                    .a (a_g),
                    .b (b_g),
                    .sa(sa_g),
                    .sb(sb_g),
                    .p (pb)
                );

                assign a_g_flat[4*K +: 4]   = a_g;
                assign b_g_flat[4*K +: 4]   = b_g;
                assign pblk_flat[9*K +: 9]  = pb;

                // sign-extend (9-bit signed) and weight by 2^(4(i+j))
                assign t_flat[32*K +: 32]   = {{23{pb[8]}}, pb} << (4*(i+j));
            end
        end
    endgenerate

    // ---- Sum of the 16 weighted block products (row sums, then a small tree)
    wire [127:0] rs_flat;      // 4 row sums x 32 bit
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_rs
            assign rs_flat[32*i +: 32] = t_flat[32*(4*i+0) +: 32]
                                       + t_flat[32*(4*i+1) +: 32]
                                       + t_flat[32*(4*i+2) +: 32]
                                       + t_flat[32*(4*i+3) +: 32];
        end
    endgenerate

    assign p = (rs_flat[31:0]   + rs_flat[63:32])
             + (rs_flat[95:64]  + rs_flat[127:96]);

endmodule
