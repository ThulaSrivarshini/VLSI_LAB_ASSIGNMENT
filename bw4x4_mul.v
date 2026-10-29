//=============================================================================
// bw4x4_mul.v
// 4x4 Baugh-Wooley multiplier with run-time selectable operand signedness.
//
//   sa = 1 : a is a signed (two's complement) 4-bit value, else unsigned
//   sb = 1 : b is a signed (two's complement) 4-bit value, else unsigned

// Purely combinational, no latches, no tri-states.
//=============================================================================
`timescale 1ns/1ps

module bw4x4_mul (
    input  wire [3:0] a,
    input  wire [3:0] b,
    input  wire       sa,
    input  wire       sb,
    output wire [8:0] p
);

    // ---- Baugh-Wooley partial-product matrix, pp[4*i+j] = a[i] * b[j] ------
    wire [15:0] pp;

    genvar i, j;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_i
            for (j = 0; j < 4; j = j + 1) begin : g_j
                if (i == 3 && j < 3) begin : neg_a_msb      // a[3]*b[j]
                    assign pp[4*i+j] = (a[i] & b[j]) ^ sa;
                end else if (j == 3 && i < 3) begin : neg_b_msb // a[i]*b[3]
                    assign pp[4*i+j] = (a[i] & b[j]) ^ sb;
                end else if (i == 3 && j == 3) begin : msb_msb  // a[3]*b[3]
                    assign pp[4*i+j] = (a[i] & b[j]) ^ (sa ^ sb);
                end else begin : plain
                    assign pp[4*i+j] = a[i] & b[j];
                end
            end
        end
    endgenerate

    // ---- Rows of the array (row j = all partial products of b[j]) ----------
    wire [8:0] row0 = {5'b0, pp[12], pp[8],  pp[4],  pp[0]} << 0;
    wire [8:0] row1 = {5'b0, pp[13], pp[9],  pp[5],  pp[1]} << 1;
    wire [8:0] row2 = {5'b0, pp[14], pp[10], pp[6],  pp[2]} << 2;
    wire [8:0] row3 = {5'b0, pp[15], pp[11], pp[7],  pp[3]} << 3;

    // ---- Constant correction row (depends only on sa / sb) -----------------
    wire [8:0] corr_mag = (sa ? 9'd56 : 9'd0)
                        + (sb ? 9'd56 : 9'd0)
                        + ((sa ^ sb) ? 9'd64 : 9'd0);
    wire [8:0] corr     = 9'd0 - corr_mag;

    // ---- Final carry-propagate summation (mod 2^9) -------------------------
    assign p = row0 + row1 + row2 + row3 + corr;

endmodule
