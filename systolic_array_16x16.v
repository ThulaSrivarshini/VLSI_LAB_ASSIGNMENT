// LEVEL 2: Pure 16x16 systolic PE interconnect. No FSM and no clock gates here.
// A propagates LEFT -> RIGHT; B propagates TOP -> BOTTOM.
// Each PE retains its own 36-bit accumulator.
// Boundary validity is supplied independently for each row/column.
module systolic_array_16x16 (
    input clk, clk_mid, clk_hi,
    input reset, clear, en_mid, en_hi,
    input [255:0] a_left_flat,    // a_left_flat[16*i +:16] is row i
    input [255:0] b_top_flat,     // b_top_flat [16*j +:16] is column j
    input [15:0] va_left,        // valid for left boundary of each row
    input [15:0] vb_top,         // valid for top boundary of each column
    output [9215:0] c_flat      // C[i][j] = c_flat[36*(16*i+j)+:36]
);
    wire [15:0] a_link [0:15][0:16];
    wire [15:0] b_link [0:16][0:15];
    wire        va_link [0:15][0:16];
    wire        vb_link [0:16][0:15];

    genvar i,j;
    generate
      for (i=0; i<16; i=i+1) begin : BOUNDARY_A
        assign a_link[i][0] = a_left_flat[16*i +: 16];
        assign va_link[i][0] = va_left[i];
      end
      for (j=0; j<16; j=j+1) begin : BOUNDARY_B
        assign b_link[0][j] = b_top_flat[16*j +: 16];
        assign vb_link[0][j] = vb_top[j];
      end
      for (i=0; i<16; i=i+1) begin : PE_ROW
        for (j=0; j<16; j=j+1) begin : PE_COL
          ps_pe u_pe (
              .clk(clk), .clk_mid(clk_mid), .clk_hi(clk_hi),
              .reset(reset), .clear(clear),
              .en_mid(en_mid), .en_hi(en_hi),
              .a_in(a_link[i][j]), .b_in(b_link[i][j]),
              .va_in(va_link[i][j]), .vb_in(vb_link[i][j]),
              .a_out(a_link[i][j+1]), .b_out(b_link[i+1][j]),
              .va_out(va_link[i][j+1]), .vb_out(vb_link[i+1][j]),
              .acc_q(c_flat[36*(16*i+j) +: 36])
          );
        end
      end
    endgenerate
endmodule
