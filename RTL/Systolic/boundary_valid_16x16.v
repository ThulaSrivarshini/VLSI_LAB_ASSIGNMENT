// LEVEL 4A: Boundary validity for ALREADY-SKEWED streaming inputs.
// The external source supplies A[i][t-i], B[t-j][j] at boundary cycle t.
// Input valid must remain high for all t=0..30; no pausing/stalling.
module boundary_valid_16x16 (
    input feeding,
    input input_valid,
    input [5:0] cycle,
    output [15:0] va_left,
    output [15:0] vb_top
);
  genvar i;
  generate
    for(i=0;i<16;i=i+1) begin : G
      assign va_left[i] = feeding && input_valid &&
                          (cycle >= i) && (cycle < i+16);
      assign vb_top[i] = feeding && input_valid &&
                         (cycle >= i) && (cycle < i+16);
    end
  endgenerate
endmodule
