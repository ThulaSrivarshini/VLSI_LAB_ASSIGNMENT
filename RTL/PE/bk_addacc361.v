// =====================================================================
// bk_addacc36.v  --  COMBINED Brent-Kung adder + accumulator with precision gating
//                    (everything for this test is in THIS one file)
//
// Top module : bk_addacc36_top
//
// Modes (mode_1 mode_0), captured only when start = 1:
//   00 INT4  -> 12-bit adder + 12-bit accumulator active, all upper bits idle
//   01 INT8  -> 20-bit adder + 20-bit accumulator active
//   10 INT16 -> full 36-bit adder + 36-bit accumulator active
//   11       -> reserved: start is ignored, mode_err pulses
// Enables (same names as in bk_adder36.v):  en_mid (INT8, INT16)   en_hi (INT16)
//
// Gating:
//   - AND gates on the a and b inputs of the adder segments S1 [19:12] and S2 [35:20]
//   - AND gates on the carries S0->S1 (en_mid) and S1->S2 (en_hi)
//   - ICG clock gates (clk_mid, clk_hi) on the accumulator slices S1 and S2
//   - the b input register is sliced on the same gated clocks
//
// The helper modules have the prefix aa_ so that this file can sit in the same
// Vivado project as bk_adder36.v without module-name clashes.
//
// Plain Verilog-2001 (works with Vivado and with Genus "-language v2001").
// =====================================================================


// ---------------------------------------------------------------------
// 1. Brent-Kung adder (N bits, carry in / carry out)
// ---------------------------------------------------------------------
module aa_bk_add #(parameter N = 12) (
  input  [N-1:0] a,
  input  [N-1:0] b,
  input          cin,
  output [N-1:0] sum,
  output         cout
);
  // constant function: ceil(log2(value))
  function integer clog2;
    input [31:0] value;
    integer t;
    begin
      clog2 = 0;
      for (t = value - 1; t > 0; t = t >> 1)
        clog2 = clog2 + 1;
    end
  endfunction

  localparam L = clog2(N);

  reg [N-1:0] g, p, p0;
  integer k, d, i;

  always @* begin
    p0 = a ^ b;                    // half-sum, used for the final sum
    g  = a & b;                    // generate
    p  = p0;                       // propagate
    g[0] = g[0] | (p0[0] & cin);   // fold carry-in into bit 0

    // up-sweep: build groups of 2, 4, 8 ... bits
    for (k = 0; k < L; k = k + 1) begin
      d = 1 << k;
      for (i = 2*d - 1; i < N; i = i + 2*d) begin
        g[i] = g[i] | (p[i] & g[i-d]);
        p[i] = p[i] & p[i-d];
      end
    end

    // down-sweep: complete the remaining prefixes
    for (k = L - 1; k >= 0; k = k - 1) begin
      d = 1 << k;
      for (i = 3*d - 1; i < N; i = i + 2*d) begin
        g[i] = g[i] | (p[i] & g[i-d]);
        p[i] = p[i] & p[i-d];
      end
    end
  end

  // carry into bit i = prefix generate of bits [i-1:0]  (bit 0 uses cin)
  assign sum  = p0 ^ {g[N-2:0], cin};
  assign cout = g[N-1];
endmodule


// ---------------------------------------------------------------------
// 2. Gated 36-bit adder: 3 segments, AND gates on inputs and carries
// ---------------------------------------------------------------------
module aa_bk_gadd36 (
  input  [35:0] a,
  input  [35:0] b,
  input         en_mid,      // 1 in INT8 and INT16
  input         en_hi,       // 1 in INT16 only
  output [35:0] sum,
  output        cout         // carry out of the top ACTIVE segment (bit 11, 19 or 35)
);
  // input gates: bits of unused segments are forced to 0
  wire [11:0] a0 = a[11:0];                      // S0: never gated
  wire [11:0] b0 = b[11:0];
  wire [7:0]  a1 = a[19:12] & {8{en_mid}};
  wire [7:0]  b1 = b[19:12] & {8{en_mid}};
  wire [15:0] a2 = a[35:20] & {16{en_hi}};
  wire [15:0] b2 = b[35:20] & {16{en_hi}};

  wire c0, c1, c2;

  aa_bk_add #(.N(12)) u_s0 (.a(a0), .b(b0), .cin(1'b0),
                         .sum(sum[11:0]),  .cout(c0));

  wire c0g = c0 & en_mid;                        // carry gate: nothing enters S1 in INT4

  aa_bk_add #(.N(8))  u_s1 (.a(a1), .b(b1), .cin(c0g),
                         .sum(sum[19:12]), .cout(c1));

  wire c1g = c1 & en_hi;                         // carry gate: nothing enters S2 in INT4/INT8

  aa_bk_add #(.N(16)) u_s2 (.a(a2), .b(b2), .cin(c1g),
                         .sum(sum[35:20]), .cout(c2));

  assign cout = en_hi ? c2 : (en_mid ? c1 : c0);
endmodule


// ---------------------------------------------------------------------
// 3. Clock gate (ICG)
// ---------------------------------------------------------------------
// =====================================================================
// aa_clk_gate  --  clock gate (ICG): latch + AND, glitch-free
//
//   en   = 1 -> gclk follows clk
//   en   = 0 -> gclk stays low (registers on gclk stop, their clock pins do not toggle)
//
// To use the real library ICG cell instead of latch + AND:
//   1. find the cell:  grep -B3 clock_gating_integrated_cell <your .lib file>
//   2. edit ICG_CELL and its pin names below
//   3. read the RTL with the macro USE_LIB_ICG defined
// =====================================================================
module aa_clk_gate (
  input  clk,
  input  en,
  output gclk
);
`ifdef USE_LIB_ICG
  ICG_CELL u_icg (.CK(clk), .E(en), .TE(1'b0), .Q(gclk));   // EDIT cell and pin names
`else
  reg en_l;
  // synopsys translate_off
  initial en_l = 1'b0;                 // simulation only: avoids X at time 0
  // synopsys translate_on
  always @(clk or en)
    if (!clk) en_l = en;               // latch: transparent while clk is low
  assign gclk = clk & en_l;            // continuous assign keeps simulation race-free
`endif
endmodule


// ---------------------------------------------------------------------
// 4. Mode capture and enables
// ---------------------------------------------------------------------
// =====================================================================
// aa_precision_ctrl  --  mode capture and enable generation
//
//   mode is captured ONLY when start = 1, so it cannot change during a run.
//   mode 00 INT4 | 01 INT8 | 10 INT16 | 11 reserved (start ignored, mode_err = 1)
//
//   en_mid : 1 in INT8 and INT16      (same name as in bk_adder36.v)
//   en_hi  : 1 in INT16 only          (same name as in bk_adder36.v)
//   clear  : 1 clock cycle AFTER a legal start (and during reset)
//   cg_en_mid / cg_en_hi : enables of the two clock gates (ICG).
//            They are forced on during reset, so every slice is cleared once.
// =====================================================================
module aa_precision_ctrl (
  input            clk,
  input            rst,            // synchronous, active high
  input            start,          // 1 clock cycle, mode must be valid
  input      [1:0] mode,
  output reg       en_mid,
  output reg       en_hi,
  output           clear,
  output           cg_en_mid,
  output           cg_en_hi,
  output reg       mode_err
);
  reg  start_q;
  wire legal = ~(mode[1] & mode[0]);       // 11 is reserved

  always @(posedge clk) begin
    if (rst) begin
      en_mid   <= 1'b0;                    // reset state = INT4
      en_hi    <= 1'b0;
      start_q  <= 1'b0;
      mode_err <= 1'b0;
    end else begin
      start_q  <= start & legal;
      mode_err <= start & ~legal;
      if (start & legal) begin
        en_mid <= mode[1] ^ mode[0];       // INT8 or INT16
        en_hi  <= mode[1] & ~mode[0];      // INT16
      end
    end
  end

  assign clear     = start_q | rst;
  assign cg_en_mid = en_mid  | rst;
  assign cg_en_hi  = en_hi   | rst;
endmodule


// ---------------------------------------------------------------------
// 5. 36-bit accumulator, three clock slices
// ---------------------------------------------------------------------
// =====================================================================
// aa_acc36  --  36-bit accumulator register in three clock slices
//
//   S0 [11:0]   clock clk       always running      (INT4, INT8, INT16)
//   S1 [19:12]  clock clk_mid   gated clock         (INT8, INT16)
//   S2 [35:20]  clock clk_hi    gated clock         (INT16 only)
//
// clk_mid / clk_hi come from two clock gates (ICG), shared by all PEs.
// A stopped slice keeps its old value and its clock pin does not toggle.
//   clear  : synchronous clear (only slices whose clock is running are cleared)
//   acc_en : per-PE enable (data valid): stops accumulation during fill / drain
// =====================================================================
module aa_acc36 (
  input         clk,
  input         clk_mid,
  input         clk_hi,
  input         clear,
  input         acc_en,
  input  [35:0] d,
  output [35:0] q
);
  reg [11:0] q0;
  reg [7:0]  q1;
  reg [15:0] q2;

  always @(posedge clk)
    if (clear)       q0 <= 12'd0;
    else if (acc_en) q0 <= d[11:0];

  always @(posedge clk_mid)
    if (clear)       q1 <= 8'd0;
    else if (acc_en) q1 <= d[19:12];

  always @(posedge clk_hi)
    if (clear)       q2 <= 16'd0;
    else if (acc_en) q2 <= d[35:20];

  assign q = {q2, q1, q0};
endmodule


// ---------------------------------------------------------------------
// 6. Adder + accumulator core, and the top (Vivado DUT / Genus top)
// ---------------------------------------------------------------------
// =====================================================================
// bk_addacc36.v  --  COMBINED  Brent-Kung adder + accumulator, with gating
//
//   needs from bk_adder36.v :  aa_bk_gadd36   (gated Brent-Kung 36-bit adder)
//   needs from this folder  :  aa_acc36, aa_precision_ctrl, aa_clk_gate
//
//        b (36 bit) ----+
//                       v
//   acc_q ----------> aa_bk_gadd36 --> sum --> aa_acc36 --> acc_q      (feedback)
//
//   Gating, by mode (INT4 = 12-bit adder + 12-bit accumulator active, rest idle):
//     1. AND gates on the a and b inputs of S1 / S2       (inside aa_bk_gadd36)
//     2. AND gates on the carries S0->S1 and S1->S2        (inside aa_bk_gadd36)
//     3. ICG clock gates on the accumulator slices S1 / S2 (clk_mid / clk_hi)
//     4. the b input register is sliced on the same gated clocks, so its
//        upper bits do not toggle in INT4 / INT8 either
//
//   b must already be a signed number of the mode's width (12 / 20 / 36 bits,
//   sign-extended); bits above the mode width are ignored by the AND gates.
// =====================================================================

// ---------------------------------------------------------------------
// core: adder + accumulator (what one PE would contain)
// ---------------------------------------------------------------------
module bk_addacc36 (
  input         clk,
  input         clk_mid,
  input         clk_hi,
  input         clear,
  input         acc_en,
  input  [35:0] b,
  input         en_mid,
  input         en_hi,
  output [35:0] acc_q
);
  wire [35:0] sum;
  wire        cout_unused;

  aa_bk_gadd36 u_add (
    .a(acc_q), .b(b),
    .en_mid(en_mid), .en_hi(en_hi),
    .sum(sum), .cout(cout_unused));

  aa_acc36 u_acc (
    .clk(clk), .clk_mid(clk_mid), .clk_hi(clk_hi),
    .clear(clear), .acc_en(acc_en),
    .d(sum), .q(acc_q));
endmodule

// ---------------------------------------------------------------------
// top: control + 2 ICGs + input registers + core   (Vivado DUT / Genus top)
//
// Run:  set mode, start = 1 for one cycle -> next cycle: clear
//       -> then acc_en = 1 and b every cycle.
//       Latency: 2 clock cycles from b to the accumulator.
// ---------------------------------------------------------------------
module bk_addacc36_top (
  input         clk,
  input         rst,
  input         start,
  input  [1:0]  mode,
  input         acc_en,
  input  [35:0] b,
  output [35:0] acc_q,
  output        mode_err
);
  wire en_mid, en_hi, clear, cg_en_mid, cg_en_hi, clk_mid, clk_hi;

  aa_precision_ctrl u_ctrl (
    .clk(clk), .rst(rst), .start(start), .mode(mode),
    .en_mid(en_mid), .en_hi(en_hi), .clear(clear),
    .cg_en_mid(cg_en_mid), .cg_en_hi(cg_en_hi), .mode_err(mode_err));

  aa_clk_gate u_cg_mid (.clk(clk), .en(cg_en_mid), .gclk(clk_mid));
  aa_clk_gate u_cg_hi  (.clk(clk), .en(cg_en_hi),  .gclk(clk_hi));

  // input registers: b in three clock slices, acc_en on the normal clock
  reg [11:0] b0_r;
  reg [7:0]  b1_r;
  reg [15:0] b2_r;
  reg        acc_en_r;

  always @(posedge clk) begin
    b0_r     <= b[11:0];
    acc_en_r <= acc_en;
  end
  always @(posedge clk_mid) b1_r <= b[19:12];
  always @(posedge clk_hi)  b2_r <= b[35:20];

  wire [35:0] b_r = {b2_r, b1_r, b0_r};

  bk_addacc36 u_core (
    .clk(clk), .clk_mid(clk_mid), .clk_hi(clk_hi),
    .clear(clear), .acc_en(acc_en_r), .b(b_r),
    .en_mid(en_mid), .en_hi(en_hi),
    .acc_q(acc_q));
endmodule
