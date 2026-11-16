// =====================================================================
// ps_pe.v  --  ONE precision-scalable processing element, with gating
//
//   needs from ps_mult16_v3.v   : ps_mult16          (your multiplier)
//   needs from bk_addacc361.v   : bk_addacc36        (gated adder + accumulator)
//                                 aa_precision_ctrl, aa_clk_gate (used by the top)
//
//   Data flow, top to bottom (3 pipeline stages):
//
//   a_in, b_in, va_in, vb_in
//        |   stage 1: operand registers (3 clock slices) ---> a_out, b_out, va_out, vb_out
//        v                                                   (forwarded to the next PE)
//   ps_mult16   operand isolation inside, enabled only when both operands are valid
//        |   p[31:0]
//        v   stage 2: product register (3 clock slices: 8 / 8 / 16 bit)
//   ps_sext_iso  SIGN EXTENSION (8 muxes) + ISOLATION (AND gates)  ->  x0, x1, x2
//        |
//        v   stage 3: bk_addacc36 = a-side AND gates + carry gates + Brent-Kung adders
//                                   + accumulator slices on clk, clk_mid, clk_hi  -> acc_q
//
//   Enables (same names as in the adder part):  en_mid (INT8, INT16)    en_hi (INT16)
//   Clock slices: low slice on clk (always), mid slice on clk_mid, high slice on clk_hi
//
//   Latency: 3 clock cycles from a_in / b_in to the accumulator.
//   Operand format: INT4 in [3:0], INT8 in [7:0], INT16 in [15:0], two's complement;
//   bits above the mode width are ignored.
// =====================================================================

// ---------------------------------------------------------------------
// Sign extension + isolation of the product, ahead of the adder
//   Product bits that count per mode:  INT4 pr[7:0]   INT8 pr[15:0]   INT16 pr[31:0]
//   Output slices (adder inputs):      x0 = S0 [11:0]  x1 = S1 [19:12]  x2 = S2 [35:20]
//
//   Gates: 8 MUX2 (sign copies) + 21 AND2 (isolation: 8 on x1, 12 + 1 shared on x2)
//   Order: extend first (mux), isolate second (AND).
// ---------------------------------------------------------------------
module ps_sext_iso (
  input  [31:0] pr,
  input         en_mid,       // 1 in INT8, INT16   (INT4 = ~en_mid)
  input         en_hi,        // 1 in INT16
  output [11:0] x0,
  output [7:0]  x1,
  output [15:0] x2
);
  // ---- sign extension: 4 + 4 muxes ----
  wire [3:0] se0 = en_mid ? pr[11:8]  : {4{pr[7]}};    // S0 bits [11:8]: INT4 copies pr[7]
  wire [3:0] se1 = en_hi  ? pr[19:16] : {4{pr[15]}};   // S1 bits [19:16]: INT8 copies pr[15]

  // ---- isolation: AND gates ----
  assign x0 = {se0, pr[7:0]};                                 // S0 always on, no gate
  assign x1 = {se1, pr[15:12]} & {8{en_mid}};                 // S1 forced to 0 in INT4
  wire   sgn = pr[31] & en_hi;                                // one AND drives the 4 sign bits
  assign x2 = {{4{sgn}}, pr[31:20] & {12{en_hi}}};            // S2 forced to 0 in INT4, INT8
endmodule

// ---------------------------------------------------------------------
// PE core: no control logic inside (control and the two ICGs are shared by all PEs)
// ---------------------------------------------------------------------
module ps_pe (
  input         clk,
  input         clk_mid,
  input         clk_hi,
  input         clear,
  input         en_mid,
  input         en_hi,
  input  [15:0] a_in,
  input  [15:0] b_in,
  input         va_in,
  input         vb_in,
  output [15:0] a_out,
  output [15:0] b_out,
  output        va_out,
  output        vb_out,
  output [35:0] acc_q
);
  // ---------------- stage 1: operand registers (forwarding registers) ----------------
  reg [3:0] a_lo,  b_lo;       // bits [3:0]   always clocked
  reg [3:0] a_mid, b_mid;      // bits [7:4]   clk_mid  (INT8, INT16)
  reg [7:0] a_hi,  b_hi;       // bits [15:8]  clk_hi   (INT16)
  reg       va_r,  vb_r;

  always @(posedge clk) begin
    a_lo <= a_in[3:0];
    b_lo <= b_in[3:0];
    va_r <= va_in;
    vb_r <= vb_in;
  end
  always @(posedge clk_mid) begin
    a_mid <= a_in[7:4];
    b_mid <= b_in[7:4];
  end
  always @(posedge clk_hi) begin
    a_hi <= a_in[15:8];
    b_hi <= b_in[15:8];
  end

  wire [15:0] a_r = {a_hi, a_mid, a_lo};
  wire [15:0] b_r = {b_hi, b_mid, b_lo};
  assign a_out  = a_r;         // taken straight from the registers, never through a gate
  assign b_out  = b_r;
  assign va_out = va_r;
  assign vb_out = vb_r;

  // ---------------- multiplier ----------------
  wire        m_valid = va_r & vb_r;                // multiplier works only on valid data
  wire [1:0]  m_mode  = {en_hi, en_mid & ~en_hi};   // 00 INT4, 01 INT8, 10 INT16
  wire signed [31:0] p;

  ps_mult16 u_mult (
    .a(a_r), .b(b_r), .mode(m_mode), .en(m_valid),
    .p(p), .illegal(), .blk_en());

  // ---------------- stage 2: product register (3 clock slices) ----------------
  reg [7:0]  p0;               // pr[7:0]    always clocked
  reg [7:0]  p1;               // pr[15:8]   clk_mid
  reg [15:0] p2;               // pr[31:16]  clk_hi
  reg        acc_en_r;         // valid, aligned with the product register

  always @(posedge clk) begin
    p0       <= p[7:0];
    acc_en_r <= m_valid;
  end
  always @(posedge clk_mid) p1 <= p[15:8];
  always @(posedge clk_hi)  p2 <= p[31:16];

  wire [31:0] pr = {p2, p1, p0};

  // ---------------- sign extension + isolation ----------------
  wire [11:0] x0;
  wire [7:0]  x1;
  wire [15:0] x2;

  ps_sext_iso u_sext (
    .pr(pr), .en_mid(en_mid), .en_hi(en_hi),
    .x0(x0), .x1(x1), .x2(x2));

  // ---------------- stage 3: gated adder + accumulator ----------------
  // bk_addacc36 also gates its inputs; for b this repeats the AND gates above
  // (same function, the synthesis tool merges them). It adds the a-side gates and the carry gates.
  bk_addacc36 u_core (
    .clk(clk), .clk_mid(clk_mid), .clk_hi(clk_hi),
    .clear(clear), .acc_en(acc_en_r), .b({x2, x1, x0}),
    .en_mid(en_mid), .en_hi(en_hi),
    .acc_q(acc_q));
endmodule

// ---------------------------------------------------------------------
// PE with its own control: mode capture, enables and the two ICGs
// (in the full array this control block exists once and feeds all PEs)
// (Vivado DUT / Genus top)
//
// Run:  set mode, start = 1 for one cycle -> next cycle: clear
//       -> then va_in = vb_in = 1 with a_in, b_in every cycle.
// ---------------------------------------------------------------------
module ps_pe_top (
  input         clk,
  input         rst,
  input         start,
  input  [1:0]  mode,
  input  [15:0] a_in,
  input  [15:0] b_in,
  input         va_in,
  input         vb_in,
  output [15:0] a_out,
  output [15:0] b_out,
  output        va_out,
  output        vb_out,
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

  ps_pe u_pe (
    .clk(clk), .clk_mid(clk_mid), .clk_hi(clk_hi),
    .clear(clear), .en_mid(en_mid), .en_hi(en_hi),
    .a_in(a_in), .b_in(b_in), .va_in(va_in), .vb_in(vb_in),
    .a_out(a_out), .b_out(b_out), .va_out(va_out), .vb_out(vb_out),
    .acc_q(acc_q));
endmodule
