// LEVEL 5: Assignment 2A streaming interface, with pre-skewed A/B boundaries.
// One global FSM and two ICGs control all 256 PEs.
// AFTER start (E0), present 31 consecutive boundary cycles at E1..E31.
// A[i][t-i] goes to row i, B[t-j][j] to column j for t=0..30.
// Reset is active-high synchronous; keep stable before active clock edge.
module tensor_unit_16x16 (
    input clk, reset, start, mode_1, mode_0, input_valid,
    input [255:0] a_left_flat,
    input [255:0] b_top_flat,
    output busy, done,
    output [9215:0] c_flat
);
    wire [1:0] mode_q;
    wire [5:0] cycle;
    wire clear, feeding, en_mid, en_hi;
    wire clk_mid, clk_hi;
    wire [15:0] va_left, vb_top;

    tensor_control_fsm u_ctrl (
        .clk(clk), .reset(reset), .start(start),
        .mode_1(mode_1), .mode_0(mode_0),
        .busy(busy), .done(done), .mode_q(mode_q), .cycle(cycle),
        .clear(clear), .feeding(feeding), .en_mid(en_mid), .en_hi(en_hi)
    );

    // en_mid/en_hi stop unnecessary clock toggling after reset/clear.
    // During reset/clear, BOTH gated clocks open to clear inactive slices.
    aa_clk_gate u_gate_mid (
        .clk(clk), .en(en_mid | reset | clear), .gclk(clk_mid));
    aa_clk_gate u_gate_hi (
        .clk(clk), .en(en_hi | reset | clear), .gclk(clk_hi));

    boundary_valid_16x16 u_valid (
        .feeding(feeding), .input_valid(input_valid), .cycle(cycle),
        .va_left(va_left), .vb_top(vb_top)
    );

    systolic_array_16x16 u_array (
        .clk(clk), .clk_mid(clk_mid), .clk_hi(clk_hi),
        .reset(reset), .clear(clear), .en_mid(en_mid), .en_hi(en_hi),
        .a_left_flat(a_left_flat), .b_top_flat(b_top_flat),
        .va_left(va_left), .vb_top(vb_top), .c_flat(c_flat)
    );
endmodule
