//=============================================================================
// tb_ps_mult16_act.v - stimulus generator for Genus switching-activity power.
// One run = one mode, selected with +MODE=0/1/2 (INT4/INT8/INT16).
// The SAME random 16-bit a/b stream (fixed seed) is used in every mode, so the
// only difference between the three VCDs is the mode.  Writes ps_mult16_m<MODE>.vcd
// Icarus : vvp sim_act +MODE=1        Vivado xsim: xsim ... -testplusarg MODE=1
//=============================================================================
`timescale 1ns/1ps
module tb_ps_mult16_act;
    reg  [15:0]        a, b;
    reg  [1:0]         mode;
    reg                en;
    wire signed [31:0] p;
    wire               illegal;
    wire [15:0]        blk_en;

    ps_mult16 dut (.a(a), .b(b), .mode(mode), .en(en),
                   .p(p), .illegal(illegal), .blk_en(blk_en));

    integer m, n, seed;
    reg [31:0] rr;
    reg [8*32-1:0] fname;

    initial begin
        if (!$value$plusargs("MODE=%d", m)) m = 2;
        $sformat(fname, "ps_mult16_m%0d.vcd", m);
        $dumpfile(fname);
        $dumpvars(0, tb_ps_mult16_act);

        seed = 32'h1234_ABCD;
        mode = m[1:0];
        en   = 1'b1;
        a = 0; b = 0;
        #10;
        for (n = 0; n < 20000; n = n + 1) begin
            rr = $random(seed);
            a = rr[15:0];
            b = rr[31:16];
            #3.333;                       // one 300 MHz cycle per vector
        end
        $finish;
    end
endmodule
