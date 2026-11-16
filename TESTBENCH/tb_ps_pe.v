//=============================================================================
// tb_ps_pe.v  --  self-checking testbench for ps_pe_top (one PE with control)
//
// For every mode: start -> 3000 operand pairs (first 4 are corner cases,
// afterwards about 25 % of the cycles have va_in and/or vb_in = 0 with random
// operands that must NOT be accumulated) -> drain the 3-stage pipeline ->
// compare acc_q with a running sum computed here with $signed arithmetic,
// masked to the mode's accumulator width (12 / 20 / 36 bit).
//
// Extra PE checks:
//   * forwarding outputs a_out / b_out / va_out / vb_out must equal the inputs
//     of the previous cycle (a_out / b_out only compared on the bits that
//     belong to the active mode, the other slices are clock gated)
//   * accumulator must not change while the pipeline is drained
//   * illegal mode (11): start ignored, mode_err pulses
//
// Run one mode to create one VCD (for Genus power analysis):
//     +MODE=0 -> ps_pe_int4.vcd
//     +MODE=1 -> ps_pe_int8.vcd
//     +MODE=2 -> ps_pe_int16.vcd
// With no +MODE argument the complete regression runs (no VCD).
// Vivado: Simulation Settings -> xsim.simulate.xsim.more_options -> -testplusarg MODE=1
//
// Files needed in the simulator:  ps_mult16_v3.v  bk_addacc361.v  ps_pe.v
//                                 tb_ps_pe.v
// VCD scope for Genus read_vcd :  tb_ps_pe/dut
//=============================================================================
`timescale 1ns/1ps

module tb_ps_pe;

    reg         clk    = 1'b0;
    reg         rst    = 1'b1;
    reg         start  = 1'b0;
    reg  [1:0]  mode   = 2'b00;
    reg  [15:0] a_in   = 16'd0;
    reg  [15:0] b_in   = 16'd0;
    reg         va_in  = 1'b0;
    reg         vb_in  = 1'b0;
    wire [15:0] a_out, b_out;
    wire        va_out, vb_out;
    wire [35:0] acc_q;
    wire        mode_err;

    integer errors = 0;

    ps_pe_top dut (
        .clk(clk), .rst(rst), .start(start), .mode(mode),
        .a_in(a_in), .b_in(b_in), .va_in(va_in), .vb_in(vb_in),
        .a_out(a_out), .b_out(b_out), .va_out(va_out), .vb_out(vb_out),
        .acc_q(acc_q), .mode_err(mode_err));

    always #2 clk = ~clk;          // clock speed is irrelevant for RTL simulation

    // signed product of the operands in the width of the mode
    function signed [63:0] sprod;
        input [1:0]  m;
        input [15:0] x;
        input [15:0] y;
        reg signed [3:0]  x4,  y4;
        reg signed [7:0]  x8,  y8;
        reg signed [15:0] x16, y16;
        begin
            case (m)
                2'd0:    begin x4  = x[3:0]; y4  = y[3:0]; sprod = x4  * y4;  end
                2'd1:    begin x8  = x[7:0]; y8  = y[7:0]; sprod = x8  * y8;  end
                default: begin x16 = x;      y16 = y;      sprod = x16 * y16; end
            endcase
        end
    endfunction

    task run_mode;
        input [1:0]   m;
        input integer nvec;
        integer           j;
        integer           rr;
        reg signed [63:0] expv;
        reg        [35:0] mask;
        reg        [15:0] opmask;
        reg        [15:0] ta, tb_;
        reg               va, vb;
        reg        [35:0] acc_hold;
        begin
            case (m)
                2'd0:    begin mask = 36'h000000FFF; opmask = 16'h000F; end
                2'd1:    begin mask = 36'h0000FFFFF; opmask = 16'h00FF; end
                default: begin mask = 36'hFFFFFFFFF; opmask = 16'hFFFF; end
            endcase
            expv = 64'sd0;

            @(negedge clk); mode = m; start = 1'b1; va_in = 1'b0; vb_in = 1'b0;
            @(negedge clk); start = 1'b0;              // accumulator clears in this cycle

            for (j = 0; j < nvec; j = j + 1) begin
                @(negedge clk);

                // forwarding check: a_out/b_out/va_out/vb_out show last cycle's inputs
                if (j > 0) begin
                    if (((a_out ^ a_in) & opmask) !== 16'd0 ||
                        ((b_out ^ b_in) & opmask) !== 16'd0 ||
                        va_out !== va_in || vb_out !== vb_in) begin
                        errors = errors + 1;
                        $display("FAIL fwd mode %0d vec %0d : a_out=%h b_out=%h va_out=%b vb_out=%b (expected a=%h b=%h va=%b vb=%b)",
                                 m, j, a_out, b_out, va_out, vb_out, a_in, b_in, va_in, vb_in);
                    end
                end

                case (j)
                    0:       begin ta = 16'h8000; tb_ = 16'h8000; end
                    1:       begin ta = 16'hFFFF; tb_ = 16'hFFFF; end
                    2:       begin ta = 16'h7FFF; tb_ = 16'h8000; end
                    3:       begin ta = 16'h0008; tb_ = 16'h0008; end
                    default: begin ta = $random;  tb_ = $random;  end
                endcase

                // validity: corner cases always valid; afterwards 75 % both valid,
                // 25 % at least one of va / vb is 0 (data must then be ignored)
                if (j < 4) begin
                    va = 1'b1; vb = 1'b1;
                end else begin
                    rr = $random & 15;
                    if      (rr < 12)  begin va = 1'b1; vb = 1'b1; end
                    else if (rr < 14)  begin va = 1'b0; vb = 1'b1; end
                    else if (rr == 14) begin va = 1'b1; vb = 1'b0; end
                    else               begin va = 1'b0; vb = 1'b0; end
                end

                a_in = ta; b_in = tb_; va_in = va; vb_in = vb;
                if (va && vb) expv = expv + sprod(m, ta, tb_);
            end

            // drain: 3 pipeline stages -> wait with invalid, random operands
            @(negedge clk); va_in = 1'b0; vb_in = 1'b0; a_in = $random; b_in = $random;
            repeat (3) begin
                @(negedge clk); a_in = $random; b_in = $random;
            end
            acc_hold = acc_q;
            repeat (2) begin
                @(negedge clk); a_in = $random; b_in = $random;     // must not change acc_q
                if (acc_q !== acc_hold) begin
                    errors = errors + 1;
                    $display("FAIL mode %0d : acc_q changed while no valid data", m);
                end
            end

            if ((acc_q & mask) !== (expv[35:0] & mask)) begin
                errors = errors + 1;
                $display("FAIL mode %0d : acc_q = %h  expected = %h", m,
                         acc_q & mask, expv[35:0] & mask);
            end else begin
                $display("PASS mode %0d : acc_q = %h", m, acc_q & mask);
            end
        end
    endtask

    integer selected_mode;

    initial begin
        // Run once per precision mode to create separate waveform files:
        //   +MODE=0 -> ps_pe_int4.vcd
        //   +MODE=1 -> ps_pe_int8.vcd
        //   +MODE=2 -> ps_pe_int16.vcd
        // With no +MODE argument, run the complete regression.
        if ($value$plusargs("MODE=%d", selected_mode)) begin
            case (selected_mode)
                0: begin
                    $dumpfile("ps_pe_int4.vcd");
                    $dumpvars(0, tb_ps_pe);
                end
                1: begin
                    $dumpfile("ps_pe_int8.vcd");
                    $dumpvars(0, tb_ps_pe);
                end
                2: begin
                    $dumpfile("ps_pe_int16.vcd");
                    $dumpvars(0, tb_ps_pe);
                end
                default: begin
                    $display("ERROR: MODE must be 0 (INT4), 1 (INT8), or 2 (INT16).");
                    $finish;
                end
            endcase
        end

        repeat (3) @(negedge clk);
        rst = 1'b0;
        @(negedge clk);

        if ($test$plusargs("MODE")) begin
            // A mode-specific run produces one VCD containing this mode's activity.
            run_mode(selected_mode[1:0], 3000);
        end else begin
            // Full regression (without a per-mode VCD).
            run_mode(2'd0, 3000);      // INT4
            run_mode(2'd1, 3000);      // INT8
            run_mode(2'd2, 3000);      // INT16
            run_mode(2'd0, 3000);      // INT16 -> INT4
            run_mode(2'd2, 3000);      // INT4  -> INT16
            run_mode(2'd1, 3000);      // INT16 -> INT8

            // Illegal mode: start is ignored, mode_err must pulse.
            @(negedge clk); mode = 2'b11; start = 1'b1;
            @(negedge clk); start = 1'b0;
            if (mode_err !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL illegal mode : mode_err did not pulse");
            end else begin
                $display("PASS illegal mode : mode_err pulsed");
            end
        end

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

endmodule
