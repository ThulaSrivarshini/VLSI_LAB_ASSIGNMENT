`timescale 1ns/1ps
// Self-checking 256-result array test with skewed boundary operands.
// Tests INT4, INT8, INT16, mode changes, negative values, and reserved 11.
//
// Plusargs:
//   +mode=<0|1|2|3>  0=INT4, 1=INT8, 2=INT16, 3=illegal mode 11 check only
//   +kind=<0..4>     0=zeros, 1=identity*B, 2=min-neg, 3=alternating, 4=random
//   +novcd           disable VCD dump
// No +mode/+kind -> run everything (VCD: tensor16_all.vcd).
// +mode only -> all kinds of that mode, +kind only -> that kind in all modes.
// Single case -> VCD named tensor16_m<mode>_k<kind>.vcd
module tb_tensor_unit_16x16;
    reg clk=0;
    always #1.667 clk = ~clk; 
    reg reset=1, start=0, mode_1=0, mode_0=0, input_valid=0;
    wire busy, done;
    reg [255:0] a_left_flat=0, b_top_flat=0;
    wire [9215:0] c_flat;
    reg [15:0] A [0:255];
    reg [15:0] B [0:255];

    tensor_unit_16x16 dut (
        .clk(clk), .reset(reset), .start(start),
        .mode_1(mode_1), .mode_0(mode_0), .input_valid(input_valid),
        .busy(busy), .done(done),
        .a_left_flat(a_left_flat), .b_top_flat(b_top_flat), .c_flat(c_flat)
    );

    function integer signed_val(input [15:0] v, input [1:0] mode);
        begin
            case (mode)
              2'b00: signed_val=$signed(v[3:0]);
              2'b01: signed_val=$signed(v[7:0]);
              default: signed_val=$signed(v);
            endcase
        end
    endfunction

    task automatic prepare_matrices(input [1:0] mode, input integer kind);
        integer i,j;
        reg [15:0] minneg, maxpos;
        begin
            case (mode)
              2'b00: begin minneg=16'hfff8; maxpos=16'h0007; end
              2'b01: begin minneg=16'hff80; maxpos=16'h007f; end
              default: begin minneg=16'h8000; maxpos=16'h7fff; end
            endcase
            for (i=0;i<16;i=i+1) begin
              for (j=0;j<16;j=j+1) begin
                case(kind)
                  0: begin A[16*i+j]=0; B[16*i+j]=0; end
                  1: begin A[16*i+j]=(i==j)?16'd1:16'd0;
                           B[16*i+j]=i*3-j*2; end
                  2: begin A[16*i+j]=minneg; B[16*i+j]=minneg; end
                  3: begin A[16*i+j]=((i+j)%2) ? minneg : maxpos;
                           B[16*i+j]=((i*j)%2) ? maxpos : minneg; end
                  default: begin A[16*i+j]=$random; B[16*i+j]=$random; end
                endcase
              end
            end
        end
    endtask

    task automatic run_case(input [1:0] mode, input integer kind);
        integer t,i,j,k,errors;
        reg signed [63:0] expected, actual;
        begin
            prepare_matrices(mode,kind);
            @(negedge clk);
            start=1; mode_1=mode[1]; mode_0=mode[0];
            input_valid=0; a_left_flat=0; b_top_flat=0;
            @(posedge clk); #1;  // start accepted; controller moves to cycle 0
            if (!busy || done) $fatal(1,"No busy after accepted start");
            start=0;
            // Boundary t=0..30: A[i,t-i], B[t-j,j].
            for(t=0;t<31;t=t+1) begin
                @(negedge clk);
                input_valid=1;
                a_left_flat=0; b_top_flat=0;
                for(i=0;i<16;i=i+1)
                    if(t>=i && t<i+16)
                        a_left_flat[16*i +: 16] = A[16*i+(t-i)];
                for(j=0;j<16;j=j+1)
                    if(t>=j && t<j+16)
                        b_top_flat[16*j +: 16] = B[16*(t-j)+j];
                @(posedge clk); #1;
                if(done || !busy) $fatal(1,"Premature done/busy drop at feed t=%0d",t);
            end
            @(negedge clk);
            input_valid=0; a_left_flat=0; b_top_flat=0;
            for(t=31;t<=47;t=t+1) begin
                @(posedge clk); #1;
                if(t<47 && (done || !busy))
                    $fatal(1,"Premature done/busy drop at drain t=%0d",t);
            end
            if(!done || busy) $fatal(1,"No done on expected final cycle");

            errors=0;
            for(i=0;i<16;i=i+1) begin
              for(j=0;j<16;j=j+1) begin
                expected=0;
                for(k=0;k<16;k=k+1)
                    expected=expected + signed_val(A[16*i+k],mode)*
                                        signed_val(B[16*k+j],mode);
                actual=$signed(c_flat[36*(16*i+j) +: 36]);
                if(actual !== expected) begin
                    if(errors<8)
                        $display("MISMATCH mode=%b kind=%0d C[%0d][%0d] got=%0d expected=%0d",
                                 mode,kind,i,j,actual,expected);
                    errors=errors+1;
                end
              end
            end
            if(errors!=0) $fatal(1,"FAILED with %0d mismatches",errors);
            $display("ARRAY PASS mode=%b kind=%0d (256 results)", mode,kind);
            @(posedge clk); #1;
            if(done || busy) $fatal(1,"done not exactly one cycle");
        end
    endtask

    integer m, kind;
    integer sel_mode, sel_kind;
    reg     run_illegal;
    string  vcd_name;

    initial begin
        // ---- plusarg parsing ----
        sel_mode = -1;
        sel_kind = -1;
        if (!$value$plusargs("mode=%d", sel_mode)) sel_mode = -1;
        if (!$value$plusargs("kind=%d", sel_kind)) sel_kind = -1;

        run_illegal = (sel_mode == 3) || (sel_mode < 0 && sel_kind < 0);

        // ---- VCD dump (one file per run) ----
        if (!$test$plusargs("novcd")) begin
            if (sel_mode >= 0 && sel_kind >= 0)
                vcd_name = $sformatf("tensor16_m%0d_k%0d.vcd", sel_mode, sel_kind);
            else if (sel_mode >= 0)
                vcd_name = $sformatf("tensor16_m%0d.vcd", sel_mode);
            else if (sel_kind >= 0)
                vcd_name = $sformatf("tensor16_k%0d.vcd", sel_kind);
            else
                vcd_name = "tensor16_all.vcd";
            $display("Dumping waveforms to %s", vcd_name);
            $dumpfile(vcd_name);
            $dumpvars(0, tb_tensor_unit_16x16);
        end

        repeat(3) @(negedge clk);
        reset=0;

        // ---- selected array tests (modes 0..2 = INT4, INT8, INT16) ----
        for (m=0; m<3; m=m+1)
            for (kind=0; kind<5; kind=kind+1)
                if ((sel_mode < 0 || sel_mode == m) &&
                    (sel_kind < 0 || sel_kind == kind))
                    run_case(m[1:0], kind);

        // ---- illegal mode 11 check ----
        if (run_illegal) begin
            @(negedge clk);
            mode_1=1; mode_0=1; start=1;
            @(posedge clk); #1;
            if(busy || done) $fatal(1,"Illegal mode 11 was accepted");
            start=0;
            $display("PASS: illegal mode 11 ignored");
        end

        $display("PASS: all selected tests passed");
        $finish;
    end
endmodule