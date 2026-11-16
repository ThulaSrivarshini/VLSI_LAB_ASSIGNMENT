// One global controller for all 256 PEs.
// FSM inputs: clk/reset/start/mode_1/mode_0. Top-level input_valid is handled by the array boundary logic.
// The accepted start edge is E0. Boundary cycle t=0 is at E1.
// Boundary operands are supplied for 31 clocks, t=0..30.
// Last PE(15,15), k=15 accumulates at E48, coincident with done.
// The input stream must be continuous (input_valid high on t=0..30).
// Reserved mode 2'b11 is ignored. New start is ignored while busy.
module tensor_control_fsm (
    input             clk,
    input             reset,
    input             start,
    input             mode_1,
    input             mode_0,
    output reg        busy,
    output reg        done,
    output reg [1:0]  mode_q,
    output reg [5:0]  cycle,
    output            clear,
    output            feeding,
    output            en_mid,
    output            en_hi
);
    reg running;
    wire legal_start = (mode_1 & mode_0) == 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            running <= 1'b0;
            busy    <= 1'b0;
            done    <= 1'b0;
            mode_q  <= 2'b00;
            cycle   <= 6'd0;
        end else begin
            done <= 1'b0;
            if (!running) begin
                busy <= 1'b0;
                if (start && legal_start) begin
                    running <= 1'b1;
                    busy    <= 1'b1;
                    mode_q  <= {mode_1, mode_0};
                    cycle   <= 6'd0;
                end
            end else if (cycle == 6'd47) begin
                // At this rising edge, the last PE's accumulator also updates.
                running <= 1'b0;
                busy    <= 1'b0;
                done    <= 1'b1;
                cycle   <= 6'd0;
            end else begin
                cycle <= cycle + 6'd1;
            end
        end
    end

    assign clear   = running && (cycle == 6'd0);
    assign feeding = running && (cycle <= 6'd30);
    assign en_mid  = (mode_q == 2'b01) || (mode_q == 2'b10);
    assign en_hi   = (mode_q == 2'b10);
endmodule
