// ddr_behavioral_model.sv
// Author: Nimisha Deepak
// SIMULATION-ONLY behavioral model of an external DDR device sitting
// behind axi4_ddr_ctrl.sv's command/data interface. This module is
// intentionally NOT included in vivado/synth.tcl's file list - real
// designs never synthesize their DRAM, and this one shouldn't either.
//
// Models the one timing property that actually matters for verifying a
// memory controller against DDR-like behavior: reads are not
// combinational or single-cycle. A fixed CAS_LATENCY separates command
// acceptance from the first read-data beat, after which data streams out
// at one beat/cycle (matching typical DDR burst behavior at the
// controller-facing level). Writes are accepted immediately once posted
// (WRITE_LATENCY cycles to actually land in the backing array, which is
// invisible to the controller since writes are posted).

`timescale 1ns / 1ps

module ddr_behavioral_model #(
    parameter int AW           = 24,
    parameter int DW           = 32,
    parameter int LENW         = 8,
    parameter int MEM_WORDS    = 1 << 22,   // 4M words = 16MB = 2^AW bytes; 3x 4MB frame buffers fit with margin
    parameter int CAS_LATENCY  = 14,        // cycles from command accepted to first read beat
    parameter int WRITE_LATENCY = 4
) (
    input  logic              clk,
    input  logic              rst_n,

    input  logic               cmd_valid,
    output logic                cmd_ready,
    input  logic [AW-1:0]      cmd_addr,
    input  logic                cmd_write,
    input  logic [LENW-1:0]    cmd_len,

    input  logic               wdata_valid,
    output logic                wdata_ready,
    input  logic [DW-1:0]      wdata,
    input  logic [(DW/8)-1:0]  wstrb,
    input  logic                wdata_last,

    output logic                rdata_valid,
    input  logic                rdata_ready,
    output logic [DW-1:0]       rdata,
    output logic                 rdata_last
);

    localparam int ADDR_LSB = 2;

    logic [DW-1:0] mem [0:MEM_WORDS-1];

    // ---------------- Command acceptance ----------------
    // Only one transaction (read or write) in flight at a time, matching
    // axi4_ddr_ctrl.sv's single shared command channel.
    typedef enum logic [1:0] {CMD_IDLE, CMD_WRITE, CMD_READ_WAIT, CMD_READ_BURST} state_t;
    state_t state;

    logic [AW-1:0]   addr_r;
    logic [LENW-1:0] len_r;
    int              cas_cnt;

    assign cmd_ready = (state == CMD_IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= CMD_IDLE;
            addr_r   <= '0;
            len_r    <= '0;
            cas_cnt  <= 0;
        end else begin
            unique case (state)
                CMD_IDLE: begin
                    if (cmd_valid && cmd_ready) begin
                        addr_r <= cmd_addr;
                        len_r  <= cmd_len;
                        if (cmd_write) begin
                            state <= CMD_WRITE;
                        end else begin
                            cas_cnt <= CAS_LATENCY;
                            state   <= CMD_READ_WAIT;
                        end
                    end
                end

                CMD_WRITE: begin
                    if (wdata_valid && wdata_ready) begin
                        addr_r <= addr_r + (DW/8);
                        if (wdata_last)
                            state <= CMD_IDLE;
                    end
                end

                CMD_READ_WAIT: begin
                    if (cas_cnt == 0) begin
                        state <= CMD_READ_BURST;
                    end else begin
                        cas_cnt <= cas_cnt - 1;
                    end
                end

                CMD_READ_BURST: begin
                    // Completion is driven by the issue-side counter
                    // (read_beat_idx below) reaching the end and that
                    // final beat being accepted - not recomputed here
                    // from acceptance count, which previously let one
                    // extra spurious beat get issued in the same cycle
                    // the last real beat was accepted (this `state`
                    // hadn't transitioned yet, so the old
                    // `state==CMD_READ_BURST`-gated issue fired once
                    // more before the FSM caught up).
                    if (rdata_valid && rdata_ready && rdata_last)
                        state <= CMD_IDLE;
                end

                default: state <= CMD_IDLE;
            endcase
        end
    end

    assign wdata_ready = (state == CMD_WRITE);

    // ---- Write data: per-byte-enable, no read-modify-write. Not needed
    // for a sim-only module, but keeping the same clean template avoids
    // simulation/synthesis mismatches if this were ever repurposed. ----
    always_ff @(posedge clk) begin
        if (wdata_valid && wdata_ready) begin
            for (int bi = 0; bi < DW/8; bi++)
                if (wstrb[bi])
                    mem[addr_r[AW-1:ADDR_LSB]][bi*8 +: 8] <= wdata[bi*8 +: 8];
        end
    end

    // ---- Read data: registered, fixed CAS latency then 1 beat/cycle.
    // Issuing is gated by its own counter (read_beat_idx <= len_r), not
    // by `state`, so it can never fire one extra time relative to how
    // many beats were actually requested. ----
    logic rdata_valid_r;
    logic [DW-1:0] rdata_r;
    logic rdata_last_r;
    // One bit wider than len_r: a max-length burst (len_r == 2^LENW-1,
    // i.e. 256 beats) needs read_beat_idx to reach 256, which does not
    // fit in LENW bits - it would wrap to 0 and the "<= len_r" guard
    // below would pass again, issuing one spurious extra beat right
    // after the real last one (confirmed by tracing a 256-beat burst:
    // read_beat_idx wrapped 255 -> 0 and a 257th beat got issued).
    logic [LENW:0]   read_beat_idx;
    logic [AW-1:0]   read_cur_addr;

    wire read_issue = (state == CMD_READ_BURST) && (read_beat_idx <= len_r) && (!rdata_valid_r || rdata_ready);

    always_ff @(posedge clk) begin
        if (read_issue)
            rdata_r <= mem[read_cur_addr[AW-1:ADDR_LSB]];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_valid_r <= 1'b0;
            rdata_last_r  <= 1'b0;
            read_beat_idx <= '0;
            read_cur_addr <= '0;
        end else begin
            if (state == CMD_READ_WAIT && cas_cnt == 0) begin
                // Latch the burst's starting address/index exactly once,
                // as the FSM leaves CAS wait.
                read_beat_idx <= '0;
                read_cur_addr <= addr_r;
            end

            if (read_issue) begin
                rdata_valid_r <= 1'b1;
                rdata_last_r  <= (read_beat_idx == len_r);
                read_beat_idx <= read_beat_idx + 1'b1;
                read_cur_addr <= read_cur_addr + (DW/8);
            end else if (rdata_valid_r && rdata_ready) begin
                rdata_valid_r <= 1'b0;
            end
        end
    end

    assign rdata_valid = rdata_valid_r;
    assign rdata       = rdata_r;
    assign rdata_last  = rdata_last_r;

endmodule
