// axi4_dma_2d.v
// Author: Nimisha Deepak
// AXI4 master that performs a 2D block copy: NUM_ROWS rows of ROW_WORDS
// 32-bit words each, reading from src_addr (advancing by src_stride bytes
// per row) and writing to dst_addr (advancing by dst_stride bytes per
// row). Each row is one AR/R burst followed by one AW/W burst, so
// non-unit strides (e.g. copying a sub-rectangle out of a larger 2D
// buffer) work without the caller unrolling per-row transfers by hand.

`timescale 1ns / 1ps

module axi4_dma_2d #(
    parameter AW    = 24,
    parameter DW    = 32,
    parameter IDW   = 2,
    parameter LENW  = 8,
    parameter QOSW  = 4,
    parameter MYID  = 2,
    parameter MAX_ROW_WORDS = 16
) (
    input  wire              clk,
    input  wire              rst_n,

    // Control/status register interface (not AXI; a simple CDC-free
    // command port since the DMA lives in the same clock domain as the
    // rest of the subsystem)
    input  wire               dma_start,
    input  wire [AW-1:0]      dma_src_addr,
    input  wire [AW-1:0]      dma_dst_addr,
    input  wire [LENW-1:0]    dma_row_words,   // 1..MAX_ROW_WORDS
    input  wire [7:0]         dma_num_rows,
    input  wire [AW-1:0]      dma_src_stride,  // bytes added to src_addr per row
    input  wire [AW-1:0]      dma_dst_stride,  // bytes added to dst_addr per row
    input  wire [QOSW-1:0]    dma_qos,
    output reg                dma_busy,
    output reg                dma_done,        // one-cycle pulse

    // AXI4 master: write channel
    output reg                m_awvalid,
    input  wire                m_awready,
    output reg  [AW-1:0]       m_awaddr,
    output reg  [LENW-1:0]     m_awlen,
    output wire [2:0]          m_awsize,
    output wire [1:0]          m_awburst,
    output wire [QOSW-1:0]     m_awqos,
    output wire [IDW-1:0]      m_awid,

    output reg                 m_wvalid,
    input  wire                m_wready,
    output wire [DW-1:0]       m_wdata,
    output wire [(DW/8)-1:0]   m_wstrb,
    output reg                 m_wlast,

    input  wire                m_bvalid,
    output reg                 m_bready,
    input  wire [1:0]          m_bresp,
    input  wire [IDW-1:0]      m_bid,

    // AXI4 master: read channel
    output reg                 m_arvalid,
    input  wire                m_arready,
    output reg  [AW-1:0]       m_araddr,
    output reg  [LENW-1:0]     m_arlen,
    output wire [2:0]          m_arsize,
    output wire [1:0]          m_arburst,
    output wire [QOSW-1:0]     m_arqos,
    output wire [IDW-1:0]      m_arid,

    input  wire                m_rvalid,
    output reg                 m_rready,
    input  wire [DW-1:0]       m_rdata,
    input  wire [1:0]          m_rresp,
    input  wire                m_rlast,
    input  wire [IDW-1:0]      m_rid
);

    assign m_awsize  = 3'b010;   // 4 bytes/beat
    assign m_arsize  = 3'b010;
    assign m_awburst = 2'b01;    // INCR
    assign m_arburst = 2'b01;    // INCR
    assign m_awqos   = dma_qos;
    assign m_arqos   = dma_qos;
    assign m_awid    = MYID[IDW-1:0];
    assign m_arid    = MYID[IDW-1:0];
    assign m_wstrb   = {(DW/8){1'b1}};

    reg [DW-1:0] row_buf [0:MAX_ROW_WORDS-1];

    localparam S_IDLE     = 3'd0,
               S_RD_ADDR  = 3'd1,
               S_RD_DATA  = 3'd2,
               S_WR_ADDR  = 3'd3,
               S_WR_DATA  = 3'd4,
               S_WR_RESP  = 3'd5,
               S_NEXT_ROW = 3'd6;

    reg [2:0]          state;
    reg [AW-1:0]        cur_src, cur_dst;
    reg [7:0]           rows_left;
    reg [LENW-1:0]      row_words_r;
    reg [$clog2(MAX_ROW_WORDS+1)-1:0] beat_idx;

    // Single-row scratch buffer: read during S_RD_DATA (dynamic write
    // address = beat_idx), drained during S_WR_ADDR/S_WR_DATA (dynamic
    // read address). The array and its read-data output register live in
    // their own plain @(posedge clk) block with no async reset - a BRAM
    // primitive has neither an asynchronous clear nor a combinational
    // read port, and this is the exact template already proven to infer
    // real Block RAM elsewhere in this project (see fifo_sync.sv). The
    // original version of this file mixed a literal index (0) and a dynamic expression
    // (beat_idx+1) for the read address across two different FSM states
    // feeding directly into the FSM's own reset-bearing always block -
    // Vivado didn't recognize that as a memory at all and built it out
    // of ~8,192 individual flip-flops with a wide fanout decode network,
    // which real post-route timing analysis then found sitting on this
    // design's critical path.
    reg          mem_we;
    reg [$clog2(MAX_ROW_WORDS)-1:0] mem_waddr;
    // mem_we/mem_waddr are registered (set on the beat-accept cycle, not
    // consumed by the memory-port block until the NEXT cycle) - so the
    // data to write must be captured into a register at that same
    // accept cycle too, not read live from m_rdata at write-time: m_rdata
    // is a streaming bus that has already moved on to the NEXT beat by
    // the time the deferred write actually fires, which silently wrote
    // beat N+1's data into beat N's slot (found by tracing row_buf's
    // contents cycle-by-cycle: row_buf[0] held source word 1, not 0).
    reg [DW-1:0] mem_wdata_hold;
    reg [DW-1:0] mem_rdata_r;
    assign m_wdata = mem_rdata_r;

    // Read side is combinational-level, not an edge pulse: the FSM below
    // asserts m_wvalid the very same edge it enters S_WR_DATA (from
    // S_WR_ADDR), so word 0's read must already have SETTLED into
    // mem_rdata_r by then, not merely be issued on that edge (issuing it
    // there would leave m_wvalid=1 paired with a stale, one-cycle-old
    // mem_rdata_r for exactly one cycle - the same class of off-by-one
    // bug documented in ddr_behavioral_model.sv). Continuously
    // "prefetching" address 0 for every cycle spent in S_WR_ADDR
    // guarantees it's already settled by the time m_awready arrives,
    // since S_WR_ADDR is entered at least one full cycle before that can
    // happen (m_awvalid itself has to go 0->1 first).
    reg mem_re;
    reg [$clog2(MAX_ROW_WORDS)-1:0] mem_raddr;
    reg [$clog2(MAX_ROW_WORDS+1)-1:0] next_beat_idx;
    always @(*) begin
        next_beat_idx = beat_idx + 1'b1;
        mem_re    = 1'b0;
        mem_raddr = beat_idx[$clog2(MAX_ROW_WORDS)-1:0];
        if (state == S_WR_ADDR) begin
            mem_re    = 1'b1;
            mem_raddr = '0;
        end else if (state == S_WR_DATA && m_wvalid && m_wready && !m_wlast) begin
            mem_re    = 1'b1;
            mem_raddr = next_beat_idx[$clog2(MAX_ROW_WORDS)-1:0];
        end
    end

    always @(posedge clk) begin
        if (mem_we)
            row_buf[mem_waddr] <= mem_wdata_hold;
        if (mem_re)
            mem_rdata_r <= row_buf[mem_raddr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            dma_busy    <= 1'b0;
            dma_done    <= 1'b0;
            m_awvalid   <= 1'b0;
            m_wvalid    <= 1'b0;
            m_wlast     <= 1'b0;
            m_bready    <= 1'b0;
            m_arvalid   <= 1'b0;
            m_rready    <= 1'b0;
            beat_idx    <= 0;
            mem_we      <= 1'b0;
        end else begin
            dma_done <= 1'b0;
            mem_we   <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (dma_start) begin
                        dma_busy    <= 1'b1;
                        cur_src     <= dma_src_addr;
                        cur_dst     <= dma_dst_addr;
                        rows_left   <= dma_num_rows;
                        row_words_r <= dma_row_words;
                        state       <= S_RD_ADDR;
                    end
                end

                S_RD_ADDR: begin
                    m_arvalid <= 1'b1;
                    m_araddr  <= cur_src;
                    m_arlen   <= row_words_r - 1'b1;
                    if (m_arvalid && m_arready) begin
                        m_arvalid <= 1'b0;
                        m_rready  <= 1'b1;
                        beat_idx  <= 0;
                        state     <= S_RD_DATA;
                    end
                end

                S_RD_DATA: begin
                    if (m_rvalid && m_rready) begin
                        mem_we    <= 1'b1;
                        mem_waddr <= beat_idx[$clog2(MAX_ROW_WORDS)-1:0];
                        mem_wdata_hold <= m_rdata;
                        beat_idx <= beat_idx + 1'b1;
                        if (m_rlast) begin
                            m_rready <= 1'b0;
                            state    <= S_WR_ADDR;
                        end
                    end
                end

                S_WR_ADDR: begin
                    m_awvalid <= 1'b1;
                    m_awaddr  <= cur_dst;
                    m_awlen   <= row_words_r - 1'b1;
                    if (m_awvalid && m_awready) begin
                        m_awvalid <= 1'b0;
                        m_wvalid  <= 1'b1;
                        beat_idx  <= 0;
                        m_wlast   <= (row_words_r == 1);
                        state     <= S_WR_DATA;
                    end
                end

                S_WR_DATA: begin
                    if (m_wvalid && m_wready) begin
                        if (m_wlast) begin
                            m_wvalid <= 1'b0;
                            m_bready <= 1'b1;
                            state    <= S_WR_RESP;
                        end else begin
                            beat_idx  <= beat_idx + 1'b1;
                            m_wlast   <= (beat_idx + 2 == row_words_r);
                        end
                    end
                end

                S_WR_RESP: begin
                    if (m_bvalid && m_bready) begin
                        m_bready <= 1'b0;
                        state    <= S_NEXT_ROW;
                    end
                end

                S_NEXT_ROW: begin
                    cur_src   <= cur_src + dma_src_stride;
                    cur_dst   <= cur_dst + dma_dst_stride;
                    rows_left <= rows_left - 1'b1;
                    if (rows_left == 1) begin
                        dma_busy <= 1'b0;
                        dma_done <= 1'b1;
                        state    <= S_IDLE;
                    end else begin
                        state <= S_RD_ADDR;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
