// axi4_ddr_ctrl.sv
// Author: Nimisha Deepak
// AXI4-to-DDR bridge/controller: SYNTHESIZED on-chip logic only. It
// terminates the single arbitrated AXI4 AW/W/B and AR/R channels,
// buffers write and read burst data in on-chip FIFOs sized to absorb one
// full max-length AXI4 burst (256 beats) each - this is what actually
// consumes the design's Block RAM - and drives a simple synchronous
// command/data interface toward an external DDR, modeled off-chip by
// ddr_behavioral_model.sv (simulation-only, never synthesized, matching
// how real designs never synthesize their DRAM). The FIFOs decouple the
// AXI-side burst cadence from the DDR-side command/data timing (a
// consumer that's briefly not ready doesn't have to stall the DDR
// model's fixed CAS-like read latency, and a producer can hand off a
// full write burst even if the DDR model isn't draining it yet).
//
// Writes are posted: BRESP is returned once the whole write burst has
// been captured into the write FIFO, not once the DDR model has actually
// drained and stored it - a standard, documented simplification (posted
// writes), not a hidden correctness gap.

`timescale 1ns / 1ps

module axi4_ddr_ctrl #(
    parameter int AW      = 24,
    parameter int DW      = 32,
    parameter int IDW     = 2,
    parameter int LENW    = 8,
    parameter int FIFO_DEPTH = 256   // words; covers one full max-length AXI4 burst
) (
    input  logic              clk,
    input  logic              rst_n,

    // ---- AXI4 slave port (from the arbitrated write/read arbiters) ----
    input  logic               s_awvalid,
    output logic                s_awready,
    input  logic [AW-1:0]      s_awaddr,
    input  logic [LENW-1:0]    s_awlen,
    input  logic [2:0]         s_awsize,
    input  logic [1:0]         s_awburst,
    input  logic [IDW-1:0]     s_awid,

    input  logic               s_wvalid,
    output logic                s_wready,
    input  logic [DW-1:0]      s_wdata,
    input  logic [(DW/8)-1:0]  s_wstrb,
    input  logic                s_wlast,

    output logic                s_bvalid,
    input  logic                s_bready,
    output logic  [1:0]        s_bresp,
    output logic  [IDW-1:0]    s_bid,

    input  logic                s_arvalid,
    output logic                s_arready,
    input  logic  [AW-1:0]     s_araddr,
    input  logic  [LENW-1:0]   s_arlen,
    input  logic  [2:0]        s_arsize,
    input  logic  [1:0]        s_arburst,
    input  logic  [IDW-1:0]    s_arid,

    output logic                s_rvalid,
    input  logic                s_rready,
    output logic  [DW-1:0]      s_rdata,
    output logic  [1:0]         s_rresp,
    output logic                 s_rlast,
    output logic  [IDW-1:0]      s_rid,

    // ---- Behavioral-DDR-facing command/data interface (off-chip model) ----
    output logic               ddr_cmd_valid,
    input  logic               ddr_cmd_ready,
    output logic [AW-1:0]      ddr_cmd_addr,
    output logic               ddr_cmd_write,
    output logic [LENW-1:0]    ddr_cmd_len,     // burst length - 1 (AXI convention)

    output logic               ddr_wdata_valid,
    input  logic                ddr_wdata_ready,
    output logic [DW-1:0]       ddr_wdata,
    output logic [(DW/8)-1:0]   ddr_wstrb,
    output logic                 ddr_wdata_last,

    input  logic                ddr_rdata_valid,
    output logic                 ddr_rdata_ready,
    input  logic [DW-1:0]       ddr_rdata,
    input  logic                 ddr_rdata_last
);

    localparam int WFIFO_W = DW + (DW/8) + 1;  // {last, strb, data}
    localparam int RFIFO_W = DW + 1;            // {last, data}

    // ==================== Write path ====================
    localparam logic [1:0] W_IDLE = 2'd0, W_CMD = 2'd1, W_DATA = 2'd2;
    logic [1:0]      wstate;
    logic [IDW-1:0]  wid_r;
    logic [AW-1:0]   aw_addr_hold;
    logic [LENW-1:0] aw_len_hold;

    logic wr_fifo_wr_valid, wr_fifo_wr_ready;
    logic [WFIFO_W-1:0] wr_fifo_wr_data;
    logic wr_fifo_rd_valid, wr_fifo_rd_ready;
    logic [WFIFO_W-1:0] wr_fifo_rd_data;

    assign wr_fifo_wr_valid = (wstate == W_DATA) && s_wvalid;
    assign wr_fifo_wr_data  = {s_wlast, s_wstrb, s_wdata};
    assign s_wready         = (wstate == W_DATA) && wr_fifo_wr_ready;

    fifo_sync #(.WIDTH(WFIFO_W), .DEPTH(FIFO_DEPTH)) u_wr_fifo (
        .clk(clk), .rst_n(rst_n),
        .wr_valid(wr_fifo_wr_valid), .wr_ready(wr_fifo_wr_ready), .wr_data(wr_fifo_wr_data),
        .rd_valid(wr_fifo_rd_valid), .rd_ready(wr_fifo_rd_ready), .rd_data(wr_fifo_rd_data)
    );

    assign ddr_wdata_valid = wr_fifo_rd_valid;
    assign wr_fifo_rd_ready = ddr_wdata_ready;
    assign {ddr_wdata_last, ddr_wstrb, ddr_wdata} = wr_fifo_rd_data;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wstate       <= W_IDLE;
            s_awready    <= 1'b0;
            s_bvalid     <= 1'b0;
            s_bresp      <= 2'b00;
            s_bid        <= '0;
            wid_r        <= '0;
            aw_addr_hold <= '0;
            aw_len_hold  <= '0;
        end else begin
            s_awready <= 1'b0;
            if (s_bvalid && s_bready) s_bvalid <= 1'b0;

            unique case (wstate)
                W_IDLE: begin
                    if (s_awvalid) begin
                        s_awready    <= 1'b1;
                        wid_r        <= s_awid;
                        aw_addr_hold <= s_awaddr;
                        aw_len_hold  <= s_awlen;
                        wstate       <= W_CMD;
                    end
                end
                W_CMD: begin
                    // Write commands take priority over a pending read
                    // command on the shared command channel (see rstate
                    // below) - a write's command phase is a single cycle,
                    // so this can't starve reads.
                    if (ddr_cmd_ready)
                        wstate <= W_DATA;
                end
                W_DATA: begin
                    if (s_wvalid && s_wready && s_wlast) begin
                        s_bvalid <= 1'b1;
                        s_bresp  <= 2'b00;
                        s_bid    <= wid_r;
                        wstate   <= W_IDLE;
                    end
                end
                default: wstate <= W_IDLE;
            endcase
        end
    end

    // ==================== Read path ====================
    localparam logic [1:0] R_IDLE = 2'd0, R_CMD = 2'd1, R_DATA = 2'd2;
    logic [1:0]      rstate;
    logic [IDW-1:0]  rid_r;
    logic [AW-1:0]   ar_addr_hold;
    logic [LENW-1:0] ar_len_hold;

    logic rd_fifo_wr_valid, rd_fifo_wr_ready;
    logic [RFIFO_W-1:0] rd_fifo_wr_data;
    logic rd_fifo_rd_valid, rd_fifo_rd_ready;
    logic [RFIFO_W-1:0] rd_fifo_rd_data;

    assign rd_fifo_wr_valid = ddr_rdata_valid;
    assign rd_fifo_wr_data  = {ddr_rdata_last, ddr_rdata};
    assign ddr_rdata_ready  = rd_fifo_wr_ready;

    fifo_sync #(.WIDTH(RFIFO_W), .DEPTH(FIFO_DEPTH)) u_rd_fifo (
        .clk(clk), .rst_n(rst_n),
        .wr_valid(rd_fifo_wr_valid), .wr_ready(rd_fifo_wr_ready), .wr_data(rd_fifo_wr_data),
        .rd_valid(rd_fifo_rd_valid), .rd_ready(rd_fifo_rd_ready), .rd_data(rd_fifo_rd_data)
    );

    assign s_rvalid         = rd_fifo_rd_valid;
    assign rd_fifo_rd_ready = s_rready;
    assign {s_rlast, s_rdata} = rd_fifo_rd_data;
    assign s_rresp          = 2'b00;
    assign s_rid             = rid_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rstate       <= R_IDLE;
            s_arready    <= 1'b0;
            rid_r        <= '0;
            ar_addr_hold <= '0;
            ar_len_hold  <= '0;
        end else begin
            s_arready <= 1'b0;
            unique case (rstate)
                R_IDLE: begin
                    // Don't accept a new AR while a write command is
                    // mid-issue on the shared command channel this cycle.
                    if (s_arvalid && wstate != W_CMD) begin
                        s_arready    <= 1'b1;
                        ar_addr_hold <= s_araddr;
                        ar_len_hold  <= s_arlen;
                        rid_r        <= s_arid;
                        rstate       <= R_CMD;
                    end
                end
                R_CMD: begin
                    if (ddr_cmd_ready && wstate != W_CMD)
                        rstate <= R_DATA;
                end
                R_DATA: begin
                    if (ddr_rdata_valid && ddr_rdata_ready && ddr_rdata_last)
                        rstate <= R_IDLE;
                end
                default: rstate <= R_IDLE;
            endcase
        end
    end

    // ---- Shared command channel: write has priority when both pending ----
    assign ddr_cmd_valid = (wstate == W_CMD) || (rstate == R_CMD && wstate != W_CMD);
    assign ddr_cmd_write = (wstate == W_CMD);
    assign ddr_cmd_addr  = (wstate == W_CMD) ? aw_addr_hold : ar_addr_hold;
    assign ddr_cmd_len   = (wstate == W_CMD) ? aw_len_hold  : ar_len_hold;

endmodule
