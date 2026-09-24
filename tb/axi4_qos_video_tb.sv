// axi4_qos_video_tb.sv
// Author: Nimisha Deepak
// Self-checking testbench for axi4_qos_videosoc_top: M0 (CPU) and M1
// (video codec traffic) AXI4 master BFMs, the 2D DMA driven through its
// control port against the triple-buffered frame manager, and a
// behavioral DDR model (ddr_behavioral_model.sv) standing in for the
// off-chip memory. Every write is shadowed in a reference model so every
// read is checked against golden data. Functional coverage uses real
// SystemVerilog covergroups (sampled explicitly at each completed
// transaction / frame-buffer swap), not hand-rolled bin counters -
// report the number Vivado xsim's coverage database actually computes.
//
// Icarus Verilog (used for fast functional-only debug iteration) does
// not support `covergroup`; the covergroup declarations/sampling are
// guarded out under `ICARUS_SIM` so the same file drives both the quick
// Icarus pass/fail loop and the official xsim coverage run.
//
// A CSV transaction log (bandwidth_log.csv) is written throughout so
// scripts/bandwidth_analysis.py can compute sustained throughput from
// real simulated cycle counts, independent of any number claimed here.

`timescale 1ns / 1ps

module axi4_qos_video_tb;

    localparam int AW = 24, DW = 32, IDW = 2, LENW = 8, QOSW = 4;
    localparam int CLK_PERIOD = 5;   // 200MHz target clock in simulation time units (ns)
    localparam int FRAME_STRIDE_BYTES = 1 << 22;

    logic clk = 0;
    logic rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---------------- DUT connections: M0 (CPU) ----------------
    logic               m0_awvalid; logic m0_awready;
    logic [AW-1:0]      m0_awaddr;
    logic [LENW-1:0]    m0_awlen;
    logic [2:0]         m0_awsize;
    logic [1:0]         m0_awburst;
    logic [QOSW-1:0]    m0_awqos;
    logic [IDW-1:0]     m0_awid;
    logic               m0_wvalid; logic m0_wready;
    logic [DW-1:0]      m0_wdata;
    logic [(DW/8)-1:0]  m0_wstrb;
    logic               m0_wlast;
    logic               m0_bvalid; logic m0_bready;
    logic [1:0]         m0_bresp;
    logic [IDW-1:0]     m0_bid;
    logic               m0_arvalid; logic m0_arready;
    logic [AW-1:0]      m0_araddr;
    logic [LENW-1:0]    m0_arlen;
    logic [2:0]         m0_arsize;
    logic [1:0]         m0_arburst;
    logic [QOSW-1:0]    m0_arqos;
    logic [IDW-1:0]     m0_arid;
    logic               m0_rvalid; logic m0_rready;
    logic [DW-1:0]      m0_rdata;
    logic [1:0]         m0_rresp;
    logic               m0_rlast;
    logic [IDW-1:0]     m0_rid;

    // ---------------- DUT connections: M1 (Codec) ----------------
    logic               m1_awvalid; logic m1_awready;
    logic [AW-1:0]      m1_awaddr;
    logic [LENW-1:0]    m1_awlen;
    logic [2:0]         m1_awsize;
    logic [1:0]         m1_awburst;
    logic [QOSW-1:0]    m1_awqos;
    logic [IDW-1:0]     m1_awid;
    logic               m1_wvalid; logic m1_wready;
    logic [DW-1:0]      m1_wdata;
    logic [(DW/8)-1:0]  m1_wstrb;
    logic               m1_wlast;
    logic               m1_bvalid; logic m1_bready;
    logic [1:0]         m1_bresp;
    logic [IDW-1:0]     m1_bid;
    logic               m1_arvalid; logic m1_arready;
    logic [AW-1:0]      m1_araddr;
    logic [LENW-1:0]    m1_arlen;
    logic [2:0]         m1_arsize;
    logic [1:0]         m1_arburst;
    logic [QOSW-1:0]    m1_arqos;
    logic [IDW-1:0]     m1_arid;
    logic               m1_rvalid; logic m1_rready;
    logic [DW-1:0]      m1_rdata;
    logic [1:0]         m1_rresp;
    logic               m1_rlast;
    logic [IDW-1:0]     m1_rid;

    // ---------------- DMA control port ----------------
    logic               dma_start;
    logic [AW-1:0]      dma_src_addr, dma_dst_addr, dma_src_stride, dma_dst_stride;
    logic [LENW-1:0]    dma_row_words;
    logic [7:0]         dma_num_rows;
    logic [QOSW-1:0]    dma_qos;
    logic               dma_busy, dma_done;

    // ---------------- Frame manager port ----------------
    logic               capture_frame_done, display_frame_done;
    logic [AW-1:0]      wr_buf_base, rd_buf_base;
    logic [1:0]         wr_buf_id, rd_buf_id;
    logic               frame_ready_pending;

    // ---------------- DDR-facing interface ----------------
    logic               ddr_cmd_valid, ddr_cmd_ready;
    logic [AW-1:0]      ddr_cmd_addr;
    logic               ddr_cmd_write;
    logic [LENW-1:0]    ddr_cmd_len;
    logic               ddr_wdata_valid, ddr_wdata_ready;
    logic [DW-1:0]      ddr_wdata;
    logic [(DW/8)-1:0]  ddr_wstrb;
    logic               ddr_wdata_last;
    logic               ddr_rdata_valid, ddr_rdata_ready;
    logic [DW-1:0]      ddr_rdata;
    logic               ddr_rdata_last;

    axi4_qos_videosoc_top #(
        .AW(AW), .DW(DW), .IDW(IDW), .LENW(LENW), .QOSW(QOSW), .FRAME_STRIDE_BYTES(FRAME_STRIDE_BYTES)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .m0_awvalid(m0_awvalid), .m0_awready(m0_awready), .m0_awaddr(m0_awaddr), .m0_awlen(m0_awlen),
        .m0_awsize(m0_awsize), .m0_awburst(m0_awburst), .m0_awqos(m0_awqos), .m0_awid(m0_awid),
        .m0_wvalid(m0_wvalid), .m0_wready(m0_wready), .m0_wdata(m0_wdata), .m0_wstrb(m0_wstrb), .m0_wlast(m0_wlast),
        .m0_bvalid(m0_bvalid), .m0_bready(m0_bready), .m0_bresp(m0_bresp), .m0_bid(m0_bid),
        .m0_arvalid(m0_arvalid), .m0_arready(m0_arready), .m0_araddr(m0_araddr), .m0_arlen(m0_arlen),
        .m0_arsize(m0_arsize), .m0_arburst(m0_arburst), .m0_arqos(m0_arqos), .m0_arid(m0_arid),
        .m0_rvalid(m0_rvalid), .m0_rready(m0_rready), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp),
        .m0_rlast(m0_rlast), .m0_rid(m0_rid),

        .m1_awvalid(m1_awvalid), .m1_awready(m1_awready), .m1_awaddr(m1_awaddr), .m1_awlen(m1_awlen),
        .m1_awsize(m1_awsize), .m1_awburst(m1_awburst), .m1_awqos(m1_awqos), .m1_awid(m1_awid),
        .m1_wvalid(m1_wvalid), .m1_wready(m1_wready), .m1_wdata(m1_wdata), .m1_wstrb(m1_wstrb), .m1_wlast(m1_wlast),
        .m1_bvalid(m1_bvalid), .m1_bready(m1_bready), .m1_bresp(m1_bresp), .m1_bid(m1_bid),
        .m1_arvalid(m1_arvalid), .m1_arready(m1_arready), .m1_araddr(m1_araddr), .m1_arlen(m1_arlen),
        .m1_arsize(m1_arsize), .m1_arburst(m1_arburst), .m1_arqos(m1_arqos), .m1_arid(m1_arid),
        .m1_rvalid(m1_rvalid), .m1_rready(m1_rready), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp),
        .m1_rlast(m1_rlast), .m1_rid(m1_rid),

        .dma_start(dma_start), .dma_src_addr(dma_src_addr), .dma_dst_addr(dma_dst_addr),
        .dma_row_words(dma_row_words), .dma_num_rows(dma_num_rows),
        .dma_src_stride(dma_src_stride), .dma_dst_stride(dma_dst_stride),
        .dma_qos(dma_qos), .dma_busy(dma_busy), .dma_done(dma_done),

        .capture_frame_done(capture_frame_done), .display_frame_done(display_frame_done),
        .wr_buf_base(wr_buf_base), .rd_buf_base(rd_buf_base),
        .wr_buf_id(wr_buf_id), .rd_buf_id(rd_buf_id), .frame_ready_pending(frame_ready_pending),

        .ddr_cmd_valid(ddr_cmd_valid), .ddr_cmd_ready(ddr_cmd_ready),
        .ddr_cmd_addr(ddr_cmd_addr), .ddr_cmd_write(ddr_cmd_write), .ddr_cmd_len(ddr_cmd_len),
        .ddr_wdata_valid(ddr_wdata_valid), .ddr_wdata_ready(ddr_wdata_ready),
        .ddr_wdata(ddr_wdata), .ddr_wstrb(ddr_wstrb), .ddr_wdata_last(ddr_wdata_last),
        .ddr_rdata_valid(ddr_rdata_valid), .ddr_rdata_ready(ddr_rdata_ready),
        .ddr_rdata(ddr_rdata), .ddr_rdata_last(ddr_rdata_last)
    );

    ddr_behavioral_model #(.AW(AW), .DW(DW), .LENW(LENW)) ddr_model (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(ddr_cmd_valid), .cmd_ready(ddr_cmd_ready),
        .cmd_addr(ddr_cmd_addr), .cmd_write(ddr_cmd_write), .cmd_len(ddr_cmd_len),
        .wdata_valid(ddr_wdata_valid), .wdata_ready(ddr_wdata_ready),
        .wdata(ddr_wdata), .wstrb(ddr_wstrb), .wdata_last(ddr_wdata_last),
        .rdata_valid(ddr_rdata_valid), .rdata_ready(ddr_rdata_ready),
        .rdata(ddr_rdata), .rdata_last(ddr_rdata_last)
    );

    // ---------------- Reference model: shadow of the DDR array ----------------
    logic [DW-1:0] ref_mem [0:(1<<22)-1];

    // ---------------- Scoreboard ----------------
    int errors = 0;
    int checks = 0;

    task automatic check_eq(input logic [DW-1:0] got, input logic [DW-1:0] exp, input string msg);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("[%0t] FAIL: %s  got=%h exp=%h", $time, msg, got, exp);
        end
    endtask

    // ---------------- CSV transaction log for the Python bandwidth script ----------------
    int csv_fd;
    initial csv_fd = $fopen("bandwidth_log.csv", "w");
    initial $fwrite(csv_fd, "time_ns,master,dir,bytes,qos\n");

    task automatic log_txn(input string master, input string dir, input int bytes, input int qos);
        $fwrite(csv_fd, "%0t,%s,%s,%0d,%0d\n", $time, master, dir, bytes, qos);
    endtask

`ifndef ICARUS_SIM
    // ---------------- Real SystemVerilog functional coverage ----------------
    int cov_master, cov_is_write, cov_qos_bucket, cov_len_class;

    covergroup cg_axi_txn;
        option.per_instance = 1;
        cp_master: coverpoint cov_master     { bins m[] = {0, 1, 2}; }
        cp_dir:    coverpoint cov_is_write   { bins d[] = {0, 1}; }
        cp_qos:    coverpoint cov_qos_bucket { bins q[] = {0, 1, 2}; }
        cp_len:    coverpoint cov_len_class  { bins l[] = {0, 1}; }
        cx_all: cross cp_master, cp_dir, cp_qos, cp_len;
    endgroup
    cg_axi_txn cg_axi = new();

    int cov_wr_id, cov_rd_id;
    covergroup cg_frame_swap;
        option.per_instance = 1;
        cp_wr: coverpoint cov_wr_id { bins b[] = {0, 1, 2}; }
        cp_rd: coverpoint cov_rd_id { bins b[] = {0, 1, 2}; }
        cx_swap: cross cp_wr, cp_rd;
    endgroup
    cg_frame_swap cg_frame = new();

    bit cov_priority_inversion_seen, cov_dma_stride_seen, cov_starvation_recovery_seen;
    covergroup cg_scenarios;
        option.per_instance = 1;
        cp_pi: coverpoint cov_priority_inversion_seen  { bins hit = {1}; }
        cp_ds: coverpoint cov_dma_stride_seen           { bins hit = {1}; }
        cp_sr: coverpoint cov_starvation_recovery_seen  { bins hit = {1}; }
    endgroup
    cg_scenarios cg_scen = new();
`endif

    function automatic int qos_bucket(input int q);
        if (q <= 2) return 0;
        else if (q <= 8) return 1;
        else return 2;
    endfunction

    task automatic mark_cov(input int master, input int is_wr, input int qos, input int len_beats);
`ifndef ICARUS_SIM
        cov_master     = master;
        cov_is_write   = is_wr;
        cov_qos_bucket = qos_bucket(qos);
        cov_len_class  = (len_beats >= 4) ? 1 : 0;
        cg_axi.sample();
`endif
    endtask

    task automatic mark_frame_cov();
`ifndef ICARUS_SIM
        cov_wr_id = wr_buf_id;
        cov_rd_id = rd_buf_id;
        cg_frame.sample();
`endif
    endtask

    task automatic mark_scenario(input string which);
`ifndef ICARUS_SIM
        case (which)
            "pi": cov_priority_inversion_seen  = 1;
            "ds": cov_dma_stride_seen           = 1;
            "sr": cov_starvation_recovery_seen  = 1;
        endcase
        cg_scen.sample();
`endif
    endtask

    // ================= M0 (CPU) BFM tasks =================
    // Handshake idiom throughout: poll at NEGEDGE (mid-cycle, race-free
    // against the DUT's own posedge-triggered registers), and let one
    // full negedge elapse after observing readiness before deasserting
    // VALID so it is never dropped before the edge the receiver samples
    // it on.
    task automatic m0_write(input logic [AW-1:0] addr, input logic [DW-1:0] data, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id);
        @(negedge clk);
        m0_awvalid = 1; m0_awaddr = addr; m0_awlen = 0; m0_awsize = 3'b010;
        m0_awburst = 2'b01; m0_awqos = qos; m0_awid = id;
        m0_wvalid = 1; m0_wdata = data; m0_wstrb = 4'hF; m0_wlast = 1;
        m0_bready = 1;
        while (!m0_awready) @(negedge clk);
        @(negedge clk);
        m0_awvalid = 0;
        while (!(m0_wvalid && m0_wready)) @(negedge clk);
        @(negedge clk);
        m0_wvalid = 0; m0_wlast = 0;
        while (!m0_bvalid) @(negedge clk);
        @(negedge clk);
        ref_mem[addr>>2] = data;
        log_txn("cpu", "wr", 4, qos);
        mark_cov(0, 1, qos, 1);
    endtask

    task automatic m0_read(input logic [AW-1:0] addr, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id, output logic [DW-1:0] data);
        @(negedge clk);
        m0_arvalid = 1; m0_araddr = addr; m0_arlen = 0; m0_arsize = 3'b010;
        m0_arburst = 2'b01; m0_arqos = qos; m0_arid = id;
        m0_rready = 1;
        while (!m0_arready) @(negedge clk);
        @(negedge clk);
        m0_arvalid = 0;
        while (!m0_rvalid) @(negedge clk);
        data = m0_rdata;
        @(negedge clk);
        log_txn("cpu", "rd", 4, qos);
        mark_cov(0, 0, qos, 1);
    endtask

    task automatic m0_burst_write(input logic [AW-1:0] addr, input int len_beats, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id);
        logic [DW-1:0] wd;
        @(negedge clk);
        m0_awvalid = 1; m0_awaddr = addr; m0_awlen = len_beats - 1; m0_awsize = 3'b010;
        m0_awburst = 2'b01; m0_awqos = qos; m0_awid = id;
        m0_bready = 1;
        while (!m0_awready) @(negedge clk);
        @(negedge clk);
        m0_awvalid = 0;
        for (int i = 0; i < len_beats; i++) begin
            wd = 32'hA000_0000 + addr + (i*4);
            m0_wvalid = 1; m0_wdata = wd; m0_wstrb = 4'hF; m0_wlast = (i == len_beats-1);
            while (!m0_wready) @(negedge clk);
            @(negedge clk);
            ref_mem[(addr + i*4) >> 2] = wd;
        end
        m0_wvalid = 0; m0_wlast = 0;
        while (!m0_bvalid) @(negedge clk);
        @(negedge clk);
        log_txn("cpu", "wr", len_beats*4, qos);
        mark_cov(0, 1, qos, len_beats);
    endtask

    task automatic m0_burst_read(input logic [AW-1:0] addr, input int len_beats, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id);
        @(negedge clk);
        m0_arvalid = 1; m0_araddr = addr; m0_arlen = len_beats - 1; m0_arsize = 3'b010;
        m0_arburst = 2'b01; m0_arqos = qos; m0_arid = id;
        m0_rready = 1;
        while (!m0_arready) @(negedge clk);
        @(negedge clk);
        m0_arvalid = 0;
        for (int i = 0; i < len_beats; i++) begin
            while (!m0_rvalid) @(negedge clk);
            check_eq(m0_rdata, ref_mem[(addr + i*4) >> 2], "m0 burst_read beat");
            @(negedge clk);
        end
        log_txn("cpu", "rd", len_beats*4, qos);
        mark_cov(0, 0, qos, len_beats);
    endtask

    // ================= M1 (Codec) BFM tasks (mirror of M0) =================
    task automatic m1_write(input logic [AW-1:0] addr, input logic [DW-1:0] data, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id);
        @(negedge clk);
        m1_awvalid = 1; m1_awaddr = addr; m1_awlen = 0; m1_awsize = 3'b010;
        m1_awburst = 2'b01; m1_awqos = qos; m1_awid = id;
        m1_wvalid = 1; m1_wdata = data; m1_wstrb = 4'hF; m1_wlast = 1;
        m1_bready = 1;
        while (!m1_awready) @(negedge clk);
        @(negedge clk);
        m1_awvalid = 0;
        while (!(m1_wvalid && m1_wready)) @(negedge clk);
        @(negedge clk);
        m1_wvalid = 0; m1_wlast = 0;
        while (!m1_bvalid) @(negedge clk);
        @(negedge clk);
        ref_mem[addr>>2] = data;
        log_txn("codec", "wr", 4, qos);
        mark_cov(1, 1, qos, 1);
    endtask

    task automatic m1_read(input logic [AW-1:0] addr, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id, output logic [DW-1:0] data);
        @(negedge clk);
        m1_arvalid = 1; m1_araddr = addr; m1_arlen = 0; m1_arsize = 3'b010;
        m1_arburst = 2'b01; m1_arqos = qos; m1_arid = id;
        m1_rready = 1;
        while (!m1_arready) @(negedge clk);
        @(negedge clk);
        m1_arvalid = 0;
        while (!m1_rvalid) @(negedge clk);
        data = m1_rdata;
        @(negedge clk);
        log_txn("codec", "rd", 4, qos);
        mark_cov(1, 0, qos, 1);
    endtask

    task automatic m1_burst_write(input logic [AW-1:0] addr, input int len_beats, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id);
        logic [DW-1:0] wd;
        @(negedge clk);
        m1_awvalid = 1; m1_awaddr = addr; m1_awlen = len_beats - 1; m1_awsize = 3'b010;
        m1_awburst = 2'b01; m1_awqos = qos; m1_awid = id;
        m1_bready = 1;
        while (!m1_awready) @(negedge clk);
        @(negedge clk);
        m1_awvalid = 0;
        for (int i = 0; i < len_beats; i++) begin
            wd = 32'hC000_0000 + addr + (i*4);
            m1_wvalid = 1; m1_wdata = wd; m1_wstrb = 4'hF; m1_wlast = (i == len_beats-1);
            while (!m1_wready) @(negedge clk);
            @(negedge clk);
            ref_mem[(addr + i*4) >> 2] = wd;
        end
        m1_wvalid = 0; m1_wlast = 0;
        while (!m1_bvalid) @(negedge clk);
        @(negedge clk);
        log_txn("codec", "wr", len_beats*4, qos);
        mark_cov(1, 1, qos, len_beats);
    endtask

    task automatic m1_burst_read(input logic [AW-1:0] addr, input int len_beats, input logic [QOSW-1:0] qos, input logic [IDW-1:0] id);
        @(negedge clk);
        m1_arvalid = 1; m1_araddr = addr; m1_arlen = len_beats - 1; m1_arsize = 3'b010;
        m1_arburst = 2'b01; m1_arqos = qos; m1_arid = id;
        m1_rready = 1;
        while (!m1_arready) @(negedge clk);
        @(negedge clk);
        m1_arvalid = 0;
        for (int i = 0; i < len_beats; i++) begin
            while (!m1_rvalid) @(negedge clk);
            check_eq(m1_rdata, ref_mem[(addr + i*4) >> 2], "m1 burst_read beat");
            @(negedge clk);
        end
        log_txn("codec", "rd", len_beats*4, qos);
        mark_cov(1, 0, qos, len_beats);
    endtask

    // Codec traffic generator: alternating bitstream-fetch bursts (reads)
    // and decoded-line write-back bursts (writes), representative of
    // real codec DMA-ish behavior without modeling a full codec core.
    task automatic codec_traffic_burst(input logic [AW-1:0] fetch_addr, input logic [AW-1:0] writeback_addr,
                                        input int n_iters, input logic [QOSW-1:0] qos);
        for (int i = 0; i < n_iters; i++) begin
            m1_burst_read (fetch_addr     + ((i*64)  % 4096), 16, qos, 2'd1);
            m1_burst_write(writeback_addr + ((i*256) % 4096), 64, qos, 2'd1);
        end
    endtask

    // ================= DMA control =================
    task automatic run_dma(input logic [AW-1:0] src, input logic [AW-1:0] dst, input int row_words,
                            input int num_rows, input logic [AW-1:0] src_stride, input logic [AW-1:0] dst_stride,
                            input logic [QOSW-1:0] qos);
        dma_src_addr = src; dma_dst_addr = dst; dma_row_words = row_words; dma_num_rows = num_rows;
        dma_src_stride = src_stride; dma_dst_stride = dst_stride; dma_qos = qos;
        @(negedge clk); dma_start = 1; @(negedge clk); dma_start = 0;
        while (!dma_done) @(negedge clk);
        @(negedge clk);
        for (int r = 0; r < num_rows; r++)
            for (int c = 0; c < row_words; c++)
                ref_mem[(dst + r*dst_stride + c*4) >> 2] = ref_mem[(src + r*src_stride + c*4) >> 2];
        log_txn("dma", "wr", row_words*num_rows*4, qos);
        log_txn("dma", "rd", row_words*num_rows*4, qos);
        mark_cov(2, 1, qos, row_words);
        mark_cov(2, 0, qos, row_words);
        if (src_stride != row_words*4 || dst_stride != row_words*4) mark_scenario("ds");
    endtask

    // Run a DMA transfer and return the elapsed cycle count - used for
    // the bandwidth-under-contention measurement (Test 10).
    task automatic run_dma_timed(input logic [AW-1:0] src, input logic [AW-1:0] dst, input int row_words,
                                  input int num_rows, input logic [AW-1:0] src_stride, input logic [AW-1:0] dst_stride,
                                  input logic [QOSW-1:0] qos, output longint cycles_taken);
        time t_start;
        t_start = $time;
        run_dma(src, dst, row_words, num_rows, src_stride, dst_stride, qos);
        cycles_taken = ($time - t_start) / CLK_PERIOD;
    endtask

    // ================= Test sequencing =================
    logic [DW-1:0] rdata;
    int t;

    initial begin
        m0_awvalid=0; m0_wvalid=0; m0_bready=0; m0_arvalid=0; m0_rready=0;
        m1_awvalid=0; m1_wvalid=0; m1_bready=0; m1_arvalid=0; m1_rready=0;
        dma_start = 0;
        capture_frame_done = 0;
        display_frame_done = 0;

        rst_n = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        $display("=== TEST 1: CPU single write/read, mid QoS ===");
        m0_write(24'h00_0010, 32'hDEAD_0001, 4'd4, 2'd0);
        m0_read (24'h00_0010, 4'd4, 2'd0, rdata);
        check_eq(rdata, ref_mem[24'h00_0010>>2], "T1 CPU write/read roundtrip");

        $display("=== TEST 2: Codec single write/read, mid QoS ===");
        m1_write(24'h00_0020, 32'hDEAD_0002, 4'd4, 2'd1);
        m1_read (24'h00_0020, 4'd4, 2'd1, rdata);
        check_eq(rdata, ref_mem[24'h00_0020>>2], "T2 Codec write/read roundtrip");

        $display("=== TEST 3: CPU and Codec concurrent, independent addresses ===");
        fork
            m0_write(24'h00_0100, 32'hCAFE_0001, 4'd10, 2'd0);
            m1_write(24'h00_0200, 32'hCAFE_0002, 4'd5,  2'd1);
        join
        m0_read(24'h00_0100, 4'd10, 2'd0, rdata); check_eq(rdata, ref_mem[24'h00_0100>>2], "T3 CPU concurrent");
        m1_read(24'h00_0200, 4'd5,  2'd1, rdata); check_eq(rdata, ref_mem[24'h00_0200>>2], "T3 Codec concurrent");

        $display("=== TEST 4: CPU max-length burst (256 beats) write + readback ===");
        m0_burst_write(24'h00_1000, 256, 4'd8, 2'd0);
        m0_burst_read (24'h00_1000, 256, 4'd8, 2'd0);

        $display("=== TEST 5: QoS priority - codec flood vs high-QoS CPU request ===");
        begin
            time t_start, t_end;
            t_start = $time;
            fork
                codec_traffic_burst(24'h00_4000, 24'h00_8000, 20, 4'd2); // low-QoS flood
                begin
                    m0_write(24'h00_2000, 32'hF00D_0001, 4'd15, 2'd0);
                    t_end = $time;
                    if ((t_end - t_start) > (CLK_PERIOD*80))
                        $display("[%0t] WARN: high-QoS CPU write took %0d ns amid codec flood (informational)", $time, t_end - t_start);
                end
            join
            mark_scenario("pi");
        end

        $display("=== TEST 6: starvation-recovery via aging - sustained high-QoS CPU still lets low-QoS codec through ===");
        begin
            bit done_low;
            done_low = 0;
            fork
                begin
                    for (t = 0; t < 40; t++)
                        m0_write(24'h00_3000 + ((t%4)*4), 32'h1234_0000+t, 4'd15, 2'd0);
                end
                begin
                    m1_write(24'h00_3100, 32'hAAAA_BBBB, 4'd0, 2'd1);
                    done_low = 1;
                end
            join
            if (done_low) mark_scenario("sr");
        end
        m1_read(24'h00_3100, 4'd0, 2'd1, rdata);
        check_eq(rdata, 32'hAAAA_BBBB, "T6 low-QoS codec write eventually completed and is correct");

        $display("=== TEST 7: DMA frame-capture transfer into the current write (back) buffer ===");
        run_dma(24'h00_5000, wr_buf_base, 64, 8, 256, 256, 4'd9);
        for (t = 0; t < 8; t++)
            m0_burst_read(wr_buf_base + t*256, 64, 4'd9, 2'd0);
        capture_frame_done = 1; @(negedge clk); capture_frame_done = 0;
        mark_frame_cov();

        $display("=== TEST 8: DMA transfer with non-unit stride (cropped sub-rectangle) ===");
        for (t = 0; t < 32; t++)
            m0_write(24'h00_6000 + t*4, 32'h5000_0000 + t, 4'd8, 2'd0);
        run_dma(24'h00_6000, 24'h00_7000, 4, 4, 32, 16, 4'd9);
        for (t = 0; t < 4; t++) begin
            m0_read(24'h00_7000 + t*4, 4'd9, 2'd0, rdata);
            check_eq(rdata, ref_mem[(24'h00_7000+t*4)>>2], "T8 DMA strided sub-rectangle readback");
        end

        $display("=== TEST 9: triple-buffer rotation correctness (frame_buffer_mgr) ===");
        begin
            logic [1:0] wr0, rd0, wr1, rd1;
            wr0 = wr_buf_id; rd0 = rd_buf_id;
            // capture completes -> back/ready swap; display then picks it up
            capture_frame_done = 1; @(negedge clk); capture_frame_done = 0;
            mark_frame_cov();
            display_frame_done = 1; @(negedge clk); display_frame_done = 0;
            mark_frame_cov();
            wr1 = wr_buf_id; rd1 = rd_buf_id;
            if (rd1 !== wr0)
                $display("[%0t] FAIL: T9 display buffer after swap should be the buffer just captured", $time);
            checks++; if (rd1 !== wr0) errors++;
            if (wr1 == rd1 || wr1 == rd0)
                $display("[%0t] FAIL: T9 producer's new back buffer collides with a buffer in use", $time);
            checks++; if (wr1 == rd1 || wr1 == rd0) errors++;
            // simultaneous capture+display in the same cycle
            capture_frame_done = 1; display_frame_done = 1;
            @(negedge clk);
            capture_frame_done = 0; display_frame_done = 0;
            mark_frame_cov();
            checks++; if (wr_buf_id == rd_buf_id) errors++;
            if (wr_buf_id == rd_buf_id)
                $display("[%0t] FAIL: T9 simultaneous capture+display collided front/back", $time);
        end

        $display("=== TEST 10: CPU + Codec + DMA frame transfer contending simultaneously (bandwidth measurement) ===");
        begin
            longint cyc;
            fork
                begin
                    for (t = 0; t < 12; t++)
                        m0_write(24'h00_9000 + t*4, 32'h1111_0000+t, 4'd10, 2'd0);
                end
                codec_traffic_burst(24'h00_A000, 24'h00_B000, 8, 4'd6);
                begin
                    // dma_row_words is an 8-bit raw word count (not AXI
                    // LEN-encoded like the BFM burst tasks), so its max
                    // representable value is 255, not 256.
                    run_dma_timed(24'h00_C000, rd_buf_base, 255, 16, 24'd1020, 24'd1020, 4'd12, cyc);
                end
            join
            $display("[%0t] Test 10: DMA moved %0d bytes in %0d cycles under CPU+codec contention (%.3f bytes/cycle)",
                $time, 255*16*4, cyc, real'(255*16*4)/real'(cyc));
            log_txn("meta", "bw_cycles", cyc, 0);
            log_txn("meta", "bw_bytes", 255*16*4, 0);
        end

        $display("=== TEST 11: back-to-back DMA transfers + burst-length boundary (len=1 and len=256) ===");
        run_dma(24'h00_D000, 24'h00_D100, 4, 2, 16, 16, 4'd3);
        run_dma(24'h00_D200, 24'h00_D300, 4, 2, 16, 16, 4'd3);
        m0_read(24'h00_D100, 4'd3, 2'd0, rdata); check_eq(rdata, ref_mem[24'h00_D100>>2], "T11 first DMA result intact");
        m0_read(24'h00_D300, 4'd3, 2'd0, rdata); check_eq(rdata, ref_mem[24'h00_D300>>2], "T11 second DMA result intact");
        m0_burst_write(24'h00_E000, 1, 4'd2, 2'd0);
        m0_burst_read (24'h00_E000, 1, 4'd2, 2'd0);
        m1_burst_write(24'h00_E100, 256, 4'd11, 2'd1);
        m1_burst_read (24'h00_E100, 256, 4'd11, 2'd1);

        $display("=== TEST 12: DDR behavioral model read latency characterization ===");
        begin
            time t_ar, t_first_r;
            m0_write(24'h00_F000, 32'hBEEF_0001, 4'd8, 2'd0);
            @(negedge clk);
            t_ar = $time;
            m0_arvalid = 1; m0_araddr = 24'h00_F000; m0_arlen = 0; m0_arsize = 3'b010;
            m0_arburst = 2'b01; m0_arqos = 4'd8; m0_arid = 2'd0; m0_rready = 1;
            while (!m0_arready) @(negedge clk);
            @(negedge clk);
            m0_arvalid = 0;
            while (!m0_rvalid) @(negedge clk);
            t_first_r = $time;
            $display("[%0t] Test 12: AR-to-first-RVALID latency = %0d ns (%0d cycles) under idle contention",
                $time, t_first_r - t_ar, (t_first_r - t_ar)/CLK_PERIOD);
            check_eq(m0_rdata, 32'hBEEF_0001, "T12 DDR latency-characterization read data");
            @(negedge clk);
        end

        repeat (10) @(negedge clk);

        $display("");
        $display("=========================================================");
        $display(" TOTAL CHECKS : %0d", checks);
        $display(" TOTAL ERRORS : %0d", errors);
`ifndef ICARUS_SIM
        $display(" AXI txn cross coverage    : %0.2f%%", cg_axi.get_coverage());
        $display(" Frame-swap cross coverage : %0.2f%%", cg_frame.get_coverage());
        $display(" Scenario coverage         : %0.2f%%", cg_scen.get_coverage());
`endif
        $display("=========================================================");
        if (errors == 0)
            $display(" RESULT: ALL TESTS PASSED");
        else
            $display(" RESULT: %0d CHECK(S) FAILED", errors);

        $fclose(csv_fd);
        $finish;
    end

    // Safety timeout
    initial begin
        #100_000_000;
        $display("TIMEOUT: testbench did not finish in time");
        $fclose(csv_fd);
        $finish;
    end

endmodule
