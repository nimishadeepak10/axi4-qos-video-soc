// axi4_qos_videosoc_top.sv
// Author: Nimisha Deepak
// QoS AXI4 memory subsystem for a video SoC: 2 external AXI4 slave ports
// (M0 = CPU/control-plane, M1 = video codec traffic) plus an internal 2D
// DMA engine (M2, frame-buffer-aware via frame_buffer_mgr), all
// QoS-arbitrated onto a single AXI4-to-DDR bridge/controller. This is the
// module synthesized standalone for the utilization/timing numbers in
// README.md - the DDR itself (ddr_behavioral_model.sv) is instantiated
// by the testbench, not here, matching how real designs never synthesize
// their DRAM.

`timescale 1ns / 1ps

module axi4_qos_videosoc_top #(
    parameter int AW   = 24,
    parameter int DW   = 32,
    parameter int IDW  = 2,
    parameter int LENW = 8,
    parameter int QOSW = 4,
    parameter int FRAME_STRIDE_BYTES = 1 << 22
) (
    input  logic              clk,
    input  logic              rst_n,

    // ---------------- Master 0: CPU (external AXI4 slave port) ----------------
    input  logic               m0_awvalid,
    output logic                m0_awready,
    input  logic [AW-1:0]      m0_awaddr,
    input  logic [LENW-1:0]    m0_awlen,
    input  logic [2:0]         m0_awsize,
    input  logic [1:0]         m0_awburst,
    input  logic [QOSW-1:0]    m0_awqos,
    input  logic [IDW-1:0]     m0_awid,
    input  logic               m0_wvalid,
    output logic                m0_wready,
    input  logic [DW-1:0]      m0_wdata,
    input  logic [(DW/8)-1:0]  m0_wstrb,
    input  logic               m0_wlast,
    output logic                m0_bvalid,
    input  logic                m0_bready,
    output logic [1:0]          m0_bresp,
    output logic [IDW-1:0]      m0_bid,
    input  logic               m0_arvalid,
    output logic                m0_arready,
    input  logic [AW-1:0]      m0_araddr,
    input  logic [LENW-1:0]    m0_arlen,
    input  logic [2:0]         m0_arsize,
    input  logic [1:0]         m0_arburst,
    input  logic [QOSW-1:0]    m0_arqos,
    input  logic [IDW-1:0]     m0_arid,
    output logic                m0_rvalid,
    input  logic                m0_rready,
    output logic [DW-1:0]       m0_rdata,
    output logic [1:0]          m0_rresp,
    output logic                m0_rlast,
    output logic [IDW-1:0]      m0_rid,

    // ---------------- Master 1: Codec traffic (external AXI4 slave port) ----------------
    input  logic               m1_awvalid,
    output logic                m1_awready,
    input  logic [AW-1:0]      m1_awaddr,
    input  logic [LENW-1:0]    m1_awlen,
    input  logic [2:0]         m1_awsize,
    input  logic [1:0]         m1_awburst,
    input  logic [QOSW-1:0]    m1_awqos,
    input  logic [IDW-1:0]     m1_awid,
    input  logic               m1_wvalid,
    output logic                m1_wready,
    input  logic [DW-1:0]      m1_wdata,
    input  logic [(DW/8)-1:0]  m1_wstrb,
    input  logic               m1_wlast,
    output logic                m1_bvalid,
    input  logic                m1_bready,
    output logic [1:0]          m1_bresp,
    output logic [IDW-1:0]      m1_bid,
    input  logic               m1_arvalid,
    output logic                m1_arready,
    input  logic [AW-1:0]      m1_araddr,
    input  logic [LENW-1:0]    m1_arlen,
    input  logic [2:0]         m1_arsize,
    input  logic [1:0]         m1_arburst,
    input  logic [QOSW-1:0]    m1_arqos,
    input  logic [IDW-1:0]     m1_arid,
    output logic                m1_rvalid,
    input  logic                m1_rready,
    output logic [DW-1:0]       m1_rdata,
    output logic [1:0]          m1_rresp,
    output logic                m1_rlast,
    output logic [IDW-1:0]      m1_rid,

    // ---------------- DMA (M2) control/status port ----------------
    input  logic                dma_start,
    input  logic [AW-1:0]       dma_src_addr,
    input  logic [AW-1:0]       dma_dst_addr,
    input  logic [LENW-1:0]     dma_row_words,
    input  logic [7:0]          dma_num_rows,
    input  logic [AW-1:0]       dma_src_stride,
    input  logic [AW-1:0]       dma_dst_stride,
    input  logic [QOSW-1:0]     dma_qos,
    output logic                 dma_busy,
    output logic                 dma_done,

    // ---------------- Triple-buffered frame manager ----------------
    input  logic                capture_frame_done,
    input  logic                display_frame_done,
    output logic [AW-1:0]       wr_buf_base,
    output logic [AW-1:0]       rd_buf_base,
    output logic [1:0]          wr_buf_id,
    output logic [1:0]          rd_buf_id,
    output logic                 frame_ready_pending,

    // ---------------- Behavioral-DDR-facing interface (off-chip, sim-only model) ----------------
    output logic               ddr_cmd_valid,
    input  logic               ddr_cmd_ready,
    output logic [AW-1:0]      ddr_cmd_addr,
    output logic               ddr_cmd_write,
    output logic [LENW-1:0]    ddr_cmd_len,

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

    localparam int NM = 3;

    // ---- Frame buffer manager ----
    frame_buffer_mgr #(.AW(AW), .FRAME_STRIDE_BYTES(FRAME_STRIDE_BYTES)) u_frame_mgr (
        .clk(clk), .rst_n(rst_n),
        .capture_frame_done(capture_frame_done), .display_frame_done(display_frame_done),
        .wr_buf_base(wr_buf_base), .rd_buf_base(rd_buf_base),
        .wr_buf_id(wr_buf_id), .rd_buf_id(rd_buf_id),
        .frame_ready_pending(frame_ready_pending)
    );

    // ---- DMA (master index 2) AXI wires ----
    logic              m2_awvalid, m2_awready, m2_wvalid, m2_wready, m2_wlast;
    logic [AW-1:0]      m2_awaddr;
    logic [LENW-1:0]    m2_awlen;
    logic [2:0]         m2_awsize;
    logic [1:0]         m2_awburst;
    logic [QOSW-1:0]    m2_awqos;
    logic [IDW-1:0]     m2_awid;
    logic [DW-1:0]      m2_wdata;
    logic [(DW/8)-1:0]  m2_wstrb;
    logic               m2_bvalid, m2_bready;
    logic [1:0]         m2_bresp;
    logic [IDW-1:0]     m2_bid;

    logic               m2_arvalid, m2_arready, m2_rvalid, m2_rready, m2_rlast;
    logic [AW-1:0]      m2_araddr;
    logic [LENW-1:0]    m2_arlen;
    logic [2:0]         m2_arsize;
    logic [1:0]         m2_arburst;
    logic [QOSW-1:0]    m2_arqos;
    logic [IDW-1:0]     m2_arid;
    logic [DW-1:0]      m2_rdata;
    logic [1:0]         m2_rresp;
    logic [IDW-1:0]     m2_rid;

    axi4_dma_2d #(
        .AW(AW), .DW(DW), .IDW(IDW), .LENW(LENW), .QOSW(QOSW), .MYID(2),
        .MAX_ROW_WORDS(256)   // one full max-length AXI4 burst per DMA row
    ) u_dma (
        .clk(clk), .rst_n(rst_n),
        .dma_start(dma_start), .dma_src_addr(dma_src_addr), .dma_dst_addr(dma_dst_addr),
        .dma_row_words(dma_row_words), .dma_num_rows(dma_num_rows),
        .dma_src_stride(dma_src_stride), .dma_dst_stride(dma_dst_stride),
        .dma_qos(dma_qos), .dma_busy(dma_busy), .dma_done(dma_done),

        .m_awvalid(m2_awvalid), .m_awready(m2_awready), .m_awaddr(m2_awaddr),
        .m_awlen(m2_awlen), .m_awsize(m2_awsize), .m_awburst(m2_awburst),
        .m_awqos(m2_awqos), .m_awid(m2_awid),
        .m_wvalid(m2_wvalid), .m_wready(m2_wready), .m_wdata(m2_wdata),
        .m_wstrb(m2_wstrb), .m_wlast(m2_wlast),
        .m_bvalid(m2_bvalid), .m_bready(m2_bready), .m_bresp(m2_bresp), .m_bid(m2_bid),

        .m_arvalid(m2_arvalid), .m_arready(m2_arready), .m_araddr(m2_araddr),
        .m_arlen(m2_arlen), .m_arsize(m2_arsize), .m_arburst(m2_arburst),
        .m_arqos(m2_arqos), .m_arid(m2_arid),
        .m_rvalid(m2_rvalid), .m_rready(m2_rready), .m_rdata(m2_rdata),
        .m_rresp(m2_rresp), .m_rlast(m2_rlast), .m_rid(m2_rid)
    );

    // ---- Concatenated per-master buses for the arbiters ----
    logic [NM-1:0]        aw_valid;
    logic [NM-1:0]        aw_ready;
    logic [NM*AW-1:0]     aw_addr;
    logic [NM*LENW-1:0]   aw_len;
    logic [NM*3-1:0]      aw_size;
    logic [NM*2-1:0]      aw_burst;
    logic [NM*QOSW-1:0]   aw_qos;
    logic [NM*IDW-1:0]    aw_id;
    assign aw_valid = {m2_awvalid, m1_awvalid, m0_awvalid};
    assign aw_addr  = {m2_awaddr,  m1_awaddr,  m0_awaddr};
    assign aw_len   = {m2_awlen,   m1_awlen,   m0_awlen};
    assign aw_size  = {m2_awsize,  m1_awsize,  m0_awsize};
    assign aw_burst = {m2_awburst, m1_awburst, m0_awburst};
    assign aw_qos   = {m2_awqos,   m1_awqos,   m0_awqos};
    assign aw_id    = {m2_awid,    m1_awid,    m0_awid};

    logic [NM-1:0]        w_valid;
    logic [NM-1:0]        w_ready;
    logic [NM*DW-1:0]     w_data;
    logic [NM*(DW/8)-1:0] w_strb;
    logic [NM-1:0]        w_last;
    assign w_valid = {m2_wvalid,  m1_wvalid,  m0_wvalid};
    assign w_data  = {m2_wdata,   m1_wdata,   m0_wdata};
    assign w_strb  = {m2_wstrb,   m1_wstrb,   m0_wstrb};
    assign w_last  = {m2_wlast,   m1_wlast,   m0_wlast};

    logic [NM-1:0]        b_valid;
    logic [NM-1:0]        b_ready;
    logic [NM*2-1:0]      b_resp;
    logic [NM*IDW-1:0]    b_id;
    assign b_ready = {m2_bready,  m1_bready,  m0_bready};

    logic [NM-1:0]        ar_valid;
    logic [NM-1:0]        ar_ready;
    logic [NM*AW-1:0]     ar_addr;
    logic [NM*LENW-1:0]   ar_len;
    logic [NM*3-1:0]      ar_size;
    logic [NM*2-1:0]      ar_burst;
    logic [NM*QOSW-1:0]   ar_qos;
    logic [NM*IDW-1:0]    ar_id;
    assign ar_valid = {m2_arvalid, m1_arvalid, m0_arvalid};
    assign ar_addr  = {m2_araddr,  m1_araddr,  m0_araddr};
    assign ar_len   = {m2_arlen,   m1_arlen,   m0_arlen};
    assign ar_size  = {m2_arsize,  m1_arsize,  m0_arsize};
    assign ar_burst = {m2_arburst, m1_arburst, m0_arburst};
    assign ar_qos   = {m2_arqos,   m1_arqos,   m0_arqos};
    assign ar_id    = {m2_arid,    m1_arid,    m0_arid};

    logic [NM-1:0]        r_valid;
    logic [NM-1:0]        r_ready;
    logic [NM*DW-1:0]     r_data;
    logic [NM*2-1:0]      r_resp;
    logic [NM-1:0]        r_last;
    logic [NM*IDW-1:0]    r_id;
    assign r_ready = {m2_rready,  m1_rready,  m0_rready};

    assign {m2_awready, m1_awready, m0_awready} = aw_ready;
    assign {m2_wready,  m1_wready,  m0_wready}  = w_ready;
    assign {m2_bvalid,  m1_bvalid,  m0_bvalid}  = b_valid;
    assign {m2_bresp,   m1_bresp,   m0_bresp}   = b_resp;
    assign {m2_bid,     m1_bid,     m0_bid}     = b_id;

    assign {m2_arready, m1_arready, m0_arready} = ar_ready;
    assign {m2_rvalid,  m1_rvalid,  m0_rvalid}  = r_valid;
    assign {m2_rdata,   m1_rdata,   m0_rdata}   = r_data;
    assign {m2_rresp,   m1_rresp,   m0_rresp}   = r_resp;
    assign {m2_rlast,   m1_rlast,   m0_rlast}   = r_last;
    assign {m2_rid,     m1_rid,     m0_rid}     = r_id;

    // ---- Arbitrated single downstream port to the DDR controller ----
    logic               s_awvalid, s_awready, s_wvalid, s_wready, s_wlast;
    logic [AW-1:0]      s_awaddr;
    logic [LENW-1:0]    s_awlen;
    logic [2:0]         s_awsize;
    logic [1:0]         s_awburst;
    logic [DW-1:0]      s_wdata;
    logic [(DW/8)-1:0]  s_wstrb;
    logic               s_bvalid, s_bready;
    logic [1:0]         s_bresp;
    logic [IDW-1:0]     s_bid, s_awid;

    logic               s_arvalid, s_arready, s_rvalid, s_rready, s_rlast;
    logic [AW-1:0]      s_araddr;
    logic [LENW-1:0]    s_arlen;
    logic [2:0]         s_arsize;
    logic [1:0]         s_arburst;
    logic [DW-1:0]      s_rdata;
    logic [1:0]         s_rresp;
    logic [IDW-1:0]     s_rid, s_arid;

    axi4_write_arb #(
        .NM(NM), .AW(AW), .DW(DW), .IDW(IDW), .LENW(LENW), .QOSW(QOSW)
    ) u_wr_arb (
        .clk(clk), .rst_n(rst_n),
        .s_awvalid(aw_valid), .s_awready(aw_ready), .s_awaddr(aw_addr), .s_awlen(aw_len),
        .s_awsize(aw_size), .s_awburst(aw_burst), .s_awqos(aw_qos), .s_awid(aw_id),
        .s_wvalid(w_valid), .s_wready(w_ready), .s_wdata(w_data), .s_wstrb(w_strb), .s_wlast(w_last),
        .s_bvalid(b_valid), .s_bready(b_ready), .s_bresp(b_resp), .s_bid(b_id),
        .m_awvalid(s_awvalid), .m_awready(s_awready), .m_awaddr(s_awaddr), .m_awlen(s_awlen),
        .m_awsize(s_awsize), .m_awburst(s_awburst), .m_awqos(), .m_awid(s_awid),
        .m_wvalid(s_wvalid), .m_wready(s_wready), .m_wdata(s_wdata), .m_wstrb(s_wstrb), .m_wlast(s_wlast),
        .m_bvalid(s_bvalid), .m_bready(s_bready), .m_bresp(s_bresp), .m_bid(s_bid),
        .grant_onehot()
    );

    axi4_read_arb #(
        .NM(NM), .AW(AW), .DW(DW), .IDW(IDW), .LENW(LENW), .QOSW(QOSW)
    ) u_rd_arb (
        .clk(clk), .rst_n(rst_n),
        .s_arvalid(ar_valid), .s_arready(ar_ready), .s_araddr(ar_addr), .s_arlen(ar_len),
        .s_arsize(ar_size), .s_arburst(ar_burst), .s_arqos(ar_qos), .s_arid(ar_id),
        .s_rvalid(r_valid), .s_rready(r_ready), .s_rdata(r_data), .s_rresp(r_resp),
        .s_rlast(r_last), .s_rid(r_id),
        .m_arvalid(s_arvalid), .m_arready(s_arready), .m_araddr(s_araddr), .m_arlen(s_arlen),
        .m_arsize(s_arsize), .m_arburst(s_arburst), .m_arqos(), .m_arid(s_arid),
        .m_rvalid(s_rvalid), .m_rready(s_rready), .m_rdata(s_rdata), .m_rresp(s_rresp),
        .m_rlast(s_rlast), .m_rid(s_rid),
        .grant_onehot()
    );

    axi4_ddr_ctrl #(
        .AW(AW), .DW(DW), .IDW(IDW), .LENW(LENW)
    ) u_ddr_ctrl (
        .clk(clk), .rst_n(rst_n),
        .s_awvalid(s_awvalid), .s_awready(s_awready), .s_awaddr(s_awaddr), .s_awlen(s_awlen),
        .s_awsize(s_awsize), .s_awburst(s_awburst), .s_awid(s_awid),
        .s_wvalid(s_wvalid), .s_wready(s_wready), .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
        .s_bvalid(s_bvalid), .s_bready(s_bready), .s_bresp(s_bresp), .s_bid(s_bid),
        .s_arvalid(s_arvalid), .s_arready(s_arready), .s_araddr(s_araddr), .s_arlen(s_arlen),
        .s_arsize(s_arsize), .s_arburst(s_arburst), .s_arid(s_arid),
        .s_rvalid(s_rvalid), .s_rready(s_rready), .s_rdata(s_rdata), .s_rresp(s_rresp),
        .s_rlast(s_rlast), .s_rid(s_rid),

        .ddr_cmd_valid(ddr_cmd_valid), .ddr_cmd_ready(ddr_cmd_ready),
        .ddr_cmd_addr(ddr_cmd_addr), .ddr_cmd_write(ddr_cmd_write), .ddr_cmd_len(ddr_cmd_len),
        .ddr_wdata_valid(ddr_wdata_valid), .ddr_wdata_ready(ddr_wdata_ready),
        .ddr_wdata(ddr_wdata), .ddr_wstrb(ddr_wstrb), .ddr_wdata_last(ddr_wdata_last),
        .ddr_rdata_valid(ddr_rdata_valid), .ddr_rdata_ready(ddr_rdata_ready),
        .ddr_rdata(ddr_rdata), .ddr_rdata_last(ddr_rdata_last)
    );

endmodule
