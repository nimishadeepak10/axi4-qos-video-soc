// axi4_read_arb.sv
// Author: Nimisha Deepak
// QoS-weighted, anti-starvation arbiter + mux for the AR/R channels of 3
// AXI4 master ports funneling into a single downstream AXI4 slave port.
// Mirrors axi4_write_arb.sv's policy and structure exactly, including the
// fixed-index tournament compare tree (see that file's header comment for
// the full rationale) and the one-cycle-ahead pick_r/found_r pipelining.

`timescale 1ns / 1ps

module axi4_read_arb #(
    parameter int NM        = 3,   // this arbiter's compare tree is hand-built for NM==3
    parameter int AW        = 24,
    parameter int DW        = 32,
    parameter int IDW       = 2,
    parameter int LENW      = 8,
    parameter int QOSW      = 4,
    parameter int AGE_SHIFT = 3,
    parameter int AGE_CAP   = 15
) (
    input  logic                     clk,
    input  logic                     rst_n,

    input  logic [NM-1:0]            s_arvalid,
    output logic [NM-1:0]            s_arready,
    input  logic [NM*AW-1:0]         s_araddr,
    input  logic [NM*LENW-1:0]       s_arlen,
    input  logic [NM*3-1:0]          s_arsize,
    input  logic [NM*2-1:0]          s_arburst,
    input  logic [NM*QOSW-1:0]       s_arqos,
    input  logic [NM*IDW-1:0]        s_arid,

    output logic [NM-1:0]            s_rvalid,
    input  logic [NM-1:0]            s_rready,
    output logic [NM*DW-1:0]         s_rdata,
    output logic [NM*2-1:0]          s_rresp,
    output logic [NM-1:0]            s_rlast,
    output logic [NM*IDW-1:0]        s_rid,

    output logic                     m_arvalid,
    input  logic                     m_arready,
    output logic [AW-1:0]            m_araddr,
    output logic [LENW-1:0]          m_arlen,
    output logic [2:0]               m_arsize,
    output logic [1:0]               m_arburst,
    output logic [QOSW-1:0]          m_arqos,
    output logic [IDW-1:0]           m_arid,

    input  logic                     m_rvalid,
    output logic                     m_rready,
    input  logic [DW-1:0]            m_rdata,
    input  logic [1:0]               m_rresp,
    input  logic                     m_rlast,
    input  logic [IDW-1:0]           m_rid,

    output logic [NM-1:0]            grant_onehot
);

    localparam logic [1:0] ST_IDLE = 2'd0, ST_ADDR = 2'd1, ST_DATA = 2'd2;
    logic [1:0] state;
    logic [1:0] winner, rr_ptr;
    logic [7:0] wait_cnt [0:NM-1];

    function automatic [7:0] eff_prio(input [QOSW-1:0] qos, input [7:0] waitc);
        logic [7:0] age;
        begin
            age = (waitc >> AGE_SHIFT);
            if (age > AGE_CAP) age = AGE_CAP[7:0];
            eff_prio = qos + age;
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NM; i++) wait_cnt[i] <= 8'd0;
        end else begin
            for (int i = 0; i < NM; i++) begin
                if (grant_onehot[i])
                    wait_cnt[i] <= 8'd0;
                else if (s_arvalid[i] && wait_cnt[i] != 8'hFF)
                    wait_cnt[i] <= wait_cnt[i] + 8'd1;
            end
        end
    end

    // Same round-1/round-2 pipelining as axi4_write_arb.sv - see that
    // file's header comment for the full rationale and the routed
    // timing measurement that motivated it.
    logic [8:0] key0, key1, key2;
    assign key0 = {eff_prio(s_arqos[0*QOSW +: QOSW], wait_cnt[0]), (rr_ptr == 2'd0)};
    assign key1 = {eff_prio(s_arqos[1*QOSW +: QOSW], wait_cnt[1]), (rr_ptr == 2'd1)};
    assign key2 = {eff_prio(s_arqos[2*QOSW +: QOSW], wait_cnt[2]), (rr_ptr == 2'd2)};

    logic sel01, v01;
    logic [8:0] key01;
    logic [1:0] idx01;
    assign sel01 = s_arvalid[0] && (!s_arvalid[1] || key0 >= key1);
    assign v01   = s_arvalid[0] || s_arvalid[1];
    assign key01 = sel01 ? key0 : key1;
    assign idx01 = sel01 ? 2'd0 : 2'd1;

    logic [8:0] key01_r, key2_r;
    logic [1:0] idx01_r;
    logic       v01_r, valid2_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key01_r  <= '0;
            idx01_r  <= 2'd0;
            v01_r    <= 1'b0;
            key2_r   <= '0;
            valid2_r <= 1'b0;
        end else begin
            key01_r  <= key01;
            idx01_r  <= idx01;
            v01_r    <= v01;
            key2_r   <= key2;
            valid2_r <= s_arvalid[2];
        end
    end

    logic selFinal, found;
    logic [1:0] pick;
    assign selFinal = v01_r && (!valid2_r || key01_r >= key2_r);
    assign found    = v01_r || valid2_r;
    assign pick     = selFinal ? idx01_r : 2'd2;

    logic [1:0] pick_r;
    logic       found_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pick_r  <= 2'd0;
            found_r <= 1'b0;
        end else begin
            pick_r  <= pick;
            found_r <= found;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            winner       <= 2'd0;
            rr_ptr       <= 2'd0;
            grant_onehot <= '0;
        end else begin
            unique case (state)
                ST_IDLE: begin
                    grant_onehot <= '0;
                    if (found_r && s_arvalid[pick_r]) begin
                        winner       <= pick_r;
                        grant_onehot <= (NM'(1'b1) << pick_r);
                        rr_ptr       <= (pick_r == NM-1) ? 2'd0 : pick_r + 2'd1;
                        state        <= ST_ADDR;
                    end
                end

                ST_ADDR: begin
                    if (m_arvalid && m_arready)
                        state <= ST_DATA;
                end

                ST_DATA: begin
                    if (m_rvalid && s_rready[winner] && m_rlast)
                        state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    always_comb begin
        m_arvalid = 1'b0;
        m_araddr  = s_araddr [winner*AW  +: AW];
        m_arlen   = s_arlen  [winner*LENW+: LENW];
        m_arsize  = s_arsize [winner*3   +: 3];
        m_arburst = s_arburst[winner*2   +: 2];
        m_arqos   = s_arqos  [winner*QOSW+: QOSW];
        m_arid    = s_arid   [winner*IDW +: IDW];

        m_rready  = 1'b0;

        if (state == ST_ADDR)
            m_arvalid = 1'b1;
        if (state == ST_DATA)
            m_rready = s_rready[winner];
    end

    genvar gi;
    generate
        for (gi = 0; gi < NM; gi = gi + 1) begin : g_mux
            assign s_arready[gi] = (state == ST_ADDR) && (winner == gi) && m_arready;
            assign s_rvalid [gi] = (state == ST_DATA) && (winner == gi) && m_rvalid;
            assign s_rdata  [gi*DW  +: DW]  = m_rdata;
            assign s_rresp  [gi*2   +: 2]   = m_rresp;
            assign s_rlast  [gi]            = m_rlast;
            assign s_rid    [gi*IDW +: IDW] = m_rid;
        end
    endgenerate

endmodule
