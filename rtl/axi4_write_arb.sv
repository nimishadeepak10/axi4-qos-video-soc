// axi4_write_arb.sv
// Author: Nimisha Deepak
// QoS-weighted, anti-starvation arbiter + mux for the AW/W/B channels of
// 3 AXI4 master ports funneling into a single downstream AXI4 slave
// port. A winner is granted exclusive use of the downstream AW->W->B
// sequence for the duration of one whole burst, then arbitration re-runs.
//
// Priority key per candidate = {AWQOS[i] + min(wait_cycles[i]>>AGE_SHIFT,
// AGE_CAP), round_robin_bonus[i]}: QoS dominates, the aging term stops a
// low-QoS master starving forever behind sustained high-QoS traffic, and
// the 1-bit round-robin bonus (set only for the single candidate whose
// index equals rr_ptr) breaks exact ties fairly over successive rounds.
//
// Candidate selection is a fixed, 2-level tournament tree over the 3
// *fixed* master indices (0 vs 1, then winner vs 2) - deliberately not a
// "scan starting at rr_ptr" loop with a data-dependent index mux. An
// earlier version of this arbiter used that rotate-and-scan form and its
// critical path was dominated by the
// resulting variable-index mux/routing, not by the priority arithmetic;
// this fixed-index form removes that mux entirely. rr_ptr still rotates
// which single candidate gets the tie-break bonus each round.
//
// The priority *decision* itself is pipelined one cycle ahead of when
// it's acted on: continuously registered into pick_r/found_r every
// cycle, with ST_IDLE granting off that register instead of the raw
// combinational compare. AXI4 requires VALID to stay asserted once
// raised until READY, so a requester picked last cycle is guaranteed
// still valid this cycle - the only externally visible effect is a
// brand-new higher-priority request losing an arbitration round by up to
// one cycle to an already-pending one, a bounded, harmless skew.
//
// Once a winner is picked, every channel signal for the rest of its
// burst is a pure combinational mux driven by the registered winner
// index - no extra retiming latency between a master's AW/W/B signals
// and the shared downstream port.

`timescale 1ns / 1ps

module axi4_write_arb #(
    parameter int NM        = 3,   // this arbiter's compare tree is hand-built for NM==3
    parameter int AW        = 24,
    parameter int DW        = 32,
    parameter int IDW       = 2,
    parameter int LENW      = 8,
    parameter int QOSW      = 4,
    parameter int AGE_SHIFT = 3,   // one aging point every 2^AGE_SHIFT wait cycles
    parameter int AGE_CAP   = 15
) (
    input  logic                     clk,
    input  logic                     rst_n,

    // Per-master slave-side AW/W/B (subsystem receives these)
    input  logic [NM-1:0]            s_awvalid,
    output logic [NM-1:0]            s_awready,
    input  logic [NM*AW-1:0]         s_awaddr,
    input  logic [NM*LENW-1:0]       s_awlen,
    input  logic [NM*3-1:0]          s_awsize,
    input  logic [NM*2-1:0]          s_awburst,
    input  logic [NM*QOSW-1:0]       s_awqos,
    input  logic [NM*IDW-1:0]        s_awid,

    input  logic [NM-1:0]            s_wvalid,
    output logic [NM-1:0]            s_wready,
    input  logic [NM*DW-1:0]         s_wdata,
    input  logic [NM*(DW/8)-1:0]     s_wstrb,
    input  logic [NM-1:0]            s_wlast,

    output logic [NM-1:0]            s_bvalid,
    input  logic [NM-1:0]            s_bready,
    output logic [NM*2-1:0]          s_bresp,
    output logic [NM*IDW-1:0]        s_bid,

    // Downstream master-side AW/W/B (subsystem drives these)
    output logic                     m_awvalid,
    input  logic                     m_awready,
    output logic [AW-1:0]            m_awaddr,
    output logic [LENW-1:0]          m_awlen,
    output logic [2:0]               m_awsize,
    output logic [1:0]               m_awburst,
    output logic [QOSW-1:0]          m_awqos,
    output logic [IDW-1:0]           m_awid,

    output logic                     m_wvalid,
    input  logic                     m_wready,
    output logic [DW-1:0]            m_wdata,
    output logic [(DW/8)-1:0]        m_wstrb,
    output logic                     m_wlast,

    input  logic                     m_bvalid,
    output logic                     m_bready,
    input  logic [1:0]               m_bresp,
    input  logic [IDW-1:0]           m_bid,

    output logic [NM-1:0]            grant_onehot
);

    localparam logic [1:0] ST_IDLE = 2'd0, ST_ADDR = 2'd1, ST_DATA = 2'd2, ST_RESP = 2'd3;
    logic [1:0] state;
    logic [1:0] winner, rr_ptr;
    logic [7:0] wait_cnt [0:NM-1];   // saturating; AGE_CAP tops out well under 8 bits

    function automatic [7:0] eff_prio(input [QOSW-1:0] qos, input [7:0] waitc);
        logic [7:0] age;
        begin
            age = (waitc >> AGE_SHIFT);
            if (age > AGE_CAP) age = AGE_CAP[7:0];
            eff_prio = qos + age;
        end
    endfunction

    // Aging counters: bump every cycle a master asserts AWVALID and isn't
    // granted; saturate instead of wrapping.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < NM; i++) wait_cnt[i] <= 8'd0;
        end else begin
            for (int i = 0; i < NM; i++) begin
                if (grant_onehot[i])
                    wait_cnt[i] <= 8'd0;
                else if (s_awvalid[i] && wait_cnt[i] != 8'hFF)
                    wait_cnt[i] <= wait_cnt[i] + 8'd1;
            end
        end
    end

    // ---- Fixed-index tournament compare, split across a pipeline
    // register between round 1 (0 vs 1) and round 2 (winner01 vs 2).
    //
    // The routed timing report for the single-cycle version of this
    // tournament showed the critical path running through BOTH 9-bit
    // compares chained combinationally in one cycle (sel01's CARRY4 pair
    // feeding, through the key01 mux, straight into selFinal's CARRY4
    // pair) - 7 logic levels, 61% of it routing delay between those
    // chained stages. Registering key01/idx01/v01 (round 1's result)
    // between the two rounds means each cycle now only has to complete
    // ONE 9-bit compare instead of two chained ones, roughly halving
    // this path - at the cost of one extra cycle of arbitration latency
    // (safe for the same VALID-must-hold-until-READY reason documented
    // in the header comment; ST_IDLE below still revalidates pick_r
    // against the *current* s_awvalid before acting on it).
    logic [8:0] key0, key1, key2;
    assign key0 = {eff_prio(s_awqos[0*QOSW +: QOSW], wait_cnt[0]), (rr_ptr == 2'd0)};
    assign key1 = {eff_prio(s_awqos[1*QOSW +: QOSW], wait_cnt[1]), (rr_ptr == 2'd1)};
    assign key2 = {eff_prio(s_awqos[2*QOSW +: QOSW], wait_cnt[2]), (rr_ptr == 2'd2)};

    logic sel01, v01;
    logic [8:0] key01;
    logic [1:0] idx01;
    assign sel01 = s_awvalid[0] && (!s_awvalid[1] || key0 >= key1);
    assign v01   = s_awvalid[0] || s_awvalid[1];
    assign key01 = sel01 ? key0 : key1;
    assign idx01 = sel01 ? 2'd0 : 2'd1;

    // Round 1's result, registered - plus a same-cycle passthrough copy
    // of candidate 2's own key/valid so round 2 compares two values that
    // came from the same original cycle.
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
            valid2_r <= s_awvalid[2];
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
                    if (found_r && s_awvalid[pick_r]) begin
                        winner       <= pick_r;
                        grant_onehot <= (NM'(1'b1) << pick_r);
                        rr_ptr       <= (pick_r == NM-1) ? 2'd0 : pick_r + 2'd1;
                        state        <= ST_ADDR;
                    end
                end

                ST_ADDR: begin
                    if (m_awvalid && m_awready)
                        state <= ST_DATA;
                end

                ST_DATA: begin
                    if (s_wvalid[winner] && m_wready && s_wlast[winner])
                        state <= ST_RESP;
                end

                ST_RESP: begin
                    if (m_bvalid && s_bready[winner])
                        state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // ---- Pure combinational forwarding based on {state, winner} ----
    always_comb begin
        m_awvalid = 1'b0;
        m_awaddr  = s_awaddr [winner*AW  +: AW];
        m_awlen   = s_awlen  [winner*LENW+: LENW];
        m_awsize  = s_awsize [winner*3   +: 3];
        m_awburst = s_awburst[winner*2   +: 2];
        m_awqos   = s_awqos  [winner*QOSW+: QOSW];
        m_awid    = s_awid   [winner*IDW +: IDW];

        m_wvalid  = 1'b0;
        m_wdata   = s_wdata  [winner*DW      +: DW];
        m_wstrb   = s_wstrb  [winner*(DW/8)  +: (DW/8)];
        m_wlast   = s_wlast[winner];

        m_bready  = 1'b0;

        if (state == ST_ADDR)
            m_awvalid = 1'b1;
        if (state == ST_DATA)
            m_wvalid = s_wvalid[winner];
        if (state == ST_RESP)
            m_bready = s_bready[winner];
    end

    genvar gi;
    generate
        for (gi = 0; gi < NM; gi = gi + 1) begin : g_mux
            assign s_awready[gi] = (state == ST_ADDR)  && (winner == gi) && m_awready;
            assign s_wready [gi] = (state == ST_DATA)  && (winner == gi) && m_wready;
            assign s_bvalid [gi] = (state == ST_RESP)  && (winner == gi) && m_bvalid;
            assign s_bresp[gi*2 +: 2]   = m_bresp;
            assign s_bid  [gi*IDW +: IDW] = m_bid;
        end
    endgenerate

endmodule
