# QoS AXI4 Memory Subsystem for a Video SoC

Author: Nimisha Deepak

A 3-master AXI4 memory subsystem for a video SoC, in SystemVerilog. Three masters (CPU, video codec, and an internal 2D DMA engine) share one memory path through QoS-weighted arbitration, a triple-buffered frame manager keeps a display path and a capture path from colliding, and an AXI4-to-DDR bridge talks to a behavioral DDR model. Verified with a self-checking testbench and real SystemVerilog covergroups in Vivado XSim, synthesized and placed and routed in Vivado, and cross-checked with a Python bandwidth-analysis script.

## Architecture

```
   M0 = CPU traffic  ──┐
   (AXI4 slave port)   │
                        ├─▶ QoS write arbiter ─┐
   M1 = Codec traffic ─┤   QoS read  arbiter ─┼─▶ axi4_ddr_ctrl ──▶ (off-chip, sim-only)
   (AXI4 slave port)    │   (fixed-index        │   AXI4-to-DDR      ddr_behavioral_model
                        │    tournament          │   bridge:          - 16MB array
   2D DMA (M2) ─────────┘    compare, see        │   wr/rd FIFOs      - 14-cycle CAS latency
   frame-aware via              below)            │   (BRAM-backed)    - 1 beat/cycle burst
   frame_buffer_mgr                                │
        ▲                                          └── ddr_cmd/wdata/rdata (simple sync handshake)
        │ capture_frame_done / display_frame_done
   triple buffer: front (display) / back (capture) / ready (spare)
```

- **`rtl/axi4_write_arb.sv` / `rtl/axi4_read_arb.sv`** QoS-weighted arbitration with anti-starvation aging, built around a fixed-index tournament compare (candidate 0 vs 1, then winner vs 2) instead of a rotating-pointer scan-and-select loop, since the scan loop's data-dependent index mux turned out to be the critical-path bottleneck. The priority decision is pipelined one cycle ahead (`pick_r`/`found_r`) before being acted on, and the tournament's two comparison rounds are themselves split across a pipeline stage (`key01_r`/`idx01_r`/`v01_r`/`key2_r`/`valid2_r`) for timing.
- **`rtl/axi4_ddr_ctrl.sv`** AXI4-to-DDR bridge. Terminates the arbitrated AW/W/B and AR/R channels, buffers a full max-length AXI4 burst (256 beats) of write and read data each in on-chip FIFOs (`rtl/fifo_sync.sv`, BRAM-backed), and drives a simple synchronous command/data interface to the DDR model. Writes are posted: BRESP returns once data is captured into the FIFO, not once DRAM has actually stored it.
- **`rtl/axi4_dma_2d.sv`** 2D block-copy DMA. Copies rows of words from a source address to a destination address with independent source/destination strides, using a small row-buffer scratch memory (`row_buf`) between the read burst and write burst of each row.
- **`rtl/frame_buffer_mgr.sv`** Lock-free triple buffer (`front`/`back`/`ready`, swap-with-spare on `capture_frame_done` and `display_frame_done`). Handles the case where both events land on the same clock cycle with an explicit combined-case branch, instead of two independent `if` statements that would silently corrupt the index mapping.
- **`rtl/ddr_behavioral_model.sv`** Simulation-only, never synthesized: a 16MB array behind a command/data handshake with a fixed 14-cycle CAS-like read latency, so the controller is verified against DDR-like timing instead of an instant single-cycle memory.
- **`rtl/axi4_qos_videosoc_top.sv`** Top level: 2 external AXI4 slave ports (CPU, codec) plus the DMA as an internal third master, arbitrated onto the DDR bridge, with the frame manager alongside.

## Verification

`tb/axi4_qos_video_tb.sv` is self-checking: every write is shadowed in a reference model and every read is checked against it. Functional coverage is measured with real SystemVerilog `covergroup`s, sampled at each completed transaction and each frame-buffer swap, run in Vivado XSim (Icarus Verilog is used separately for fast functional iteration, since it doesn't support `covergroup`, guarded by `` `ifdef ICARUS_SIM ``).

| Metric | Result |
|---|---|
| Directed tests | 12 (CPU/codec single and burst r/w, concurrent masters, QoS priority under a codec flood, aging-based starvation recovery, DMA frame-capture into the triple buffer, non-unit-stride DMA, triple-buffer rotation correctness, 3-way CPU+codec+DMA contention with bandwidth measurement, back-to-back DMA plus burst-length boundary, DDR-model read-latency characterization) |
| Self-checked assertions | 1488 |
| Failures | 0 |
| AXI transaction cross coverage (`cg_axi_txn`: master x read/write x QoS bucket x burst-length class) | 92.78% |
| Frame-buffer-swap cross coverage (`cg_frame_swap`: write-buffer-id x read-buffer-id) | 70.37% |
| Scenario coverage (priority-inversion, DMA stride, aging-recovery bins) | 100% |

Bugs found and fixed during development:

1. **DDR model issuing one extra read beat.** The read-issue logic was gated on `state == CMD_READ_BURST`, but the FSM's own transition to `CMD_IDLE` happens one cycle after the last beat is accepted, not on the same cycle it's issued. On that transition cycle the old gate fired once more, corrupting the next transaction's first read. Fixed by decoupling issuing onto its own counter (`read_beat_idx <= len_r`), independent of `state`.
2. **That counter overflowing on a 256-beat burst.** `read_beat_idx` was sized 8 bits (0-255) to match `len_r`, but needs to represent 256 to correctly stop issuing. At 255+1 it wrapped to 0 and the `<= len_r` guard passed again, issuing a spurious 257th beat. Fixed by widening the counter to 9 bits.
3. **Frame manager could collide front and back on a simultaneous event.** Two independent `if` statements for `capture_frame_done`/`display_frame_done` would corrupt the index mapping if both fired the same cycle. Fixed with an explicit three-way case (both together, capture-only, display-only).
4. **A deferred BRAM write captured the wrong beat's data.** While rewriting `row_buf` so Vivado would actually infer Block RAM for it (see Synthesis below), the write enable/address were registered one cycle ahead of the array write, but the write data was still read live off the streaming AXI bus at write-execution time instead of being captured alongside the address, so each word landed one beat late. Fixed by adding a held register (`mem_wdata_hold`) captured on the same cycle as the write address.

Run it yourself:
```bash
# Fast functional iteration (Icarus, no coverage)
iverilog -g2012 -D ICARUS_SIM -o sim/tb.vvp tb/axi4_qos_video_tb.sv rtl/*.sv
vvp sim/tb.vvp

# Official run with real covergroups (Vivado XSim)
cd sim
xvlog -sv ../rtl/*.sv ../tb/axi4_qos_video_tb.sv
xelab -debug typical axi4_qos_video_tb -s video_sim -timescale 1ns/1ps -cov_db_name video_cov
xsim video_sim -R
```

## Bandwidth analysis

`scripts/bandwidth_analysis.py` reads the CSV transaction log the testbench writes (`bandwidth_log.csv`) and computes sustained throughput from measured cycle counts:

```
$ python scripts/bandwidth_analysis.py --csv bandwidth_log.csv --freq-mhz 208.59
 DMA transfer under contention: 16320 bytes in 7560 cycles
 Measured throughput          : 2.159 bytes/cycle
 Projected sustained bandwidth: 429.4 MB/s
```

This is measured over one representative contended DMA transfer (255 words/row, 16 rows, concurrent with CPU write traffic and codec read/write burst traffic) and scaled by the real achieved clock frequency (208.6MHz, see Synthesis below), not a full 1920x1080 frame simulated beat-by-beat (that's over a million AXI beats and not practical to simulate directly). The gap versus a target of 520MB/s comes from arbitration and DDR-latency overhead under 3-way contention; closing it further would mean deeper per-command pipelining in `axi4_ddr_ctrl` or overlapping the DMA's read and write phases, which are currently sequential per row.

## Synthesis (Vivado, full place and route)

Targets `xc7a100tcsg324-1` as a same-fabric stand-in for Zynq-7020, since this Vivado install doesn't have the Zynq device-support package installed. Both parts share the identical 7-series logic fabric (same slice/LUT/FF/BRAM/CARRY4 primitives).

| Metric | Result |
|---|---|
| Clock | 200MHz constraint, met (WNS positive, real achievable about 208.6MHz) |
| WNS / TNS | +0.206ns / 0.000ns (0 of 892 endpoints failing) |
| Slice LUTs | 507 (0.80% of 63,400) |
| Registers (FF) | 420 (0.33% of 126,800) |
| Block RAM | 2 tiles (1x RAMB36E1 + 2x RAMB18E1, 1.48%) |

Reports: `vivado/reports/utilization.rpt`, `vivado/reports/timing_summary.rpt`, `vivado/reports/timing_worst_paths.rpt`.

Getting timing to close took two changes:

1. **Arbiter tournament pipelining.** The 2-round fixed-index priority compare was originally combinational end-to-end in one cycle, and the critical path ran through both chained 9-bit compares. Splitting it with a pipeline register between round 1 and round 2 means each cycle only completes one compare instead of two chained ones, which took WNS from -1.141ns to -0.473ns at the cost of one extra arbitration cycle of latency (safe, since AXI4 holds VALID until READY).
2. **`row_buf` BRAM inference in the DMA.** The original code mixed a literal index and a dynamic `beat_idx`-based index for `row_buf`'s read port across two FSM states feeding into the same reset-bearing always block, so Vivado didn't recognize it as a memory and built it out of about 8,192 individual flip-flops with a wide fanout decode network, which then became the new critical path. Rewriting it to the same reset-free, registered-address BRAM template used in `fifo_sync.sv` let Vivado infer real Block RAM: LUTs dropped from 3,203 to 507 and registers from 8,674 to 420, and the critical path moved off `row_buf` entirely, closing timing with +0.206ns to spare.

The design keeps almost no on-chip data storage by design (one 256-deep burst FIFO pair plus a 16-word DMA row buffer), since frame data lives in the off-chip-modeled DDR, which is why LUT/FF/BRAM usage comes in well under a typical budget for this class of design.

## Reproducing these results

```bash
# Functional simulation (fast, no coverage)
iverilog -g2012 -D ICARUS_SIM -o sim/tb.vvp tb/axi4_qos_video_tb.sv rtl/*.sv
vvp sim/tb.vvp

# Official coverage run (Vivado XSim)
cd sim && xvlog -sv ../rtl/*.sv ../tb/axi4_qos_video_tb.sv && \
  xelab -debug typical axi4_qos_video_tb -s video_sim -timescale 1ns/1ps -cov_db_name video_cov && \
  xsim video_sim -R

# Bandwidth analysis
python scripts/bandwidth_analysis.py --csv sim/bandwidth_log.csv

# Vivado synthesis + full place & route (writes vivado/reports/*)
vivado -mode batch -source vivado/synth.tcl
```
