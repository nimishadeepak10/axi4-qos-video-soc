// frame_buffer_mgr.sv
// Author: Nimisha Deepak
// Classic lock-free triple buffer: three frame-sized regions in DDR
// address space, addressed by base = buf_id * FRAME_STRIDE_BYTES. At any
// time, `front` is being displayed/read, `back` is being written by the
// capture/decode producer, and `ready` holds the most recently completed
// frame not yet picked up for display - all three indices are always
// pairwise distinct.
//
// - On capture_frame_done: swap(back, ready), mark a fresh frame ready.
//   The producer's new `back` is whatever was `ready` before (the oldest
//   buffer nobody is currently touching), so it never collides with the
//   buffer currently being displayed.
// - On display_frame_done: if a fresh frame is ready, swap(front, ready)
//   so the consumer picks it up; otherwise keep displaying the current
//   `front` (no new frame yet - a normal, harmless case, not an error).
//
// Because every swap only ever involves `ready`, and front/back/ready
// start as the 3 distinct indices {0,1,2}, they remain pairwise distinct
// after any sequence of swaps - the producer can never be told to write
// into the buffer currently being displayed, and vice versa, without any
// blocking or handshake between producer and consumer.

`timescale 1ns / 1ps

module frame_buffer_mgr #(
    parameter int AW                 = 24,
    parameter int FRAME_STRIDE_BYTES = 1 << 22   // 4MB per buffer slot (1080p YUV422 fits with margin)
) (
    input  logic             clk,
    input  logic             rst_n,

    input  logic             capture_frame_done,   // producer finished writing `back`
    input  logic             display_frame_done,   // consumer finished reading `front`, wants next

    output logic [AW-1:0]    wr_buf_base,          // producer target this cycle onward
    output logic [AW-1:0]    rd_buf_base,          // consumer target this cycle onward
    output logic [1:0]       wr_buf_id,
    output logic [1:0]       rd_buf_id,
    output logic             frame_ready_pending    // debug/coverage: a completed frame is waiting to be displayed
);

    logic [1:0] front_idx, back_idx, ready_idx;
    logic       ready_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            front_idx   <= 2'd0;
            back_idx    <= 2'd1;
            ready_idx   <= 2'd2;
            ready_valid <= 1'b0;
        end else begin
            // Mutually exclusive branches - both events landing on the
            // same cycle need their own combined transition, not two
            // independent sequential swaps (naively doing swap(back,ready)
            // then swap(front,ready) in the same cycle would make front
            // and ready both land on the old `ready` index, colliding
            // with back - handled explicitly as its own case instead).
            if (capture_frame_done && display_frame_done) begin
                // Producer finished `back` right as the consumer asked
                // for the next frame: display jumps straight to the
                // freshly completed buffer, producer resumes into
                // whatever the consumer just vacated. `ready` (still the
                // old spare) is untouched either way.
                front_idx   <= back_idx;
                back_idx    <= front_idx;
                ready_valid <= 1'b0;
            end else if (capture_frame_done) begin
                // swap(back, ready)
                back_idx    <= ready_idx;
                ready_idx   <= back_idx;
                ready_valid <= 1'b1;
            end else if (display_frame_done && ready_valid) begin
                // swap(front, ready)
                front_idx   <= ready_idx;
                ready_idx   <= front_idx;
                ready_valid <= 1'b0;
            end
            // display_frame_done with !ready_valid: no new frame yet,
            // keep showing the current front - not an error.
        end
    end

    assign wr_buf_id           = back_idx;
    assign rd_buf_id           = front_idx;
    assign wr_buf_base         = back_idx  * FRAME_STRIDE_BYTES;
    assign rd_buf_base         = front_idx * FRAME_STRIDE_BYTES;
    assign frame_ready_pending = ready_valid;

endmodule
