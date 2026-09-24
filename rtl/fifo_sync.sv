// fifo_sync.sv
// Author: Nimisha Deepak
// Plain single-clock, first-word-fall-through synchronous FIFO, written
// to infer Block RAM: the memory array and its registered read-data
// output live in their own always_ff with no asynchronous reset (a BRAM
// primitive has none), while the pointers and valid/ready flags carry
// the reset instead. The read side holds its registered output until
// accepted, then prefetches the next word.

`timescale 1ns / 1ps

module fifo_sync #(
    parameter int WIDTH = 32,
    parameter int DEPTH = 256                 // power of 2
) (
    input  logic             clk,
    input  logic             rst_n,

    input  logic             wr_valid,
    output logic             wr_ready,
    input  logic [WIDTH-1:0] wr_data,

    output logic             rd_valid,
    input  logic             rd_ready,
    output logic [WIDTH-1:0] rd_data
);

    localparam int PTRW = $clog2(DEPTH);

    logic [WIDTH-1:0] mem [0:DEPTH-1];

    // Wrap-around pointers into the mem array; array_full only accounts
    // for words still sitting in the array (not the one word that may
    // additionally be held in the rd_data output register), so the
    // FIFO's guaranteed usable depth is DEPTH, one slot conservative -
    // simple and provably free of overflow rather than squeezing out
    // the last word of capacity.
    logic [PTRW:0] wr_ptr, rd_ptr;

    wire array_full  = (wr_ptr[PTRW] != rd_ptr[PTRW]) && (wr_ptr[PTRW-1:0] == rd_ptr[PTRW-1:0]);
    wire array_empty = (wr_ptr == rd_ptr);

    assign wr_ready = !array_full;

    logic mem_we;
    assign mem_we = wr_valid && wr_ready;

    always_ff @(posedge clk) begin
        if (mem_we)
            mem[wr_ptr[PTRW-1:0]] <= wr_data;
    end

    logic mem_re;
    assign mem_re = (!rd_valid || rd_ready) && !array_empty;

    always_ff @(posedge clk) begin
        if (mem_re)
            rd_data <= mem[rd_ptr[PTRW-1:0]];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr   <= '0;
            rd_ptr   <= '0;
            rd_valid <= 1'b0;
        end else begin
            if (mem_we) wr_ptr <= wr_ptr + 1'b1;

            if (mem_re) begin
                rd_ptr   <= rd_ptr + 1'b1;
                rd_valid <= 1'b1;
            end else if (rd_valid && rd_ready) begin
                rd_valid <= 1'b0;
            end
        end
    end

endmodule
