`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the graphics-ROM byte-fetch adapter.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  This is the only file in core/rtl/video/ that knows the SDRAM fetch
//  protocol.  The playfield (badlands_pf.sv) and the motion objects
//  (badlands_mo_fetch.sv) both ask for "one byte at this address, tell me when
//  it is here".  One instance per client, parameterised only by address width.
//
//  Why one byte, and why the budget is two H counts: both stamp paths read one
//  8-bit device at a time — the playfield's six 27512s are chip-selected so
//  that exactly one is enabled, and the three motion-object 27512s likewise.
//  Each device holds its address for two H counts and the byte is latched by
//  the F374 shifters at the boundary into the following even count:
//
//      PF: byte b is read during H = 8c+4+2b, 8c+5+2b
//      MO: byte b is read during n = 14+2b (mod 16)
//
//  so "address valid" to "byte must be latched" is 2 H counts = 16 clk_sys
//  cycles at 57.272724 MHz.  The memory side acks within 15 clk_sys of `req`
//  rising, so the consumers pulse `fetch` on the `ce_7m` cycle that ends the
//  count before the address window, `req` rises exactly on the count boundary,
//  and `data` is combinational on the acknowledge cycle so the pixel latch can
//  still take a byte that arrives in the very last cycle.
//
//  The one-cycle request gap: the arbiter waits for `req` to drop before
//  serving this client again, so one request = one transaction.  A worst-case
//  ack can land in the same cycle in which the next `fetch` is pulsed, so that
//  fetch is parked in `pend` for one cycle rather than holding `req` high
//  across two transactions.  That costs a cycle only after a 15-cycle ack,
//  i.e. only after a row-buffer miss, and the request after a miss is always a
//  hit: 3 of 4 PF bytes and 7 of 8 MO bytes come out of the client's own row
//  buffer and are acked in 1 cycle.
//
//  Each client's address is a byte offset inside its own region:
//
//      PF  pf_addr[18:0] = {device[2:0], stamp[10:0], row[2:0], /4H, 2H}
//          device = 0 4N | 1 2N | 2 4S | 3 2S | 4 4U | 5 2U
//      MO  mo_addr[17:0] = {device[1:0], RA14:RA5, RA4:RA2, /8HD6H,/4HD2H,/2H}
//          device = {MOA17,MOA16} = 0 14R | 1 10R | 2 14T
//          device 3 is socket 10T, which is not populated — badlands_mo_fetch
//          must not let that address reach memory.
//
//  The bytes are the raw ROM bytes.  MAME's `ROMREGION_INVERT` is MAME's, not
//  the board's: the pen buses `/PF3:0` and `/MO3:0` are active low, which is
//  the polarity the 27512 already holds.  Nothing in this path inverts.
//============================================================================

module badlands_gfx_client #(
	parameter int AW = 19
)(
	input  logic          clk,
	input  logic          reset,

	// ---- video side ------------------------------------------------------
	input  logic          fetch,        // one clk pulse: start a byte fetch
	input  logic [AW-1:0] fetch_addr,
	output logic [7:0]    data,         // combinational on the ack cycle
	output logic          data_valid,   // a byte has arrived for the last fetch
	output logic          late,         // a fetch arrived with one outstanding

	// ---- memory side ------------------------------------------------------
	output logic          req,          // level, held with the address
	output logic [AW-1:0] addr,         // byte address inside the region
	input  logic          ack,          // one-clk pulse; rdata valid in it
	input  logic [7:0]    rdata
);

	logic [7:0]    data_r;
	logic          pend;
	logic [AW-1:0] pend_addr;

	wire ack_now = req & ack;

	assign data = ack_now ? rdata : data_r;

	always_ff @(posedge clk) begin
		late <= 1'b0;
		if (reset) begin
			req        <= 1'b0;
			addr       <= '0;
			pend       <= 1'b0;
			pend_addr  <= '0;
			data_r     <= 8'hFF;   // an un-driven stamp bus reads all ones
			data_valid <= 1'b0;
		end else begin
			if (ack_now) begin
				data_r     <= rdata;
				data_valid <= 1'b1;
			end
			if (fetch) begin
				data_valid <= 1'b0;
				// A fetch while one is still outstanding would lose the
				// previous byte.  The outstanding request is left alone (the
				// address may not change mid-transaction) and the new fetch is
				// dropped; it does not happen at the guaranteed latency.
				if (req && !ack_now) late <= 1'b1;
			end

			if (ack_now) begin
				req <= 1'b0;                       // always give the arbiter
				if (fetch) begin                   // its one low cycle
					pend      <= 1'b1;
					pend_addr <= fetch_addr;
				end
			end else if (fetch && !req) begin
				req  <= 1'b1;
				addr <= fetch_addr;
			end else if (pend) begin
				req  <= 1'b1;
				addr <= pend_addr;
				pend <= 1'b0;
			end
		end
	end

endmodule
