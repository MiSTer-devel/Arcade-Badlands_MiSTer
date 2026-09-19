`timescale 1ns/1ps
//============================================================================
//  Bad Lands graphics memory -- the two byte-wide read clients (playfield and
//  motion object stamp ROMs) plus the loader write port, arbitrated onto the
//  single-port `badlands_sdram`.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Ported from
//  `Arcade-Blasteroids_MiSTer/rtl/mem/blstroid_gfx_mem.sv` (GPL-3.0,
//  same author; itself from Xybots and Toobin').  The byte interface, the
//  per-client row buffers and `rfsh_ok` are Bad Lands' own.
//
//  Why byte clients: the board enables exactly one 27512 at a time on the
//  playfield side (/CE and /OE from PFA17 / PFA16 / /PFBEN0 / /PFBEN1) and on
//  the motion object side (from MOA17 / MOA16), and each fetch is 8 bits =
//  2 pixels.  One SDRAM access per byte would be four times the traffic
//  needed, so each client keeps one stamp-row buffer and only a miss reaches
//  the SDRAM:
//
//    client | row address bits                 | fetched as         | hits
//    PF     | A1:A0 = {/4H, 2H} over 8 counts  | burst-2 at         | 3 of 4
//           |                                  | {addr[18:2],2'b0}  |
//    MO     | A2:A0 = {/8HD6H,/4HD2H,/2H}      | burst-4 at         | 7 of 8
//           | over 16 counts                   | {addr[17:3],3'b0}  |
//
//  Protocol: raise `*_req` with the address and hold both; `*_ack` is a
//  one-clk_sys pulse, and `*_data` is valid in that cycle and held until this
//  client's next ack; then drop `*_req`.  The arbiter waits for the drop
//  before serving that client again, so one request = one transaction.
//
//  Latency bound, 16 clk_sys = 2 H counts: the PF stamp ROM address settles at
//  H = 8c+4 and the byte is latched entering 8c+6; an MO byte read during
//  counts (k, k+1) is latched entering k+2.  A buffer hit acks in the next
//  clk_sys cycle, a PF miss takes ~13 clk and an MO miss ~15.  Two things keep
//  the bound met: the clients cannot miss in the same window (a PF miss only
//  at H mod 8 == 4, an MO miss only at H mod 16 == 14, and neither burst is
//  still running when the other starts), and AUTO_REFRESH is fenced off by
//  `rfsh_ok`, the one addition to the ported Blasteroids controller.
//
//  Arbitration: the loader write port beats both readers, but it only runs
//  while `ioctl_download` is high, when neither reader exists yet.  The two
//  readers alternate -- when both ask, whichever was not served last goes
//  first -- rather than running a fixed priority, because both layers have a
//  hard per-stamp deadline and neither may be the permanent loser.
//
//  Controller contract: when `sd_ready` is high, pulse `sd_req` for one cycle
//  with addr/we/blen(/wdata); read data returns on `sd_rdata` with one
//  `sd_valid` pulse per word, consecutive for a burst, and `sd_ready` returns
//  high when the access completes.  A request coinciding with a due refresh is
//  latched inside the controller, so the single-cycle pulse is safe.
//
//  Word packing: the even SDRAM byte address is in D7:0 and the odd in D15:8,
//  so a burst's first word carries bytes 0 and 1, the second bytes 2 and 3,
//  and a row buffer is simply `{... , w1, w0}` with byte k at bits [8k+7:8k].
//============================================================================

module badlands_gfx_mem #(
	parameter int AW = 24,
	parameter logic [AW-1:0] TILE_BASE = 24'h000000,   // WORD address of `tiles`
	parameter logic [AW-1:0] SPR_BASE  = 24'h030000    // WORD address of `sprites`
)(
	input  logic          clk,          // clk_sys 57.272724 MHz
	input  logic          reset,

	// ---- playfield stamp-byte client ----
	input  logic          pf_req,       // level, held with the address
	input  logic [18:0]   pf_addr,      // byte address inside `tiles`, 0..0x5FFFF
	output logic          pf_ack,       // one-cycle pulse; pf_data valid in it
	output logic  [7:0]   pf_data,      // held until this client's next ack

	// ---- motion-object stamp-byte client ----
	input  logic          mo_req,
	input  logic [17:0]   mo_addr,      // byte address inside `sprites`, 0..0x2FFFF
	output logic          mo_ack,
	output logic  [7:0]   mo_data,

	// ---- the refresh window ----
	input  logic  [2:0]   h_mod8,       // H counter bits 2:0
	output logic          rfsh_ok,      // -> badlands_sdram

	// ---- loader write port (word address, from badlands_sdram_loader) ----
	input  logic          dl_wr,
	input  logic [AW-1:0] dl_waddr,
	input  logic [15:0]   dl_wdata,
	output logic          dl_ack,

	// ---- badlands_sdram controller port ----
	output logic          sd_req,
	output logic [AW-1:0] sd_addr,
	output logic          sd_we,
	output logic  [1:0]   sd_blen,
	output logic [15:0]   sd_wdata,
	input  logic          sd_ready,
	input  logic          sd_valid,
	input  logic [15:0]   sd_rdata
);

	// rfsh_ok is assigned below, once pf_need / mo_need exist.

	// ---- the two row buffers --------------------------------------------
	logic [31:0] pf_buf;   logic [16:0] pf_tag;   logic pf_bufv;
	logic [63:0] mo_buf;   logic [14:0] mo_tag;   logic mo_bufv;

	wire pf_hit = pf_bufv && (pf_tag == pf_addr[18:2]);
	wire mo_hit = mo_bufv && (mo_tag == mo_addr[17:3]);

	// ---- per-client request state ---------------------------------------
	typedef enum logic [1:0] { C_IDLE, C_MISS, C_HOLD } cstate_t;
	cstate_t pf_st, mo_st;

	// the central FSM raises these for one cycle when a fill completes; the
	// buffer and its tag are updated on the SAME edge, so the client reads
	// pf_buf / mo_buf directly in that cycle.
	logic pf_fill, mo_fill;

	// ---- the central arbiter / SDRAM sequencer --------------------------
	typedef enum logic [1:0] { G_IDLE, G_REQ, G_WAIT, G_HOLD } gstate_t;
	gstate_t st;

	typedef enum logic [1:0] { OP_WR, OP_MO, OP_PF } op_t;
	op_t op;

	logic [1:0]  wcnt;
	logic [47:0] wbuf;
	logic        last_mo;

	wire pf_need = (pf_st == C_MISS);
	wire mo_need = (mo_st == C_MISS);

	// ---- the refresh window ----------------------------------------------
	// A refresh may start only at h_mod8 == 0 and only while neither client is
	// asking.  Both conditions are needed, because a refresh (5 clk) and a
	// miss (13 clk) do not both fit between two requests, and a request is a
	// level held with the address, so `*_need` covers every cycle of a fill.
	// Fetches are issued every two H counts, so a request can only rise at
	// clk 0, 16, 32 or 48 of a group; a refresh started anywhere inside
	// h_mod8 == 0 (clk 0..7) is finished by clk 12, before the clk-16 request.
	// The window is 8 clk of every 64 and tREFI is 446 clk_sys, so about seven
	// windows pass per refresh -- and a deferred refresh is never dropped.
	assign rfsh_ok = (h_mod8 == 3'd0)
	              && !pf_req && !mo_req && !pf_need && !mo_need;

	// when both readers ask, the one that did NOT go last goes now
	wire take_mo = mo_need && (!pf_need || !last_mo);
	wire take_pf = pf_need && (!mo_need ||  last_mo);

	// PF: 4 bytes at {addr[18:2],2'b00} -> burst-2 from word {addr[18:2],1'b0}
	// MO: 8 bytes at {addr[17:3],3'b000} -> burst-4 from word {addr[17:3],2'b00}
	wire [AW-1:0] pf_wa = TILE_BASE + {{(AW-18){1'b0}}, pf_addr[18:2], 1'b0};
	wire [AW-1:0] mo_wa = SPR_BASE  + {{(AW-17){1'b0}}, mo_addr[17:3], 2'b00};

	// The tag is latched with the address the burst was issued for, not read
	// back from the client at fill time: a client that moved its address
	// mid-transaction would otherwise label the buffer with an address it does
	// not contain, and every later hit would be wrong.
	logic [16:0] pf_tag_l;
	logic [14:0] mo_tag_l;

	wire [1:0] wlast = (op == OP_MO) ? 2'd3 : 2'd1;

	always_ff @(posedge clk) begin
		if (reset) begin
			st <= G_IDLE; sd_req <= 1'b0; sd_we <= 1'b0; sd_blen <= 2'd0;
			dl_ack <= 1'b0; last_mo <= 1'b0;
			pf_fill <= 1'b0; mo_fill <= 1'b0;
			pf_bufv <= 1'b0; mo_bufv <= 1'b0;
		end else begin
			dl_ack  <= 1'b0;
			sd_req  <= 1'b0;
			pf_fill <= 1'b0;
			mo_fill <= 1'b0;
			case (st)
				// ---- pick a client (write first, then alternate readers) ----
				G_IDLE: begin
					wcnt <= 2'd0;
					if (dl_wr) begin
						op <= OP_WR; sd_addr <= dl_waddr; sd_we <= 1'b1;
						sd_blen <= 2'd0; sd_wdata <= dl_wdata;
						if (sd_ready) begin sd_req <= 1'b1; st <= G_REQ; end
					end else if (take_mo) begin
						op <= OP_MO; sd_addr <= mo_wa; sd_we <= 1'b0; sd_blen <= 2'd3;
						mo_tag_l <= mo_addr[17:3];
						if (sd_ready) begin sd_req <= 1'b1; st <= G_REQ; last_mo <= 1'b1; end
					end else if (take_pf) begin
						op <= OP_PF; sd_addr <= pf_wa; sd_we <= 1'b0; sd_blen <= 2'd1;
						pf_tag_l <= pf_addr[18:2];
						if (sd_ready) begin sd_req <= 1'b1; st <= G_REQ; last_mo <= 1'b0; end
					end
				end
				// ---- req asserted for exactly this cycle ----
				G_REQ: begin
					sd_req <= 1'b0; sd_we <= 1'b0;
					st <= G_WAIT;
				end
				// ---- await completion (burst words on consecutive sd_valid) ----
				G_WAIT: begin
					if (op == OP_WR) begin
						if (sd_ready) begin dl_ack <= 1'b1; st <= G_HOLD; end
					end else if (sd_valid) begin
						if (wcnt != wlast) begin
							wbuf <= {sd_rdata, wbuf[47:16]};   // shift in, low word first
							wcnt <= wcnt + 2'd1;
						end else begin
							if (op == OP_MO) begin
								mo_buf  <= {sd_rdata, wbuf[47:0]};
								mo_tag  <= mo_tag_l;
								mo_bufv <= 1'b1;
								mo_fill <= 1'b1;
							end else begin
								pf_buf  <= {sd_rdata, wbuf[47:32]};
								pf_tag  <= pf_tag_l;
								pf_bufv <= 1'b1;
								pf_fill <= 1'b1;
							end
							st <= G_HOLD;
						end
					end
				end
				// ---- one transaction per request: wait for the client to drop it ----
				G_HOLD: begin
					case (op)
						OP_WR:   if (!dl_wr)                st <= G_IDLE;
						OP_MO:   if (mo_st != C_MISS)       st <= G_IDLE;
						default: if (pf_st != C_MISS)       st <= G_IDLE;   // OP_PF
					endcase
				end
				default: st <= G_IDLE;
			endcase
		end
	end

	// ---- byte selection --------------------------------------------------
	function automatic logic [7:0] sel4(input logic [31:0] b, input logic [1:0] i);
		case (i)
			2'd0:    sel4 = b[7:0];
			2'd1:    sel4 = b[15:8];
			2'd2:    sel4 = b[23:16];
			default: sel4 = b[31:24];
		endcase
	endfunction

	function automatic logic [7:0] sel8(input logic [63:0] b, input logic [2:0] i);
		case (i)
			3'd0:    sel8 = b[7:0];
			3'd1:    sel8 = b[15:8];
			3'd2:    sel8 = b[23:16];
			3'd3:    sel8 = b[31:24];
			3'd4:    sel8 = b[39:32];
			3'd5:    sel8 = b[47:40];
			3'd6:    sel8 = b[55:48];
			default: sel8 = b[63:56];
		endcase
	endfunction

	// ---- the two client state machines -----------------------------------
	always_ff @(posedge clk) begin
		if (reset) begin
			pf_st <= C_IDLE; pf_ack <= 1'b0; pf_data <= 8'h00;
			mo_st <= C_IDLE; mo_ack <= 1'b0; mo_data <= 8'h00;
		end else begin
			pf_ack <= 1'b0;
			mo_ack <= 1'b0;

			case (pf_st)
				C_IDLE: if (pf_req) begin
					if (pf_hit) begin
						pf_data <= sel4(pf_buf, pf_addr[1:0]);
						pf_ack  <= 1'b1;
						pf_st   <= C_HOLD;
					end else pf_st <= C_MISS;
				end
				C_MISS: if (pf_fill) begin
					pf_data <= sel4(pf_buf, pf_addr[1:0]);
					pf_ack  <= 1'b1;
					pf_st   <= C_HOLD;
				end
				default: if (!pf_req) pf_st <= C_IDLE;      // C_HOLD
			endcase

			case (mo_st)
				C_IDLE: if (mo_req) begin
					if (mo_hit) begin
						mo_data <= sel8(mo_buf, mo_addr[2:0]);
						mo_ack  <= 1'b1;
						mo_st   <= C_HOLD;
					end else mo_st <= C_MISS;
				end
				C_MISS: if (mo_fill) begin
					mo_data <= sel8(mo_buf, mo_addr[2:0]);
					mo_ack  <= 1'b1;
					mo_st   <= C_HOLD;
				end
				default: if (!mo_req) mo_st <= C_IDLE;      // C_HOLD
			endcase
		end
	end

endmodule
