`timescale 1ns/1ps
//============================================================================
//  Bad Lands SDRAM loader-writer -- packs the download byte stream into 16-bit
//  words and writes both graphics regions through the single
//  `badlands_gfx_mem` write port.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Ported from
//  `Arcade-Blasteroids_MiSTer/rtl/mem/blstroid_sdram_loader.sv`
//  (GPL-3.0, same author; itself from Xybots and Toobin').
//
//  Unlike Blasteroids, there is no half-interleave here.  Bad Lands' MO stamps
//  come from three ordinary 27512s at 14R / 10R / 14T selected by
//  {MOA17,MOA16}, so the socket index is just the top two bits of a flat byte
//  address and one stamp row is eight consecutive bytes.  Both regions map
//  linearly and the address arithmetic is a single shift:
//
//      SDRAM word address = SDRAM byte address >> 1
//
//  The map:
//    SDRAM byte 0x000000-0x05FFFF   384 KB  tiles    <- stream 0x050000
//    SDRAM byte 0x060000-0x08FFFF   192 KB  sprites  <- stream 0x0B0000
//    SDRAM byte 0x090000-                   free -- socket 10T is empty and
//                                           nothing may be mirrored there
//  Both bases are word aligned, so the map is a pure shift with no carries:
//    TILE_BASE = word 0x000000 , SPR_BASE = word 0x030000 (= byte 0x060000).
//
//  Byte lanes: the even byte address goes to D7:0 and the odd to D15:8.  This
//  is the opposite convention from the program ROM, which is big-endian
//  because the MRA interleaves it into 68000 words; the stamp ROMs are
//  byte-addressed devices with no word order of their own, so the natural
//  little-endian packing is used and both sides of the datapath agree on it.
//
//  Even and odd bytes of a word are always consecutive in the download stream,
//  so the even byte is latched and the word is emitted on the odd byte.
//  `wr_busy` is combinational and is asserted while a word is outstanding and
//  in the cycle a word completes; drive `ioctl_wait` with it so the byte
//  stream can never outrun the SDRAM.
//
//  This is a single write path through the shared `badlands_gfx_mem`
//  controller port, not a separate write client behind the read arbiter.
//  Write clients placed behind the arbiter get pruned by Quartus; after any
//  arbiter change, grep output_files/*.map.rpt for "Lost fanout" on the
//  request/we nets.
//============================================================================

module badlands_sdram_loader #(
	parameter int AW = 24,
	parameter logic [AW-1:0] TILE_BASE = 24'h000000,   // word 0 = byte 0x000000
	parameter logic [AW-1:0] SPR_BASE  = 24'h030000    // word     = byte 0x060000
)(
	input  logic          clk,
	input  logic          reset,

	// ---- byte strobes from badlands_rom_loader (the SDRAM regions only) ----
	input  logic          tiles_wr,
	input  logic [18:0]   tiles_addr,     // byte 0..0x5FFFF
	input  logic          sprites_wr,
	input  logic [17:0]   sprites_addr,   // byte 0..0x2FFFF
	input  logic  [7:0]   ld_data,
	output logic          wr_busy,        // -> ioctl_wait (back-pressure)

	// ---- gfx_mem write port ----
	output logic          dl_wr,
	output logic [AW-1:0] dl_waddr,
	output logic [15:0]   dl_wdata,
	input  logic          dl_ack
);

	// ---- is this byte the even (first) or the word-completing (odd) one? ----
	wire tile_ev = tiles_wr   && !tiles_addr[0];
	wire tile_od = tiles_wr   &&  tiles_addr[0];
	wire spr_ev  = sprites_wr && !sprites_addr[0];
	wire spr_od  = sprites_wr &&  sprites_addr[0];

	wire any_ev = tile_ev | spr_ev;
	wire any_od = tile_od | spr_od;     // word-completing

	logic [7:0] lo_buf;                 // the latched even byte -> D7:0

	wire [AW-1:0] word_addr = tile_od
		? (TILE_BASE + {{(AW-18){1'b0}}, tiles_addr[18:1]})
		: (SPR_BASE  + {{(AW-17){1'b0}}, sprites_addr[17:1]});
	wire [15:0]   word_data = {ld_data, lo_buf};   // odd -> D15:8, even -> D7:0

	typedef enum logic [0:0] { L_IDLE, L_WR } lstate_t;
	lstate_t st;

	// combinational back-pressure: busy while writing, or the cycle a word
	// completes
	assign wr_busy = (st == L_WR) || (st == L_IDLE && any_od);

	always_ff @(posedge clk) begin
		if (reset) begin
			st <= L_IDLE; dl_wr <= 1'b0;
		end else begin
			// The even byte is latched whatever the state.  Latching only in
			// L_IDLE would drop an even byte that arrived while a word was
			// outstanding, and the next odd byte would then pair with a stale
			// low byte -- a silently corrupted word rather than a lost one.
			// `wr_busy` makes that unreachable, but the guard is free.
			if (any_ev) lo_buf <= ld_data;
			case (st)
				L_IDLE: begin
					dl_wr <= 1'b0;
					if (any_od) begin
						dl_waddr <= word_addr;
						dl_wdata <= word_data;
						dl_wr    <= 1'b1;
						st       <= L_WR;
					end
				end
				L_WR: begin
					if (dl_ack) begin dl_wr <= 1'b0; st <= L_IDLE; end
				end
				default: st <= L_IDLE;
			endcase
		end
	end

	// Simulation-only overrun check, in its own process so no synthesisable
	// always_ff contains a system task.  An odd byte arriving while the
	// previous word is still outstanding cannot be recovered; it means
	// `ioctl_wait` is not wired to `wr_busy`.
`ifndef ALTERA_RESERVED_QIS
	always @(posedge clk)
		if (!reset && st == L_WR && any_od)
			$display("ERROR: badlands_sdram_loader overrun - a word-completing byte arrived while a write was outstanding; ioctl_wait is not wired to wr_busy");
`endif

endmodule
