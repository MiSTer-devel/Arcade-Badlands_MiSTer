`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the PLAYFIELD path: the 9V/7V word latches, the 18S bank flop
//  and 16P /PFBEN0/1, the six-27512 stamp fetch, 11V's palette latch, the
//  7T/7R pixel shifters and 7N's drive onto /CRAMA7:1.  Schematic sheet 3.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The VRAM word: D15:13 palette (-> 11V LS175 -> 7N -> /CRAMA7:5), D12 =
//  PFA17 and D11 = PFA16 (chip selects, not ROM address bits — the "PFA" names
//  are left over from an abandoned one-big-ROM layout), D10:0 = stamp -> ROM
//  A15:A5.
//
//  /PFBEN1 = NAND(18S Q, PFA17) and /PFBEN0 = NAND(18S /Q, PFA17), so exactly
//  one of the six stamp ROMs is ever enabled:
//
//      D12 bank D11 | device | tiles offset   MAME code
//       0    x   0  |  4N    | 0x00000        0x000-0x7FF
//       0    x   1  |  2N    | 0x10000        0x800-0xFFF
//       1    0   0  |  4S    | 0x20000        0x1000-0x17FF
//       1    0   1  |  2S    | 0x30000        0x1800-0x1FFF
//       1    1   0  |  4U    | 0x40000        0x2000-0x27FF
//       1    1   1  |  2U    | 0x50000        0x2800-0x2FFF
//
//  MAME's `code = (data & 0x1fff) + (BIT(data,12) ? (bank << 12) : 0)` is the
//  same map flattened into one index.  The ROM data bus is 8 bits, so a fetch
//  is two pixels and four fetches cover one 8-pixel stamp row.
//
//  The pipeline, inside a group of eight counts c: the PF word is complete on
//  RAMD15:0 at 8c+3, where 9V/7V latch it (4H rising) and PFA17/16 and A15:A5
//  settle; ROM byte 0 ({/4H,2H} = 00) is read over 8c+4 and 8c+5; at 8c+5 11V
//  latches the palette (4HD2H rising) and 7T/7R latch byte 0 (/1H rising);
//  bytes 1, 2 and 3 follow every two counts.  So a stamp fetched in group c
//  puts its 8 pixels on /PF3:0 at H = 8c+6 .. 8c+13 — a fixed +6-count offset.
//  Each edge above is the `ce_7m` cycle of that count, since `ce_7m` marks the
//  last clk_sys cycle of a count, and 456 mod 8 = 0, so the four 4H-derived
//  edges are periodic mod 8 with no line-wrap anomaly.
//
//  Pen polarity: the shifter outputs /PF3:0 are active low and the 27512 holds
//  exactly that polarity, so nothing here inverts.  Byte D7:D4 is the even
//  pixel and D3:D0 the odd one, pen bit 3 = the nibble's MSB.  A playfield pen
//  of 0 is a normal opaque colour on this board — GAL 9N tests /PF3 only, so
//  the playfield has no transparency.
//
//  18S 74F74 half 1 has /PRE and /CLR both tied to +5 V, so the bank bit's
//  power-on value is indeterminate; `BANK_POR` parameterises it and 0 matches
//  MAME.
//============================================================================

module badlands_pf #(
	// Power-on value of the 18S bank bit (see the header).
	parameter bit BANK_POR = 1'b0
)(
	input  logic        clk,          // clk_sys
	input  logic        reset,        // synchronous, active high
	input  logic        ce_7m,        // marks the last clk_sys cycle of count h
	input  logic        ce_m7_fall,   // 7N latches on /7M rising = 7M falling

	// Only the low three H and V bits reach sheet 3: the stamp arrives through
	// the VRAM word, the row is {4V,2V,1V} and the byte index is {/4H,2H}.
	input  logic [2:0]  h,            // 4H, 2H, 1H
	input  logic [2:0]  v,            // 4V, 2V, 1V  -> ROM A4:A2

	// ---- VRAM video port -------------------------------------------------
	input  logic [15:0] ramd,         // RAMD15:0
	input  logic        ramd_pf_stb,  // high for the whole count h mod 8 == 3

	// ---- 18S bank flop: /PFBANK (0xFEC001 write, D0) ---------------------
	input  logic        pfbank_wr,    // one clk_sys pulse at the /PFBANK edge
	input  logic        pfbank_d0,

	// ---- gfx client (tiles region) ---------------------------------------
	output logic        pf_req,
	output logic [18:0] pf_addr,
	input  logic        pf_ack,
	input  logic [7:0]  pf_data,

	// ---- outputs ---------------------------------------------------------
	output logic [3:0]  pf_pen_n,     // /PF3:0 — active low, 7T/7R wire-tied Q
	output logic [2:0]  pf_pal_n,     // 11V /Q2:0 = NOT palette[2:0]
	output logic [6:0]  cram_pf_n,    // 7N LS374 Q = /CRAMA7:1 while /PF is low
	output logic        pf_bank,      // 18S Q
	output logic        pf_late       // the gfx client missed its 2-count window
);

	// =====================================================================
	// 1. 18S 74F74 half 1 — the playfield bank bit
	// =====================================================================
	// D = the 68000's D0, CK = /PFBANK rising, /S = /CLR = +5 V, no reset.
	logic bank;
	always_ff @(posedge clk) begin
		if (reset)           bank <= BANK_POR;
		else if (pfbank_wr)  bank <= pfbank_d0;
	end
	assign pf_bank = bank;

	// =====================================================================
	// 2. 9V / 7V LS374 — the RAM word, latched on 4H rising
	// =====================================================================
	// 4H rises at H = 8c+4, i.e. on the ce_7m that ends count 8c+3 — the count
	// in which RAMD15:0 is the complete PF word (`ramd_pf_stb`).
	logic [15:0] word_r;
	wire         word_latch = ce_7m & ramd_pf_stb;
	always_ff @(posedge clk) begin
		if (reset)           word_r <= 16'h0000;
		else if (word_latch) word_r <= ramd;
	end

	// The byte-0 fetch is issued on the same edge, so its address must use the
	// word being latched, not the one being replaced.  Only the ROM-address and
	// chip-select fields are needed here; the palette (D15:13) is taken from the
	// 9V latch by 11V two counts later.
	wire [12:0] word_now = word_latch ? ramd[12:0] : word_r[12:0];

	// =====================================================================
	// 3. The device select — 16P F00 and the six /CE + /OE pairs
	// =====================================================================
	function automatic logic [2:0] device(input logic pfa17, input logic pfa16,
	                                      input logic bnk);
		begin
			if (!pfa17)      device = {2'b00, pfa16};                // 4N / 2N
			else if (!bnk)   device = pfa16 ? 3'd3 : 3'd2;           // 2S / 4S
			else             device = pfa16 ? 3'd5 : 3'd4;           // 2U / 4U
		end
	endfunction

	// =====================================================================
	// 4. The fetch — one byte per two H counts
	// =====================================================================
	// A fetch is pulsed on the ce_7m that ends an odd count, so `req` rises on
	// the boundary into the even count where the ROM address window opens and
	// the client has the full 16 clk_sys of the 2-count window.  A fetch only
	// happens with 1H = 1, so H+1 in the {4H,2H} field is just h[2:1] + 1 with
	// 1H going to 0.
	wire [1:0]  h_next21 = h[2:1] + 2'd1;
	wire        do_fetch = ce_7m & h[0];

	// ROM pins: A15:A5 = stamp[10:0], A4:A2 = {4V,2V,1V}, A1:A0 = {/4H, 2H},
	// all evaluated for the first count of the 2-count window.
	wire [10:0] stamp_now = word_now[10:0];
	wire [1:0]  byte_idx  = { ~h_next21[1], h_next21[0] };
	wire [18:0] fetch_a   = { device(word_now[12], word_now[11], bank),
	                          stamp_now, v, byte_idx };

	// The client presents `data` combinationally on its acknowledge cycle, so
	// the shifters can still take a byte that arrives in the last clk_sys cycle
	// of the 2-count window.
	wire [7:0] rom_byte;
	wire       rom_valid;

	badlands_gfx_client #(.AW(19)) u_client (
		.clk(clk), .reset(reset),
		.fetch(do_fetch), .fetch_addr(fetch_a),
		.data(rom_byte), .data_valid(rom_valid), .late(pf_late),
		.req(pf_req), .addr(pf_addr), .ack(pf_ack), .rdata(pf_data)
	);

	// =====================================================================
	// 5. 7T / 7R F374 — the pixel pair
	// =====================================================================
	// Both latch on /1H rising (= 1H falling = entry into an even count) and
	// their Q pins are wire-tied: 7T drives while 1H = 0, 7R while 1H = 1.
	logic [7:0] pen_byte;
	// `rom_valid` is not consulted: the fetch schedule is fixed by the raster,
	// `pf_late` flags a missed byte, and the board's F374 latches
	// unconditionally too.
	// verilator lint_off UNUSEDSIGNAL
	wire        unused_ok = rom_valid;
	// verilator lint_on UNUSEDSIGNAL
	always_ff @(posedge clk) begin
		if (reset)            pen_byte <= 8'hFF;   // all-ones = pen 0
		else if (do_fetch)    pen_byte <= rom_byte;
	end
	assign pf_pen_n = h[0] ? pen_byte[3:0] : pen_byte[7:4];

	// =====================================================================
	// 6. 11V LS175 — the palette, latched on 4HD2H rising
	// =====================================================================
	// 4HD2H rises at H = 8c+6, i.e. on the ce_7m ending count 8c+5, and its D
	// inputs are the 9V-latched RAMD15:13, not live RAMD.  Only the /Q outputs
	// are used, so this register holds the complement.
	always_ff @(posedge clk) begin
		if (reset)                             pf_pal_n <= 3'b111;
		else if (ce_7m && (h[2:0] == 3'd5))    pf_pal_n <= ~word_r[15:13];
	end

	// =====================================================================
	// 7. 7N LS374 — the drive onto /CRAMA7:1
	// =====================================================================
	// CK = /7M rising = 7M falling (the middle of a count); /OE = /PF, so 7N
	// owns the shared bus only while the priority logic has selected the
	// playfield.  The tri-state mux itself lives in badlands_video.sv.
	always_ff @(posedge clk) begin
		if (reset)           cram_pf_n <= 7'h7F;
		else if (ce_m7_fall) cram_pf_n <= { pf_pal_n, pf_pen_n };
	end

endmodule
