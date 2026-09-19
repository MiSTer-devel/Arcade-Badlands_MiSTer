`timescale 1ns/1ps
//============================================================================
//  Bad Lands — the motion-object STAMP FETCH: 13V LS175 and 15V F74 (the
//  MOA17/MOA16 chip selects), the three-27512 address map, and the 7T/7R HIGH
//  nibble that carries /MO3:0.  Schematic sheet 4 grids 13V, 15V, 14R/10R/14T.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The ROM address:
//      A15:A6 = RA14:RA5     (10 bits -> 1024 stamps per device)
//      A5:A3  = RA4:RA2      (the stamp row, VMATCH's `line`)
//      A2     = /8HD6H       A1 = /4HD2H       A0 = /2H
//
//  so `b = 4*(/8HD6H) + 2*(/4HD2H) + 1*(/2H)` runs a clean 0..7 starting at
//  n = 14 — exactly where 8HD6H (= VMATCH OUTCLK, = the 15V clock) installs the
//  new RA14:2 and MOA17/16.  One byte address per two counts, 8 bytes = the
//  16 pixels of one object:
//
//      n  | 14,15 | 0,1 | 2,3 | 4,5 | 6,7 | 8,9 | 10,11 | 12,13
//      b  |   0   |  1  |  2  |  3  |  4  |  5  |   6   |   7
//
//  The byte index is decoded from the live /8HD6H, /4HD2H and /2H, never from a
//  counter.  That matters at the line wrap: 456 is not a multiple of 16, the
//  group beginning at H = 448 is only 8 counts long, and the real sweep jumps
//  (b = 4 at H = 454, b = 5 at H = 0, b = 2 at H = 2) instead of running on to
//  6 and 7.  A counter would invent bytes the board never reads.
//
//  The devices:
//      13V LS175, CK = PVCLK rising (n = 5 -> 6):  Q0 = RAMD11, Q1 = RAMD10
//      15V F74,   CK = 8HD6H rising (n = 13 -> 14): MOA17 = 13V Q0,
//                                                   MOA16 = 13V Q1
//      {MOA17, MOA16} = code[11:10], taken from the MOP word and republished in
//      step with RA14:2.  It bypasses VMATCH, which is why the row carry stops
//      at bit 9 on the board (see MO_CODE_CARRY).
//
//      {MOA17,MOA16} | device | sprites offset | codes
//            00      |  14R   |    0x00000     | 0x000-0x3FF
//            01      |  10R   |    0x10000     | 0x400-0x7FF
//            10      |  14T   |    0x20000     | 0x800-0xBFF
//            11      | 10T — not installed     | 0xC00-0xFFF
//
//  Socket 10T is empty, so no ROM drives the bus and the pull-ups leave it at
//  0xFF = pen 0 = transparent on both nibbles.  This module therefore
//  suppresses the memory request for that device and forces the pixel byte to
//  0xFF, so `mo_addr` never reaches 0x30000..0x3FFFF.
//
//  The pixels come out of the 7T/7R high nibble — the same two F374s as the
//  playfield, split across sheets 3 and 4.  They latch on /1H rising (entry
//  into an even count) and their Q pins are wire-tied, 7T driving while 1H = 0
//  and 7R while 1H = 1.  D7:D4 is the even pixel, D3:D0 the odd one, pen bit 3
//  = the nibble's MSB, and the raw ROM byte is already the active-low pen bus,
//  so nothing here inverts.
//============================================================================

module badlands_mo_fetch #(
	// 0 = the schematic (code[11:10] bypasses the row adder through 13V/15V),
	// 1 = MAME (the whole 12-bit code is incremented).  They differ only when
	// code[9:0] + row crosses 1024.
	parameter bit MO_CODE_CARRY = 1'b0
)(
	input  logic        clk,
	input  logic        reset,
	input  logic        ce_7m,

	input  logic [2:0]  h,             // 4H, 2H, 1H
	input  logic        h8d6h,         // the 8HD6H level
	input  logic        h8d6h_rise,    // ce_7m-aligned 8HD6H 0 -> 1
	input  logic        h8d6h_fall,    // ce_7m-aligned 8HD6H 1 -> 0
	input  logic        pvclk_rise,    // 13V LS175 clock

	input  logic [11:10] ramd,         // 13V D0 = RAMD11, D1 = RAMD10

	input  logic [12:0] ra,            // VMATCH RA14:RA2, registered at OUTCLK
	input  logic [12:0] ra_next,       // the same, combinational
	input  logic [1:0]  code_hi_carry,      // MAME's carried code[11:10]
	input  logic [1:0]  code_hi_carry_next,

	// ---- gfx client (sprites region) -------------------------------------
	output logic        mo_req,
	output logic [17:0] mo_addr,
	input  logic        mo_ack,
	input  logic [7:0]  mo_data,

	// ---- outputs ---------------------------------------------------------
	output logic [3:0]  mo_pen_n,      // /MO3:0 — active low
	output logic [1:0]  moa,           // {MOA17, MOA16} (observation)
	output logic        mo_late
);

	// =====================================================================
	// 1. 13V LS175 and 15V F74 — the chip selects
	// =====================================================================
	logic [1:0] q13v;      // {Q0, Q1} = {code[11], code[10]}
	always_ff @(posedge clk) begin
		if (reset)           q13v <= 2'b00;
		else if (pvclk_rise) q13v <= {ramd[11], ramd[10]};
	end

	logic [1:0] moa_r;
	always_ff @(posedge clk) begin
		if (reset)            moa_r <= 2'b00;
		else if (h8d6h_rise)  moa_r <= MO_CODE_CARRY ? code_hi_carry_next : q13v;
	end
	assign moa = MO_CODE_CARRY ? code_hi_carry : moa_r;

	// The byte-0 fetch is issued on the very edge that clocks 15V and VMATCH's
	// OUTCLK register, so its address must use the values those registers are
	// about to take.
	wire [1:0]  moa_now = h8d6h_rise
	                    ? (MO_CODE_CARRY ? code_hi_carry_next : q13v)
	                    : moa;
	wire [12:0] ra_now  = h8d6h_rise ? ra_next : ra;

	// =====================================================================
	// 2. The byte index — decoded from the live strobes
	// =====================================================================
	// A fetch is pulsed on the ce_7m that ends an odd count, so `req` rises on
	// the boundary into the even count that opens the 2-count address window.
	// The three address pins must therefore be evaluated for that next count:
	//    /2H    : H is odd here, so 2H of H+1 is not 2H of H  ->  /2H = h[1]
	//    /4HD2H : 4HD2H = 4H delayed two counts, exactly periodic mod 8 because
	//             456 mod 8 = 0, so 4HD2H(H+1) = bit 2 of ((H-1) mod 8)
	//    /8HD6H : take the level and apply this edge's marker, which is the only
	//             way to get the short wrap group right
	wire [2:0] hm1        = h + 3'd7;             // (H - 1) mod 8
	wire       n4hd2h_nxt = ~hm1[2];
	// verilator lint_off UNUSEDSIGNAL
	wire       unused_hm1 = &{1'b0, hm1[1:0]};
	// verilator lint_on UNUSEDSIGNAL
	wire       n2h_nxt    =  h[1];
	wire       h86_nxt    = h8d6h_rise ? 1'b1 : (h8d6h_fall ? 1'b0 : h8d6h);
	wire [2:0] byte_idx   = { ~h86_nxt, n4hd2h_nxt, n2h_nxt };

	// =====================================================================
	// 3. The fetch
	// =====================================================================
	wire        empty_socket = (moa_now == 2'b11);      // 10T is not installed
	wire        do_fetch     = ce_7m & h[0] & ~empty_socket;
	wire [17:0] fetch_a      = { moa_now, ra_now, byte_idx };

	wire [7:0]  rom_byte;
	wire        rom_valid;
	// verilator lint_off UNUSEDSIGNAL
	wire        unused_ok = rom_valid;
	// verilator lint_on UNUSEDSIGNAL

	badlands_gfx_client #(.AW(18)) u_client (
		.clk(clk), .reset(reset),
		.fetch(do_fetch), .fetch_addr(fetch_a),
		.data(rom_byte), .data_valid(rom_valid), .late(mo_late),
		.req(mo_req), .addr(mo_addr), .ack(mo_ack), .rdata(mo_data)
	);

	// =====================================================================
	// 4. 7T / 7R F374 high nibble — /MO3:0
	// =====================================================================
	// One extra flag rides with the pixel byte: whether the window it came from
	// addressed the empty 10T socket, in which case the bus reads all ones.
	logic       empty_win;
	logic [7:0] pen_byte;
	always_ff @(posedge clk) begin
		if (reset) begin
			empty_win <= 1'b1;
			pen_byte  <= 8'hFF;
		end else if (ce_7m && h[0]) begin
			pen_byte  <= empty_win ? 8'hFF : rom_byte;
			empty_win <= empty_socket;
		end
	end
	assign mo_pen_n = h[0] ? pen_byte[3:0] : pen_byte[7:4];

endmodule
