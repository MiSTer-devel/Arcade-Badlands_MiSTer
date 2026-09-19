`timescale 1ns/1ps
//============================================================================
//  Bad Lands power-on reset and watchdog -- the Q1 2N5306 / R30 / C33 POR node
//  and the 28A 74LS197 of schematic sheet 1.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Structure ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_watchdog.sv` (GPL-3.0,
//  same author, an LS90 doing the same job).
//
//  ---- the POR node -------------------------------------------------------
//      +5V -- R30 100K --+-- C33 0.1uF -- GND
//                        +-- "RES" 2-pin jumper -- GND
//                        +-- Q1 2N5306 base
//      Q1 emitter --+-- R31 220 -- GND
//                   +--------------------> 28A LS197 pin 13 (/CLR)
//  tau = 100 K x 0.1 uF = 10 ms, so the Darlington emitter passes the LS197's
//  V_IH of about 2.0 V roughly 11 ms after +5 V is good.  That figure is the
//  POR_US parameter.
//
//  ---- 28A 74LS197 --------------------------------------------------------
//      D (11) = +5 V ; C (3) = B (10) = A (4) = GND    => the preset is 8
//      CLK1 (8) = VBLANK ; CLK2 (6) = QA (5)           (the two stages cascade)
//      /CLR (13) = the POR node                        (asynchronous)
//      /LOAD (1) = /WDOG through 13J F00 + 15K F04     (asynchronous)
//      QD (12) = /RES ; QB/QC unconnected
//
//  A 74197 counts on the falling edge of its clock input, so the counter
//  advances at the start of each vertical blank.  From the preset 8 the
//  sequence is 9,10,11,12,13,14,15,0, so the 8th VBLANK edge rolls QD to 0.
//
//    * a write to 0xFE0001 (/WDOG) asynchronously loads 8 => QD = 1, running
//    * eight VBLANKs with no write => QD = 0 => /RES asserted
//    * /RES then stays low for the next 8 VBLANKs (counts 0..7) before QD
//      returns high: a repeating 8-frame reset pulse, ~133.5 ms each half at
//      a 262-line frame
//    * at power-up /CLR holds the count at 0 for POR_US, so /RES is asserted
//      from the first instant and the board starts 8 VBLANKs later -- about
//      11 ms + 8 frames ~ 145 ms in all
//
//  ---- the WATCHDOG jumper ------------------------------------------------
//      +5V -- R93 10K --+-- 13J F00 pin 4
//                       +-- WATCHDOG jumper (2-pin, normally open) -- GND
//      /WDOG ------------- 13J F00 pin 5 ; 13J pin 6 -> 15K F04 -> 28A /LOAD
//  Fitting the jumper nails 13J pin 4 low, so 13J pin 6 is high and /LOAD is
//  low permanently: the counter freezes at 8 with QD = 1 and the watchdog is
//  disabled.  That is the input `wdog_jumper`, default 0 = open = the shipped
//  build.
//
//  ---- what /RES reaches, and does not ------------------------------------
//  Exactly four loads: 68000 /HALT (via R58 1 K series), 68000 /RESET (R59),
//  22J F74 half 1 /CLR (the EEPROM unlock flop, so the EEPROM locks), and 19J
//  LS08 pins 4 and 10 (forces the EEPROM /OE low and gates /AUDRES).
//
//  It does not reach the SOS-2, the LETA, the PF bank flop 18S, the
//  MO/priority flops 15V/18R, the PIXCLK flop 14E, the sound IRQ latch 14E or
//  either mailbox flag in 15J; the sound section is reset only by /AUDRES.
//  And because R58/R59 are series resistors rather than pull-ups, a 68000
//  `RESET` instruction cannot pull this net down at all.
//
//  MAME uses a generic WATCHDOG_TIMER with library-default seconds-scale
//  timing, about 20x too long; this module is the schematic's.
//============================================================================

module badlands_watchdog #(
	// clk_sys, for turning POR_US into a cycle count
	parameter int CLK_HZ = 57272724,
	// the R30/C33 rise time, ~11 ms
	parameter int POR_US = 11000
)(
	input  logic       clk,          // clk_sys
	input  logic       ext_reset,    // MiSTer: OSD reset / ioctl_download -- re-runs POR
	input  logic       vblank,       // SOS-2 pin 22, active high
	input  logic       wdog_n,       // LS138 24C Y0, level, active low (write 0xFE0001)
	input  logic       wdog_jumper,  // 1 = jumper fitted = watchdog disabled

	output logic       res_n,        // 28A QD = the /RES net, active low
	output logic       por,          // the POR node, inverted
	output logic [3:0] count         // 28A QD..QA
);

	// ---------------- the POR node ----------------------------------------
	localparam int POR_CYCLES = int'((longint'(POR_US) * CLK_HZ) / 1000000);
	localparam int POR_W      = (POR_CYCLES <= 1) ? 1 : $clog2(POR_CYCLES + 1);

	logic [POR_W-1:0] por_cnt;
	logic             por_done;

	always_ff @(posedge clk) begin
		if (ext_reset) begin
			por_cnt  <= '0;
			por_done <= 1'b0;
		end else if (!por_done) begin
			if (por_cnt >= POR_W'(POR_CYCLES - 1)) por_done <= 1'b1;
			else                                   por_cnt  <= por_cnt + 1'b1;
		end
	end

	// /CLR is low (asserted) while the C33 node has not yet reached V_IH.
	wire clr_n = por_done;
	assign por = ~clr_n;

	// ---------------- 13J F00 + 15K F04 -> /LOAD --------------------------
	//   13J pin 6 = NAND( R93 node , /WDOG ) ; 15K inverts it into /LOAD
	//   => /LOAD = jumper_open AND /WDOG
	//   jumper open   -> R93 node high -> /LOAD = /WDOG  (a write loads 8)
	//   jumper fitted -> R93 node low  -> /LOAD = 0      (frozen at 8)
	wire r93_node = ~wdog_jumper;
	wire load_n   = r93_node & wdog_n;

	// ---------------- 28A 74LS197 ----------------------------------------
	// Counts on the falling edge of CLK1 = VBLANK, i.e. at the start of
	// vertical blank.  /CLR overrides /LOAD, and both are asynchronous.
	logic vblank_q;
	always_ff @(posedge clk) vblank_q <= vblank;
	wire vblank_fall = ~vblank & vblank_q;

	logic [3:0] q;
	always_ff @(posedge clk) begin
		if (!clr_n)           q <= 4'd0;        // /CLR (13) -- POR, dominates
		else if (!load_n)     q <= 4'd8;        // /LOAD (1) -- D,C,B,A = 1,0,0,0
		else if (vblank_fall) q <= q + 4'd1;
	end

	assign count = q;
	assign res_n = q[3];                        // QD (12) = /RES

endmodule
