`timescale 1ns/1ps
//============================================================================
//  Bad Lands (Atari Games, 1989) -- the main <-> sound mailboxes: the 15F and
//  13F 74LS374 latches and the two 74F74 flags in 15J, from schematic sheet 5.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.  See LICENSE, NOTICE.md.
//
//  Not an Atari SCOM.  The Blasteroids / Xybots / Vindicators cores model a
//  pair of SCOM customs exchanging a byte serially (`blstroid_sound_comm.sv`,
//  GPL-3.0, same author); only that module's shape -- one clock, edge strobes
//  on both sides -- is borrowed.  Sheet 5 draws two plain LS374s and one 74F74
//  package: no serial frame, no handshake protocol, no reset of any kind.
//
//  Wiring:
//   15F LS374  main -> sound: CP = /AUDWR (0xFE8000 W), /OE = /RDP ($2802),
//                             D = 68000 D15:8, Q = SD7:0
//   13F LS374  sound -> main: CP = /WRP ($2A02 W), /OE = /AUDRD (0xFEA000 R),
//                             D = SD7:0, Q = 68000 D15:8
//   15J 74F74, both halves D = +5 V and /PRE = +5 V:
//     half 1 (pins 9-13): CLK = /WRP,   /CLR = /AUDRD,
//                         /Q = /AUDIRQ  -> 68000 IRQ2 and 16C pin 6
//     half 2 (pins 1-6):  CLK = /AUDWR, /CLR = /RDP,
//                         Q  = AUDFULL  -> GAL 9N -> D8 at 0xFC0000,
//                         /Q = /MAINIRQ -> 6502 /NMI and 16C pin 11
//  The LS138 that makes /RDP and /WRP has no direction term, so a read of
//  $2A02 sets the response flag just as a write does.
//
//  Two things this module must get right:
//   1. No reset reaches 15J or either LS374 -- not /AUDRES, not /RES, not the
//      power-on reset.  A sound reset can leave AUDFULL and /AUDIRQ set from
//      before it, and the 6502 then comes out of reset with a stale command
//      already flagged.  No reset term appears below; the registers merely
//      power up at 0 the way every FPGA flip-flop does.
//   2. The /CLR is a level and it dominates a simultaneous clock, as a 74F74's
//      asynchronous clear does.  If the 68000 writes a command in the cycle the
//      6502 reads one, the 6502 gets the old byte, the new byte is latched into
//      15F, and the flag ends up clear -- the command is delivered but loses its
//      NMI.  That is a real board race, not a modelling artefact.
//
//  The firmware copes: the NMI handler at $57A0 waits for /RDIO bit 5 to clear
//  and then does `lda $280A` (reading the latch also releases the NMI), and the
//  response path at $453E polls bit 5 before and after `sta $2A02`, so the 6502
//  never overwrites an unread response.
//============================================================================

module badlands_mailbox
(
	input  logic       clk,

	// ---- 68000 side ----
	input  logic       audwr_stb,     // 0xFE8000 write (/AUDWR), one clk
	input  logic [7:0] audwr_data,    // D15:8 of that write
	input  logic       audrd_stb,     // 0xFEA000 read  (/AUDRD), one clk
	output logic [7:0] audrd_data,    // 13F Q -> 68000 D15:8
	output logic       audfull,       // 15J half 2 Q  -> D8 at 0xFC0000
	output logic       audirq_n,      // 15J half 1 /Q -> 68000 IRQ2

	// ---- 6502 side ----
	input  logic       rdp_stb,       // /RDP  ($2802 + mirrors), one ce_2h clk
	input  logic       wrp_stb,       // /WRP  ($2A02 + mirrors), one ce_2h clk
	input  logic [7:0] wrp_data,      // SD7:0 at the write
	output logic [7:0] rdp_data,      // 15F Q -> SD7:0
	output logic       mainirq_n      // 15J half 2 /Q -> 6502 /NMI
);

	// No reset term (see the header).  The initialisers are the FPGA power-up
	// state only; the board's is undefined.
	logic [7:0] l15f;                 // main -> sound command latch
	logic [7:0] l13f;                 // sound -> main response latch
	logic       f_full;               // 15J half 2  (AUDFULL / /MAINIRQ)
	logic       f_resp;               // 15J half 1  (/AUDIRQ)
	initial begin l15f = 8'h00; l13f = 8'h00; f_full = 1'b0; f_resp = 1'b0; end

	assign rdp_data   = l15f;
	assign audrd_data = l13f;
	assign audfull    = f_full;
	assign mainirq_n  = ~f_full;
	assign audirq_n   = ~f_resp;

	// `always @(posedge clk)` and not `always_ff`: these registers have no reset,
	// so their power-up value comes from the `initial` above -- and IEEE 1800
	// 9.2.2.4 forbids an always_ff variable being written by another process.
	always @(posedge clk) begin
		logic ff, fr;

		// ---- 15F / 13F LS374 data latches: no clear, /OE does not affect Q ----
		if (audwr_stb) l15f <= audwr_data;
		if (wrp_stb)   l13f <= wrp_data;

		// ---- 15J half 2: CLK = /AUDWR, /CLR = /RDP ----
		ff = f_full;
		if (audwr_stb) ff = 1'b1;
		if (rdp_stb)   ff = 1'b0;     // the asynchronous /CLR dominates
		f_full <= ff;

		// ---- 15J half 1: CLK = /WRP, /CLR = /AUDRD ----
		fr = f_resp;
		if (wrp_stb)   fr = 1'b1;
		if (audrd_stb) fr = 1'b0;     // the asynchronous /CLR dominates
		f_resp <= fr;
	end

endmodule
