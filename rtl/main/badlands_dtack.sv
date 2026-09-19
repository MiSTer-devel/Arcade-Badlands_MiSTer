`timescale 1ns/1ps
//============================================================================
//  Bad Lands /DTACK -- the 28R 74LS163A wait-state sequencer of schematic
//  sheet 2, plus the 19J LS08 that ORs in the video RAM's /RTIME.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later;
//  see LICENSE, NOTICE.md.  Structure ported from
//  `Arcade-Blasteroids_MiSTer/rtl/main/blstroid_dtack.sv` (GPL-3.0,
//  same author, a 12C LS163A doing the same job).
//
//  ---- the part, as sheet 2 wires it --------------------------------------
//    6,5,4,3  D,C,B,A  = /RAM , /AUDRESD , /EEROM , /AUDRESD
//    9        /LOAD    = QD (pin 11)          -- the counter loads itself
//    1        /CLR     = AS (high while /AS asserted; synchronous clear)
//    2        CLK      = /7M
//    7        ENP      = 10J F04 pin 2 = /RCO
//    10       ENT      = /VPA
//    15       RCO      -> 10J F04 pin 1
//
//    19J LS08: pin 2 = /RTIME (GAL 21R pin 14), pin 1 = /RCO, pin 3 = /DTACK
//      => /DTACK = /RTIME AND /RCO, i.e. asserted when RCO = 1 or /RTIME = 0.
//
//  While /AS is inactive the '163 is cleared on every /7M edge.  On /AS the
//  first /7M edge loads V = (D,C,B,A).  If V >= 8 the counter counts up to 15,
//  where RCO asserts /DTACK and ENP (= /RCO) stops it.  If V < 8, QD = 0 keeps
//  /LOAD asserted and the counter reloads forever, so /DTACK must then come
//  from /RTIME.  ENT = /VPA kills the counter during an interrupt acknowledge,
//  which is therefore terminated by the 68000's own VPA path and nothing else.
//
//  ---- the wait table -----------------------------------------------------
//  A 68000 cycle is 4 clocks plus one per counter increment.
//
//   | access                                        |/RAM|/AUDRESD|/EEROM| V  | inc | cycle |
//   | ROM, /CRAM, all 8 /MISC strobes, /MISC2 read, |  1 |   1    |  1   | 15 |  0  | 4 clk |
//   |   and anything undecoded                      |    |        |      |    |     | 0 wait|
//   | EEPROM 0xFD0000                               |  1 |   1    |  0   | 13 |  2  | 6 clk |
//   | audio-reset write 0xFC0000                    |  1 |   0    |  1   | 10 |  5  | 9 clk |
//   | video/work RAM 0xFFE000+                      |  0 |   1    |  1   |  7 |never| /RTIME|
//
//  The "anything undecoded" row is what makes the program's `tst.l $00070000`
//  and `jmp $00070000` harmless: 0x070000 has A23 = 0, so /RAM, /EEROM and
//  /AUDRESD are all inactive, the counter loads 15, RCO asserts at once and
//  the cycle completes with 0 wait states.  The 68000 does not hang and the
//  watchdog is not invoked, so no special case is needed here.
//
//  /AUDRESD = R/W + /MISC2 (17J F32 pins 10, 9 -> 8) is low only on a write to
//  0xFC0000.  A read of 0xFC0000 leaves it high and gets the 0-wait row, so
//  the `clr.w $00FC0000` at reset takes 0 waits on its read and 5 on its
//  write.
//
//  The '163 is clocked by /7M, so it steps on `ce_7m_n` -- the same tick
//  badlands_cpu uses for fx68k's enPhi1.  No derived clocks.
//============================================================================

module badlands_dtack
(
	input  logic clk,        // clk_sys
	input  logic ce_7m_n,    // /7M rising (pcnt == 4) -- 28R pin 2

	input  logic as,         // active-high copy of /AS  -- 28R pin 1 (/CLR)
	input  logic ram_n,      // /RAM     -- 28R pin 6 (D)
	input  logic audresd_n,  // /AUDRESD -- 28R pins 5 (C) and 3 (A)
	input  logic eerom_n,    // /EEROM   -- 28R pin 4 (B)
	input  logic vpa_n,      // /VPA     -- 28R pin 10 (ENT)
	input  logic rtime_n,    // GAL 21R pin 14 -> 19J LS08 pin 2

	output logic dtack       // active high (= ~/DTACK), for badlands_cpu
);

	logic [3:0] q;

	wire [3:0] load_val = {ram_n, audresd_n, eerom_n, audresd_n};  // D,C,B,A

	// The net tied to ENT is /VPA, so ENT is high (enabled) while /VPA is
	// deasserted and low during an IACK.  On the LS163A the ripple carry is
	// RCO = QA.QB.QC.QD.ENT, so /VPA kills RCO as well as the count.
	wire ent    = vpa_n;             // pin 10
	wire rco    = (q == 4'hF) & ent; // pin 15
	wire rco_n  = ~rco;              // 10J F04 pin 2 -> ENP (pin 7)
	wire load_n = q[3];              // pin 9 /LOAD = QD (pin 11)

	always_ff @(posedge clk) begin
		if (ce_7m_n) begin
			if (!as)                q <= 4'd0;         // synchronous /CLR = AS low
			else if (!load_n)       q <= load_val;     // synchronous LOAD
			else if (rco_n & ent)   q <= q + 4'd1;     // ENP & ENT
			// else hold: ENP low (RCO reached) or ENT low (IACK)
		end
	end

	// 19J LS08: /DTACK = /RTIME AND /RCO  =>  asserted when RCO or !/RTIME
	assign dtack = rco | ~rtime_n;

endmodule
