`timescale 1ns/1ps
//============================================================================
//  Bad Lands — GAL 16S (136074-1003) at 16S, plus the two 16P F00 gates that
//  turn its o14/o15 into /MOW0 and /MOW1.  Schematic sheet 4 grid 16S / 16P.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  GAL16V8 in 16L8 mode: all eight outputs combinational, `.oe = vcc`, no
//  registers, no clock.  Pin 6 is a bare unconnected stub and pin 7 (/LASTH) is
//  wired but appears in no equation; no term is invented for either.
//
//      1  VMD      = 18S/2 Q  (VMATCHD registered on /8H; low = "matches")
//      2..5        = 1H, 2H, 4H, 8H
//      7, 8        = /LASTH (unused), /FIRSTH
//      9  1V
//      11 MOPENNZ  = 24J F20 = NAND(/MO3,/MO2,/MO1,/MO0) = 1 when the pen != 0
//      12, 13      = /MOD0, /MOD1                 (LB DIE0 / DIE1)
//      14, 15      -> 16P F00 with MODCLK -> /MOW0, /MOW1
//      16..19      = /MOL0, /MOR0, /MOL1, /MOR1
//
//  The equations verbatim.  `/FIRSTH` bare is the net (high = NOT in the
//  first-H window); `FIRSTH` bare is its complement (high = in the window).
//
//      /MOD0 = 0  when  (/FIRSTH == 1V)
//      /MOD1 = 0  when  (/FIRSTH != 1V)
//
//      o14   = /FIRSTH & ( /1V + (/VMD & MOPENNZ) )
//      o15   = /FIRSTH & (  1V + (/VMD & MOPENNZ) )
//      /MOW0 = NAND(o14, MODCLK)
//      /MOW1 = NAND(o15, MODCLK)
//
//      /MOL0 = 0  when  1H &  2H & 4H &  8H & /FIRSTH &  1V     ; H mod 16 == 15
//      /MOL1 = 0  when  1H &  2H & 4H &  8H & /FIRSTH & /1V
//      /MOR0 = 0  when  1H & /2H & 4H & /8H &  FIRSTH & /1V     ; H mod 16 == 5
//      /MOR1 = 0  when  1H & /2H & 4H & /8H &  FIRSTH &  1V
//
//  `H mod 16 == 15` never coincides with /FIRSTH low on a 456-count line, so
//  the /FIRSTH term in /MOL is redundant on this board.  It is kept exactly as
//  dumped, in case the raster is ever re-parameterised.
//
//  What they mean: on an even line (1V = 0) DIE1 is active all line, so bank 1
//  is the write bank; /MOL1 loads its write address (the object's X) once per
//  group and /MOW1 fires only where the entry matches this line and the pen
//  != 0 — transparency and vertical clipping done in hardware, one pixel at a
//  time.  /MOW0 is asserted unconditionally while DIE0 is inactive, so bank 0,
//  the display bank, is overwritten with the data-in path's idle value in the
//  same pixel period in which it is read out: the clear-on-read pass.  On an
//  odd line everything swaps.  /FIRSTH (H = 454..5, entirely inside horizontal
//  blanking) suppresses every write and every address load; it is the bank-swap
//  guard band, and 1V changes in the middle of it, which is why the DIE
//  equations are written as XORs with /FIRSTH.
//============================================================================

module badlands_lb_ctrl (
	// ---- GAL 16S inputs ---------------------------------------------------
	input  logic       vmd,        // pin 1  — 18S/2 Q, low = this entry matches
	input  logic [3:0] h,          // pins 2-5 = 1H, 2H, 4H, 8H
	input  logic       lasth_n,    // pin 7  — wired but used by no term
	input  logic       firsth_n,   // pin 8
	input  logic       v1,         // pin 9
	input  logic       mopennz,    // pin 11 — 24J F20, 1 when the MO pen != 0
	input  logic       modclk,     // 16P F00's other input

	// ---- GAL 16S / 16P outputs -------------------------------------------
	output logic       mod0_n,     // pin 12 -> LB DIE0
	output logic       mod1_n,     // pin 13 -> LB DIE1
	output logic       mow0_n,     // 16P F00 pins 9,10 -> 8
	output logic       mow1_n,     // 16P F00 pins 13,12 -> 11
	// The ungated 16S pins 14 and 15.  /MOW = NAND(o1x, MODCLK) is a pulse one
	// MODCLK-high phase wide, and that phase straddles the H count boundary
	// (ph 6,7 of count k and ph 0,1 of count k+1) while o1x itself changes at
	// the boundary.  The line buffer therefore takes the DCLK rising edge with
	// o1x sampled in the count that owns that edge — the only assignment that
	// gives one write per count with self-consistent data.
	output logic       o14,
	output logic       o15,
	output logic       mol0_n,     // pin 16
	output logic       mor0_n,     // pin 17
	output logic       mol1_n,     // pin 18
	output logic       mor1_n      // pin 19
);

	wire h1 = h[0], h2 = h[1], h4 = h[2], h8 = h[3];

	// /LASTH is an input pin of the real part and is deliberately unused.
	// verilator lint_off UNUSEDSIGNAL
	wire unused_lasth = lasth_n;
	// verilator lint_on UNUSEDSIGNAL

	// The transparency + vertical-clip term shared by o14 and o15.
	wire draw = ~vmd & mopennz;

	assign mod0_n = ~(firsth_n == v1);
	assign mod1_n = ~(firsth_n != v1);

	assign o14 = firsth_n & (~v1 | draw);
	assign o15 = firsth_n & ( v1 | draw);

	assign mow0_n = ~(o14 & modclk);
	assign mow1_n = ~(o15 & modclk);

	assign mol0_n = ~( h1 &  h2 & h4 &  h8 &  firsth_n &  v1);
	assign mol1_n = ~( h1 &  h2 & h4 &  h8 &  firsth_n & ~v1);
	assign mor0_n = ~( h1 & ~h2 & h4 & ~h8 & ~firsth_n & ~v1);
	assign mor1_n = ~( h1 & ~h2 & h4 & ~h8 & ~firsth_n &  v1);

endmodule
