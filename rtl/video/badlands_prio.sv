`timescale 1ns/1ps
//============================================================================
//  Bad Lands — GAL 9N (136074-1004) at 9N and the 18R F74 half 2 that
//  registers its decision.  Schematic sheet 4 grids 9N, 18R.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The device is a GAL16V8 in 16L8 mode; four of the eight outputs are used and
//  macrocells 13-16 are configured as inputs.
//
//      1            7M
//      2,3,4,5      /CRAMA1, /CRAMA2, /CRAMA3, /CRAMA4 — the pen on the bus
//      6,7,8        /PF0, /PF1, /PF2 — wired, but referenced by no term
//      9            /PF3 — taken directly from 7T/7R, not from the shared bus
//      11           LB DO7 — the buffered, inverted motion-object priority bit
//      12           D8 -> the 68000 data bus (tri-state)
//      13,14,15     /MISC2, R/W, AUDFULL
//      16           /MO, fed back from 18R pin 8
//      17           PRIO -> 10K D0 -> colour RAM A8
//      18           not connected — internal feedback only
//      19           -> 18R pin 12 (D)
//
//  The fuse equations:
//      o12    = i15                       D8    = AUDFULL
//      o12.oe = /i13 & i14                D8.oe = (/MISC2 low) AND (R/W high)
//      o17    = i16                       PRIO  = /MO   (a buffer)
//      o18    = i9                        pin18 = /PF3  (internal only)
//      /o19   = /i1 & i11 & /o18
//             + /i1 & i2 & i3 & i4 & i5
//
//      pin19  = NOT[ (/7M AND DO7 AND PF3) OR (/7M AND pen == 0) ]
//
//  with PF3 = NOT(/PF3) and pen[3:0] = NOT(/CRAMA4:1).  DO7 = NOT(priority),
//  because 17K supplies the /Q of RAMD3.
//
//  The two-phase 7M protocol is the point of the `/7M` literal in both terms.
//  18R is clocked by 14M, so it re-decides twice per pixel, once in each 7M
//  half-period.  With 7M high over ph 0..3 of an H count and low over ph 4..7,
//  and 14M rising at ph 0 and ph 4:
//    * while 7M = 1 both product terms are killed, so pin19 = 1 and the edge at
//      ph 4 sets /PF = 1, /MO = 0 — the motion-object layer is forced onto
//      /CRAMA7:1 for ph 4..7 whatever it contains, which is how the GAL gets to
//      see the MO pen and its priority bit at all;
//    * while 7M = 0 the GAL evaluates the real rule on that MO pen, on DO7 and
//      on the playfield's own pen bit 3 (pin 9 is driven directly, so it is
//      available even while the MO layer owns the bus), and the edge at ph 0 of
//      the next count publishes the winner for ph 0..3 — the window in which 7N
//      also holds the playfield index and the LB holds its DO, and at the end
//      of which 10K latches the result.
//
//  The rule this produces: the motion object wins when mo_pen != 0 AND
//  ( priority OR NOT pf_pen[3] ), which is what MAME's `screen_update_badlands`
//  does.
//
//  Pins 6, 7, 8 (/PF0:2) are wired and used by no term, so the playfield's
//  transparency is never tested: a playfield pen of 0 is a normal opaque colour
//  on this board, unlike the motion objects.
//============================================================================

module badlands_prio (
	input  logic       clk,
	input  logic       reset,

	// ---- GAL 9N inputs ----------------------------------------------------
	input  logic       m7,          // pin 1
	input  logic [6:0] crama_n,     // {/CRAMA7 .. /CRAMA1}; pins 2-5 use [3:0]
	input  logic       pf3_n,       // pin 9  — direct from 7T/7R
	input  logic       do7,         // pin 11 — LB DO7 = NOT(stored priority)

	// ---- the D8 tri-state term, brought out as a port so badlands_video can
	//      hand it to the main bus: AUDFULL is read on D8 at 0xFC0000 -------
	input  logic       misc2_n,     // pin 13
	input  logic       rw,          // pin 14  (1 = read)
	input  logic       audfull,     // pin 15
	output logic       d8,          // pin 12 value
	output logic       d8_oe,       // pin 12 output enable

	// ---- 18R F74 half 2, CK = 14M ----------------------------------------
	// `ce_m14_rise` from badlands_sos2_sync is high for the one clk_sys cycle
	// whose ending posedge is the 14M rising edge (ph 3 and ph 7), so the flop
	// samples pin 19 as it stood over ph 3 (a 7M = 1 half) and over ph 7 (a
	// 7M = 0 half) — the two-phase protocol above.
	input  logic       ce_m14_rise,

	output logic       pf_n,        // 18R Q  (pin 9) = /PF -> 7N /OE
	output logic       mo_n,        // 18R /Q (pin 8) = /MO -> LB DOE, 9N pin 16
	output logic       prio,        // 9N pin 17 = /MO -> 10K D0 -> CRAM A8
	output logic       pin19        // 9N pin 19 (observation point)
);

	// ---- pin 12: AUDFULL on D8, enabled by a READ of /MISC2 --------------
	assign d8    = audfull;
	assign d8_oe = ~misc2_n & rw;

	// ---- pin 19 ----------------------------------------------------------
	//   /o19 = /7M & DO7 & /o18      with o18 = /PF3, so /o18 = PF3
	//        + /7M & /CRAMA1 & /CRAMA2 & /CRAMA3 & /CRAMA4   (i.e. pen == 0)
	wire m7_n     = ~m7;
	wire pf3      = ~pf3_n;                 // the true playfield pen bit 3
	wire pen_zero = &crama_n[3:0];          // /CRAMA4:1 all high  <=>  pen == 0
	// /CRAMA7:5 (the palette) reaches the GAL on no pin at all: the whole bus is
	// taken as one port here for readability, but only pins 2-5 exist.
	// verilator lint_off UNUSEDSIGNAL
	wire unused_pal = &{1'b0, crama_n[6:4]};
	// verilator lint_on UNUSEDSIGNAL
	wire o19_n    = (m7_n & do7 & pf3) | (m7_n & pen_zero);
	assign pin19  = ~o19_n;

	// ---- 18R 74F74 half 2: D = pin 19, CK = 14M rising, no reset ---------
	logic q18r;
	always_ff @(posedge clk) begin
		if (reset)           q18r <= 1'b1;  // power-up only: the F74's /S and
		else if (ce_m14_rise) q18r <= pin19; // /CLR are both tied to +5 V
	end

	assign pf_n = q18r;
	assign mo_n = ~q18r;
	assign prio = ~q18r;                    // o17 = i16 = /MO

endmodule
