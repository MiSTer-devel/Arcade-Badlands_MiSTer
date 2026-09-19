`timescale 1ns/1ps
//============================================================================
//  Bad Lands — GAL 26R (136074-2000) at 26R, the horizontal strobe generator,
//  plus the 13J F00 inverter that makes `8HD6H` and the 13V LS175 `/HSYNC`
//  delay network (R98/R99/R100).  Schematic sheets 1 (grid 26R) and 4.
//
//  Copyright (C) 2026 the Bad Lands MiSTer core authors.  GPL-3.0-or-later.
//
//  The device is a GAL16V8 in 16R6 mode.  CLK (pin 1) = 7M and /OE (pin 11) =
//  GND, so the part never tri-states.  VBLANK is an ordinary input on pin 12,
//  256H an input on pin 19, and pins 2-9 are 1H,2H,4H,8H,16H,32H,64H,128H.
//  The outputs are all registered on 7M rising:
//      13 /BLANK   14 /HSYNCDO   15 /8HD6H   16 PVCLK   17 /FIRSTH   18 /LASTH
//
//  In the fuse dump's `/rfN := <sum>` form the flip-flop stores `<sum>` and the
//  pin carries its complement.  Five of the six nets are printed with a leading
//  slash, so for them pin == net == NOT(sum).  PVCLK has no slash, so its pin
//  equals NOT(sum16) — which is why the sum below is written for /PVCLK.
//
//  The one-count register shift is why this module registers the sum of the
//  CURRENT h at every ce_7m: the GAL's flip-flops capture the H value valid
//  before the 7M rising edge and the SOS-2's H counter advances on that same
//  edge, so the pin during count N carries the sum evaluated for count N-1.
//  `ce_7m` marks the last clk_sys cycle of count `h`, so `q <= sum(h)` there
//  puts sum(h) on the pin throughout count h+1, which is what the GAL does.
//
//  Resulting assertion sets (`n = H mod 16`):
//
//     /BLANK    low   H = 0..7 and 344..455, plus all of VBLANK  (120 + VBLANK)
//     visible   high  H = 8..343                                 336 counts
//     /HSYNCDO  low   H = 359..390                               32 counts
//     /HSYNC    low   H = 374..405   (13V, +15 counts)           32 counts
//     /8HD6H    low   n in {14,15,0,1,2,3,4,5}                    8 of 16
//      8HD6H    high  n in {14,15,0,1,2,3,4,5}   (13J inverter)
//     PVCLK     high  n in {6,7}  (rises 5->6, falls 7->8)        2 of 16
//     /FIRSTH   low   H = 454,455,0,1,2,3,4,5                     8 counts
//     /LASTH    low   H = 448..455                                8 counts
//
//  8HD6H in active-high sense is literally 8H delayed by 6 H counts, which
//  confirms the 1H/2H/4H/8H pin map and H_TOTAL = 456 (the /FIRSTH set is
//  contiguous only across a 455 -> 0 wrap).
//
//  The /HSYNC delay: 13V 74LS175 is clocked by PVCLK rising (once per 16
//  counts, at the n = 5 -> 6 boundary).  /HSYNCDO enters stage 2 (D2); Q2 feeds
//  R98 and stage 3 (D3); Q3 feeds R100.
//
//     R99  (tap = /HSYNCDO, 0 delay)  not installed   -> HSYNC_TAP = 0
//     R98  (tap = 13V Q2, 1 PVCLK)    installed       -> HSYNC_TAP = 1 (default)
//     R100 (tap = 13V Q3, 2 PVCLK)    not installed   -> HSYNC_TAP = 2
//
//  Because both /HSYNCDO and PVCLK are registered on the same 7M edges, 13V
//  samples the value /HSYNCDO held during the count before the edge, so the
//  shift is quantised to H = 6 (mod 16): the edge at 374 sees /HSYNCDO low
//  during 373 and the edge at 406 sees it high during 405.  /HSYNC is
//  therefore low for H = 374..405 — a uniform +15-count shift, 32 counts wide.
//
//  `hblank` is a core contract signal, not a PCB net.  The board has one
//  blanking net, /BLANK, and it already contains VBLANK (the `+ i12` product
//  term).  MiSTer's arcade_video wants separate hblank and vblank, so this
//  module also registers the same sum without the VBLANK term and exports it.
//  Nothing inside the pipeline uses it; /BLANK (`blank_n`) is what clears
//  1K/5K.
//============================================================================

module badlands_hstrobes #(
	// 0 = R99 (/HSYNCDO raw), 1 = R98 (13V Q2, the fitted option),
	// 2 = R100 (13V Q3).
	parameter int HSYNC_TAP = 1
)(
	input  logic       clk,        // clk_sys
	input  logic       reset,      // synchronous, active high; the GAL has no
	                               // reset pin, so this is an initialiser only
	input  logic       ce_7m,      // marks the last clk_sys cycle of count `h`

	// ---- GAL 26R inputs (pins 2-9, 12, 19) -------------------------------
	input  logic [8:0] h,          // {256H,128H,64H,32H,16H,8H,4H,2H,1H}
	input  logic       vblank,     // pin 12, SOS-2 pin 22, active high

	// ---- GAL 26R pins (all registered on 7M rising) ----------------------
	output logic       blank_n,    // pin 13  /BLANK
	output logic       hsyncdo_n,  // pin 14  /HSYNCDO
	output logic       h8d6h_n,    // pin 15  /8HD6H
	output logic       pvclk,      // pin 16  PVCLK   (no slash: active high)
	output logic       firsth_n,   // pin 17  /FIRSTH
	output logic       lasth_n,    // pin 18  /LASTH

	// ---- 13J F00 pins 1,2 -> 3 (a NAND used as an inverter) --------------
	output logic       h8d6h,      // 8HD6H = NOT(/8HD6H)

	// ---- 13V LS175 + R98/R99/R100 ----------------------------------------
	output logic       hsync_n,    // /HSYNC to the monitor and to 8J

	// ---- core contract (not PCB nets) ------------------------------------
	output logic       hblank,     // /BLANK's sum with the VBLANK term removed
	output logic       pvclk_rise, // one ce_7m-aligned pulse, PVCLK 0 -> 1
	output logic       pvclk_fall, // one ce_7m-aligned pulse, PVCLK 1 -> 0
	output logic       h8d6h_rise, // one ce_7m-aligned pulse, 8HD6H 0 -> 1
	output logic       h8d6h_fall  // one ce_7m-aligned pulse, 8HD6H 1 -> 0
);

	// =====================================================================
	// 1. GAL 26R — the equations verbatim in net names.  Every literal below
	//    is one product term of the fuse dump, in the same order.
	// =====================================================================
	wire h1h   =  h[0], n1hn  = ~h[0];
	wire n2h   = ~h[1], h2h   = h[1];
	wire n4h   = ~h[2], h4h   = h[2];
	wire n8h   = ~h[3], h8h   = h[3];
	wire n16h  = ~h[4], h16h  = h[4];
	wire n32h  = ~h[5], h32h  = h[5];
	wire n64h  = ~h[6], h64h  = h[6];
	wire n128h = ~h[7], h128h = h[7];
	wire n256h = ~h[8], h256h = h[8];

	// BLANK    := /1H & /8H & /16H & /32H & /64H & /128H & /256H
	//           + /2H & /8H & /16H & /32H & /64H & /128H & /256H
	//           + /4H & /8H & /16H & /32H & /64H & /128H & /256H
	//           +  1H &  2H &  4H &  16H &  64H & 256H
	//           +  8H & 16H & 64H & 256H
	//           + 32H & 64H & 256H
	//           + 128H & 256H
	//           + VBLANK
	wire blank_sum =
		  (n1hn & n8h & n16h & n32h & n64h & n128h & n256h)
		| (n2h  & n8h & n16h & n32h & n64h & n128h & n256h)
		| (n4h  & n8h & n16h & n32h & n64h & n128h & n256h)
		| (h1h & h2h & h4h & h16h & h64h & h256h)
		| (h8h & h16h & h64h & h256h)
		| (h32h & h64h & h256h)
		| (h128h & h256h)
		| vblank;

	// The same sum WITHOUT the VBLANK product term — the core contract signal.
	wire hblank_sum =
		  (n1hn & n8h & n16h & n32h & n64h & n128h & n256h)
		| (n2h  & n8h & n16h & n32h & n64h & n128h & n256h)
		| (n4h  & n8h & n16h & n32h & n64h & n128h & n256h)
		| (h1h & h2h & h4h & h16h & h64h & h256h)
		| (h8h & h16h & h64h & h256h)
		| (h32h & h64h & h256h)
		| (h128h & h256h);

	// HSYNCD0  := /2H & /8H & /16H & /32H & /64H & 128H & 256H
	//           + /4H & /8H & /16H & /32H & /64H & 128H & 256H
	//           +  2H &  4H & 32H & 64H & /128H & 256H
	//           +  8H & 32H & 64H & /128H & 256H
	//           + 16H & 32H & 64H & /128H & 256H
	wire hsyncd0_sum =
		  (n2h & n8h & n16h & n32h & n64h & h128h & h256h)
		| (n4h & n8h & n16h & n32h & n64h & h128h & h256h)
		| (h2h & h4h & h32h & h64h & n128h & h256h)
		| (h8h & h32h & h64h & n128h & h256h)
		| (h16h & h32h & h64h & n128h & h256h);

	// 8HD6H    := /1H & /2H & /8H  +  /4H & /8H  +  2H & 4H & 8H  +  1H & 4H & 8H
	wire h8d6h_sum =
		  (n1hn & n2h & n8h)
		| (n4h & n8h)
		| (h2h & h4h & h8h)
		| (h1h & h4h & h8h);

	// /PVCLK   := 8H + (1H & 2H) + (/1H & /2H) + /4H
	//   i.e.  PVCLK := /8H & 4H & (1H XOR 2H)      (the PIN is the complement)
	wire pvclk_sum_n = h8h | (h1h & h2h) | (n1hn & n2h) | n4h;
	wire pvclk_sum   = ~pvclk_sum_n;

	// FIRSTH   := /1H & /2H & /8H & /16H & /32H & /64H & /128H & /256H
	//           + /4H & /8H & /16H & /32H & /64H & /128H & /256H
	//           +  2H &  4H & /8H & /16H & /32H & 64H & 128H & 256H
	//           +  1H &  4H & /8H & /16H & /32H & 64H & 128H & 256H
	wire firsth_sum =
		  (n1hn & n2h & n8h & n16h & n32h & n64h & n128h & n256h)
		| (n4h & n8h & n16h & n32h & n64h & n128h & n256h)
		| (h2h & h4h & n8h & n16h & n32h & h64h & h128h & h256h)
		| (h1h & h4h & n8h & n16h & n32h & h64h & h128h & h256h);

	// LASTH    := /1H & /8H & /16H & /32H & 64H & 128H & 256H
	//           + /2H & /8H & /16H & /32H & 64H & 128H & 256H
	//           + /4H & /8H & /16H & /32H & 64H & 128H & 256H
	//           +  1H &  2H &  4H & 8H & 16H & 32H & /64H & 128H & 256H
	wire lasth_sum =
		  (n1hn & n8h & n16h & n32h & h64h & h128h & h256h)
		| (n2h  & n8h & n16h & n32h & h64h & h128h & h256h)
		| (n4h  & n8h & n16h & n32h & h64h & h128h & h256h)
		| (h1h & h2h & h4h & h8h & h16h & h32h & n64h & h128h & h256h);

	// --- the six D flip-flops, clocked by 7M rising = the count boundary ---
	logic q_blank, q_hblank, q_hsyncd0, q_8hd6h, q_pvclk, q_firsth, q_lasth;
	always_ff @(posedge clk) begin
		if (reset) begin
			q_blank <= 1'b1; q_hblank <= 1'b1; q_hsyncd0 <= 1'b0;
			q_8hd6h <= 1'b0; q_pvclk  <= 1'b0; q_firsth  <= 1'b0;
			q_lasth <= 1'b0;
		end else if (ce_7m) begin
			q_blank   <= blank_sum;
			q_hblank  <= hblank_sum;
			q_hsyncd0 <= hsyncd0_sum;
			q_8hd6h   <= h8d6h_sum;
			q_pvclk   <= pvclk_sum;
			q_firsth  <= firsth_sum;
			q_lasth   <= lasth_sum;
		end
	end

	// --- pins: every net whose name carries a slash is the complement ------
	assign blank_n   = ~q_blank;
	assign hsyncdo_n = ~q_hsyncd0;
	assign h8d6h_n   = ~q_8hd6h;
	assign pvclk     =  q_pvclk;     // no slash: the pin is active high
	assign firsth_n  = ~q_firsth;
	assign lasth_n   = ~q_lasth;
	assign hblank    =  q_hblank;

	// 13J 74F00 pins 1 = /8HD6H, 2 = +5 V, 3 = 8HD6H (a NAND as an inverter).
	assign h8d6h = ~h8d6h_n;

	// Edge markers, one ce_7m cycle wide, for the consumers that use these nets
	// as clocks on the board (VMATCH INCLK/OUTCLK, 13V, 15V, 17K).  A signal
	// registered at `ce_7m` rises on that same clk edge, so the marker is "the
	// sum is about to be loaded and the register is currently 0".
	// 8HD6H tracks q_8hd6h: pin 15 carries /8HD6H = NOT q_8hd6h and 13J inverts
	// it back, so an 8HD6H rising edge is q_8hd6h going 0 -> 1.
	assign pvclk_rise = ce_7m &  pvclk_sum & ~q_pvclk;
	assign pvclk_fall = ce_7m & ~pvclk_sum &  q_pvclk;
	assign h8d6h_rise = ce_7m &  h8d6h_sum & ~q_8hd6h;
	assign h8d6h_fall = ce_7m & ~h8d6h_sum &  q_8hd6h;

	// =====================================================================
	// 2. 13V 74LS175 — the /HSYNC delay line, clocked by PVCLK rising
	// =====================================================================
	// PVCLK rises on the clk edge that ends the count with n = 5, and on that
	// same edge the GAL reloads /HSYNCDO.  A 74LS175 clocked by PVCLK
	// therefore captures the value /HSYNCDO held during that n = 5 count, i.e.
	// `hsyncdo_n` before this cycle's update.
	logic q2, q3;
	always_ff @(posedge clk) begin
		if (reset) begin
			q2 <= 1'b1;
			q3 <= 1'b1;
		end else if (pvclk_rise) begin
			q2 <= hsyncdo_n;   // pre-update value = /HSYNCDO during count n = 5
			q3 <= q2;
		end
	end

	always_comb begin
		case (HSYNC_TAP)
			0:       hsync_n = hsyncdo_n;   // R99  — not installed
			2:       hsync_n = q3;          // R100 — not installed
			default: hsync_n = q2;          // R98  — the fitted option
		endcase
	end

endmodule
